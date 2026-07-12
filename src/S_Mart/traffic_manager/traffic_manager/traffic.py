"""TrafficManager — route + index 기반 예약 관리 (교착 감지는 별도 단계).

핵심 개념:
- 경로는 항상 순수 최단(다익스트라, 예약 미반영). 작업노드 사이 페널티/홈 엣지
  차단은 그래프 비용에 이미 반영.
- 예약 테이블: node -> robot_id (누가 점유 중).
- reserve_forward: 경로를 따라 "직진 끝(코너) 또는 예약 가능한 곳"까지만 확보.
  → 예약 단위 = 주행 단위 = 직선 run. (코너 캡: 코너 너머는 안 쥠 → 독점 최소화)
- next_segment: 확보된 구간을 그대로 반환 (구조상 항상 직선 — 분할 불필요).
- arrive/update_position: 통과한 뒤 노드 즉시 release. 멈춘 로봇은 항상 자기
  현재 노드 하나만 점유 → 로봇 2대의 모든 상호 차단이 head-on으로 수렴 (감지 완전).

교착 해소 = 대피(back-off):
- head-on 감지 시 양쪽의 "대피 비용"(상대의 현재+남은 경로 밖 최근접 노드까지
  거리)을 대칭 계산 → 싼 쪽이 양보. 동률이면 laden(적재) 로봇 통과, 최종 id.
- 양보자 새 경로 = [현재→대피 노드] + (대피 노드→목적지 최단 재계산).
  대피 후 대기/우회는 예약 시스템이 자연 처리 (승자는 아무것도 안 함).
- 대피 경로는 같은 노드를 두 번 지날 수 있음 → index 갱신은 항상 전방 탐색.
- 대피 불가(양쪽 다 후보 없음) 시 구식 우회(_reroute, 엣지 차단 포함) 폴백.
"""
from collections import deque


def _direction(g, a, b):
    """격자 진행 방향 → 'E'/'W'/'N'/'S' (좌표 비교)."""
    ax, ay = g.xy(a)
    bx, by = g.xy(b)
    dx, dy = bx - ax, by - ay
    if abs(dx) >= abs(dy):
        return 'E' if dx > 0 else 'W'
    return 'N' if dy > 0 else 'S'


class RobotState:
    def __init__(self, robot_id, route, laden=False):
        self.robot_id = robot_id
        self.route = route          # [노드, ...] 전체 경로 (대피 시 중복 노드 가능)
        self.index = 0              # 현재 위치 = route[index]
        self.reserved_end = 0       # 예약 확보된 마지막 index (index 이상)
        self.laden = laden          # 팔레트 적재 여부 (교착 동률 타이브레이크)
        self.waiting = False        # 현재 대기 중(next_segment가 [])인지
        self.escape_end = None      # 대피 중이면 대피 노드의 route index (도착 시 해제)
                                    # — 대피 중인 로봇의 승자를 연쇄 양보에서 보호


class TrafficManager:
    ARRIVE_RADIUS = 0.12                  # 노드 통과 판정 반경(m)

    def __init__(self, router):
        self.router = router
        self.graph = router.graph
        self.robots = {}                 # robot_id -> RobotState
        self.reservations = {}           # node -> robot_id
        # 승자 우선권(hold): 양보자 -> (승자, 비켜준 노드).
        # 양보 취지는 "승자 먼저"인데 예약은 폴링 레이스라, 패자가 비켜준
        # 노드를 먼저 재예약하면 재대치→재양보 반복 위험. 패자는 대피 완료
        # 후 '승자가 그 노드를 쓸 때까지' 전진 예약을 동결한다.
        # (B안 시뮬 실증 후 이식, 2026-07-10)
        self._hold = {}

    def _hold_active(self, robot_id):
        """hold 유지 판정 + 조건 소멸 시 해제.

        해제: ①내 복귀 경로가 그 노드를 안 지남(레이스 불가 — 즉시 출발)
              ②승자가 통과했거나 그 위에 도착(rem[0] — 목적지인 경우 포함)
              ③승자 경로가 바뀌어 더는 안 감.
        대피 노드는 승자 경로 밖이므로 hold 대기가 승자를 막을 수 없음
        → 승자는 반드시 전진 → hold는 반드시 풀림 (교착 없음)."""
        h = self._hold.get(robot_id)
        if h is None:
            return False
        winner, node = h
        st = self.robots[robot_id]
        if node not in st.route[st.index:]:
            del self._hold[robot_id]
            return False
        st_w = self.robots.get(winner)
        if st_w is not None:
            rem = st_w.route[st_w.index:]
            if node in rem and rem[0] != node:
                return True              # 승자가 아직 접근 중 — 대기 유지
        del self._hold[robot_id]
        return False

    # ── 경로 설정 ─────────────────────────────────────────
    def set_route(self, robot_id, start, goal, laden=False, blocked=None,
                  blocked_edges=None):
        """다익스트라로 경로 생성 → 저장. 시작 노드 즉시 예약. 성공 bool.

        laden: 팔레트 적재 여부.
        blocked / blocked_edges: 폴백 우회용 탐색 제약 (평상시 None).
        """
        route = self.router.shortest_path(start, goal, blocked=blocked,
                                          blocked_edges=blocked_edges)
        if not route:
            return False
        return self._install_route(robot_id, route, laden)

    def _install_route(self, robot_id, route, laden):
        """계산된 route를 로봇에 장착. 기존 예약(현재 위치 제외) 정리. 성공 bool."""
        start = route[0]
        if self.reservations.get(start, robot_id) != robot_id:
            return False                 # 시작 노드가 남에게 점유됨
        if robot_id in self.robots:
            for n in [k for k, v in self.reservations.items()
                      if v == robot_id and k != start]:
                del self.reservations[n]
        st = RobotState(robot_id, route, laden=laden)
        self.robots[robot_id] = st
        self.reservations[start] = robot_id
        return True

    # ── 예약: 직진 끝(코너) or 예약 가능한 곳까지 (코너 캡) ─
    def reserve_forward(self, robot_id):
        """예약 끝에서 전방으로, 같은 방향이 유지되고 비어있는 동안만 확보.

        코너(방향 전환) 노드는 run의 끝으로 포함하고 그 너머는 쥐지 않는다.
        → 확보 구간 = 항상 직선 run. 반환: 새 reserved_end.
        """
        st = self.robots[robot_id]
        if st.escape_end is None and self._hold_active(robot_id):
            return st.reserved_end       # 승자 통과 대기 — 전진 예약 동결
        i = st.reserved_end
        prev_dir = None
        while i + 1 < len(st.route):
            d = _direction(self.graph, st.route[i], st.route[i + 1])
            if prev_dir is not None and d != prev_dir:
                break                    # 코너 → run 종료 (코너 노드까지 확보됨)
            nxt = st.route[i + 1]
            owner = self.reservations.get(nxt)
            if owner is not None and owner != robot_id:
                break                    # 남이 점유 → 여기까지
            self.reservations[nxt] = robot_id
            prev_dir = d
            i += 1
        st.reserved_end = i
        return st.reserved_end

    # ── 다음 주행 구간 (확보 구간 = 직선 run 그대로) ───────
    def next_segment(self, robot_id):
        """예약된 직선 run 반환. 갈 곳 없으면 [] (대기)."""
        st = self.robots[robot_id]
        if st.index >= len(st.route) - 1:
            st.waiting = False
            return []                            # 이미 목적지 도착
        self.reserve_forward(robot_id)
        if st.reserved_end <= st.index:
            st.waiting = True                    # 다음 노드 막힘 → 대기 표시
            return []
        st.waiting = False
        return st.route[st.index + 1: st.reserved_end + 1]

    # ── 도착 / 해제 ──────────────────────────────────────
    def _advance(self, robot_id, new_index):
        """index 갱신 + 예약 창(index~reserved_end) 밖의 지나온 노드 release."""
        st = self.robots[robot_id]
        st.index = new_index
        if st.escape_end is not None and st.index >= st.escape_end:
            st.escape_end = None         # 대피 노드 도착 → 대피 완료
        keep = set(st.route[st.index: st.reserved_end + 1])
        for j in range(0, st.index):
            n = st.route[j]
            if n in keep:
                continue                 # 대피 경로 중복 노드: 앞에 다시 나오면 유지
            if self.reservations.get(n) == robot_id:
                del self.reservations[n]

    def arrive(self, robot_id, node):
        """node 도착 → index 갱신 + 지나온 뒤 노드 즉시 release.

        대피 경로는 같은 노드가 두 번 나올 수 있어 현재 index 이후에서 탐색.
        (로봇 0.25m < 노드간격 0.30m라 노드 중심에 서면 꼬리가 뒤 통로에
         안 걸쳐 충돌 위험 낮음. 뒤 노드를 안 쥐므로 최종도착 시 통로 막힘도 없음.)
        """
        st = self.robots[robot_id]
        try:
            idx = st.route.index(node, st.index)
        except ValueError:
            return
        self._advance(robot_id, idx)

    def update_position(self, robot_id, x, y):
        """로봇 amcl 위치 → 예약 전방 노드 반경 진입 시 통과 처리(release).

        traffic이 위치를 직접 보고 release (로봇은 직선 run 끝점까지 주행만).
        반환: 새로 통과 판정된 노드 or None.
        """
        st = self.robots.get(robot_id)
        if st is None:
            return None
        passed_i = None
        i = st.index + 1
        while i <= st.reserved_end and i < len(st.route):
            nx, ny = self.graph.xy(st.route[i])
            if (x - nx) ** 2 + (y - ny) ** 2 <= self.ARRIVE_RADIUS ** 2:
                passed_i = i
            i += 1
        if passed_i is None:
            return None
        self._advance(robot_id, passed_i)
        return st.route[passed_i]

    # ── 조회 ─────────────────────────────────────────────
    def current_node(self, robot_id):
        st = self.robots[robot_id]
        return st.route[st.index]

    def next_node(self, robot_id):
        st = self.robots[robot_id]
        return st.route[st.index + 1] if st.index + 1 < len(st.route) else None

    def is_done(self, robot_id):
        st = self.robots[robot_id]
        return st.index >= len(st.route) - 1

    def reserved_nodes(self, robot_id):
        return [n for n, v in self.reservations.items() if v == robot_id]

    def _goal(self, robot_id):
        return self.robots[robot_id].route[-1]

    # ── 교착 감지 · 중재 (주기적 호출) ────────────────────
    def resolve_deadlock(self):
        """대기 로봇 중재 — 2단계.

        ① 상호 대치(head-on): 서로의 다음 노드 = 상대의 현재 노드.
           양쪽 대피 비용을 대칭 비교해 양보자 선정.
        ② 조기 양보: 대기 중인 로봇이 '주행 중인 상대의 남은 경로 위'에
           서 있으면, 상대가 인접까지 오기 전에 미리 비켜줌 → 상대는
           정차 없이 통과 (실기에서 대치 성립까지의 대기 낭비 제거).
           같은 방향 추종/스쳐 지나감은 조건상 발동 안 함.

        반환: {robot_id: 'reroute' | 'stuck'}. (없으면 {})
        'stuck' = 대피·우회 모두 불가 (안전망 — 상위에서 warn).
        """
        actions = {}
        waiting = [r for r, st in self.robots.items() if st.waiting]
        handled = set()

        # ① 상호 대치 — 대피 비용 비교
        checked = set()
        for a in waiting:
            for b in self.robots:
                if a == b or (b, a) in checked:
                    continue
                checked.add((a, b))
                if not self._is_head_on(a, b):
                    continue
                if (self.robots[a].escape_end is not None
                        or self.robots[b].escape_end is not None):
                    continue             # 한쪽이 대피 이동 중 — 곧 풀림, 재중재 금지
                handled.add(a)
                handled.add(b)
                yielder = self._pick_yielder(a, b)
                if yielder is not None:
                    other = b if yielder == a else a
                    if self._yield_route(yielder, other):
                        actions[yielder] = 'reroute'
                        continue
                # 대피 불가 → 구식 우회(엣지 차단 포함) 폴백
                loser = yielder if yielder is not None else max(a, b)
                other = b if loser == a else a
                if self._reroute(loser, other):
                    actions[loser] = 'reroute'
                else:
                    actions[a] = 'stuck'

        # ② 조기 양보 — 어차피 막힌 김에 상대 길을 미리 비켜줌 (적재 여부 무관).
        # 발동 조건:
        #   내가 대기 중 + 나를 막은 게 바로 그 상대(내 다음 노드를 y가 예약)
        #   + 내 자리가 y의 남은 경로 위 (= y가 이리로 오는 중, 충돌 불가피)
        # 보호 가드 (라이브락 방지 — 실기 발견):
        #   - y가 대피 이동 중이면 양보 금지: 나는 승자, y의 복귀 경로가 내 자리를
        #     지난다는 이유로 나까지 양보하면 둘 다 비켰다 둘 다 복귀 → 무한 반복.
        #   - 내가 대피 중이어도 재양보 금지.
        # laden은 여기서 안 봄(확정): "먼저 막힌 놈이 비킨다"가 기본 규칙 —
        # 적재차의 한 칸 옆걸음은 평소 laden 주행과 위험 차이가 없고, 상대가
        # 무정차 통과라 해소가 가장 빠름. laden 우선권은 동시 대치(①)에서만.
        for x in waiting:
            if x in handled:
                continue
            st_x = self.robots[x]
            if st_x.escape_end is not None:
                continue
            cur_x = st_x.route[st_x.index]
            nxt_x = st_x.route[st_x.index + 1]
            blocker = self.reservations.get(nxt_x)
            if blocker is None or blocker == x:
                continue                     # 지금은 안 막힘 (waiting 플래그가 낡음)
            st_y = self.robots[blocker]
            if st_y.escape_end is not None:
                continue                     # 상대가 비켜주는 중 — 나는 잠깐 대기
            if cur_x in st_y.route[st_y.index:]:
                # 실패(대피 불가)면 그냥 대기 유지 — 인접 대치가 되면 ①이 처리
                if self._yield_route(x, blocker):
                    actions[x] = 'reroute'
        return actions

    def _is_head_on(self, a, b):
        na, nb = self.next_node(a), self.next_node(b)
        ca, cb = self.current_node(a), self.current_node(b)
        return na == cb and nb == ca

    # ── 대피 (back-off) ──────────────────────────────────
    def _escape_path(self, robot_id, opponent):
        """robot이 opponent의 (현재+남은 경로) 밖으로 나가는 최단 대피 경로.

        BFS. 통과 금지: 남이 예약 중인 노드 (상대 현재 노드 포함).
        상대의 남은 경로 노드는 통과는 가능, 최종 정지 지점만 불가.
        반환: [현재, ..., 대피 노드] or None.
        """
        st = self.robots[robot_id]
        opp = self.robots[opponent]
        opp_path = set(opp.route[opp.index:])
        start = st.route[st.index]
        q = deque([[start]])
        seen = {start}
        while q:
            path = q.popleft()
            node = path[-1]
            if node != start and node not in opp_path:
                return path
            for nb, _ in self.graph.neighbors(node):
                if nb in seen:
                    continue
                owner = self.reservations.get(nb)
                if owner is not None and owner != robot_id:
                    continue             # 남의 점유 노드는 통과 불가
                seen.add(nb)
                q.append(path + [nb])
        return None

    def _pick_yielder(self, a, b):
        """양보(대피)할 로봇 결정 — 대칭 대피 비용 비교.

        ① 대피 비용(상대 경로 밖 최근접 노드까지 칸수) 싼 쪽이 양보
        ② 동률이면 laden(적재) 로봇 통과, 빈 로봇 양보
        ③ robot_id 타이브레이크 (결정적)
        양쪽 다 대피 불가면 None (폴백으로).
        """
        pa = self._escape_path(a, b)
        pb = self._escape_path(b, a)
        if pa is None and pb is None:
            return None
        if pa is None:
            return b
        if pb is None:
            return a
        ca, cb = len(pa) - 1, len(pb) - 1
        if ca != cb:
            return a if ca < cb else b
        la, lb = self.robots[a].laden, self.robots[b].laden
        if la != lb:
            return b if la else a        # 적재 로봇 통과 → 빈 쪽 양보
        return min(a, b)

    def _yield_route(self, robot_id, opponent):
        """양보자 새 경로 = [현재→대피 노드] + (대피 노드→목적지 최단).

        대피 후의 대기/우회는 예약 시스템이 자연 처리:
        재계산 경로가 승자 구간을 다시 지나면 해제될 때까지 대기 후 진행,
        딴 길이 더 짧으면 그리로 우회. 승자는 아무 조치 없음.
        """
        esc = self._escape_path(robot_id, opponent)
        if not esc:
            return False
        st = self.robots[robot_id]
        yielded_node = st.route[st.index]        # 내가 비켜주는 자리
        tail = self.router.shortest_path(esc[-1], self._goal(robot_id))
        if not tail:
            return False
        route = esc + tail[1:]           # 대피 노드 중복 제거하고 이어붙임
        if not self._install_route(robot_id, route, st.laden):
            return False
        # 대피 노드 도착까지 '대피 중' 표시 — 이 로봇의 승자를 연쇄 양보에서 보호
        self.robots[robot_id].escape_end = len(esc) - 1
        # 승자가 이 자리를 지나갈 때까지 복귀(전진 예약) 금지 — 재선점 레이스 방지
        self._hold[robot_id] = (opponent, yielded_node)
        return True

    # ── 폴백: 구식 우회 (대피 불가 시) ────────────────────
    def _reroute(self, robot_id, opponent):
        st = self.robots[robot_id]
        cur = self.current_node(robot_id)
        goal = self._goal(robot_id)
        opp_cur = self.current_node(opponent)
        if goal == opp_cur:
            # swap: 내 목적지 = 상대 현재 노드 → 노드 회피 불가.
            # 직행 엣지만 빼고 반대편 접근 경로 생성.
            return self.set_route(robot_id, cur, goal, laden=st.laden,
                                  blocked_edges={frozenset((cur, opp_cur))})
        avoid = {opp_cur, self.next_node(opponent)}
        avoid.discard(None)
        return self.set_route(robot_id, cur, goal, laden=st.laden, blocked=avoid)
