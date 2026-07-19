"""TrafficManager — route + index 기반 예약 관리 (교착 감지는 별도 단계).

전체 경로 예약 + 예약 반영 라우팅 (B안 — A/B 검증 후 2026-07-12 채택.
A안(corner-cap)은 git 히스토리 참조, 비교 데이터는 Gazebo/traffic_results):

핵심 개념:
- 라우팅: 다익스트라 + 남이 예약한 노드행 엣지 비용 2배(소프트 페널티, router.py)
  → 교차가 심할수록 예약 구간을 애초에 우회. 작업노드 사이 페널티/홈 엣지
  차단은 그래프 비용에 이미 반영.
- 예약 테이블: node -> robot_id (누가 점유 중).
- reserve_forward: 경로 설정 시 '경로 전체'를 즉시 예약. 남의 예약에 막히면
  거기까지만 — 이후 next_segment마다 재시도. (대피 중엔 대피 노드까지만,
  hold 중엔 동결 — 메서드 주석 참조)
- next_segment: 예약은 전체지만 주행 계약은 불변 — FSM은 직선 run 단위,
  첫 코너에서 잘라 반환.
- arrive/update_position: 통과한 뒤 노드 즉시 release. 대기 로봇은 예약을
  소진한 상태라 발밑만 점유 → 양쪽 대기 교착은 head-on으로 수렴 (감지 완전,
  A안과 등가 — 분석 증명 2026-07-10).

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
    ARRIVE_RADIUS = 0.07                  # 노드 통과 판정 반경(m). 0.12→0.07:
                                          # 저속 정지(~0.12m) 시 접근 노드를 조기 "통과" 오인
                                          # → 장애물 재경로 출발점 꼬임 방지. 3Hz 샘플 바닥(0.03) 위.

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
        # 장애물 차단 노드(전역): {노드명, ...}. 로봇이 recovery로 감지 보고 시 등록.
        # 장애물은 노드(교차점) 위에 놓임 → 노드를 막아야 모든 진입 방향이 회피됨.
        # (엣지만 막으면 다른 문으로 그 노드에 들어가 재충돌 — 2026-07-15 실기 발견).
        # hard block, 시간 만료 없음 — 로봇이 통과하면 자동 해제 / GUI·수동 unblock. 전역.
        self._blocked_nodes = set()

    # ── 장애물 차단 노드 (전역) ────────────────────────────
    def block_node(self, n):
        """노드 차단 등록. 이후 모든 라우팅/예약이 이 노드를 회피."""
        self._blocked_nodes.add(n)

    def unblock_node(self, n):
        """노드 차단 해제 (통과 자가치유 / GUI 확인 / 수동)."""
        self._blocked_nodes.discard(n)

    def clear_blocks(self):
        """전체 차단 해제."""
        self._blocked_nodes.clear()

    def blocked_nodes(self):
        """현재 차단된 노드 집합 (GUI 상태 동기화용 — traffic_node reconcile)."""
        return set(self._blocked_nodes)

    def _node_blocked(self, n):
        return n in self._blocked_nodes

    def next_node_blocked(self, robot_id):
        """로봇의 다음 진행 노드가 장애물 차단인지 (reroute 트리거 판별용)."""
        st = self.robots.get(robot_id)
        if st is None or st.index + 1 >= len(st.route):
            return False
        return self._node_blocked(st.route[st.index + 1])

    def report_obstacle(self, robot_id):
        """로봇이 다음 노드에서 지속 장애물 감지 → 노드 전역 차단 + 대응 결정.

        노드는 traffic의 index 기준(route[index+1], 진입하려던 노드) — index가 0.07로 정확.
        차단(block_node)은 모든 경우 공통(다른 로봇 반영). 본인 대응만 갈림:
          'reroute'      → route로 우회 (통과 노드 장애물)
          'goal_blocked' → 막힌 노드=최종 목적지 → 우회 무의미, 대기 (사람 필요)
          'no_route'     → 우회로 없음(노드가 유일 관문) → 대기 (사람 필요)
          None           → 막을 노드 없음 (이미 목적지 위)
        반환: (kind, blocked_node | None, route | None)
        """
        st = self.robots.get(robot_id)
        if st is None or st.index + 1 >= len(st.route):
            return None, None, None
        a, b = st.route[st.index], st.route[st.index + 1]
        self.block_node(b)                       # 장애물이 놓인 노드 차단 (공통)
        if b == self._goal(robot_id):            # 막힌 노드 = 최종 목적지
            return 'goal_blocked', b, None       # 재경로 안 함 — 대기+알림
        if self.set_route(robot_id, a, self._goal(robot_id), laden=st.laden):
            return 'reroute', b, list(self.robots[robot_id].route)
        return 'no_route', b, None               # 우회로 없음 — 대기+알림

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

    def _others(self, robot_id):
        """남이 예약 중인 노드 집합 (라우팅 소프트 페널티 대상)."""
        return {n for n, r in self.reservations.items() if r != robot_id}

    # ── 경로 설정 ─────────────────────────────────────────
    def set_route(self, robot_id, start, goal, laden=False, blocked=None,
                  blocked_edges=None):
        """페널티 반영 다익스트라로 경로 생성 → 저장 → 전체 즉시 예약. 성공 bool.

        laden: 팔레트 적재 여부.
        blocked / blocked_edges: 폴백 우회용 탐색 제약 (평상시 None).
        """
        nodes = set(self._blocked_nodes)     # 장애물 차단 노드(전역) 항상 반영
        if blocked:
            nodes |= set(blocked)            # 폴백 우회용 추가 회피 노드와 병합
        route = self.router.shortest_path(
            start, goal, blocked=nodes, blocked_edges=blocked_edges,
            penalized=self._others(robot_id))
        if not route:
            return False
        if not self._install_route(robot_id, route, laden):
            return False
        self.reserve_forward(robot_id)       # 경로 전체 즉시 확보
        return True

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

    # ── 예약: 경로 전체 (남의 예약 앞까지) ─────────────────
    # 단 '대피 중'에는 대피 노드까지만 확보: 대피 경로는 자기 현재 노드를
    # 다시 지나는 중복 꼬리(예: N4→N9→N4→N5)를 가질 수 있는데, 전체 예약으로
    # 창을 열면 update_position이 중복 노드를 현재 위치와 매칭해 '가짜 통과'
    # (index 점프 + escape_end 조기 해제 + 예약 오염 → 양쪽 동시 양보,
    #  유령 위치 때문에 상대가 실물 로봇 위로 라우팅). 2026-07-09 시뮬 실증.
    def reserve_forward(self, robot_id):
        """예약 끝(reserved_end)에서 전방으로 빈 노드를 계속 확보. 반환: 새 reserved_end.

        평시엔 경로 끝까지(전체 예약), 남의 예약을 만나면 거기서 멈춤 —
        이후 next_segment 폴링마다 재호출되며 풀린 만큼 더 확보 (재시도 루프의 실체).
        """
        st = self.robots[robot_id]
        if st.escape_end is not None:
            limit = st.escape_end            # 대피 중: 대피 노드까지만 (중복 꼬리 가짜통과 방지)
        elif self._hold_active(robot_id):
            limit = st.index                 # 승자 통과 대기: 전진 예약 동결
        else:
            limit = len(st.route) - 1        # 평시: 경로 끝까지 전체 예약
        i = st.reserved_end
        while i + 1 <= limit:
            nxt = st.route[i + 1]
            if self._node_blocked(nxt):
                break                        # 장애물 차단 노드 — 넘어가면 장애물로 감.
                                             # 여기서 멈춰 next_node_blocked로 재경로 유도
            owner = self.reservations.get(nxt)
            if owner is not None and owner != robot_id:
                break                        # 남이 점유 → 여기까지, 다음 폴링에 재시도
            self.reservations[nxt] = robot_id
            i += 1
        st.reserved_end = i                  # index ≤ reserved_end 불변식 유지
        return st.reserved_end

    # ── 다음 주행 구간 — 예약은 전체지만 주행 단위는 직선 run ─
    def next_segment(self, robot_id):
        """확보 구간을 첫 코너에서 잘라 반환. 갈 곳 없으면 [] (대기)."""
        st = self.robots[robot_id]
        if st.index >= len(st.route) - 1:
            st.waiting = False
            return []                            # 이미 목적지 도착
        self.reserve_forward(robot_id)
        if st.reserved_end <= st.index:
            st.waiting = True                    # 다음 노드 막힘 → 대기 표시
            return []
        st.waiting = False
        j = st.index
        prev = None
        while j + 1 <= st.reserved_end:
            d = _direction(self.graph, st.route[j], st.route[j + 1])
            if prev is not None and d != prev:
                break                            # 코너 → run 종료
            prev = d
            j += 1
        return st.route[st.index + 1: j + 1]

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

    def update_position(self, robot_id, x, y):
        """로봇 amcl 위치 → 예약 전방 노드 반경 진입 시 통과 처리(release).

        traffic이 위치를 직접 보고 release (로봇은 직선 run 끝점까지 주행만).
        반환: 새로 통과 판정된 노드 or None.
        """
        st = self.robots.get(robot_id)
        if st is None:
            return None                      # 경로 없는 로봇의 pose는 무시
        passed_i = None
        i = st.index + 1
        while i <= st.reserved_end and i < len(st.route):   # 내 예약 창 안에서만 탐색
            nx, ny = self.graph.xy(st.route[i])
            if (x - nx) ** 2 + (y - ny) ** 2 <= self.ARRIVE_RADIUS ** 2:
                passed_i = i                 # 창 안에서 가장 멀리 도달한 노드로 갱신
            i += 1
        if passed_i is None:
            return None
        # 통과한 노드가 차단돼 있었으면 자동 해제 — 물리적으로 지나감 = 치워졌다는 증거.
        # (목적지 막힘 B안: 재시도하다 장애물 없어지면 통과 → 여기서 차단 정상화 → 시스템 자가치유)
        for j in range(st.index + 1, passed_i + 1):
            if self._node_blocked(st.route[j]):
                self.unblock_node(st.route[j])
        self._advance(robot_id, passed_i)    # index 전진 + 지나온 노드 release
        return st.route[passed_i]

    # ── 조회 ─────────────────────────────────────────────
    def current_node(self, robot_id):
        """현재(마지막 통과) 노드 = route[index]."""
        st = self.robots[robot_id]
        return st.route[st.index]

    def next_node(self, robot_id):
        """다음 진행 예정 노드. 목적지에 서 있으면 None."""
        st = self.robots[robot_id]
        return st.route[st.index + 1] if st.index + 1 < len(st.route) else None

    def is_done(self, robot_id):
        """목적지 도착 여부 (index가 경로 끝)."""
        st = self.robots[robot_id]
        return st.index >= len(st.route) - 1

    def _goal(self, robot_id):
        """이 로봇의 최종 목적지 노드 (대피 재경로에서도 불변)."""
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
        """정면 대치 판정 — 서로의 다음 노드가 상대의 현재 노드 (맞물림)."""
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
        opp_path = set(opp.route[opp.index:])    # 이 집합 "밖"이 대피 성공 조건
        start = st.route[st.index]
        q = deque([[start]])
        seen = {start}
        while q:
            path = q.popleft()                   # BFS → 처음 찾은 곳 = 최소 칸수 대피처
            node = path[-1]
            if node != start and node not in opp_path:
                return path                      # 상대 경로 밖 도달 — 여기 서 있으면 안전
            for nb, _ in self.graph.neighbors(node):
                if nb in seen:
                    continue
                if self._node_blocked(nb):
                    continue             # 장애물 차단 노드는 대피 경로로도 통과 불가
                owner = self.reservations.get(nb)
                if owner is not None and owner != robot_id:
                    continue             # 남의 점유 노드는 통과 불가
                seen.add(nb)
                q.append(path + [nb])
        return None                              # 갈 곳 없음 → _pick_yielder가 폴백 판단

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
        tail = self.router.shortest_path(esc[-1], self._goal(robot_id),
                                         blocked=set(self._blocked_nodes),
                                         penalized=self._others(robot_id))
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
        """상대의 현재·다음 노드를 피해 전체 경로 재계산 (대피가 불가능할 때만).

        일반 케이스 = 노드 회피(blocked), swap 케이스 = 직행 엣지만 차단.
        """
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
