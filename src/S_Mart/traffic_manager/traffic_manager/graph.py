"""노드/엣지 그래프 — nodes.yaml 로드 + 4-인접 엣지 자동 생성.

- 대각선 없이 가로/세로 인접(0.30m)만 연결, blocked 노드 제외.
- 엣지 비용 = 거리(0.30). 단 작업공간 랙 사이 엣지는 2배(0.60)로 페널티
  → 다른 로봇이 작업 중인 랙 사이를 통과 트래픽이 지나가지 않게 우회 유도.
  대상: N1-N6(냉장랙), N16-N21(냉동랙), N5-N10(출고-입고).
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory

# 인접 판정: 0.30m ± 오차
_STEP = 0.30
_TOL = 0.06

# 2배 비용 엣지 (작업공간 사이 통과 억제). 작업 후 비워지므로 우회 유도만.
_PENALTY_EDGES = {
    frozenset(('N1', 'N6')),    # 냉장랙 사이
    frozenset(('N16', 'N21')),  # 냉동랙 사이
    frozenset(('N5', 'N10')),   # 출고-입고 사이
}
_PENALTY_FACTOR = 2.0

# 완전 차단 엣지. 홈은 임무 올 때까지 무기한 점유 → 통과 시 deadlock → 아예 막음.
# (홈 진입/진출은 다른 엣지(N19/N24)로 하므로 복귀엔 지장 없음)
_BLOCKED_EDGES = {
    frozenset(('N20', 'N25')),  # home_B - home_A 사이 통과 금지
}

# 생성자에서 모든 계산이 끝남. 이후 읽기만 하는 객체. (런타임에 그래프 변경 X)
class Graph:
    """노드 좌표 + 인접(엣지) 그래프.

    nodes: {name: {x, y, role, dock, blocked}}
    adj:   {name: [(이웃, 비용), ...]}   (blocked 제외)
    """

    # 노드 로드
    def __init__(self, nodes_file=None):
        if nodes_file is None:
            nodes_file = os.path.join(
                get_package_share_directory('traffic_manager'), 'graph', 'nodes.yaml'
            )
        with open(nodes_file) as f:
            self.nodes = yaml.safe_load(f)['nodes']
        self.free = [n for n in self.nodes if not self.nodes[n].get('blocked')]    # 장애물 노드 지움 (blocked 노드 제외)
        self.adj = self._build_adjacency()

    # 엣지 생성
    def _build_adjacency(self):
        adj = {n: [] for n in self.free}
        for a in self.free:
            for b in self.free:
                if a == b:
                    continue
                dx = abs(self.nodes[a]['x'] - self.nodes[b]['x'])
                dy = abs(self.nodes[a]['y'] - self.nodes[b]['y'])
                # 완전 차단 엣지는 연결 안 함 (홈 통과 금지)
                if frozenset((a, b)) in _BLOCKED_EDGES:
                    continue
                # 가로 또는 세로로 딱 한 칸(0.30m) 떨어진 경우만 (대각선 X)
                horiz = abs(dx - _STEP) < _TOL and dy < _TOL
                vert = abs(dy - _STEP) < _TOL and dx < _TOL
                if horiz or vert:
                    adj[a].append((b, self._edge_cost(a, b)))
        return adj

    # 엣지 비용 연산
    def _edge_cost(self, a, b):
        cost = _STEP
        if frozenset((a, b)) in _PENALTY_EDGES:
            cost *= _PENALTY_FACTOR
        return round(cost, 3)

    # Router(다익스트라) 사용
    def neighbors(self, node):
        return self.adj.get(node, [])

    # Traffic (통과 판정 및 방향계산) 사용
    def xy(self, node):
        nd = self.nodes[node]
        return nd['x'], nd['y']

    # Router (출발 도착시 사용)
    def is_blocked(self, node):
        return self.nodes.get(node, {}).get('blocked', False)

    # Router 사용
    def exists(self, node):
        return node in self.nodes
