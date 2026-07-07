"""경로 생성 — 다익스트라 최단경로 (노드 리스트 반환).

Graph의 인접/비용을 사용. 작업공간 랙 사이 엣지 페널티가 비용에 반영돼 있어,
경유 트래픽은 자연히 통로로 우회한다(출발/도착이면 어쩔 수 없이 지남).
"""
import heapq


class Router:
    def __init__(self, graph):
        self.graph = graph

    def shortest_path(self, start, goal, blocked=None, blocked_edges=None):
        """start→goal 최단 노드 리스트 반환. 없으면 None.

        blocked: 피할 노드 집합 (교착 리라우팅용). start/goal은 제외 안 함.
        blocked_edges: 이번 탐색에서만 안 쓸 엣지 집합 ({frozenset((a, b)), ...}).
          swap 교착용 — 목적지 자체는 막을 수 없으니 직행 엣지만 빼서
          반대편으로 접근하는 경로를 뽑는다. 예약과 무관한 탐색 제약.
        (blocked/미존재 노드는 None.)
        """
        g = self.graph
        block = set(blocked) if blocked else set()
        edges = blocked_edges or set()
        block.discard(start)               # 출발/도착은 막지 않음
        block.discard(goal)
        if not g.exists(start) or not g.exists(goal):
            return None
        if g.is_blocked(start) or g.is_blocked(goal):
            return None
        if start == goal:
            return [start]

        # (누적비용, 노드, 경로)
        pq = [(0.0, start, [start])]
        best = {start: 0.0}
        while pq:
            cost, node, path = heapq.heappop(pq)
            if node == goal:
                return path
            if cost > best.get(node, float('inf')):
                continue
            for nb, w in g.neighbors(node):
                if nb in block:            # 리라우팅: 이 노드 회피
                    continue
                if frozenset((node, nb)) in edges:  # swap: 직행 엣지 회피
                    continue
                nc = cost + w
                if nc < best.get(nb, float('inf')):
                    best[nb] = nc
                    heapq.heappush(pq, (nc, nb, path + [nb]))
        return None
