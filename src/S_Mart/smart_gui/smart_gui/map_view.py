"""창고 맵 — 5×5 노드-그래프 뷰.

그래프는 traffic_manager.graph.Graph를 그대로 쓴다(nodes.yaml·인접 규칙의 단일 출처).
좌표는 map 프레임 그대로이며, /traffic/pose로 오는 로봇 좌표와 같은 프레임이라
변환 없이 겹쳐 그린다.

map 프레임 → 화면: x는 오른쪽, y는 **위쪽**(화면 y와 반대). 종횡비는 유지한다
(0.30m 격자가 화면에서도 정사각이어야 눈으로 거리를 읽을 수 있음).
"""

from PyQt5.QtCore import QPointF, QRectF, Qt
from PyQt5.QtGui import QBrush, QColor, QFont, QPainter, QPen
from PyQt5.QtWidgets import QSizePolicy, QWidget

from traffic_manager.graph import Graph

from . import theme

# 노드 role → (색, 표시명). nodes.yaml의 role 값과 1:1.
ROLE_STYLE = {
    'rack_chilled': (theme.BLUE, '냉장랙'),
    'rack_frozen': (theme.CYAN, '냉동랙'),
    'outbound': (theme.AMBER, '출고'),
    'inbound': (theme.GREEN, '입고'),
    'home_A': (theme.VIOLET, '홈'),
    'home_B': (theme.VIOLET, '홈'),
    'corridor': (theme.DIM, '통로'),
    'obstacle': (theme.RED, '장애물'),
}

_MARGIN_M = 0.15          # map 프레임 여백(m) — dock 화살표·로봇이 잘리지 않을 만큼만
_DOCK_VEC = {'E': (1, 0), 'W': (-1, 0), 'N': (0, 1), 'S': (0, -1)}


class MapView(QWidget):
    """노드 그래프 + 실시간 로봇 위치."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(360, 360)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._graph = Graph()
        self._poses = {}        # robot → (x, y)  map 프레임
        self._stale = set()     # 연결 끊긴 로봇 (흐리게)
        self._dyn_blocks = {}   # 런타임 차단 노드 {node: kind} (nodes.yaml 정적 blocked와 별개)

        xs = [n['x'] for n in self._graph.nodes.values()]
        ys = [n['y'] for n in self._graph.nodes.values()]
        self._bounds = (min(xs) - _MARGIN_M, min(ys) - _MARGIN_M,
                        max(xs) + _MARGIN_M, max(ys) + _MARGIN_M)

    # ── 외부 입력 ─────────────────────────────────────────────

    def set_pose(self, robot: str, x: float, y: float):
        self._poses[robot] = (x, y)
        self.update()

    def set_stale(self, robot: str, stale: bool):
        if stale:
            self._stale.add(robot)
        else:
            self._stale.discard(robot)
        self.update()

    def set_dynamic_blocks(self, blocks: dict):
        """traffic 런타임 차단 노드 {node: kind}. block/clear 이벤트로 갱신."""
        self._dyn_blocks = dict(blocks)
        self.update()

    # ── 좌표 변환 ─────────────────────────────────────────────

    def _to_screen(self, x: float, y: float) -> QPointF:
        x0, y0, x1, y1 = self._bounds
        scale = min(self.width() / (x1 - x0), self.height() / (y1 - y0))
        # 종횡비 유지 후 남는 공간은 가운데 정렬
        ox = (self.width() - (x1 - x0) * scale) / 2
        oy = (self.height() - (y1 - y0) * scale) / 2
        return QPointF(ox + (x - x0) * scale,
                       oy + (y1 - y) * scale)      # y 반전

    # ── 그리기 ────────────────────────────────────────────────

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor(theme.PANEL2))

        self._draw_edges(p)
        self._draw_nodes(p)
        self._draw_dyn_blocks(p)
        self._draw_robots(p)
        self._draw_legend(p)

    def _draw_edges(self, p: QPainter):
        p.setPen(QPen(QColor(theme.LINE), 2))
        drawn = set()
        for a, neighbors in self._graph.adj.items():
            for b, _cost in neighbors:
                if (b, a) in drawn:
                    continue
                drawn.add((a, b))
                na, nb = self._graph.nodes[a], self._graph.nodes[b]
                p.drawLine(self._to_screen(na['x'], na['y']),
                           self._to_screen(nb['x'], nb['y']))

    def _draw_nodes(self, p: QPainter):
        font = QFont(p.font())
        font.setPointSize(7)
        p.setFont(font)

        for name, n in self._graph.nodes.items():
            center = self._to_screen(n['x'], n['y'])
            role = n.get('role', 'corridor')
            color, _label = ROLE_STYLE.get(role, (theme.DIM, role))
            blocked = n.get('blocked', False)

            if blocked:
                # 중앙 장애물 벽에 가려 주행 불가한 노드 — 경로에서 아예 빠진다
                p.setPen(QPen(QColor(theme.RED), 1.5))
                p.setBrush(Qt.NoBrush)
                r = 5.0
                p.drawLine(QPointF(center.x() - r, center.y() - r),
                           QPointF(center.x() + r, center.y() + r))
                p.drawLine(QPointF(center.x() - r, center.y() + r),
                           QPointF(center.x() + r, center.y() - r))
                continue

            is_work = role not in ('corridor', 'obstacle')
            radius = 9.0 if is_work else 4.0
            p.setBrush(QBrush(QColor(color)))
            p.setPen(QPen(QColor(theme.BG), 2) if is_work else Qt.NoPen)
            p.drawEllipse(center, radius, radius)

            if is_work:
                self._draw_dock_arrows(p, center, n.get('dock'), color)

            # 노드 이름은 작업 노드만 (통로까지 쓰면 글자가 서로 겹침)
            if is_work:
                p.setPen(QPen(QColor(theme.MUTED)))
                p.drawText(QRectF(center.x() - 22, center.y() + 11, 44, 12),
                           Qt.AlignCenter, name)

    def _draw_dyn_blocks(self, p: QPainter):
        """런타임 차단 노드 오버레이. reroute=우회(주황 링), goal_blocked/no_route=대기(빨강 링).

        정적 nodes.yaml blocked(빨강 X)와 구분되도록 링+X로 그린다.
        """
        for node, kind in self._dyn_blocks.items():
            n = self._graph.nodes.get(node)
            if not n:
                continue
            center = self._to_screen(n['x'], n['y'])
            color = theme.ORANGE if kind == 'reroute' else theme.RED
            p.setPen(QPen(QColor(color), 2.5))
            p.setBrush(Qt.NoBrush)
            r = 11.0
            p.drawEllipse(center, r, r)
            x = r * 0.5
            p.drawLine(QPointF(center.x() - x, center.y() - x),
                       QPointF(center.x() + x, center.y() + x))
            p.drawLine(QPointF(center.x() - x, center.y() + x),
                       QPointF(center.x() + x, center.y() - x))

    def _draw_dock_arrows(self, p: QPainter, center: QPointF, dock, color: str):
        """도킹 진입 방향 표시. nodes.yaml의 dock은 문자열 또는 리스트(N5=[E,N])."""
        if not dock:
            return
        dirs = dock if isinstance(dock, list) else [dock]
        p.setPen(QPen(QColor(color), 2))
        for d in dirs:
            vec = _DOCK_VEC.get(d)
            if not vec:
                continue
            dx, dy = vec[0], -vec[1]      # 화면 y 반전
            start = QPointF(center.x() + dx * 10, center.y() + dy * 10)
            end = QPointF(center.x() + dx * 18, center.y() + dy * 18)
            p.drawLine(start, end)

    def _draw_robots(self, p: QPainter):
        font = QFont(p.font())
        font.setPointSize(8)
        font.setBold(True)
        p.setFont(font)

        for robot, (x, y) in self._poses.items():
            color = QColor(theme.ROBOT_COLORS.get(robot, theme.INK))
            if robot in self._stale:
                color.setAlpha(70)        # 위치가 옛날 값 — 흐리게
            center = self._to_screen(x, y)

            p.setBrush(QBrush(color))
            p.setPen(QPen(QColor(theme.BG), 2))
            p.drawEllipse(center, 11, 11)

            p.setPen(QPen(QColor(theme.BG)))
            p.drawText(QRectF(center.x() - 14, center.y() - 7, 28, 14),
                       Qt.AlignCenter, robot.replace('AMR_', 'R'))

    def _draw_legend(self, p: QPainter):
        font = QFont(p.font())
        font.setPointSize(8)
        font.setBold(False)
        p.setFont(font)

        items = [(theme.BLUE, '냉장랙'), (theme.CYAN, '냉동랙'), (theme.AMBER, '출고'),
                 (theme.GREEN, '입고'), (theme.VIOLET, '홈'), (theme.RED, '장애물'),
                 (theme.ORANGE, '우회차단')]
        x, y = 10, self.height() - 16
        for color, label in items:
            p.setBrush(QBrush(QColor(color)))
            p.setPen(Qt.NoPen)
            p.drawEllipse(QPointF(x + 4, y), 4, 4)
            p.setPen(QPen(QColor(theme.MUTED)))
            p.drawText(QPointF(x + 12, y + 4), label)
            x += 16 + p.fontMetrics().horizontalAdvance(label) + 10
