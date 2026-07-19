"""대시보드 구성 위젯 — 목업의 .panel / .robot / .kpi / .row 대응.

여기 있는 위젯은 전부 "받은 데이터를 그린다"만 한다. 조회·구독은 main.py가 하고
set_*() 로 밀어넣는다 (위젯이 DB나 ROS를 직접 잡으면 테스트도 재사용도 불가).
"""

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor, QPainter, QPen, QPolygonF
from PyQt5.QtCore import QPointF
from PyQt5.QtWidgets import (
    QFrame, QHBoxLayout, QLabel, QPushButton, QSizePolicy, QVBoxLayout, QWidget,
)

from . import theme

# 상태 → (표시명, 색). robot_fsm이 외부로 내보내는 3종 + UI가 판정하는 stale.
STATUS_STYLE = {
    'idle': ('대기', theme.MUTED),
    'busy': ('작업중', theme.CYAN),
    'error': ('오류', theme.RED),
    'stale': ('연결 끊김', theme.RED),
}

TASK_TYPE_STYLE = {
    'outbound': ('출고', theme.BLUE),
    'inbound': ('입고', theme.GREEN),
    'reclaim': ('회수', theme.ORANGE),
}

ORDER_STATUS_STYLE = {
    'pending': ('pending', theme.MUTED),
    'processing': ('processing', theme.CYAN),
    'awaiting_pickup': ('awaiting', theme.AMBER),
    'delivered': ('delivered', theme.MUTED),
    'cancelled': ('cancelled', theme.RED),
}


def _chip(text: str, color: str, alpha: float = 0.15) -> QLabel:
    lbl = QLabel(text)
    lbl.setStyleSheet(theme.chip_qss(color, alpha))
    lbl.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
    return lbl


class Panel(QFrame):
    """제목줄 + 본문을 가진 카드. 본문은 body 레이아웃에 쌓는다."""

    def __init__(self, title: str, cnt: str = '', parent=None):
        super().__init__(parent)
        self.setObjectName('Panel')

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        # 제목줄은 **위젯으로 감싸 높이를 고정**한다. 레이아웃으로 그냥 넣으면 패널에 남는
        # 세로 공간이 생겼을 때(맵 패널처럼 stretch를 받는 경우) 제목줄도 같이 늘어나
        # 헤더가 아래로 밀리고 본문이 하단으로 쏠린다.
        head_w = QWidget()
        head_w.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        head = QHBoxLayout(head_w)
        head.setContentsMargins(14, 11, 14, 11)
        title_lbl = QLabel(title)
        title_lbl.setObjectName('PanelHead')
        head.addWidget(title_lbl)
        head.addStretch(1)
        self.cnt_lbl = QLabel(cnt)
        self.cnt_lbl.setObjectName('PanelHeadCnt')
        head.addWidget(self.cnt_lbl)
        outer.addWidget(head_w)

        line = QFrame()
        line.setFixedHeight(1)
        line.setStyleSheet(f'background: {theme.LINE};')
        outer.addWidget(line)

        self.body = QVBoxLayout()
        self.body.setContentsMargins(14, 12, 14, 12)
        self.body.setSpacing(8)
        outer.addLayout(self.body, 1)   # 남는 공간은 본문만 가져간다

    def set_cnt(self, text: str):
        self.cnt_lbl.setText(text)

    def clear_body(self):
        _clear_layout(self.body)


def _clear_layout(layout):
    """레이아웃을 비운다.

    중첩 레이아웃(addLayout)은 item.widget()이 None이라 위젯만 지우면 그대로 남아
    갱신할 때마다 내용이 쌓인다. 재귀로 안쪽까지 걷어내야 한다.
    """
    while layout.count():
        item = layout.takeAt(0)
        w = item.widget()
        if w is not None:
            w.deleteLater()
            continue
        child = item.layout()
        if child is not None:
            _clear_layout(child)
            child.deleteLater()


class BatteryBar(QWidget):
    """배터리 게이지. fleet_manager의 30% 배정 임계값을 색으로 드러낸다."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(7)
        self._pct = None

    def set_pct(self, pct):
        self._pct = pct
        self.update()

    def paintEvent(self, _e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setPen(QPen(QColor(theme.LINE), 1))
        p.setBrush(QColor('#0a1322'))
        p.drawRoundedRect(0, 0, self.width() - 1, self.height() - 1, 3, 3)

        if self._pct is None:
            return
        color = theme.GREEN if self._pct >= 0.6 else (
            theme.AMBER if self._pct >= 0.3 else theme.RED)
        w = int((self.width() - 2) * max(0.0, min(1.0, self._pct)))
        if w > 0:
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(color))
            p.drawRoundedRect(1, 1, w, self.height() - 3, 3, 3)


class Sparkline(QWidget):
    """KPI 타일용 미니 추이. 값이 없으면 아무것도 안 그린다(가짜 선 금지)."""

    def __init__(self, color: str, parent=None):
        super().__init__(parent)
        self.setFixedHeight(26)
        self._values = []
        self._color = color

    def set_values(self, values):
        self._values = list(values or [])
        self.update()

    def paintEvent(self, _e):
        if not self._values or max(self._values) == 0:
            return
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        top = max(self._values)
        step = self.width() / max(1, len(self._values) - 1)
        poly = QPolygonF([
            QPointF(i * step, self.height() - 2 - (v / top) * (self.height() - 4))
            for i, v in enumerate(self._values)
        ])
        p.setPen(QPen(QColor(self._color), 2))
        p.drawPolyline(poly)


class KpiTile(QFrame):
    def __init__(self, label: str, spark_color: str = theme.CYAN, parent=None):
        super().__init__(parent)
        self.setObjectName('Panel')
        lay = QVBoxLayout(self)
        lay.setContentsMargins(16, 14, 16, 14)
        lay.setSpacing(2)

        lbl = QLabel(label)
        lbl.setObjectName('KpiLbl')
        lay.addWidget(lbl)

        self.val = QLabel('—')
        self.val.setObjectName('KpiVal')
        lay.addWidget(self.val)

        self.sub = QLabel('')
        self.sub.setObjectName('KpiSub')
        lay.addWidget(self.sub)

        self.spark = Sparkline(spark_color)
        lay.addWidget(self.spark)

    def set_value(self, text: str, sub: str = '', sub_color: str = theme.MUTED):
        self.val.setText(text)
        self.sub.setText(sub)
        self.sub.setStyleSheet(f'color: {sub_color}; font-size: 11px; font-weight: 700;')


class RobotCard(QWidget):
    """로봇 1대. ROS(상태·배터리)와 DB(현재 임무)를 한 카드에서 합친다."""

    def __init__(self, robot_id: str, domain: int, parent=None):
        super().__init__(parent)
        self.robot_id = robot_id
        color = theme.ROBOT_COLORS.get(robot_id, theme.CYAN)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 6, 0, 6)
        lay.setSpacing(8)

        top = QHBoxLayout()
        top.setSpacing(10)
        avatar = QLabel(robot_id.replace('AMR_', 'R'))
        avatar.setFixedSize(34, 34)
        avatar.setAlignment(Qt.AlignCenter)
        avatar.setStyleSheet(
            f'background: {color}; color: #06121f; border-radius: 9px; font-weight: 800;')
        top.addWidget(avatar)

        names = QVBoxLayout()
        names.setSpacing(0)
        name = QLabel(robot_id)
        name.setObjectName('RName')
        sub = QLabel(f'TurtleBot3 Burger · 도메인 {domain}')
        sub.setObjectName('RSub')
        names.addWidget(name)
        names.addWidget(sub)
        top.addLayout(names)
        top.addStretch(1)

        self.status_chip = _chip('연결 끊김', theme.RED)
        top.addWidget(self.status_chip)
        lay.addLayout(top)

        batt = QHBoxLayout()
        batt.setSpacing(8)
        self.batt_bar = BatteryBar()
        batt.addWidget(QLabel('🔋'))
        batt.addWidget(self.batt_bar, 1)
        self.batt_lbl = QLabel('—')
        self.batt_lbl.setStyleSheet(f'font-family: {theme.MONO}; color: {theme.INK};')
        self.batt_lbl.setFixedWidth(38)
        self.batt_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        batt.addWidget(self.batt_lbl)
        lay.addLayout(batt)

        self.meta = QLabel('현재 임무 · —')
        self.meta.setObjectName('RMeta')
        self.meta.setWordWrap(True)
        lay.addWidget(self.meta)

    def set_status(self, status: str):
        text, color = STATUS_STYLE.get(status, (status, theme.MUTED))
        self.status_chip.setText(text)
        self.status_chip.setStyleSheet(theme.chip_qss(color))

    def set_battery(self, pct):
        self.batt_bar.set_pct(pct)
        self.batt_lbl.setText('—' if pct is None else f'{pct * 100:.0f}%')

    def set_task(self, task):
        """DB tasks 1행(dict) 또는 None."""
        if not task:
            self.meta.setText(
                f'현재 임무 · <b style="color:{theme.INK}">—</b><br>'
                f'<span style="color:{theme.DIM}">배정된 임무 없음</span>')
            return
        type_name, _ = TASK_TYPE_STYLE.get(task['type'], (task['type'], theme.MUTED))
        # picked_at이 찍혔으면 source에서 물건을 이미 들었다는 뜻 (fleet이 source_arrived로 스탬프)
        phase = '적재 완료(picked)' if task.get('picked_at') else 'source 이동 중'
        self.meta.setText(
            f'현재 임무 · <b style="color:{theme.INK}">#{task["id"]} '
            f'{task["product_name"]} {type_name}</b><br>'
            f'위치 · <b style="color:{theme.INK}">{task["source_location_id"]} → '
            f'{task["target_location_id"]}</b> · {phase}')


def make_row(chip_text, chip_color, main: str, meta: str, badge_text=None,
             badge_color=theme.MUTED) -> QWidget:
    """임무 큐·주문 목록의 한 줄 (목업 .row)."""
    row = QWidget()
    lay = QHBoxLayout(row)
    lay.setContentsMargins(0, 6, 0, 6)
    lay.setSpacing(10)

    if chip_text:
        lay.addWidget(_chip(chip_text, chip_color, 0.16))

    texts = QVBoxLayout()
    texts.setSpacing(1)
    m = QLabel(main)
    m.setObjectName('RowMain')
    s = QLabel(meta)
    s.setObjectName('RowMeta')
    texts.addWidget(m)
    texts.addWidget(s)
    lay.addLayout(texts, 1)

    if badge_text:
        lay.addWidget(_chip(badge_text, badge_color))
    return row


def make_alert(icon: str, title: str, detail: str, color: str) -> QWidget:
    """알림 한 줄 (목업 .alert — 좌측 심각도 색 바)."""
    row = QWidget()
    lay = QHBoxLayout(row)
    lay.setContentsMargins(0, 6, 0, 6)
    lay.setSpacing(9)

    bar = QFrame()
    bar.setFixedWidth(3)
    bar.setStyleSheet(f'background: {color}; border-radius: 1px;')
    lay.addWidget(bar)

    lay.addWidget(QLabel(icon))

    texts = QVBoxLayout()
    texts.setSpacing(1)
    t = QLabel(title)
    t.setObjectName('RowMain')
    t.setWordWrap(True)
    d = QLabel(detail)
    d.setObjectName('RowMeta')
    d.setWordWrap(True)
    texts.addWidget(t)
    texts.addWidget(d)
    lay.addLayout(texts, 1)
    return row


def make_obstacle_alert(icon: str, title: str, detail: str, color: str,
                        on_clear=None) -> QWidget:
    """장애물 차단 알림. on_clear가 있으면 [제거] 버튼(일반 노드 막힘=우회),
    없으면 버튼 없이 자동 해제 안내(목적지 막힘·우회로 없음=대기)."""
    row = QWidget()
    lay = QHBoxLayout(row)
    lay.setContentsMargins(0, 6, 0, 6)
    lay.setSpacing(9)

    bar = QFrame()
    bar.setFixedWidth(3)
    bar.setStyleSheet(f'background: {color}; border-radius: 1px;')
    lay.addWidget(bar)

    lay.addWidget(QLabel(icon))

    texts = QVBoxLayout()
    texts.setSpacing(1)
    t = QLabel(title)
    t.setObjectName('RowMain')
    t.setWordWrap(True)
    d = QLabel(detail)
    d.setObjectName('RowMeta')
    d.setWordWrap(True)
    texts.addWidget(t)
    texts.addWidget(d)
    lay.addLayout(texts, 1)

    if on_clear is not None:
        btn = QPushButton('제거')
        btn.setObjectName('RBtn')
        btn.clicked.connect(on_clear)
        btn.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        lay.addWidget(btn)
    return row


def make_empty(text: str) -> QLabel:
    lbl = QLabel(text)
    lbl.setObjectName('Empty')
    lbl.setAlignment(Qt.AlignCenter)
    lbl.setContentsMargins(0, 14, 0, 14)
    return lbl
