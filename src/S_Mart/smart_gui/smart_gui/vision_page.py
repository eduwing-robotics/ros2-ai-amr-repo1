"""비전/카메라 페이지 — 모든 카메라를 한 곳에.

목업의 '로봇 상세'는 이 페이지에 흡수됐다. 그 페이지에서 실제로 살아남는 건 전방 카메라
하나뿐이었기 때문(FSM/BT 내부상태·ArUco 정렬오차는 도메인12에서 관측 불가, robot_fsm은
busy/idle/error 3종만 외부 발행. 나머지는 대시보드 카드와 중복).

구독은 **이 페이지가 보일 때만** 한다(showEvent/hideEvent). 로봇 전방 카메라는 WiFi를
건너오므로 안 보는 동안 구독을 잡고 있으면 도킹용 estimator의 대역폭을 갉아먹는다.
"""

import time
from datetime import datetime

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import (
    QButtonGroup, QFrame, QHBoxLayout, QLabel, QPushButton, QSizePolicy,
    QVBoxLayout, QWidget,
)

from . import theme
from .camera_link import CAMERAS, GATE_CAMS, ROBOT_CAMS
from .panels import Panel, make_empty

# 이 시간 넘게 프레임이 없으면 '신호 없음'. 로봇캠 10fps·게이트캠 5Hz라 넉넉히 잡음.
NO_SIGNAL_SEC = 2.0

MAX_LOG = 30       # 메모리에 들고 있는 이벤트 수 (event_logs 테이블과 무관한 실시간 로그)
LOG_ROWS = 4       # 화면에 보이는 줄 수
ROW_H = 22


class CameraTile(QFrame):
    """카메라 한 대. 프레임이 없으면 없다고 말한다(정지 화면을 붙잡고 있지 않는다)."""

    def __init__(self, cam_name: str, compact: bool = False, parent=None):
        super().__init__(parent)
        self.setObjectName('Panel')
        self.cam_name = cam_name
        self._last_frame = 0.0
        self._pix = None

        lay = QVBoxLayout(self)
        lay.setContentsMargins(10, 10, 10, 10)
        lay.setSpacing(6)

        head = QHBoxLayout()
        title = QLabel(CAMERAS[cam_name].label)
        title.setObjectName('PanelHead')
        head.addWidget(title)
        head.addStretch(1)
        self.live = QLabel('● 신호 없음')
        self.live.setStyleSheet(f'color: {theme.DIM}; font-size: 10px; font-weight: 700;')
        head.addWidget(self.live)
        lay.addLayout(head)

        self.view = QLabel()
        self.view.setAlignment(Qt.AlignCenter)
        self.view.setMinimumHeight(120 if compact else 260)
        self.view.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.view.setStyleSheet(
            f'background: #0a1120; border: 1px solid {theme.LINE}; border-radius: 8px;'
            f'color: {theme.DIM};')
        self.view.setText('신호 없음')
        lay.addWidget(self.view, 1)

        self.meta = QLabel(f'{CAMERAS[cam_name].topic} · 도메인 {CAMERAS[cam_name].domain}')
        self.meta.setObjectName('RowMeta')
        lay.addWidget(self.meta)

    def set_frame(self, data: bytes):
        pix = QPixmap()
        if not pix.loadFromData(data, 'JPEG'):
            return
        self._pix = pix
        self._last_frame = time.monotonic()
        self._rescale()

    def _rescale(self):
        if self._pix is None:
            return
        self.view.setPixmap(self._pix.scaled(
            self.view.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def resizeEvent(self, e):
        super().resizeEvent(e)
        self._rescale()

    def tick(self):
        """프레임이 끊겼는지 확인. 끊기면 화면을 비운다."""
        alive = self._pix is not None and (time.monotonic() - self._last_frame) < NO_SIGNAL_SEC
        if alive:
            self.live.setText('● LIVE')
            self.live.setStyleSheet(
                f'color: {theme.RED}; font-size: 10px; font-weight: 700;')
        else:
            self.live.setText('● 신호 없음')
            self.live.setStyleSheet(
                f'color: {theme.DIM}; font-size: 10px; font-weight: 700;')
            if self._pix is not None:
                self._pix = None
                self.view.clear()
                self.view.setText('신호 없음')


class VisionPage(QWidget):
    """로봇 전방(탭) + 게이트 3대 + 감지 이벤트 로그."""

    def __init__(self, cams, ros, db, parent=None):
        super().__init__(parent)
        self.cams = cams
        self.ros = ros
        self.db = db
        self.robot = ROBOT_CAMS[0]
        self._log = []

        lay = QVBoxLayout(self)
        lay.setContentsMargins(16, 16, 16, 16)
        lay.setSpacing(14)

        title_row = QHBoxLayout()
        title = QLabel('비전 / 카메라')
        title.setObjectName('PageTitle')
        sub = QLabel('로봇 전방 · 게이트 감지 · 화면이 열려 있는 동안만 구독')
        sub.setObjectName('PageSub')
        title_row.addWidget(title)
        title_row.addWidget(sub)
        title_row.addStretch(1)
        lay.addLayout(title_row)

        # ── 로봇 탭 ──
        tabs = QHBoxLayout()
        tabs.setSpacing(6)
        group = QButtonGroup(self)
        for robot in ROBOT_CAMS:
            b = QPushButton(f'  {robot}  ')
            b.setObjectName('NavItem')
            b.setCheckable(True)
            b.setChecked(robot == self.robot)
            b.setFixedWidth(120)
            b.clicked.connect(lambda _c, r=robot: self._select_robot(r))
            group.addButton(b)
            tabs.addWidget(b)
        tabs.addStretch(1)
        lay.addLayout(tabs)

        # ── 로봇 전방 + 그 로봇 상태 ──
        top = QHBoxLayout()
        top.setSpacing(14)
        self.robot_tiles = {r: CameraTile(r) for r in ROBOT_CAMS}
        for r, tile in self.robot_tiles.items():
            # 레이아웃에 넣어 부모를 붙인 **뒤에** 가시성을 정한다. 부모 없는 위젯에
            # setVisible(True)를 하면 잠깐 최상위 창이 되고, 그 상태로 reparent되면
            # 스택이 이 페이지를 current로 올려버린다.
            top.addWidget(tile, 1)
            tile.setVisible(r == self.robot)
        top.addWidget(self._build_side(), 0)
        lay.addLayout(top, 3)

        # ── 게이트 3대 ──
        gates = QHBoxLayout()
        gates.setSpacing(14)
        self.gate_tiles = {}
        for name in GATE_CAMS:
            tile = CameraTile(name, compact=True)
            self.gate_tiles[name] = tile
            gates.addWidget(tile, 1)
        lay.addLayout(gates, 2)

        # ── 감지 이벤트 로그 ──
        self.log_panel = Panel('감지 이벤트', '/detection/*')
        # 높이 = 제목줄 + LOG_ROWS 행이 눌리지 않는 최소치. 이걸 줄이면 행이 납작해져 글자가 사라진다.
        self.log_panel.setFixedHeight(52 + LOG_ROWS * (ROW_H + 8) + 16)
        self.log_panel.body.addWidget(make_empty('감지 이벤트 없음'))
        lay.addWidget(self.log_panel)

        ros.detection.connect(self._on_detection)
        cams.frame.connect(self._on_frame)

    def _build_side(self) -> QWidget:
        """전방 카메라 옆 로봇 상태 — 도킹 정렬을 눈으로 볼 때 맥락이 필요하다."""
        panel = Panel('로봇 상태', 'FLEET')
        panel.setFixedWidth(240)
        self.side_status = QLabel('—')
        self.side_status.setObjectName('RMeta')
        self.side_status.setWordWrap(True)
        panel.body.addWidget(self.side_status)
        panel.body.addStretch(1)
        return panel

    # ── 구독 수명 = 페이지 표시 여부 ──────────────────────────

    def showEvent(self, e):
        super().showEvent(e)
        self.cams.start(self.robot)
        for name in GATE_CAMS:
            self.cams.start(name)

    def hideEvent(self, e):
        super().hideEvent(e)
        # 안 보이는 동안 구독을 놓는다 = WiFi/네트워크 비용 0
        self.cams.stop_all()

    def _select_robot(self, robot: str):
        if robot == self.robot:
            return
        self.cams.stop(self.robot)          # 다른 로봇 캠은 즉시 끊는다
        self.robot = robot
        for r, tile in self.robot_tiles.items():
            tile.setVisible(r == robot)
        if self.isVisible():
            self.cams.start(robot)

    # ── 수신 ──────────────────────────────────────────────────

    def _on_frame(self, name: str, data: bytes):
        tile = self.robot_tiles.get(name) or self.gate_tiles.get(name)
        if tile is not None:
            tile.set_frame(data)

    def _on_detection(self, kind: str, summary: str):
        label = {'inbound': ('입고 감지', theme.GREEN),
                 'placed': ('출고 놓임', theme.AMBER),
                 'cleared': ('수령/치워짐', theme.CYAN)}.get(kind, (kind, theme.MUTED))
        self._log.insert(0, (datetime.now().strftime('%H:%M:%S'), label[0], label[1], summary))
        del self._log[MAX_LOG:]
        self._render_log()

    def _render_log(self):
        self.log_panel.clear_body()
        self.log_panel.set_cnt(f'{len(self._log)}건')
        if not self._log:
            self.log_panel.body.addWidget(make_empty('감지 이벤트 없음'))
            return
        for ts, name, color, summary in self._log[:LOG_ROWS]:
            row = QHBoxLayout()
            row.setContentsMargins(0, 0, 0, 0)
            t = QLabel(ts)
            t.setStyleSheet(f'font-family: {theme.MONO}; color: {theme.MUTED};')
            t.setFixedWidth(64)
            k = QLabel(name)
            k.setStyleSheet(f'color: {color}; font-weight: 700;')
            k.setFixedWidth(90)
            s = QLabel(summary)
            s.setObjectName('RowMain')
            row.addWidget(t)
            row.addWidget(k)
            row.addWidget(s, 1)
            w = QWidget()
            w.setFixedHeight(ROW_H)
            w.setLayout(row)
            self.log_panel.body.addWidget(w)
        self.log_panel.body.addStretch(1)

    # ── 1초 틱 (main이 호출) ──────────────────────────────────

    def tick(self, status: str, battery, task):
        for tile in list(self.robot_tiles.values()) + list(self.gate_tiles.values()):
            tile.tick()

        batt = '—' if battery is None else f'{battery * 100:.0f}%'
        lines = [f'<b style="color:{theme.INK}">{self.robot}</b>',
                 f'상태 · {status}', f'배터리 · {batt}']
        if task:
            lines.append(f'임무 · #{task["id"]} {task["product_name"]}')
            lines.append(f'{task["source_location_id"]} → {task["target_location_id"]}')
        else:
            lines.append('임무 · 없음')
        self.side_status.setText('<br>'.join(lines))
