"""Smart Mart 관제 UI — 메인 윈도우 (대시보드 페이지).

실행: ros2 run smart_gui admin_gui     (ROS_DOMAIN_ID=12, 서버 워크스테이션)

데이터 경로 3개:
  ROS(도메인12)  robot_status·battery_state·traffic/pose  → RosLink 시그널 → 위젯
  DB NOTIFY      new_order·order_status_updated·…         → DbLink.notified → 재조회
  DB 폴링(1s)    tasks (NOTIFY 없음)                      → _tick

모든 위젯 갱신은 GUI 스레드에서만 일어난다. RosLink/DbLink는 시그널만 쏜다.
"""

import signal
import sys
from datetime import datetime

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication, QButtonGroup, QFrame, QHBoxLayout, QLabel, QMainWindow,
    QPushButton, QScrollArea, QStackedWidget, QVBoxLayout, QWidget,
)

from . import theme
from .camera_link import CameraHub
from .db_link import BATTERY_MIN, DbLink
from .map_view import MapView
from .ops_page import OpsPage
from .panels import (
    ORDER_STATUS_STYLE, TASK_TYPE_STYLE, KpiTile, Panel, RobotCard, make_alert,
    make_empty, make_obstacle_alert, make_row,
)
from .ros_link import ROBOT_IDS, RosLink
from .vision_page import VisionPage

# 로봇 → 온보드 도메인 (표시용). 격리 규칙은 project_domain_deployment 참조.
ROBOT_DOMAINS = {'AMR_1': 30, 'AMR_2': 31}


def _fmt_age(sec: int) -> str:
    """경과 시간 사람이 읽는 형식. 오래된 주문이 '27018분'으로 나오지 않게."""
    if sec < 60:
        return f'{sec}초'
    if sec < 3600:
        return f'{sec // 60}분 {sec % 60}초'
    if sec < 86400:
        return f'{sec // 3600}시간 {sec % 3600 // 60}분'
    return f'{sec // 86400}일 전'


class UtilizationTracker:
    """가동률 — 이력 테이블이 없어 **UI가 켜진 이후**만 샘플링해서 낸다.

    1초 틱마다 각 로봇이 busy였는지 세는 근사치. 프로세스를 재시작하면 0부터.
    (진짜 가동률을 내려면 robot_status 이력을 남기는 테이블이 필요 — 2차 백로그)
    """

    def __init__(self):
        self._busy = 0        # 로봇-초 (로봇 2대가 1초 busy면 2)
        self._total = 0       # 관측된 로봇-초
        self._seconds = 0     # 샘플링한 실제 시간(초)

    def sample(self, statuses: dict):
        """statuses = 이번 틱에 관측된 로봇들(연결 끊긴 로봇은 분모에서 제외)."""
        if not statuses:
            return
        self._seconds += 1
        for status in statuses.values():
            self._total += 1
            if status == 'busy':
                self._busy += 1

    def ratio(self):
        return (self._busy / self._total * 100.0) if self._total else None

    @property
    def seconds(self) -> int:
        return self._seconds


class MainWindow(QMainWindow):
    def __init__(self, ros: RosLink, db: DbLink):
        super().__init__()
        self.ros = ros
        self.db = db
        self.cams = CameraHub()
        self.util = UtilizationTracker()

        self._status = {r: None for r in ROBOT_IDS}     # ROS 수신 원본
        self._battery = {r: None for r in ROBOT_IDS}
        self._queue_len = 0
        self._obstacles = {}     # node → {'kind':..., 'robot':...}  traffic 차단 상태
        # 1초마다 도는 패널은 내용이 바뀔 때만 다시 그린다 (깜빡임·CPU 낭비 방지)
        self._sig = {}

        self.setWindowTitle('S-Mart 관제 시스템')
        self.resize(1480, 900)

        root = QWidget()
        self.setCentralWidget(root)
        lay = QVBoxLayout(root)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        lay.addWidget(self._build_topbar())

        shell = QHBoxLayout()
        shell.setContentsMargins(0, 0, 0, 0)
        shell.setSpacing(0)
        shell.addWidget(self._build_nav())

        self.stack = QStackedWidget()
        self.stack.addWidget(self._build_dashboard())        # index 0
        self.vision = VisionPage(self.cams, ros, db)
        self.stack.addWidget(self.vision)                    # index 1
        self.ops = OpsPage(ros, db)
        self.stack.addWidget(self.ops)                       # index 2
        # 시작 페이지를 명시적으로 못박는다. Qt의 "첫 위젯이 current" 기본동작에 기대면
        # 자식 위젯의 show 타이밍에 따라 다른 페이지가 current가 될 수 있고, 그러면
        # 비전 페이지가 떠서 **기동하자마자 카메라를 구독**한다(= lazy 구독 설계 무력화).
        self.stack.setCurrentIndex(0)
        shell.addWidget(self.stack, 1)
        lay.addLayout(shell, 1)

        # ── 시그널 배선 ──
        ros.status_changed.connect(self._on_status)
        ros.battery_changed.connect(self._on_battery)
        ros.pose_changed.connect(self._on_pose)
        ros.obstacle_changed.connect(self._on_obstacle)
        db.notified.connect(self._on_notify)
        db.error.connect(self._on_db_error)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(1000)

        self._slow = QTimer(self)
        self._slow.timeout.connect(self._refresh_slow)
        self._slow.start(15000)

        self._refresh_slow()
        self._tick()

    # ── 상단바 ────────────────────────────────────────────────

    def _build_topbar(self) -> QWidget:
        bar = QFrame()
        bar.setObjectName('TopBar')
        lay = QHBoxLayout(bar)
        lay.setContentsMargins(20, 12, 20, 12)
        lay.setSpacing(18)

        brand = QLabel('🛒 S-Mart')
        brand.setObjectName('Brand')
        lay.addWidget(brand)
        tag = QLabel('관제 시스템')
        tag.setObjectName('BrandTag')
        lay.addWidget(tag)

        self.sys_lbl = QLabel('● 연결 확인 중')
        self.sys_lbl.setStyleSheet(f'color: {theme.MUTED}; font-weight: 600;')
        lay.addWidget(self.sys_lbl)
        lay.addStretch(1)

        self.pill = QLabel('로봇 0 가동 · 임무 0')
        self.pill.setObjectName('Pill')
        lay.addWidget(self.pill)

        self.clock = QLabel('--:--:--')
        self.clock.setObjectName('Clock')
        lay.addWidget(self.clock)
        return bar

    def _build_nav(self) -> QWidget:
        nav = QFrame()
        nav.setObjectName('Nav')
        nav.setFixedWidth(200)
        lay = QVBoxLayout(nav)
        lay.setContentsMargins(10, 12, 10, 12)
        lay.setSpacing(4)

        group = QButtonGroup(self)
        sec = QLabel('모니터링')
        sec.setObjectName('NavSec')
        lay.addWidget(sec)

        # 목업 6페이지 → 3페이지로 축소(근거: context/07 §①).
        # 로봇 상세는 비전에 흡수(전방 카메라 외엔 도메인12에서 관측 불가), 창고/재고는
        # 대시보드 재고 패널과 중복, 이벤트/알림은 임무·주문의 섹션으로.
        for text, page in [('🏠  대시보드', 0), ('📹  비전/카메라', 1)]:
            lay.addWidget(self._nav_item(text, page, group))
        sec2 = QLabel('운영')
        sec2.setObjectName('NavSec')
        lay.addWidget(sec2)
        lay.addWidget(self._nav_item('📋  임무·주문', 2, group))

        lay.addStretch(1)
        self.nav_foot = QLabel('ROS2 Jazzy · 도메인 12')
        self.nav_foot.setObjectName('NavFoot')
        self.nav_foot.setWordWrap(True)
        lay.addWidget(self.nav_foot)
        return nav

    def _nav_item(self, text: str, page, group: QButtonGroup) -> QPushButton:
        """page = stack 인덱스. None이면 아직 안 만든 페이지 → 비활성."""
        btn = QPushButton(text)
        btn.setObjectName('NavItem')
        btn.setCheckable(True)
        btn.setEnabled(page is not None)
        if page is None:
            btn.setToolTip('아직 구현 안 됨 — 페이지 단위로 붙이는 중')
            return btn
        group.addButton(btn)
        btn.setChecked(page == 0)
        btn.clicked.connect(lambda _c, p=page: self.stack.setCurrentIndex(p))
        return btn

    # ── 대시보드 페이지 ───────────────────────────────────────

    def _build_dashboard(self) -> QWidget:
        page = QWidget()
        lay = QVBoxLayout(page)
        lay.setContentsMargins(16, 16, 16, 16)
        lay.setSpacing(14)

        title_row = QHBoxLayout()
        title = QLabel('대시보드')
        title.setObjectName('PageTitle')
        sub = QLabel('전체 현황 요약 · 실시간')
        sub.setObjectName('PageSub')
        title_row.addWidget(title)
        title_row.addWidget(sub)
        title_row.addStretch(1)
        lay.addLayout(title_row)

        lay.addLayout(self._build_kpis())

        cols = QHBoxLayout()
        cols.setSpacing(14)
        cols.addWidget(self._build_left_col(), 0)
        cols.addWidget(self._build_map_panel(), 1)
        cols.addWidget(self._build_right_col(), 0)
        lay.addLayout(cols, 1)
        return page

    def _build_kpis(self) -> QHBoxLayout:
        row = QHBoxLayout()
        row.setSpacing(14)
        self.kpi_done = KpiTile('오늘 처리 임무', theme.CYAN)
        self.kpi_util = KpiTile('로봇 가동률', theme.GREEN)
        self.kpi_time = KpiTile('평균 배송 시간', theme.AMBER)
        self.kpi_mis = KpiTile('오배송률', theme.MUTED)
        for tile in (self.kpi_done, self.kpi_util, self.kpi_time, self.kpi_mis):
            row.addWidget(tile)
        return row

    def _build_left_col(self) -> QWidget:
        col = QWidget()
        col.setFixedWidth(300)
        lay = QVBoxLayout(col)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(14)

        fleet = Panel('로봇 상태', 'FLEET')
        self.cards = {}
        for i, robot in enumerate(ROBOT_IDS):
            card = RobotCard(robot, ROBOT_DOMAINS.get(robot, 0))
            self.cards[robot] = card
            fleet.body.addWidget(card)
            if i == 0:
                sep = QFrame()
                sep.setFixedHeight(1)
                sep.setStyleSheet(f'background: {theme.LINE};')
                fleet.body.addWidget(sep)
        lay.addWidget(fleet)

        self.inv_panel = Panel('재고 현황', 'INVENTORY')
        lay.addWidget(self.inv_panel)
        lay.addStretch(1)
        return col

    def _build_map_panel(self) -> QWidget:
        panel = Panel('창고 맵', '실시간 · 5×5 노드 그래프')
        panel.body.setContentsMargins(0, 0, 0, 0)
        self.map = MapView()
        panel.body.addWidget(self.map)
        return panel

    def _build_right_col(self) -> QWidget:
        col = QWidget()
        col.setFixedWidth(330)
        outer = QVBoxLayout(col)
        outer.setContentsMargins(0, 0, 0, 0)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        inner = QWidget()
        lay = QVBoxLayout(inner)
        lay.setContentsMargins(0, 0, 6, 0)
        lay.setSpacing(14)

        self.alert_panel = Panel('알림', '0건')
        self.queue_panel = Panel('임무 큐', 'TASKS')
        self.order_panel = Panel('최근 주문', 'ORDERS')
        for p in (self.alert_panel, self.queue_panel, self.order_panel):
            lay.addWidget(p)
        lay.addStretch(1)

        scroll.setWidget(inner)
        outer.addWidget(scroll)
        return col

    # ── ROS 시그널 (GUI 스레드) ───────────────────────────────

    def _on_status(self, robot: str, status: str):
        self._status[robot] = status
        self.cards[robot].set_status(status)

    def _on_battery(self, robot: str, pct: float):
        self._battery[robot] = pct
        self.cards[robot].set_battery(pct)

    def _on_pose(self, robot: str, x: float, y: float):
        self.map.set_pose(robot, x, y)

    def _on_obstacle(self, event: str, node: str, kind: str, robot: str):
        """traffic 장애물 차단/해제 이벤트 → 맵 오버레이 + 알림 즉시 갱신."""
        if event == 'block':
            self._obstacles[node] = {'kind': kind, 'robot': robot}
        elif event == 'clear':
            self._obstacles.pop(node, None)
        self.map.set_dynamic_blocks({n: o['kind'] for n, o in self._obstacles.items()})
        self._refresh_alerts()

    def _clear_obstacle(self, node: str):
        """제거 버튼 → /traffic/unblock 발행. 실제 해제 표시는 traffic의 clear 이벤트로."""
        self.ros.publish_unblock(node)

    def _on_notify(self, channel: str):
        if channel in ('new_order', 'order_status_updated', 'order_cancelled'):
            self._refresh_orders()
            self._refresh_alerts()
        elif channel == 'location_updated':
            self._refresh_inventory()
        if self.ops.isVisible():
            self.ops.refresh()

    def _on_db_error(self, msg: str):
        self.sys_lbl.setText('● DB 연결 실패')
        self.sys_lbl.setStyleSheet(f'color: {theme.RED}; font-weight: 600;')
        self.sys_lbl.setToolTip(msg)

    # ── 주기 갱신 ─────────────────────────────────────────────

    def _changed(self, key: str, value) -> bool:
        """value가 직전과 다를 때만 True — 패널 재구성 여부 판단용."""
        if self._sig.get(key) == value:
            return False
        self._sig[key] = value
        return True

    def _tick(self):
        """1초: 시계 · stale 판정 · 로봇 현재임무 · 임무 큐 · 알림 · 가동률 샘플."""
        self.clock.setText(datetime.now().strftime('%H:%M:%S'))

        effective = {}
        tasks = {}
        for robot in ROBOT_IDS:
            stale = self.ros.is_stale(robot)
            self.map.set_stale(robot, stale)
            if stale:
                # 1Hz 하트비트가 끊긴 것 = 로봇/브릿지/WiFi 문제. 마지막 값 붙잡고 있지 않는다.
                self.cards[robot].set_status('stale')
                self.cards[robot].set_battery(None)
                effective[robot] = 'stale'
            else:
                status = self._status[robot] or 'idle'
                self.cards[robot].set_status(status)
                self.cards[robot].set_battery(self._battery[robot])
                effective[robot] = status
            tasks[robot] = self.db.current_task(robot)
            self.cards[robot].set_task(tasks[robot])

        vr = self.vision.robot
        self.vision.tick(effective[vr], self._battery[vr], tasks[vr])

        self.util.sample({r: s for r, s in effective.items() if s != 'stale'})
        self._refresh_util()
        self._refresh_queue()
        # 알림은 ROS 상태(stale·error·배터리)에서 파생되므로 로봇과 같은 주기로 돌아야 한다.
        # 느린 타이머에 두면 로봇이 살아난 뒤에도 "연결 끊김"이 남는다.
        self._refresh_alerts()
        self._refresh_topbar(effective)

    def _refresh_slow(self):
        """15초: 변동이 느리고 NOTIFY로도 커버되는 것들 (누락 대비 백업 갱신)."""
        self._refresh_inventory()
        self._refresh_orders()
        self._refresh_kpis()
        # tasks엔 NOTIFY가 없어 임무 테이블은 이 주기로만 따라잡는다(1초로 돌리면 선택이 계속 튄다)
        if self.ops.isVisible():
            self.ops.refresh()

    def _refresh_topbar(self, effective: dict):
        active = sum(1 for s in effective.values() if s == 'busy')
        self.pill.setText(f'로봇 {active} 가동 · 임무 {self._queue_len}')

        stale = [r for r, s in effective.items() if s == 'stale']
        errors = [r for r, s in effective.items() if s == 'error']
        if not self.db.alive:
            text, color = '● DB 연결 실패', theme.RED
        elif errors:
            text, color = f'● 로봇 오류: {", ".join(errors)}', theme.RED
        elif stale:
            text, color = f'● 연결 끊김: {", ".join(stale)}', theme.AMBER
        else:
            text, color = '● 시스템 정상', theme.GREEN
        self.sys_lbl.setText(text)
        self.sys_lbl.setStyleSheet(f'color: {color}; font-weight: 600;')

        live = len([r for r in ROBOT_IDS if r not in stale])
        self.nav_foot.setText(
            f'ROS2 Jazzy · 도메인 12\n로봇 {live}/{len(ROBOT_IDS)} 수신 · '
            f'psql {"연결" if self.db.alive else "끊김"}')

    def _refresh_inventory(self):
        rows = self.db.inventory()
        self.inv_panel.clear_body()
        if not rows:
            self.inv_panel.body.addWidget(make_empty('재고 데이터 없음'))
            return

        for stype, label, color in (('chilled', '냉장 A 창고', theme.BLUE),
                                    ('frozen', '냉동 B 창고', theme.CYAN)):
            slots = [r for r in rows if r['storage_type'] == stype]
            used = [r for r in slots if r['product_name']]
            head = QHBoxLayout()
            head.addWidget(QLabel(label))
            head.addStretch(1)
            cnt = QLabel(f'{len(used)} / {len(slots)} 칸')
            cnt.setStyleSheet('color: #fff; font-weight: 700;')
            head.addWidget(cnt)
            self.inv_panel.body.addLayout(head)

            bar = QFrame()
            bar.setFixedHeight(6)
            pct = int(len(used) / len(slots) * 100) if slots else 0
            bar.setStyleSheet(
                f'background: qlineargradient(x1:0, y1:0, x2:1, y2:0, '
                f'stop:0 {color}, stop:{max(pct / 100, 0.001)} {color}, '
                f'stop:{min(pct / 100 + 0.001, 1)} #0a1322, stop:1 #0a1322);'
                f'border-radius: 3px;')
            self.inv_panel.body.addWidget(bar)

        counts = {}
        reserved = 0
        for r in rows:
            if r['product_name']:
                counts[r['product_name']] = counts.get(r['product_name'], 0) + 1
            if r['reserved_by']:
                reserved += 1
        summary = ' · '.join(
            f'{name} <b style="color:#fff">{n}</b>' for name, n in sorted(counts.items()))
        lbl = QLabel(summary or '적재된 품목 없음')
        lbl.setObjectName('RMeta')
        lbl.setWordWrap(True)
        self.inv_panel.body.addWidget(lbl)

        self.inv_panel.set_cnt(f'예약 잠금 {reserved}칸' if reserved else 'INVENTORY')

    def _refresh_queue(self):
        rows = self.db.task_queue()
        self._queue_len = len(rows)
        if not self._changed('queue', [tuple(sorted(r.items())) for r in rows]):
            return
        self.queue_panel.clear_body()
        self.queue_panel.set_cnt(f'{len(rows)}건' if rows else 'TASKS')
        if not rows:
            self.queue_panel.body.addWidget(make_empty('대기·진행 중인 임무 없음'))
            return
        for t in rows:
            type_name, color = TASK_TYPE_STYLE.get(t['type'], (t['type'], theme.MUTED))
            badge_color = theme.CYAN if t['status'] == 'assigned' else theme.MUTED
            meta = f'{t["source_location_id"]} → {t["target_location_id"]}'
            if t['robot_id']:
                meta += f' · {t["robot_id"]}'
            self.queue_panel.body.addWidget(make_row(
                type_name, color, f'#{t["id"]} {t["product_name"]}', meta,
                t['status'], badge_color))

    def _refresh_orders(self):
        rows = self.db.recent_orders()
        self.order_panel.clear_body()
        if not rows:
            self.order_panel.body.addWidget(make_empty('주문 없음'))
            return
        for o in rows:
            label, color = ORDER_STATUS_STYLE.get(o['status'], (o['status'], theme.MUTED))
            meta = o['user_name']
            if o['cancel_reason']:
                meta += f' · {o["cancel_reason"]}'
            self.order_panel.body.addWidget(make_row(
                None, None, f'#{o["id"]} {o["product_name"]}', meta, label, color))

    def _refresh_alerts(self):
        """알림 = 별도 테이블이 아니라 현재 상태에서 **파생**한다."""
        alerts = []

        for robot in ROBOT_IDS:
            if self.ros.is_stale(robot):
                alerts.append(('🔴', f'{robot} 연결 끊김',
                               'robot_status 1Hz 하트비트 수신 중단 · 브릿지/WiFi/로봇 확인',
                               theme.RED))
            elif self._status[robot] == 'error':
                alerts.append(('🔴', f'{robot} 오류 상태',
                               'FSM ERROR — fleet 자동 배정에서 제외됨', theme.RED))
            pct = self._battery[robot]
            if pct is not None and pct < BATTERY_MIN:
                alerts.append(('🔋', f'{robot} 배터리 부족',
                               f'{pct * 100:.0f}% · 배정 임계값 {BATTERY_MIN * 100:.0f}% 미만',
                               theme.AMBER))

        for o in self.db.alert_orders():
            if o['cancel_reason'] == 'misdelivery':
                alerts.append(('🔴', f'오배송 문의 · 주문 #{o["id"]}',
                               f'{o["product_name"]} · 관제 확인 필요', theme.RED))
            elif o['status'] == 'awaiting_pickup':
                # 목업의 "미수령 임박 · 자동취소까지 0:30"은 여기서 낼 수 없다.
                # awaiting_pickup 전환 시각 컬럼이 orders에 없어서 카운트다운의 기준점이
                # 없기 때문 (미수령 타이머는 task_manager 메모리에만 있음).
                # 있는 값(주문 생성 시각)만 그 이름 그대로 보여준다.
                alerts.append(('🟡', f'수령 대기 · 주문 #{o["id"]}',
                               f'{o["product_name"]} · 주문 생성 {_fmt_age(o["age_sec"] or 0)}',
                               theme.AMBER))

        # 장애물 차단 알람(맵과 같은 상태에서 파생). 위젯에 버튼이 붙어 tuple로 못 담으므로
        # 별도 렌더하되, 변경 감지 시그니처에는 포함해 차단/해제 시 패널이 다시 그려지게 한다.
        ob_sig = tuple(sorted(
            (n, o['kind'], o['robot']) for n, o in self._obstacles.items()))
        if not self._changed('alerts', (tuple(alerts), ob_sig)):
            return
        self.alert_panel.clear_body()
        self.alert_panel.set_cnt(f'{len(alerts) + len(self._obstacles)}건')
        if not alerts and not self._obstacles:
            self.alert_panel.body.addWidget(make_empty('알림 없음'))
            return

        # 장애물(로봇 정지 위험)을 맨 위에. reroute=우회+[제거], goal_blocked/no_route=대기·자동해제.
        OB_STYLE = {
            'reroute':      ('🚧', '경유 노드 막힘', '우회 중 · 장애물 치운 뒤 [제거] 클릭', theme.ORANGE),
            'goal_blocked': ('⛔', '목적지 노드 막힘', '대기 중 · 장애물 제거 시 통과하면 자동 해제', theme.RED),
            'no_route':     ('⛔', '우회로 없음', '대기 중 · 장애물 제거 시 통과하면 자동 해제', theme.RED),
        }
        for node, o in self._obstacles.items():
            icon, title, detail, color = OB_STYLE.get(
                o['kind'], ('⛔', '노드 차단', '대기 중', theme.RED))
            robot = o['robot'] or '로봇'
            # 일반(reroute)만 수동 제거 — 우회한 로봇이 그 노드를 안 지나 자동해제가 안 되기 때문.
            on_clear = (lambda _c=False, n=node: self._clear_obstacle(n)) \
                if o['kind'] == 'reroute' else None
            self.alert_panel.body.addWidget(make_obstacle_alert(
                icon, f'{title} · {node}', f'{robot} · {detail}', color, on_clear))

        for icon, title, detail, color in alerts:
            self.alert_panel.body.addWidget(make_alert(icon, title, detail, color))

    def _refresh_util(self):
        """가동률만 1초 주기 — ROS 상태에서 파생되므로 DB KPI와 갱신 주기가 다르다."""
        util = self.util.ratio()
        self.kpi_util.set_value('—' if util is None else f'{util:.0f} %',
                                f'UI 세션 기준 · {_fmt_age(self.util.seconds)} 관측')

    def _refresh_kpis(self):
        k = self.db.kpis()

        self.kpi_done.set_value(f'{k["done_today"]} 건' if k['done_today'] else '—',
                                '오늘 완료된 task' if k['done_today'] else '오늘 완료 없음')
        self.kpi_done.spark.set_values(self.db.hourly_done())

        if k['avg_sec']:
            m, s = divmod(int(k['avg_sec']), 60)
            self.kpi_time.set_value(f'{m}:{s:02d}', 'assigned → completed 평균')
        else:
            self.kpi_time.set_value('—', '완료 임무 없음')

        if k['mis_rate'] is None:
            self.kpi_mis.set_value('—', '오늘 종료된 주문 없음')
        else:
            self.kpi_mis.set_value(f'{k["mis_rate"]:.1f} %', 'cancel_reason=misdelivery',
                                   theme.RED if k['mis_rate'] > 0 else theme.MUTED)

    def closeEvent(self, event):
        # 카메라 컨텍스트를 먼저 닫는다 — 스핀 중인 executor를 두고 context를 내리면 코어덤프.
        self.cams.shutdown()
        self.ros.shutdown()
        super().closeEvent(event)


def main(args=None):
    # Ctrl+C로 죽을 수 있게 (Qt 이벤트루프는 기본적으로 SIGINT를 안 받는다)
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    app = QApplication(sys.argv)
    app.setStyleSheet(theme.QSS)

    ros = RosLink()
    db = DbLink()
    win = MainWindow(ros, db)
    win.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
