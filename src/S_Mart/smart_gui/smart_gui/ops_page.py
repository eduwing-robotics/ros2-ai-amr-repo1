"""임무·주문 페이지 — 유일하게 관제가 시스템에 개입하는 화면.

개입 2종은 **둘 다 기존 계약을 그대로 탄다**. UI가 로봇이나 task에 직접 명령하지 않는다:
  주문 취소 → orders=cancelled + NOTIFY order_cancelled → task_manager가 로봇 계층에 전파
  오배송 회수 → /reclaim_request 발행 → task_manager가 reclaim task 생성

임무 행을 고르면 event_logs를 그 task로 필터한다(FK event_logs.task_id).
"""

from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import (
    QAbstractItemView, QHBoxLayout, QHeaderView, QLabel, QMessageBox, QPushButton,
    QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)

from . import theme
from .panels import ORDER_STATUS_STYLE, TASK_TYPE_STYLE, Panel

TASK_STATUS_COLOR = {
    'pending': theme.MUTED, 'assigned': theme.CYAN, 'done': theme.DIM,
    'cancelled': theme.RED, 'failed': theme.RED,
}
EVENT_COLOR = {
    'created': theme.MUTED, 'assigned': theme.CYAN, 'picked': theme.BLUE,
    'done': theme.GREEN, 'cancelled': theme.RED, 'failed': theme.RED,
    'timed_out': theme.AMBER,
}

# 이 상태의 주문만 취소 가능 — 서버 POST /orders/{id}/cancel과 동일 규칙.
CANCELLABLE = ('pending', 'processing')
# 물건이 게이트로 이미 나간 주문 = 취소가 아니라 회수 대상.
RECLAIMABLE = ('awaiting_pickup', 'cancelled')


def _ts(dt) -> str:
    return dt.strftime('%m/%d %H:%M:%S') if dt else '—'


def _table(headers, stretch: int) -> QTableWidget:
    """stretch = 남는 폭을 가져갈 컬럼. 나머지는 내용에 맞춘다.

    전 컬럼 Stretch로 두면 시각·경로가 '07/17 …'처럼 잘린다(컬럼 수가 많아서).
    """
    t = QTableWidget(0, len(headers))
    t.setHorizontalHeaderLabels(headers)
    t.verticalHeader().setVisible(False)
    t.setSelectionBehavior(QAbstractItemView.SelectRows)
    t.setSelectionMode(QAbstractItemView.SingleSelection)
    t.setEditTriggers(QAbstractItemView.NoEditTriggers)
    t.setShowGrid(False)
    t.setAlternatingRowColors(False)
    t.setStyleSheet(f"""
        QTableWidget {{ background: transparent; border: none; }}
        QTableWidget::item {{ padding: 6px 4px; border-bottom: 1px solid {theme.LINE};
                              color: {theme.MUTED}; }}
        QTableWidget::item:selected {{ background: rgba(34,211,238,0.10); color: {theme.INK}; }}
        QHeaderView::section {{ background: {theme.PANEL}; color: {theme.DIM};
            border: none; border-bottom: 1px solid {theme.LINE};
            padding: 7px 4px; font-size: 10px; font-weight: 700; }}
    """)
    h = t.horizontalHeader()
    h.setSectionResizeMode(QHeaderView.ResizeToContents)
    h.setSectionResizeMode(stretch, QHeaderView.Stretch)
    return t


def _cell(text, color=None, mono=False) -> QTableWidgetItem:
    it = QTableWidgetItem(str(text))
    if color:
        it.setForeground(QColor(color))
    if mono:
        f = it.font()
        f.setFamily('monospace')
        it.setFont(f)
    return it


class OpsPage(QWidget):
    def __init__(self, ros, db, parent=None):
        super().__init__(parent)
        self.ros = ros
        self.db = db
        self._sel_task = None       # 선택된 task_id → event_logs 필터

        lay = QVBoxLayout(self)
        lay.setContentsMargins(16, 16, 16, 16)
        lay.setSpacing(14)

        head = QHBoxLayout()
        title = QLabel('임무 · 주문')
        title.setObjectName('PageTitle')
        sub = QLabel('임무 생애주기 · 주문 관리 · 수동 개입')
        sub.setObjectName('PageSub')
        head.addWidget(title)
        head.addWidget(sub)
        head.addStretch(1)
        self.msg = QLabel('')
        self.msg.setObjectName('RowMeta')
        head.addWidget(self.msg)
        lay.addLayout(head)

        top = QHBoxLayout()
        top.setSpacing(14)
        top.addWidget(self._build_tasks(), 3)
        top.addWidget(self._build_orders(), 2)
        lay.addLayout(top, 3)

        lay.addWidget(self._build_events(), 2)

    def _build_tasks(self) -> Panel:
        p = Panel('임무 큐', 'TASKS')
        p.body.setContentsMargins(0, 0, 0, 0)
        self.tasks_tbl = _table(
            ['ID', '타입', '상품', 'source → target', '로봇', '상태', 'assigned', 'picked', '완료'],
            stretch=3)
        self.tasks_tbl.itemSelectionChanged.connect(self._on_task_selected)
        p.body.addWidget(self.tasks_tbl)
        return p

    def _build_orders(self) -> Panel:
        p = Panel('주문', 'ORDERS')
        p.body.setContentsMargins(0, 0, 0, 0)
        self.orders_tbl = _table(['ID', '상품', '고객', '상태', '취소사유'], stretch=2)
        self.orders_tbl.itemSelectionChanged.connect(self._on_order_selected)
        p.body.addWidget(self.orders_tbl)

        btns = QHBoxLayout()
        btns.setContentsMargins(12, 8, 12, 10)
        btns.setSpacing(6)
        self.btn_cancel = QPushButton('주문 취소')
        self.btn_cancel.setObjectName('RBtn')
        self.btn_cancel.setEnabled(False)
        self.btn_cancel.clicked.connect(self._cancel_order)
        self.btn_reclaim = QPushButton('reclaim 발행')
        self.btn_reclaim.setObjectName('RBtn')
        self.btn_reclaim.setEnabled(False)
        self.btn_reclaim.clicked.connect(self._publish_reclaim)
        btns.addWidget(self.btn_cancel)
        btns.addWidget(self.btn_reclaim)
        p.body.addLayout(btns)
        return p

    def _build_events(self) -> Panel:
        self.ev_panel = Panel('이벤트 로그', 'event_logs · 전체')
        self.ev_panel.body.setContentsMargins(0, 0, 0, 0)
        self.ev_tbl = _table(['시각', 'task', '임무', '이벤트', '로봇'], stretch=2)
        self.ev_panel.body.addWidget(self.ev_tbl)
        return self.ev_panel

    # ── 갱신 ──────────────────────────────────────────────────

    def showEvent(self, e):
        super().showEvent(e)
        self.refresh()

    def refresh(self):
        self._fill_tasks(self.db.all_tasks())
        self._fill_orders(self.db.all_orders())
        self._fill_events()

    def _fill_tasks(self, rows):
        self._tasks = rows
        t = self.tasks_tbl
        t.blockSignals(True)
        t.setRowCount(len(rows))
        for i, r in enumerate(rows):
            type_name, type_color = TASK_TYPE_STYLE.get(r['type'], (r['type'], theme.MUTED))
            t.setItem(i, 0, _cell(f'#{r["id"]}', '#ffffff', mono=True))
            t.setItem(i, 1, _cell(type_name, type_color))
            t.setItem(i, 2, _cell(r['product_name']))
            t.setItem(i, 3, _cell(f'{r["source_location_id"]} → {r["target_location_id"]}',
                                  mono=True))
            t.setItem(i, 4, _cell(r['robot_id'] or '—'))
            t.setItem(i, 5, _cell(r['status'], TASK_STATUS_COLOR.get(r['status'], theme.MUTED)))
            t.setItem(i, 6, _cell(_ts(r['assigned_at']), mono=True))
            t.setItem(i, 7, _cell(_ts(r['picked_at']), mono=True))
            t.setItem(i, 8, _cell(_ts(r['completed_at']), mono=True))
        t.blockSignals(False)

    def _fill_orders(self, rows):
        self._orders = rows
        t = self.orders_tbl
        t.blockSignals(True)
        t.setRowCount(len(rows))
        for i, r in enumerate(rows):
            label, color = ORDER_STATUS_STYLE.get(r['status'], (r['status'], theme.MUTED))
            t.setItem(i, 0, _cell(f'#{r["id"]}', '#ffffff', mono=True))
            t.setItem(i, 1, _cell(r['product_name']))
            t.setItem(i, 2, _cell(r['user_name']))
            t.setItem(i, 3, _cell(label, color))
            reason = r['cancel_reason']
            t.setItem(i, 4, _cell(reason or '—',
                                  theme.RED if reason == 'misdelivery' else theme.MUTED))
        t.blockSignals(False)

    def _fill_events(self):
        rows = self.db.event_logs(self._sel_task)
        self.ev_panel.set_cnt(
            f'event_logs · task #{self._sel_task}' if self._sel_task else 'event_logs · 전체')
        t = self.ev_tbl
        t.setRowCount(len(rows))
        for i, r in enumerate(rows):
            type_name, _ = TASK_TYPE_STYLE.get(r['task_type'], (r['task_type'], theme.MUTED))
            t.setItem(i, 0, _cell(_ts(r['occurred_at']), mono=True))
            t.setItem(i, 1, _cell(f'#{r["task_id"]}', mono=True))
            t.setItem(i, 2, _cell(f'{type_name} {r["product_name"]}'))
            t.setItem(i, 3, _cell(r['event'], EVENT_COLOR.get(r['event'], theme.MUTED)))
            t.setItem(i, 4, _cell(r['robot_id'] or '—'))

    # ── 선택 / 개입 ───────────────────────────────────────────

    def _cur(self, table, rows):
        i = table.currentRow()
        return rows[i] if 0 <= i < len(rows) else None

    def _on_task_selected(self):
        task = self._cur(self.tasks_tbl, self._tasks)
        self._sel_task = task['id'] if task else None
        self._fill_events()

    def _on_order_selected(self):
        o = self._cur(self.orders_tbl, self._orders)
        self.btn_cancel.setEnabled(bool(o) and o['status'] in CANCELLABLE)
        self.btn_reclaim.setEnabled(bool(o) and o['status'] in RECLAIMABLE)
        if o and o['status'] not in CANCELLABLE:
            self.btn_cancel.setToolTip('pending/processing만 취소 가능 (서버 규칙과 동일). '
                                       '게이트에 나간 물건은 reclaim 대상')
        else:
            self.btn_cancel.setToolTip('')

    def _cancel_order(self):
        o = self._cur(self.orders_tbl, self._orders)
        if not o:
            return
        if QMessageBox.question(
                self, '주문 취소',
                f'주문 #{o["id"]} ({o["product_name"]})을 취소합니다.\n'
                f'로봇이 수행 중이면 취소 신호가 로봇 계층까지 전파됩니다.') != QMessageBox.Yes:
            return
        err = self.db.cancel_order(o['id'])
        self._flash(f'주문 #{o["id"]} 취소 — 로봇 계층 전파됨' if not err else f'실패: {err}',
                    theme.GREEN if not err else theme.RED)
        self.refresh()

    def _publish_reclaim(self):
        o = self._cur(self.orders_tbl, self._orders)
        if not o:
            return
        if QMessageBox.question(
                self, 'reclaim 발행',
                f'주문 #{o["id"]} ({o["product_name"]}) 회수 임무를 요청합니다.\n'
                f'되돌릴 선반은 task_manager가 정합니다.') != QMessageBox.Yes:
            return
        self.ros.publish_reclaim(o['id'])
        self._flash(f'/reclaim_request 발행 — order #{o["id"]} '
                    f'(task_manager가 처리하면 임무 큐에 나타남)', theme.CYAN)

    def _flash(self, text: str, color: str):
        self.msg.setText(text)
        self.msg.setStyleSheet(f'color: {color}; font-size: 12px; font-weight: 600;')
