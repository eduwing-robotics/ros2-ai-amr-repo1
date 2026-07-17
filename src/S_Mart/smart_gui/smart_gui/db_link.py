"""PostgreSQL 조회 + NOTIFY 구독.

커넥션 2개로 나눈 이유는 task_manager와 동일:
  _q     : 조회 전용 (GUI 스레드의 QTimer가 사용)
  _listen: LISTEN 전용, **AUTOCOMMIT 필수** — 트랜잭션 안에서는 NOTIFY를 못 받는다.

NOTIFY는 orders/locations 변경만 알려준다. tasks에는 NOTIFY가 없어서
임무 큐·로봇 현재임무는 폴링(main.py의 1초 타이머)으로 갱신한다.
"""

import json
import select
import threading

import psycopg2
import psycopg2.extras
from PyQt5.QtCore import QObject, pyqtSignal

_DSN = 'dbname=s_mart user=codelab password=codelab host=localhost connect_timeout=3'

# fleet_manager가 배정 제외하는 배터리 임계값. 알림 기준을 여기 맞춘다.
BATTERY_MIN = 0.3

_CHANNELS = ['new_order', 'order_status_updated', 'order_cancelled', 'location_updated']


class DbLink(QObject):
    """DB 조회 래퍼. 결과는 dict 리스트로 반환하고 위젯은 모른다."""

    notified = pyqtSignal(str)      # NOTIFY 채널명 → 즉시 갱신 트리거
    error = pyqtSignal(str)         # 연결/쿼리 실패 → 상단바 표시용

    def __init__(self):
        super().__init__()
        self._q = None
        self._alive = False
        self._connect()

        self._listen_thread = threading.Thread(target=self._listen_loop, daemon=True)
        self._listen_thread.start()

    def _connect(self):
        try:
            self._q = psycopg2.connect(_DSN, cursor_factory=psycopg2.extras.RealDictCursor)
            self._alive = True
        except psycopg2.Error as e:
            self._alive = False
            self.error.emit(str(e).strip())

    @property
    def alive(self) -> bool:
        return self._alive

    def _fetch(self, sql: str, args=()) -> list:
        """조회 1회. 실패하면 빈 리스트 — UI는 '데이터 없음'으로 흘러가고 죽지 않는다."""
        if not self._alive:
            self._connect()
            if not self._alive:
                return []
        try:
            with self._q.cursor() as cur:
                cur.execute(sql, args)
                return [dict(r) for r in cur.fetchall()]
        except psycopg2.Error as e:
            self._q.rollback() if self._q and not self._q.closed else None
            self._alive = False
            self.error.emit(str(e).strip())
            return []

    # ── LISTEN 스레드 ────────────────────────────────────────

    def _listen_loop(self):
        conn = None
        while True:
            if conn is None or conn.closed:
                try:
                    conn = psycopg2.connect(_DSN)
                    conn.set_isolation_level(
                        psycopg2.extensions.ISOLATION_LEVEL_AUTOCOMMIT)
                    with conn.cursor() as cur:
                        for ch in _CHANNELS:
                            cur.execute(f'LISTEN {ch}')
                except psycopg2.Error:
                    conn = None
                    threading.Event().wait(2.0)   # 재연결 백오프
                    continue
            try:
                if select.select([conn], [], [], 1.0) == ([], [], []):
                    continue
                conn.poll()
                while conn.notifies:
                    n = conn.notifies.pop(0)
                    self.notified.emit(n.channel)
            except psycopg2.Error:
                conn = None

    # ── 대시보드 쿼리 ─────────────────────────────────────────

    def current_task(self, robot_id: str):
        """로봇의 현재 임무 1건.

        assigned가 2건 이상 보일 수 있어(과거 fleet 버그 이력) 최신 배정을 집는다.
        """
        rows = self._fetch(
            """
            SELECT id, type::text, product_name, source_location_id,
                   target_location_id, picked_at, assigned_at
              FROM tasks
             WHERE robot_id = %s AND status = 'assigned'
             ORDER BY assigned_at DESC NULLS LAST
             LIMIT 1
            """,
            (robot_id,),
        )
        return rows[0] if rows else None

    def inventory(self) -> list:
        return self._fetch(
            """
            SELECT location_id, storage_type::text, product_name, inbound_at, reserved_by
              FROM locations
             ORDER BY location_id
            """
        )

    def task_queue(self, limit: int = 20) -> list:
        return self._fetch(
            """
            SELECT id, type::text, status::text, product_name, robot_id,
                   source_location_id, target_location_id
              FROM tasks
             WHERE status IN ('pending', 'assigned')
             ORDER BY created_at
             LIMIT %s
            """,
            (limit,),
        )

    def recent_orders(self, limit: int = 8) -> list:
        return self._fetch(
            """
            SELECT o.id, o.product_name, o.status::text, o.cancel_reason::text,
                   o.created_at, u.name AS user_name
              FROM orders o
              JOIN users u ON u.id = o.user_id
             ORDER BY o.created_at DESC
             LIMIT %s
            """,
            (limit,),
        )

    def alert_orders(self) -> list:
        """알림 소스: 수령 대기 중(경과시간 표시)  +  오배송 문의(관제 개입 필요)."""
        return self._fetch(
            """
            SELECT id, product_name, status::text, cancel_reason::text, created_at,
                   EXTRACT(EPOCH FROM (now() - created_at))::int AS age_sec
              FROM orders
             WHERE status = 'awaiting_pickup'
                OR cancel_reason = 'misdelivery'
             ORDER BY created_at DESC
             LIMIT 10
            """
        )

    def kpis(self) -> dict:
        """오늘 처리량 · 평균 배송시간 · 오배송률. tasks가 비면 전부 None."""
        done = self._fetch(
            """
            SELECT count(*) AS n,
                   avg(EXTRACT(EPOCH FROM (completed_at - assigned_at))) AS avg_sec
              FROM tasks
             WHERE status = 'done'
               AND completed_at >= date_trunc('day', now())
            """
        )
        mis = self._fetch(
            """
            SELECT count(*) FILTER (WHERE cancel_reason = 'misdelivery') AS mis,
                   count(*) FILTER (WHERE status IN ('delivered', 'cancelled')) AS closed
              FROM orders
             WHERE created_at >= date_trunc('day', now())
            """
        )
        d = done[0] if done else {}
        m = mis[0] if mis else {}
        closed = m.get('closed') or 0
        return {
            'done_today': d.get('n') or 0,
            'avg_sec': d.get('avg_sec'),
            'mis_rate': (m.get('mis') / closed * 100.0) if closed else None,
        }

    def hourly_done(self) -> list:
        """스파크라인용 — 오늘 시간대별 완료 건수 (0~23시, 없으면 0)."""
        rows = self._fetch(
            """
            SELECT EXTRACT(HOUR FROM completed_at)::int AS h, count(*) AS n
              FROM tasks
             WHERE status = 'done'
               AND completed_at >= date_trunc('day', now())
             GROUP BY 1
             ORDER BY 1
            """
        )
        buckets = [0] * 24
        for r in rows:
            buckets[r['h']] = r['n']
        return buckets

    # ── 임무·주문 페이지 ──────────────────────────────────────

    def all_tasks(self, limit: int = 100) -> list:
        """전체 생애주기(대시보드 큐와 달리 done/cancelled/failed 포함)."""
        return self._fetch(
            """
            SELECT id, type::text, status::text, product_name, robot_id, order_id,
                   source_location_id, target_location_id,
                   assigned_at, picked_at, completed_at, cancel_requested_at, created_at
              FROM tasks
             ORDER BY id DESC
             LIMIT %s
            """,
            (limit,),
        )

    def all_orders(self, limit: int = 100) -> list:
        return self._fetch(
            """
            SELECT o.id, o.product_name, o.status::text, o.cancel_reason::text,
                   o.created_at, u.name AS user_name
              FROM orders o
              JOIN users u ON u.id = o.user_id
             ORDER BY o.id DESC
             LIMIT %s
            """,
            (limit,),
        )

    def event_logs(self, task_id=None, limit: int = 100) -> list:
        """task_id를 주면 그 임무의 이벤트만 (event_logs.task_id FK)."""
        if task_id is None:
            return self._fetch(
                """
                SELECT e.id, e.task_id, e.event::text, e.robot_id, e.occurred_at,
                       t.type::text AS task_type, t.product_name
                  FROM event_logs e
                  JOIN tasks t ON t.id = e.task_id
                 ORDER BY e.occurred_at DESC, e.id DESC
                 LIMIT %s
                """,
                (limit,),
            )
        return self._fetch(
            """
            SELECT e.id, e.task_id, e.event::text, e.robot_id, e.occurred_at,
                   t.type::text AS task_type, t.product_name
              FROM event_logs e
              JOIN tasks t ON t.id = e.task_id
             WHERE e.task_id = %s
             ORDER BY e.occurred_at DESC, e.id DESC
             LIMIT %s
            """,
            (task_id, limit),
        )

    # ── 관제 개입 ─────────────────────────────────────────────

    def cancel_order(self, order_id: int) -> str:
        """관리자 수동 취소. 성공하면 '', 실패하면 사유 문자열.

        고객 앱 취소와 **완전히 같은 경로**를 탄다: orders를 cancelled(user)로 바꾸고
        NOTIFY order_cancelled → task_manager가 로봇 계층으로 전파(context/14 방식1).
        여기서 로봇에 직접 명령하지 않는 게 핵심 — 취소 전파는 이미 있는 계약이다.

        허용 상태를 pending/processing으로 막는 것도 서버(`POST /orders/{id}/cancel`)와 동일.
        awaiting_pickup은 물건이 이미 게이트에 나가 있어 '취소'가 아니라 reclaim 대상이다.
        """
        if not self._alive:
            return 'DB 연결 없음'
        try:
            with self._q.cursor() as cur:
                cur.execute(
                    """
                    UPDATE orders SET status = 'cancelled', cancel_reason = 'user'
                     WHERE id = %s AND status IN ('pending', 'processing')
                    """,
                    (order_id,),
                )
                if cur.rowcount == 0:
                    self._q.rollback()
                    return '취소 불가 상태 (pending/processing만 가능)'
                payload = json.dumps({'order_id': order_id})
                # 로봇 계층 전파 (task_manager가 LISTEN)
                cur.execute("SELECT pg_notify('order_cancelled', %s)", (payload,))
                # 고객 UI 갱신 (FastAPI가 LISTEN → WebSocket broadcast)
                cur.execute(
                    "SELECT pg_notify('order_status_updated', %s)",
                    (json.dumps({'order_id': order_id, 'status': 'cancelled',
                                 'cancel_reason': 'user'}),))
            self._q.commit()
            return ''
        except psycopg2.Error as e:
            self._q.rollback()
            self.error.emit(str(e).strip())
            return str(e).strip()
