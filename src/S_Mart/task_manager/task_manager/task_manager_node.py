#!/usr/bin/env python3
"""Task Manager ROS2 노드 — 임무 생성 전담 (할당은 Fleet Manager가 담당)

트리거 4종:
  1. PostgreSQL LISTEN new_order            → outbound task 생성      (서브 스레드)
  2. PostgreSQL LISTEN order_status_updated → awaiting_pickup 감지 시 미수령 타이머 시작
                                              (서브 스레드)
  3. /detection/inbound 구독  → inbound task 생성                    (메인 스레드 콜백)
  4. /detection/cleared 구독  → 게이트가 비면 outbound done, delivered (메인 스레드 콜백)
  + 1Hz 자체 검사             → 타이머 만료 시 outbound cancelled + reclaim 생성

생성된 task는 /new_task 토픽으로 Fleet Manager에 신호를 보냄.

★ 미수령 타이머를 여기서 소유하는 이유 (2026-07-16 실주행 사고):
  예전엔 감지 노드가 타이머를 들고 no_pickup을 발행했다. 그런데 감지 노드는
  '물건이 보이는 순간' 타이머를 시작하는 반면 여기는 주문이 awaiting_pickup이
  된 뒤에만 신호를 받는다. 로봇이 물건을 내려놓고 완료 보고까지 49초가 걸려
  30초 타이머가 먼저 터졌고, no_pickup은 "awaiting_pickup 주문 없음"으로 버려졌다.
  감지 노드는 그 뒤 COOLDOWN에 갇혀 다시는 신호를 못 보냈고, 주문은
  awaiting_pickup에 영구 고착 → 게이트·선반이 같이 죽었다.
  시계와 주문 상태가 다른 프로세스에 있으면 반드시 어긋난다. 이제 둘 다 여기 있다.

  ※ 마감 시각은 메모리(_deadlines)에 둔다. 이 노드가 재시작하면 진행 중이던
    awaiting_pickup 주문의 마감이 사라져 만료 판정이 영영 안 된다(= 위 고착 재발).
    그때 물리면 orders.awaiting_pickup_at 컬럼을 추가해 DB에서 계산할 것.

연결 구조 (스레드 간 psycopg2 커넥션 공유 불가 → 스레드별 전용 커넥션):
  - _listen_conn (AUTOCOMMIT): 서브 스레드 전용, LISTEN+poll만 사용
  - _sub_db     (기본):        서브 스레드 전용, outbound task INSERT·게이트 조회 트랜잭션
  - _main_db    (기본):        메인 스레드 전용, 나머지 모든 DB 작업

reserved_by 해제 시점:
  - /detection/cleared 수신 시 → 고객이 실제로 상품을 가져간 것이 확인된 시점에만 해제
  - 미수령 시 → outbound task_id에서 reclaim task_id로 교체 (슬롯 잠금 유지)
  - 이렇게 해야 미수령 시 원래 슬롯이 다른 임무에 뺏기지 않음
"""

import json
import select
import threading
import time
from datetime import datetime, timezone

import psycopg2
import psycopg2.extensions
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

_DSN = 'dbname=s_mart user=codelab password=codelab host=localhost'


class TaskManagerNode(Node):
    def __init__(self):
        super().__init__('task_manager')

        # 서브 스레드: LISTEN+poll 전용 (AUTOCOMMIT 필수 — 트랜잭션 안에서는 NOTIFY 수신 불가)
        self._listen_conn = psycopg2.connect(_DSN)
        self._listen_conn.set_isolation_level(
            psycopg2.extensions.ISOLATION_LEVEL_AUTOCOMMIT
        )

        # 서브 스레드: outbound task INSERT 트랜잭션용
        self._sub_db = psycopg2.connect(_DSN)

        # 메인 스레드: inbound/pickup/no_pickup 처리용
        self._main_db = psycopg2.connect(_DSN)

        # 미수령 시한 — 냉동은 녹으므로 더 짧게. awaiting_pickup 시점부터 센다.
        self.declare_parameter('chilled_timeout', 30.0)
        self.declare_parameter('frozen_timeout', 15.0)

        # 미수령 마감 — {게이트: (order_id, time.monotonic() 기준 마감)}.
        # 서브 스레드(LISTEN)가 쓰고 메인 스레드(1Hz 검사)가 읽고 지우므로 락 필요.
        # 벽시계 대신 monotonic — NTP 보정에 흔들리지 않게.
        self._deadlines = {}
        self._deadlines_lock = threading.Lock()

        # Fleet Manager로 새 task 알림
        self._pub = self.create_publisher(String, '/new_task', 10)

        # 고객 능동 취소 → 로봇에 취소 신호 (도메인 브릿지가 12→30/31 중계)
        # 로봇이 payload의 robot_id로 필터. order_id는 로봇이 보고에 되싣어 fleet이 대상 특정.
        self._cancel_pub = self.create_publisher(String, '/cancel_mission', 10)

        # PLACE 중 취소(자연수렴) → fleet이 게이트 회수 reclaim을 위임하는 요청
        self.create_subscription(String, '/reclaim_request', self._on_reclaim_request, 10)

        # 입고 감지 — 카메라가 IN-1 게이트에서 상품명만 감지하여 발행
        # {"product_name": "사과"} — 슬롯은 Task Manager가 선택
        self.create_subscription(String, '/detection/inbound', self._on_inbound, 10)

        # 게이트 비움 감지 — {"slot":"OUT-1"}. 고객 수령인지 reclaim 회수인지는
        # 감지 노드가 모른다. 주문 상태로 여기서 판단한다 (awaiting_pickup일 때만 수령).
        self.create_subscription(String, '/detection/cleared', self._on_cleared, 10)

        # 미수령 마감 검사 — 1Hz면 충분 (시한이 15s/30s 단위)
        self.create_timer(1.0, self._check_expired)

        # LISTEN 서브 스레드 시작 (daemon=True: 메인 스레드 종료 시 자동 종료)
        threading.Thread(target=self._listen_orders, daemon=True).start()

        self.get_logger().info('Task Manager 시작')

    # ── 서브 스레드: LISTEN new_order ─────────────────────────────────────────

    def _listen_orders(self):
        """PostgreSQL NOTIFY 대기 → 채널별 처리.

          new_order            → outbound task 생성
          order_status_updated → awaiting_pickup이면 미수령 타이머 시작
                                 (fleet_manager가 로봇 완료 보고 시 발행)

        rclpy.spin()이 메인 스레드를 점유하기 때문에 서브 스레드에서 실행.
        NOTIFY가 올 때까지 select()로 블로킹 대기하며 메인 스레드와 간섭 없음.
        """
        cur = self._listen_conn.cursor()
        cur.execute('LISTEN new_order')
        cur.execute('LISTEN order_status_updated')
        cur.execute('LISTEN order_cancelled')
        self.get_logger().info('LISTEN new_order, order_status_updated, order_cancelled 등록 완료')

        while rclpy.ok():
            # 5초마다 종료 신호 체크 (rclpy.ok()가 False가 되면 루프 탈출)
            if select.select([self._listen_conn], [], [], 5.0)[0]:
                self._listen_conn.poll()
                for notify in self._listen_conn.notifies:
                    try:
                        payload = json.loads(notify.payload)
                        if notify.channel == 'new_order':
                            self._create_outbound_task(payload['order_id'])
                        elif notify.channel == 'order_cancelled':
                            self._on_order_cancelled(payload['order_id'])
                        elif payload.get('status') == 'awaiting_pickup':
                            self._arm_pickup_timer(payload['order_id'])
                    except Exception as e:
                        self.get_logger().error(
                            f'NOTIFY 처리 실패 ({notify.channel}): {e}')
                self._listen_conn.notifies.clear()

    def _arm_pickup_timer(self, order_id):
        """awaiting_pickup 진입 → 그 게이트의 미수령 마감 등록 (서브 스레드).

        시한은 상품의 storage_type(DB)으로 고른다 — 감지 노드가 분류를 실어보낼
        필요가 없다. 게이트도 감지 신호가 아니라 DB의 outbound task에서 읽는다.
        """
        cur = self._sub_db.cursor()
        try:
            cur.execute(
                """
                SELECT t.target_location_id, p.storage_type
                FROM tasks t
                JOIN orders o ON o.id = t.order_id
                JOIN products p ON p.name = o.product_name
                WHERE t.order_id = %s AND t.type = 'outbound'
                ORDER BY t.created_at DESC LIMIT 1
                """,
                (order_id,)
            )
            row = cur.fetchone()
            self._sub_db.commit()
            if not row:
                self.get_logger().warn(
                    f'order_id={order_id}의 outbound task 없음 — 미수령 타이머 미등록')
                return
            gate, storage = row
            timeout = (self.get_parameter('frozen_timeout').value
                       if storage == 'frozen'
                       else self.get_parameter('chilled_timeout').value)
            with self._deadlines_lock:
                self._deadlines[gate] = (order_id, time.monotonic() + timeout)
            self.get_logger().info(
                f'수령 대기 시작: order_id={order_id}, gate={gate}, '
                f'{storage} → {timeout:.0f}s 후 미수령 판정')
        except Exception as e:
            self._sub_db.rollback()
            self.get_logger().error(f'미수령 타이머 등록 실패 (order_id={order_id}): {e}')

    def _on_order_cancelled(self, order_id):
        """LISTEN order_cancelled (서브 스레드) → 고객 능동 취소를 로봇 계층으로 전파.

        order는 서버가 이미 cancelled(user)로 만들어둠. 여기선 outbound task 상태로 분기:
          - pending  : 아직 로봇 미배정 → task 취소 + 선반 예약만 해제. 로봇 신호 불필요.
          - assigned : 로봇 수행 중 → /cancel_mission 신호 + cancel_requested_at 스탬프.
                       DB 최종 처리는 로봇 보고를 받는 fleet이 함(로봇이 어느 상태서 멈출지
                       여기선 모름 — idle/busy/error 3종만 밖에서 보임).
        서브 스레드 전용 _sub_db 사용.
        """
        cur = self._sub_db.cursor()
        try:
            now = datetime.now(timezone.utc)
            cur.execute(
                """
                SELECT id, status, robot_id, source_location_id FROM tasks
                WHERE order_id = %s AND type = 'outbound'
                ORDER BY created_at DESC LIMIT 1
                """,
                (order_id,)
            )
            row = cur.fetchone()
            if not row:
                self.get_logger().warn(f'취소: order_id={order_id} outbound task 없음 — 무시')
                self._sub_db.rollback()
                return
            task_id, status, robot_id, shelf = row

            if status in ('done', 'cancelled'):
                self.get_logger().info(f'취소: order_id={order_id} task 이미 {status} — 무시')
                self._sub_db.rollback()
                return

            if status == 'pending':
                cur.execute(
                    "UPDATE tasks SET status = 'cancelled', completed_at = %s WHERE id = %s",
                    (now, task_id)
                )
                # 물건 안 움직였으므로 선반 예약(reserved_by)만 해제 — product_name/inbound_at 유지
                cur.execute(
                    'UPDATE locations SET reserved_by = NULL WHERE location_id = %s',
                    (shelf,)
                )
                cur.execute(
                    'INSERT INTO event_logs (task_id, event, occurred_at) VALUES (%s, %s, %s)',
                    (task_id, 'cancelled', now)
                )
                # 선반 예약이 풀려 available 재고가 늘었다 → 고객 UI 실시간 갱신
                cur.execute(
                    'SELECT pg_notify(%s, %s)',
                    ('location_updated', json.dumps({'location_id': shelf}))
                )
                self._sub_db.commit()
                self.get_logger().info(
                    f'취소(pending): order_id={order_id} task_id={task_id} cancelled, 선반 {shelf} 예약 해제')
                return

            if status == 'assigned':
                # 반납 중 로봇 사망 대비 흔적 남김(죽어있던 cancel_requested_at 컬럼 활용)
                cur.execute(
                    'UPDATE tasks SET cancel_requested_at = %s WHERE id = %s',
                    (now, task_id)
                )
                self._sub_db.commit()
                if robot_id:
                    self._cancel_pub.publish(String(data=json.dumps(
                        {'robot_id': robot_id, 'order_id': order_id})))
                    self.get_logger().info(
                        f'취소(assigned): order_id={order_id} → /cancel_mission {robot_id}')
                else:
                    self.get_logger().warn(
                        f'취소(assigned): order_id={order_id} robot_id 없음 — 신호 못 보냄')

        except Exception as e:
            self._sub_db.rollback()
            self.get_logger().error(f'취소 처리 실패 (order_id={order_id}): {e}')

    # ── 메인 스레드: 1Hz 미수령 검사 ──────────────────────────────────────────

    def _check_expired(self):
        """마감 지난 게이트 → 미수령 처리. 마감은 먼저 지워 중복 처리 방지."""
        now = time.monotonic()
        with self._deadlines_lock:
            expired = [(gate, oid) for gate, (oid, due) in self._deadlines.items()
                       if now >= due]
            for gate, _ in expired:
                del self._deadlines[gate]
        for gate, order_id in expired:
            self.get_logger().warn(f'미수령 시한 경과: order_id={order_id}, gate={gate}')
            self._cancel_and_reclaim(gate)

    # ── 메인 스레드: ROS2 콜백 ────────────────────────────────────────────────

    def _on_inbound(self, msg: String):
        """입고 감지 콜백.

        수신 형식: {"product_name": "사과"}
        카메라는 상품명만 감지. 어느 슬롯에 넣을지는 Task Manager가 결정.
        """
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f'inbound 페이로드 파싱 실패: {e}')
            return
        self._create_inbound_task(data['product_name'])

    def _on_cleared(self, msg: String):
        """게이트 비움 콜백 — {"slot":"OUT-1"}.

        그 게이트에 awaiting_pickup 주문이 있으면 = 고객이 가져간 것 → delivered.
        없으면 = 미수령 후 reclaim 로봇이 집어간 것(주문은 이미 cancelled)이거나
        오감지 → _mark_done_and_delivered가 경고 후 무시한다. DB 상태가 방어막이라
        감지 노드 쪽에 COOLDOWN 같은 장치가 필요 없다.
        """
        slot = self._parse_slot(msg)
        if not slot:
            self.get_logger().warn('slot 없는 cleared 신호 — 무시 (대상 게이트 특정 불가)')
            return
        # 수령됐으니 미수령 마감 취소. 마감이 없어도(이미 만료·다른 게이트) 무해.
        with self._deadlines_lock:
            self._deadlines.pop(slot, None)
        self._mark_done_and_delivered(slot)

    @staticmethod
    def _parse_slot(msg: String):
        """감지 페이로드에서 출고 게이트(slot) 추출. 없거나 파싱 실패 시 None."""
        try:
            return json.loads(msg.data).get('slot')
        except (ValueError, AttributeError):
            return None

    # ── 상태 업데이트 ─────────────────────────────────────────────────────────

    def _mark_done_and_delivered(self, slot):
        """수령 확인 → outbound task done + order delivered + 슬롯 해방.

        slot(OUT-1/OUT-2)의 awaiting_pickup 주문만 처리 (2공간 구분).
        해당 주문 없으면 = reclaim 회수 or 오감지 → 경고 후 종료.
        """
        cur = self._main_db.cursor()
        try:
            now = datetime.now(timezone.utc)

            # 해당 게이트의 awaiting_pickup 주문 조회
            cur.execute(
                """
                SELECT o.id FROM orders o
                JOIN tasks t ON t.order_id = o.id AND t.type = 'outbound'
                WHERE o.status = 'awaiting_pickup' AND t.target_location_id = %s
                ORDER BY t.created_at DESC LIMIT 1
                """,
                (slot,)
            )
            order_row = cur.fetchone()
            if not order_row:
                self.get_logger().warn(
                    f'awaiting_pickup 주문 없음 (slot={slot}) — 오감지 또는 이미 처리됨')
                self._main_db.rollback()
                return
            order_id = order_row[0]

            # outbound task 조회
            cur.execute(
                """
                SELECT id, source_location_id FROM tasks
                WHERE order_id = %s AND type = 'outbound'
                ORDER BY created_at DESC LIMIT 1
                """,
                (order_id,)
            )
            row = cur.fetchone()
            if not row:
                self.get_logger().error(f'order_id={order_id}의 outbound task 없음')
                self._main_db.rollback()
                return
            task_id, shelf = row       # shelf=원래 선반(source). slot은 게이트(target)

            # outbound task = done
            cur.execute(
                "UPDATE tasks SET status = 'done', completed_at = %s WHERE id = %s",
                (now, task_id)
            )

            # order = delivered
            cur.execute(
                "UPDATE orders SET status = 'delivered' WHERE id = %s",
                (order_id,)
            )
            # FastAPI WebSocket broadcast 트리거 (고객 UI 실시간 갱신)
            cur.execute(
                'SELECT pg_notify(%s, %s)',
                ('order_status_updated', json.dumps({'order_id': order_id, 'status': 'delivered'}))
            )

            # 선반 완전 해방 — 고객이 상품을 가져간 것이 확인된 시점에만 해제
            cur.execute(
                "UPDATE locations SET reserved_by = NULL, product_name = NULL, inbound_at = NULL WHERE location_id = %s",
                (shelf,)
            )
            # FastAPI WebSocket broadcast 트리거 (고객 UI 재고 실시간 갱신)
            cur.execute(
                'SELECT pg_notify(%s, %s)',
                ('location_updated', json.dumps({'location_id': shelf}))
            )

            cur.execute(
                'INSERT INTO event_logs (task_id, event, occurred_at) VALUES (%s, %s, %s)',
                (task_id, 'done', now)
            )

            self._main_db.commit()
            self.get_logger().info(
                f'수령 완료: order_id={order_id}, task_id={task_id}, '
                f'gate={slot}, shelf={shelf} 해방'
            )

        except Exception as e:
            self._main_db.rollback()
            self.get_logger().error(f'수령 처리 실패: {e}')

    def _create_reclaim(self, cur, order_id, now):
        """게이트에 놓인 상품을 원래 선반으로 되돌리는 reclaim task 생성 (공용 헬퍼).

        미수령(no_pickup)과 고객취소(user·PLACE 자연수렴)가 공유한다. order 상태·
        cancel_reason은 여기서 안 건드린다(호출자 책임 — 미수령은 no_pickup 설정, 고객취소는
        서버가 이미 user로 해둠). 호출자의 트랜잭션 안에서 실행, commit 안 함.

        reclaim source = outbound의 target(물건이 놓인 게이트), target = outbound의 source(선반).
        슬롯 잠금은 outbound → reclaim로 교체(슬롯 해방하지 않고 reclaim 완료까지 유지).
        반환: reclaim_task_id (outbound task 없으면 None).
        """
        cur.execute(
            """
            SELECT id, source_location_id, product_name, target_location_id FROM tasks
            WHERE order_id = %s AND type = 'outbound'
            ORDER BY created_at DESC LIMIT 1
            """,
            (order_id,)
        )
        row = cur.fetchone()
        if not row:
            self.get_logger().error(f'order_id={order_id}의 outbound task 없음')
            return None
        outbound_task_id, shelf, product_name, gate = row

        cur.execute(
            "UPDATE tasks SET status = 'cancelled', completed_at = %s WHERE id = %s",
            (now, outbound_task_id)
        )
        cur.execute(
            """
            INSERT INTO tasks
                (type, status, product_name, order_id,
                 source_location_id, target_location_id, created_at)
            VALUES (%s, %s, %s, %s, %s, %s, %s)
            RETURNING id
            """,
            ('reclaim', 'pending', product_name, order_id, gate, shelf, now)
        )
        reclaim_task_id = cur.fetchone()[0]
        cur.execute(
            'UPDATE locations SET reserved_by = %s WHERE location_id = %s',
            (reclaim_task_id, shelf)
        )
        cur.execute(
            'INSERT INTO event_logs (task_id, event, occurred_at) VALUES (%s, %s, %s)',
            (outbound_task_id, 'cancelled', now)
        )
        cur.execute(
            'INSERT INTO event_logs (task_id, event, occurred_at) VALUES (%s, %s, %s)',
            (reclaim_task_id, 'created', now)
        )
        return reclaim_task_id

    def _cancel_and_reclaim(self, slot):
        """미수령 → order cancelled(no_pickup) + reclaim task 생성 + 슬롯 잠금 유지.

        slot(OUT-1/OUT-2)의 awaiting_pickup 주문만 처리 (2공간 구분).
        해당 주문 없으면 = 이미 수령됐거나 처리됨 → 경고 후 종료.
        """
        cur = self._main_db.cursor()
        try:
            now = datetime.now(timezone.utc)

            # 해당 게이트의 awaiting_pickup 주문 조회
            cur.execute(
                """
                SELECT o.id FROM orders o
                JOIN tasks t ON t.order_id = o.id AND t.type = 'outbound'
                WHERE o.status = 'awaiting_pickup' AND t.target_location_id = %s
                ORDER BY t.created_at DESC LIMIT 1
                """,
                (slot,)
            )
            order_row = cur.fetchone()
            if not order_row:
                self.get_logger().warn(
                    f'awaiting_pickup 주문 없음 (slot={slot}) — 오감지 또는 이미 처리됨')
                self._main_db.rollback()
                return
            order_id = order_row[0]

            # order = cancelled (no_pickup) — 취소 사유 설정은 미수령 경로 몫
            cur.execute(
                """
                UPDATE orders SET status = 'cancelled', cancel_reason = 'no_pickup'
                WHERE id = %s
                """,
                (order_id,)
            )
            cur.execute(
                'SELECT pg_notify(%s, %s)',
                ('order_status_updated', json.dumps({
                    'order_id': order_id,
                    'status': 'cancelled',
                    'cancel_reason': 'no_pickup',
                }))
            )

            reclaim_task_id = self._create_reclaim(cur, order_id, now)
            if reclaim_task_id is None:
                self._main_db.rollback()
                return
            self._main_db.commit()

            self.get_logger().info(
                f'미수령: order_id={order_id} (slot={slot}) → reclaim task_id={reclaim_task_id} 생성, 슬롯 잠금 유지')
            self._pub.publish(
                String(data=json.dumps({'task_id': reclaim_task_id, 'task_type': 'reclaim'}))
            )

        except Exception as e:
            self._main_db.rollback()
            self.get_logger().error(f'미수령 처리 실패: {e}')

    def _on_reclaim_request(self, msg: String):
        """fleet이 PLACE 중 고객취소를 감지 → 게이트에 놓인 상품을 선반으로 되돌릴 reclaim 요청.

        order는 서버가 이미 cancelled(user)로 해둠 → order 상태·cancel_reason 안 건드림.
        메인 스레드 콜백이므로 _main_db 사용.
        """
        try:
            order_id = json.loads(msg.data)['order_id']
        except Exception as e:
            self.get_logger().error(f'reclaim_request 파싱 실패: {e}')
            return
        cur = self._main_db.cursor()
        try:
            now = datetime.now(timezone.utc)
            reclaim_task_id = self._create_reclaim(cur, order_id, now)
            if reclaim_task_id is None:
                self._main_db.rollback()
                return
            self._main_db.commit()
            self.get_logger().info(
                f'취소(PLACE) 회수: order_id={order_id} → reclaim task_id={reclaim_task_id}')
            self._pub.publish(
                String(data=json.dumps({'task_id': reclaim_task_id, 'task_type': 'reclaim'}))
            )
        except Exception as e:
            self._main_db.rollback()
            self.get_logger().error(f'reclaim_request 처리 실패 (order_id={order_id}): {e}')

    # ── task INSERT 함수들 ─────────────────────────────────────────────────────

    def _create_outbound_task(self, order_id: int):
        """주문 접수(NOTIFY new_order) → outbound task INSERT.

        슬롯 선택: 제품명 + 저장타입 일치 + inbound_at IS NOT NULL + reserved_by IS NULL
        FIFO: inbound_at 오름차순으로 가장 먼저 입고된 슬롯 우선.
        서브 스레드에서 호출되므로 _sub_db 사용.
        """
        cur = self._sub_db.cursor()
        try:
            # 주문에서 상품명 조회
            cur.execute('SELECT product_name FROM orders WHERE id = %s', (order_id,))
            row = cur.fetchone()
            if not row:
                self.get_logger().error(f'order_id={order_id} 없음')
                self._sub_db.rollback()
                return
            product_name = row[0]

            # FIFO: 해당 상품이 있고 예약 안 된 슬롯 중 가장 먼저 입고된 것
            cur.execute(
                """
                SELECT location_id FROM locations
                WHERE product_name = %s
                  AND inbound_at IS NOT NULL
                  AND reserved_by IS NULL
                ORDER BY inbound_at ASC
                LIMIT 1
                """,
                (product_name,)
            )
            slot = cur.fetchone()
            if not slot:
                self.get_logger().warn(f'{product_name} 재고 없음 — outbound task 생성 불가')
                self._sub_db.rollback()
                return
            source_loc = slot[0]

            # 빈 출고 게이트 선택 (OUT-1/OUT-2 중 현재 점유 안 된 것).
            # 점유 = 진행 중 outbound가 target하거나, 진행 중 reclaim이 source로 쓰는 게이트.
            cur.execute(
                """
                SELECT g FROM (VALUES ('OUT-1'), ('OUT-2')) AS gates(g)
                WHERE g NOT IN (
                    SELECT t.target_location_id FROM tasks t
                    JOIN orders o ON o.id = t.order_id
                    WHERE t.type = 'outbound'
                      AND o.status IN ('processing', 'awaiting_pickup')
                    UNION
                    SELECT t.source_location_id FROM tasks t
                    WHERE t.type = 'reclaim' AND t.status IN ('pending', 'assigned')
                )
                ORDER BY g ASC
                LIMIT 1
                """
            )
            gate_row = cur.fetchone()
            if not gate_row:
                self.get_logger().warn(
                    '빈 출고 게이트 없음 (OUT-1·OUT-2 모두 점유) — outbound task 보류')
                self._sub_db.rollback()
                return
            out_gate = gate_row[0]

            now = datetime.now(timezone.utc)

            # outbound task INSERT (source: 슬롯, target: 선택된 출고 게이트)
            cur.execute(
                """
                INSERT INTO tasks
                    (type, status, product_name, order_id,
                     source_location_id, target_location_id, created_at)
                VALUES (%s, %s, %s, %s, %s, %s, %s)
                RETURNING id
                """,
                ('outbound', 'pending', product_name, order_id, source_loc, out_gate, now)
            )
            task_id = cur.fetchone()[0]

            # 주문 상태 → processing (임무 생성 완료)
            cur.execute(
                "UPDATE orders SET status = 'processing' WHERE id = %s",
                (order_id,)
            )
            # FastAPI WebSocket broadcast 트리거 (고객 UI 실시간 갱신)
            cur.execute(
                'SELECT pg_notify(%s, %s)',
                ('order_status_updated', json.dumps({'order_id': order_id, 'status': 'processing'}))
            )

            # 슬롯 예약 — pickup 확인 전까지 잠금 유지
            cur.execute(
                'UPDATE locations SET reserved_by = %s WHERE location_id = %s',
                (task_id, source_loc)
            )

            cur.execute(
                'INSERT INTO event_logs (task_id, event, occurred_at) VALUES (%s, %s, %s)',
                (task_id, 'created', now)
            )

            self._sub_db.commit()

            self.get_logger().info(
                f'outbound task 생성: task_id={task_id}, order_id={order_id}, '
                f'slot={source_loc} → gate={out_gate}'
            )
            self._pub.publish(
                String(data=json.dumps({'task_id': task_id, 'task_type': 'outbound'}))
            )

        except Exception as e:
            self._sub_db.rollback()
            self.get_logger().error(f'outbound task 생성 실패: {e}')

    def _create_inbound_task(self, product_name: str):
        """입고 감지 → inbound task INSERT.

        카메라는 상품명만 감지. Task Manager가 products 조회로 storage_type 확인 후
        같은 storage_type의 빈 슬롯(inbound_at IS NULL, reserved_by IS NULL)을 선택.
        source: IN-1 (입고 게이트), target: Task Manager가 선택한 빈 슬롯.
        메인 스레드에서 호출되므로 _main_db 사용.
        """
        cur = self._main_db.cursor()
        try:
            # products에서 storage_type 조회
            cur.execute(
                'SELECT storage_type FROM products WHERE name = %s',
                (product_name,)
            )
            row = cur.fetchone()
            if not row:
                self.get_logger().error(f'상품 없음: {product_name}')
                self._main_db.rollback()
                return
            storage_type = row[0]

            # 같은 storage_type의 빈 슬롯 선택 (inbound_at IS NULL = 비어있는 슬롯)
            cur.execute(
                """
                SELECT location_id FROM locations
                WHERE storage_type = %s
                  AND inbound_at IS NULL
                  AND reserved_by IS NULL
                ORDER BY location_id ASC
                LIMIT 1
                """,
                (storage_type,)
            )
            slot = cur.fetchone()
            if not slot:
                self.get_logger().warn(f'{storage_type} 창고 자리 없음 — inbound task 생성 불가')
                self._main_db.rollback()
                return
            target_loc = slot[0]

            now = datetime.now(timezone.utc)

            # inbound task INSERT (source: 입고 게이트, target: 선택된 빈 슬롯)
            cur.execute(
                """
                INSERT INTO tasks
                    (type, status, product_name,
                     source_location_id, target_location_id, created_at)
                VALUES (%s, %s, %s, %s, %s, %s)
                RETURNING id
                """,
                ('inbound', 'pending', product_name, 'IN-1', target_loc, now)
            )
            task_id = cur.fetchone()[0]

            # 슬롯 예약 — 로봇이 상품을 넣을 때까지 다른 임무가 이 슬롯을 못 씀
            cur.execute(
                'UPDATE locations SET reserved_by = %s WHERE location_id = %s',
                (task_id, target_loc)
            )

            cur.execute(
                'INSERT INTO event_logs (task_id, event, occurred_at) VALUES (%s, %s, %s)',
                (task_id, 'created', now)
            )

            self._main_db.commit()

            self.get_logger().info(
                f'inbound task 생성: task_id={task_id}, product={product_name}({storage_type}), target={target_loc}'
            )
            self._pub.publish(
                String(data=json.dumps({'task_id': task_id, 'task_type': 'inbound'}))
            )

        except Exception as e:
            self._main_db.rollback()
            self.get_logger().error(f'inbound task 생성 실패: {e}')

    def destroy_node(self):
        self._listen_conn.close()
        self._sub_db.close()
        self._main_db.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
