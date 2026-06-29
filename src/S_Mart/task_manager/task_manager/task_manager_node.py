#!/usr/bin/env python3
"""Task Manager ROS2 노드 — 임무 생성 전담 (할당은 Fleet Manager가 담당)

트리거 4종:
  1. PostgreSQL LISTEN new_order    → outbound task 생성          (서브 스레드)
  2. /detection/inbound  구독       → inbound task 생성           (메인 스레드 콜백)
  3. /detection/pickup   구독       → outbound task done, delivered (메인 스레드 콜백)
  4. /detection/no_pickup 구독      → outbound task cancelled + reclaim task 생성 (메인 스레드 콜백)

생성된 task는 /new_task 토픽으로 Fleet Manager에 신호를 보냄.

연결 구조 (스레드 간 psycopg2 커넥션 공유 불가 → 스레드별 전용 커넥션):
  - _listen_conn (AUTOCOMMIT): 서브 스레드 전용, LISTEN+poll만 사용
  - _sub_db     (기본):        서브 스레드 전용, outbound task INSERT 트랜잭션
  - _main_db    (기본):        메인 스레드 전용, 나머지 모든 DB 작업

reserved_by 해제 시점:
  - /detection/pickup 수신 시 → 고객이 실제로 상품을 가져간 것이 확인된 시점에만 해제
  - no_pickup 시 → outbound task_id에서 reclaim task_id로 교체 (슬롯 잠금 유지)
  - 이렇게 해야 no_pickup 시 원래 슬롯이 다른 임무에 뺏기지 않음
"""

import json
import select
import threading
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

        # Fleet Manager로 새 task 알림
        self._pub = self.create_publisher(String, '/new_task', 10)

        # 입고 감지 — 카메라가 IN-1 게이트에서 상품명만 감지하여 발행
        # {"product_name": "사과"} — 슬롯은 Task Manager가 선택
        self.create_subscription(String, '/detection/inbound', self._on_inbound, 10)

        # 수령 감지 — 1분 이내 게이트에서 물건이 사라짐 (고객 수령), 페이로드 없음
        self.create_subscription(String, '/detection/pickup', self._on_pickup, 10)

        # 미수령 감지 — 1분 지나도 게이트에 물건이 그대로, 페이로드 없음
        self.create_subscription(String, '/detection/no_pickup', self._on_no_pickup, 10)

        # LISTEN 서브 스레드 시작 (daemon=True: 메인 스레드 종료 시 자동 종료)
        threading.Thread(target=self._listen_orders, daemon=True).start()

        self.get_logger().info('Task Manager 시작')

    # ── 서브 스레드: LISTEN new_order ─────────────────────────────────────────

    def _listen_orders(self):
        """PostgreSQL NOTIFY new_order 대기 → outbound task 생성.

        rclpy.spin()이 메인 스레드를 점유하기 때문에 서브 스레드에서 실행.
        NOTIFY가 올 때까지 select()로 블로킹 대기하며 메인 스레드와 간섭 없음.
        """
        cur = self._listen_conn.cursor()
        cur.execute('LISTEN new_order')
        self.get_logger().info('LISTEN new_order 등록 완료')

        while rclpy.ok():
            # 5초마다 종료 신호 체크 (rclpy.ok()가 False가 되면 루프 탈출)
            if select.select([self._listen_conn], [], [], 5.0)[0]:
                self._listen_conn.poll()
                for notify in self._listen_conn.notifies:
                    try:
                        payload = json.loads(notify.payload)
                        self._create_outbound_task(payload['order_id'])
                    except Exception as e:
                        self.get_logger().error(f'outbound task 생성 실패: {e}')
                self._listen_conn.notifies.clear()

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

    def _on_pickup(self, msg: String):
        """수령 감지 콜백 — 1분 이내 게이트에서 물건이 사라짐.

        AI Server가 페이로드 없이 발행. awaiting_pickup 주문을 DB에서 직접 조회.
        outbound task = done, order = delivered, reserved_by = NULL (슬롯 완전 해방).
        """
        self._mark_done_and_delivered()

    def _on_no_pickup(self, msg: String):
        """미수령 감지 콜백 — 1분 지나도 게이트에 물건이 그대로.

        AI Server가 페이로드 없이 발행. awaiting_pickup 주문을 DB에서 직접 조회.
        outbound task = cancelled, order = cancelled(no_pickup).
        reclaim task 생성 후 reserved_by를 reclaim_task_id로 교체 (슬롯 잠금 유지).
        """
        self._cancel_and_reclaim()

    # ── 상태 업데이트 ─────────────────────────────────────────────────────────

    def _mark_done_and_delivered(self):
        """수령 확인 → outbound task done + order delivered + 슬롯 해방.

        OUT-1이 하나이므로 awaiting_pickup 주문은 항상 유일.
        오감지 시 awaiting_pickup 주문 없음 → 경고 후 종료.
        """
        cur = self._main_db.cursor()
        try:
            now = datetime.now(timezone.utc)

            # awaiting_pickup 주문 조회 (OUT-1 하나 → 항상 유일)
            cur.execute(
                "SELECT id FROM orders WHERE status = 'awaiting_pickup' LIMIT 1"
            )
            order_row = cur.fetchone()
            if not order_row:
                self.get_logger().warn('awaiting_pickup 주문 없음 — 오감지 또는 이미 처리됨')
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
            task_id, slot = row

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

            # 슬롯 완전 해방 — 고객이 상품을 가져간 것이 확인된 시점에만 해제
            cur.execute(
                "UPDATE locations SET reserved_by = NULL, product_name = NULL, inbound_at = NULL WHERE location_id = %s",
                (slot,)
            )
            # FastAPI WebSocket broadcast 트리거 (고객 UI 재고 실시간 갱신)
            cur.execute(
                'SELECT pg_notify(%s, %s)',
                ('location_updated', json.dumps({'location_id': slot}))
            )

            cur.execute(
                'INSERT INTO event_logs (task_id, event, occurred_at) VALUES (%s, %s, %s)',
                (task_id, 'done', now)
            )

            self._main_db.commit()
            self.get_logger().info(
                f'수령 완료: order_id={order_id}, task_id={task_id}, slot={slot} 해방'
            )

        except Exception as e:
            self._main_db.rollback()
            self.get_logger().error(f'수령 처리 실패: {e}')

    def _cancel_and_reclaim(self):
        """미수령 → outbound task cancelled + reclaim task 생성 + 슬롯 잠금 유지.

        OUT-1이 하나이므로 awaiting_pickup 주문은 항상 유일.
        오감지 시 awaiting_pickup 주문 없음 → 경고 후 종료.
        """
        cur = self._main_db.cursor()
        try:
            now = datetime.now(timezone.utc)

            # awaiting_pickup 주문 조회 (OUT-1 하나 → 항상 유일)
            cur.execute(
                "SELECT id FROM orders WHERE status = 'awaiting_pickup' LIMIT 1"
            )
            order_row = cur.fetchone()
            if not order_row:
                self.get_logger().warn('awaiting_pickup 주문 없음 — 오감지 또는 이미 처리됨')
                self._main_db.rollback()
                return
            order_id = order_row[0]

            # outbound task 조회
            cur.execute(
                """
                SELECT id, source_location_id, product_name FROM tasks
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
            outbound_task_id, original_slot, product_name = row

            # outbound task = cancelled (임무 실패 — 고객 미수령)
            cur.execute(
                "UPDATE tasks SET status = 'cancelled', completed_at = %s WHERE id = %s",
                (now, outbound_task_id)
            )

            # order = cancelled (no_pickup)
            cur.execute(
                """
                UPDATE orders SET status = 'cancelled', cancel_reason = 'no_pickup'
                WHERE id = %s
                """,
                (order_id,)
            )
            # FastAPI WebSocket broadcast 트리거 (고객 UI 실시간 갱신)
            cur.execute(
                'SELECT pg_notify(%s, %s)',
                ('order_status_updated', json.dumps({
                    'order_id': order_id,
                    'status': 'cancelled',
                    'cancel_reason': 'no_pickup',
                }))
            )

            # reclaim task 생성 (source: OUT-1 게이트, target: 원래 슬롯)
            cur.execute(
                """
                INSERT INTO tasks
                    (type, status, product_name, order_id,
                     source_location_id, target_location_id, created_at)
                VALUES (%s, %s, %s, %s, %s, %s, %s)
                RETURNING id
                """,
                ('reclaim', 'pending', product_name, order_id, 'OUT-1', original_slot, now)
            )
            reclaim_task_id = cur.fetchone()[0]

            # 슬롯 잠금을 outbound_task_id → reclaim_task_id로 교체
            # (슬롯을 해방하지 않고 reclaim 임무가 완료될 때까지 잠금 유지)
            cur.execute(
                'UPDATE locations SET reserved_by = %s WHERE location_id = %s',
                (reclaim_task_id, original_slot)
            )

            cur.execute(
                'INSERT INTO event_logs (task_id, event, occurred_at) VALUES (%s, %s, %s)',
                (outbound_task_id, 'cancelled', now)
            )
            cur.execute(
                'INSERT INTO event_logs (task_id, event, occurred_at) VALUES (%s, %s, %s)',
                (reclaim_task_id, 'created', now)
            )

            self._main_db.commit()

            self.get_logger().info(
                f'미수령: order_id={order_id}, outbound task_id={outbound_task_id} cancelled, '
                f'reclaim task_id={reclaim_task_id}, slot={original_slot} 잠금 유지'
            )
            self._pub.publish(
                String(data=json.dumps({'task_id': reclaim_task_id, 'task_type': 'reclaim'}))
            )

        except Exception as e:
            self._main_db.rollback()
            self.get_logger().error(f'미수령 처리 실패: {e}')

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

            now = datetime.now(timezone.utc)

            # outbound task INSERT (source: 슬롯, target: 출고 게이트)
            cur.execute(
                """
                INSERT INTO tasks
                    (type, status, product_name, order_id,
                     source_location_id, target_location_id, created_at)
                VALUES (%s, %s, %s, %s, %s, %s, %s)
                RETURNING id
                """,
                ('outbound', 'pending', product_name, order_id, source_loc, 'OUT-1', now)
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
                f'outbound task 생성: task_id={task_id}, order_id={order_id}, slot={source_loc}'
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
            raise

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
