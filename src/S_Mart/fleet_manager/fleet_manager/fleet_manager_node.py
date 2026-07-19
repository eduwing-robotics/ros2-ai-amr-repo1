#!/usr/bin/env python3
"""Fleet Manager ROS2 노드 — 로봇 배정 + DB 상태 전환 전담

역할:
  - /new_task 수신 → 배정 가능 로봇 선택(LRU) → /assignment 발행 → DB 갱신
  - /task_report 수신 → task 타입별 DB 상태 전환
  - 각 로봇의 /robot_status, /battery_state 구독 → 배정 가능 여부 판단

로봇 선택 전략 (LRU):
  조건: robot_status == 'idle' AND battery >= BATTERY_MIN(30%)
  우선순위: _last_used 가장 오래된(타임스탬프 작은) 로봇
  → 두 로봇이 모두 가용할 때 균등 배분, 한 대만 가용하면 그 로봇 즉시 배정

스레드:
  단일 스레드 (rclpy.spin()). 모든 콜백이 ms 수준 빠른 DB 작업이므로
  멀티스레드 불필요. 공유 상태(딕셔너리, DB 커넥션)에 락 없이 안전.
"""

import json
import time
from datetime import datetime, timezone

import psycopg2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

_DSN = 'dbname=s_mart user=codelab password=codelab host=localhost'

# 배터리 30% 미만이면 배정 제외
BATTERY_MIN = 0.3

# 관리 로봇 목록 (ROS2 토픽명 규칙: 영문자·숫자·'_'만 허용 → 하이픈 대신 언더스코어)
ROBOT_IDS = ['AMR_1', 'AMR_2']


class FleetManagerNode(Node):
    def __init__(self):
        super().__init__('fleet_manager')

        # ── 구독: 임무 흐름 ──────────────────────────────────────────────────
        self.create_subscription(String, '/new_task',    self._on_new_task,    10)
        self.create_subscription(String, '/task_report', self._on_task_report, 10)

        # ── 구독: 로봇 상태 (각 로봇이 자신의 상태를 직접 발행) ──────────────
        # lambda 기본 인자(r=robot_id)로 루프 변수 캡처 — 클로저 버그 방지
        for robot_id in ROBOT_IDS:
            self.create_subscription(
                String,
                f'/{robot_id}/robot_status',
                lambda msg, r=robot_id: self._on_robot_status(r, msg),
                10,
            )
            self.create_subscription(
                BatteryState,
                f'/{robot_id}/battery_state',
                lambda msg, r=robot_id: self._on_battery_state(r, msg),
                10,
            )

        # ── 발행: Path Plan Manager로 배정 지시 ─────────────────────────────
        self._pub_assignment = self.create_publisher(String, '/assignment', 10)

        # ── 발행: PLACE 중 고객취소 감지 시 게이트 회수 위임 (task_manager가 reclaim 생성) ──
        self._pub_reclaim = self.create_publisher(String, '/reclaim_request', 10)

        # ── DB 커넥션 (메인 스레드 전용, LISTEN 없으므로 1개로 충분) ──────────
        self._db = psycopg2.connect(_DSN)

        # ── 로봇 상태 딕셔너리 (로봇 토픽 수신값 저장, FM이 직접 변경 X) ──────
        # 초기값 idle: 기동 직후 _check_pending이 pending task를 시도할 수 있도록
        self._robot_status   = {r: 'idle' for r in ROBOT_IDS}  # idle/busy/returning/error
        self._battery_status = {r: 1.0    for r in ROBOT_IDS}  # 0.0~1.0 (기본 100%)

        # ── LRU: 마지막 임무 완료 시각 (Unix timestamp, 0 = 한 번도 안 씀) ──
        # 값이 작을수록 더 오래 쉰 로봇 → 배정 우선순위 높음
        self._last_used = {r: 0.0 for r in ROBOT_IDS}

        self.get_logger().info('Fleet Manager 시작')

        # ── 기동 시 복구: 재시작 전 남아있던 pending task 즉시 처리 ──────────
        self._check_pending()

    # ── 콜백: 로봇 상태 수신 ─────────────────────────────────────────────────

    def _on_robot_status(self, robot_id: str, msg: String):
        """로봇이 /{robot_id}/robot_status 에 자신의 상태를 발행할 때 호출.

        수신값을 _robot_status 에 저장. idle이 되면 대기 임무 확인.
        FM이 직접 상태를 변경하지 않고 로봇 발행값만 반영하는 것이 설계 원칙.
        """
        status = msg.data
        self._robot_status[robot_id] = status
        self.get_logger().debug(f'{robot_id} 상태 수신: {status}')

        if status == 'idle':
            # 홈 복귀 완료 → 대기 중인 임무 확인 후 즉시 배정
            self._check_pending()
        elif status == 'error':
            # 배정 제외는 자동 (idle 아니므로). 로그만 남김
            # 해당 로봇의 assigned task 복구는 2차 이후 구현
            self.get_logger().warn(f'{robot_id} 오류 상태 — 자동 배정 제외')

    def _on_battery_state(self, robot_id: str, msg: BatteryState):
        """로봇이 /{robot_id}/battery_state 에 배터리 상태를 발행할 때 호출.

        msg.percentage: 0.0(0%) ~ 1.0(100%).
        저장만 하고 직접 배정을 트리거하지 않음 — _try_assign 호출 시 참조.
        """
        self._battery_status[robot_id] = msg.percentage
        if msg.percentage < BATTERY_MIN:
            self.get_logger().warn(
                f'{robot_id} 배터리 부족: {msg.percentage * 100:.0f}%'
                f' (최소 {BATTERY_MIN * 100:.0f}%)'
            )

    # ── 콜백: 임무 흐름 ──────────────────────────────────────────────────────

    def _on_new_task(self, msg: String):
        """Task Manager가 /new_task 에 새 임무를 발행할 때 호출.

        수신 형식: {"task_id": 5, "task_type": "outbound"}
        task_id 파싱 후 _try_assign에 위임.
        """
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f'new_task 페이로드 파싱 실패: {e}')
            return
        self._try_assign(data['task_id'])

    def _on_task_report(self, msg: String):
        """Path Plan Manager가 /task_report 에 주행 결과를 발행할 때 호출.

        수신 형식: {"robot_id": "AMR_1", "event": "source_arrived" | "target_done"}
        event 종류에 따라 처리 메서드를 분기.
        """
        try:
            data     = json.loads(msg.data)
            robot_id = data['robot_id']
            event    = data['event']
        except Exception as e:
            self.get_logger().error(f'task_report 페이로드 파싱 실패: {e}')
            return

        if event == 'source_arrived':
            self._handle_source_arrived(robot_id)
        elif event == 'target_done':
            self._handle_target_done(robot_id)
        elif event in ('cancel_aborted', 'cancel_returned'):
            # 고객취소 보고 — order_id로 대상 task 특정(로봇에 assigned가 여러 개일 수 있음)
            self._finish_customer_cancel(robot_id, data.get('order_id'), event)
        else:
            self.get_logger().warn(f'알 수 없는 task_report event: {event}')

    # ── 핵심 배정 로직 ────────────────────────────────────────────────────────

    def _try_assign(self, task_id: int):
        """pending task를 배정 가능 로봇에 할당.

        _on_new_task와 _check_pending 양쪽에서 호출되는 공유 메서드.
        로봇 선택 (LRU): idle + battery >= 30% 후보 중 last_used가 가장 오래된 로봇.
        가용 로봇 없으면 리턴 — task는 pending 유지, 로봇이 idle 발행 시 재시도.
        """
        cur = self._db.cursor()
        try:
            # task 상세 조회
            cur.execute(
                """
                SELECT type, source_location_id, target_location_id, product_name, order_id
                FROM tasks WHERE id = %s
                """,
                (task_id,)
            )
            row = cur.fetchone()
            if not row:
                self.get_logger().error(f'task_id={task_id} DB에 없음')
                self._db.rollback()
                return
            task_type, source, target, product_name, order_id = row

            # 배정 가능 후보: idle + 배터리 30% 이상
            candidates = [
                r for r in ROBOT_IDS
                if self._robot_status[r] == 'idle'
                and self._battery_status[r] >= BATTERY_MIN
            ]
            if not candidates:
                self.get_logger().info(
                    f'task_id={task_id} 배정 보류 — 가용 로봇 없음 (pending 유지)'
                )
                self._db.rollback()
                return

            # LRU: last_used 가장 작은(오래 쉰) 로봇 선택
            robot_id = min(candidates, key=lambda r: self._last_used[r])

            now = datetime.now(timezone.utc)

            # [트랜잭션] tasks assigned 업데이트 + event_logs 동시 커밋
            cur.execute(
                """
                UPDATE tasks
                SET status = 'assigned', robot_id = %s, assigned_at = %s
                WHERE id = %s
                """,
                (robot_id, now, task_id)
            )
            cur.execute(
                """
                INSERT INTO event_logs (task_id, event, robot_id, occurred_at)
                VALUES (%s, %s, %s, %s)
                """,
                (task_id, 'assigned', robot_id, now)
            )
            self._db.commit()

            # /assignment 발행 → Path Plan Manager가 로봇에 주행 지시
            payload = json.dumps({
                'robot_id': robot_id,
                'source':   source,
                'target':   target,
            })
            self._pub_assignment.publish(String(data=payload))

            self.get_logger().info(
                f'배정 완료: task_id={task_id}({task_type}) → {robot_id}'
                f' [{source} → {target}]'
            )

        except Exception as e:
            self._db.rollback()
            self.get_logger().error(f'_try_assign 실패 (task_id={task_id}): {e}')

    # ── task_report 처리 ──────────────────────────────────────────────────────

    def _handle_source_arrived(self, robot_id: str):
        """로봇이 소스 위치(슬롯/게이트)에 도착했을 때 처리.

        [트랜잭션] picked_at 기록 + event_logs 'picked' 동시 커밋.
        """
        cur = self._db.cursor()
        try:
            now = datetime.now(timezone.utc)

            # 해당 로봇의 assigned task 조회.
            # outbound는 target_done 후에도 assigned를 유지(pickup 확인까지)하므로 한 로봇에
            # assigned가 2개 공존할 수 있다 → picked_at IS NULL로 아직 안 집은 것만,
            # assigned_at DESC로 가장 최근 배정분을 고른다. ORDER BY 없는 LIMIT 1은
            # 어느 행이 나올지 정의되지 않아 이전 task의 picked_at을 덮어쓸 수 있다.
            cur.execute(
                """
                SELECT id FROM tasks
                WHERE robot_id = %s AND status = 'assigned' AND picked_at IS NULL
                ORDER BY assigned_at DESC
                LIMIT 1
                """,
                (robot_id,)
            )
            row = cur.fetchone()
            if not row:
                self.get_logger().warn(
                    f'{robot_id}: assigned task 없음 (source_arrived 무시)'
                )
                self._db.rollback()
                return
            task_id = row[0]

            # [트랜잭션] picked_at + event_logs 'picked' 동시 커밋
            cur.execute(
                'UPDATE tasks SET picked_at = %s WHERE id = %s',
                (now, task_id)
            )
            cur.execute(
                """
                INSERT INTO event_logs (task_id, event, robot_id, occurred_at)
                VALUES (%s, %s, %s, %s)
                """,
                (task_id, 'picked', robot_id, now)
            )
            self._db.commit()

            self.get_logger().info(f'{robot_id} 소스 도착: task_id={task_id}')

        except Exception as e:
            self._db.rollback()
            self.get_logger().error(f'source_arrived 처리 실패 ({robot_id}): {e}')

    def _handle_target_done(self, robot_id: str):
        """로봇이 타깃 위치에 상품을 내려놓고 완료 보고했을 때 처리.

        task 타입별 DB 처리:
          inbound/reclaim: task done + locations 채움 + event_logs 'done'
          outbound: orders awaiting_pickup + NOTIFY order_status_updated
                    (task는 assigned 유지 — Task Manager가 pickup/no_pickup 후 최종 처리)

        모든 타입에서 _last_used 갱신 (LRU 기준).
        """
        cur = self._db.cursor()
        try:
            now = datetime.now(timezone.utc)

            # 해당 로봇의 assigned task 상세 조회.
            # source_arrived와 같은 이유로 ORDER BY 필수 — 지금 막 내려놓은 건 가장 최근
            # 배정분이다. 이전 outbound(assigned + awaiting_pickup)를 잡으면 안 된다.
            cur.execute(
                """
                SELECT id, type, target_location_id, product_name, order_id
                FROM tasks
                WHERE robot_id = %s AND status = 'assigned'
                ORDER BY assigned_at DESC
                LIMIT 1
                """,
                (robot_id,)
            )
            row = cur.fetchone()
            if not row:
                self.get_logger().warn(
                    f'{robot_id}: assigned task 없음 (target_done 무시)'
                )
                self._db.rollback()
                return
            task_id, task_type, target_loc, product_name, order_id = row

            if task_type in ('inbound', 'reclaim'):
                # [트랜잭션] task done + locations 채움 + event_logs 동시 커밋
                cur.execute(
                    "UPDATE tasks SET status = 'done', completed_at = %s WHERE id = %s",
                    (now, task_id)
                )
                cur.execute(
                    """
                    UPDATE locations
                    SET product_name = %s, inbound_at = %s, reserved_by = NULL
                    WHERE location_id = %s
                    """,
                    (product_name, now, target_loc)
                )
                cur.execute(
                    """
                    INSERT INTO event_logs (task_id, event, robot_id, occurred_at)
                    VALUES (%s, %s, %s, %s)
                    """,
                    (task_id, 'done', robot_id, now)
                )
                # FastAPI가 LISTEN location_updated 수신 → WebSocket broadcast → 고객 재고 실시간 갱신
                cur.execute(
                    'SELECT pg_notify(%s, %s)',
                    ('location_updated', json.dumps({'location_id': target_loc}))
                )
                self._db.commit()
                self.get_logger().info(
                    f'{robot_id} {task_type} 완료: task_id={task_id},'
                    f' slot={target_loc} 채움'
                )

            elif task_type == 'outbound':
                # PLACE 완료 시점에 order가 이미 취소됐는지 확인(FOR UPDATE로 cancel_order와 직렬화).
                # PLACE는 원자구간이라 취소가 와도 못 끊고 게이트에 놓인다 → 여기서 자연수렴 판정.
                cur.execute("SELECT status FROM orders WHERE id = %s FOR UPDATE", (order_id,))
                ostatus_row = cur.fetchone()
                ostatus = ostatus_row[0] if ostatus_row else None

                if ostatus == 'cancelled':
                    # PLACE 중(또는 직전) 고객취소됨. 물건은 게이트에 놓임 → awaiting_pickup으로
                    # 바꾸지 않고(cancel_reason=user 보존) task_manager에 게이트 회수 reclaim 위임.
                    self._db.commit()   # FOR UPDATE 잠금 해제
                    self._pub_reclaim.publish(String(data=json.dumps({'order_id': order_id})))
                    self.get_logger().info(
                        f'{robot_id} outbound target_done인데 order_id={order_id} 취소됨'
                        f' → reclaim 위임(게이트 회수)')
                else:
                    # [트랜잭션] orders awaiting_pickup + NOTIFY 동시 커밋
                    # task는 assigned 유지 — Task Manager가 /detection/pickup|no_pickup 후 최종 처리
                    cur.execute(
                        "UPDATE orders SET status = 'awaiting_pickup' WHERE id = %s",
                        (order_id,)
                    )
                    # FastAPI가 LISTEN order_status_updated 수신 → WebSocket broadcast → 고객 UI 갱신
                    cur.execute(
                        'SELECT pg_notify(%s, %s)',
                        ('order_status_updated', json.dumps({
                            'order_id': order_id,
                            'status': 'awaiting_pickup',
                        }))
                    )
                    self._db.commit()
                    self.get_logger().info(
                        f'{robot_id} outbound 완료: task_id={task_id},'
                        f' order_id={order_id} → awaiting_pickup'
                    )

            # LRU 갱신: 임무 완료 시각 기록 (task 타입 무관)
            self._last_used[robot_id] = time.time()

        except Exception as e:
            self._db.rollback()
            self.get_logger().error(f'target_done 처리 실패 ({robot_id}): {e}')

    def _finish_customer_cancel(self, robot_id: str, order_id, kind: str):
        """고객취소 로봇 보고 처리 (방식1 자체반납).

          cancel_aborted  : TO_SOURCE(빈손) 중단 — 물건 안 움직임
          cancel_returned : TO_TARGET 자체반납 완료 — 로봇이 선반에 되돌려놓음

        두 경우 DB 효과는 동일: outbound task cancelled + 선반 예약(reserved_by)만 해제.
        outbound 동안 선반 product_name은 안 비워지므로(수령 확정 때만 비움) 재고는 원래
        그대로다 → product_name/inbound_at 손대지 않는다(inbound_at=now()는 재고 나이 오염).
        order는 서버가 cancelled(user)로 해둠 → 안 건드림.
        """
        if order_id is None:
            self.get_logger().error(f'{kind}: order_id 없음 ({robot_id}) — 대상 특정 불가')
            return
        cur = self._db.cursor()
        try:
            now = datetime.now(timezone.utc)
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
                self.get_logger().warn(f'{kind}: order_id={order_id} outbound task 없음 (무시)')
                self._db.rollback()
                return
            task_id, slot = row

            cur.execute(
                "UPDATE tasks SET status = 'cancelled', completed_at = %s WHERE id = %s",
                (now, task_id)
            )
            cur.execute(
                'UPDATE locations SET reserved_by = NULL WHERE location_id = %s',
                (slot,)
            )
            cur.execute(
                """
                INSERT INTO event_logs (task_id, event, robot_id, occurred_at)
                VALUES (%s, %s, %s, %s)
                """,
                (task_id, 'cancelled', robot_id, now)
            )
            # 선반 예약이 풀려 available 재고가 늘었다 → 고객 UI 실시간 갱신
            # (order_cancelled broadcast는 주문목록만 갱신, 재고는 location_updated로만 갱신됨)
            cur.execute(
                'SELECT pg_notify(%s, %s)',
                ('location_updated', json.dumps({'location_id': slot}))
            )
            self._db.commit()
            self._last_used[robot_id] = time.time()   # 로봇은 이제 자유 — LRU 갱신
            self.get_logger().info(
                f'{robot_id} {kind}: task_id={task_id}(order_id={order_id}) cancelled,'
                f' 선반 {slot} 예약 해제')

        except Exception as e:
            self._db.rollback()
            self.get_logger().error(f'{kind} 처리 실패 ({robot_id}, order_id={order_id}): {e}')

    # ── 대기 임무 확인 ────────────────────────────────────────────────────────

    def _check_pending(self):
        """pending 상태의 가장 오래된 task를 꺼내 배정 시도.

        호출 시점:
          1. 로봇이 'idle' 상태 발행 시 (_on_robot_status)
          2. 노드 기동 직후 (재시작 복구)
        """
        cur = self._db.cursor()
        try:
            # created_at ASC: 가장 먼저 생성된 task를 먼저 처리 (FIFO)
            cur.execute(
                """
                SELECT id FROM tasks
                WHERE status = 'pending'
                ORDER BY created_at ASC
                LIMIT 1
                """
            )
            row = cur.fetchone()
            if row:
                self._try_assign(row[0])
        except Exception as e:
            self.get_logger().error(f'_check_pending 실패: {e}')

    # ── 종료 처리 ─────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._db.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FleetManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
