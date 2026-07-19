"""ROS2(도메인 12) → Qt 신호 브릿지.

rclpy 콜백은 executor 스레드에서 돈다. 위젯은 GUI 스레드에서만 만질 수 있으므로
이 클래스는 **오직 Qt 시그널만 emit** 한다 (PyQt가 스레드 경계에서 자동으로 큐잉).
위젯 참조를 여기로 들이지 말 것 — 그 순간 크래시 원인이 된다.

구독 토픽은 전부 smart_domain_bridge가 로봇 도메인(30/31) → 서버(12)로 넘겨주는 것들:
  /{robot}/robot_status  std_msgs/String   'idle'|'busy'|'error'  (1Hz)
  /{robot}/battery_state sensor_msgs/BatteryState  percentage 0.0~1.0
  /traffic/pose          std_msgs/String   {"robot":..., "x":..., "y":...}
"""

import json
import threading
import time

import rclpy
from PyQt5.QtCore import QObject, pyqtSignal
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

ROBOT_IDS = ['AMR_1', 'AMR_2']

# robot_status가 1Hz라 이 시간 넘게 조용하면 연결이 끊긴 것으로 본다.
# WiFi 포화로 구독이 wedge된 전례(context/13 §8)가 있어 UI가 반드시 드러내야 하는 상태.
STALE_SEC = 3.0


def _norm_battery(pct: float) -> float:
    """BatteryState.percentage를 0.0~1.0으로 정규화.

    sensor_msgs/BatteryState 스펙은 percentage = 0.0~1.0인데
    **turtlebot3_node는 0~100으로 발행한다**(실측 2026-07-17: voltage 12.0에 percentage 83.33).
    발행자를 우리가 못 고치므로 수신 측에서 정규화한다.
    ※ 같은 이유로 fleet_manager의 배터리 가드(`percentage < 0.3`)도 무력 상태 — 별도 수정 필요.
    """
    pct = float(pct)
    return pct / 100.0 if pct > 1.0 else pct


class RosLink(QObject):
    """rclpy 노드 + 실행 스레드. 수신값은 시그널로만 GUI에 전달."""

    status_changed = pyqtSignal(str, str)          # robot, 'idle'|'busy'|'error'
    battery_changed = pyqtSignal(str, float)       # robot, 0.0~1.0
    pose_changed = pyqtSignal(str, float, float)   # robot, x, y
    detection = pyqtSignal(str, str)               # 토픽 종류, 사람이 읽을 요약

    def __init__(self):
        super().__init__()
        self._lock = threading.Lock()
        self._last_seen = {r: 0.0 for r in ROBOT_IDS}

        rclpy.init()
        self._node = Node('smart_admin_gui')

        for robot in ROBOT_IDS:
            # 기본인자로 루프 변수를 캡처 — 클로저 늦은 바인딩 방지
            self._node.create_subscription(
                String, f'/{robot}/robot_status',
                lambda msg, r=robot: self._on_status(r, msg), 10)
            self._node.create_subscription(
                BatteryState, f'/{robot}/battery_state',
                lambda msg, r=robot: self._on_battery(r, msg), 10)

        self._node.create_subscription(String, '/traffic/pose', self._on_pose, 10)

        # AI 감지 이벤트 — task_manager의 임무 트리거 소스. 문자열이라 상시 구독해도 공짜.
        # (옛 문서의 pickup/no_pickup이 아니라 현재 코드는 placed/cleared)
        self._node.create_subscription(
            String, '/detection/inbound',
            lambda m: self._on_detection('inbound', m), 10)
        self._node.create_subscription(
            String, '/detection/placed',
            lambda m: self._on_detection('placed', m), 10)
        self._node.create_subscription(
            String, '/detection/cleared',
            lambda m: self._on_detection('cleared', m), 10)

        # 관제 개입: 오배송 회수 위임 (task_manager가 구독)
        self._reclaim_pub = self._node.create_publisher(String, '/reclaim_request', 10)

        self._exec = MultiThreadedExecutor()
        self._exec.add_node(self._node)
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self):
        try:
            self._exec.spin()
        except (rclpy.executors.ExternalShutdownException, RuntimeError):
            pass

    # ── 콜백 (executor 스레드) ────────────────────────────────

    def _on_status(self, robot: str, msg: String):
        with self._lock:
            self._last_seen[robot] = time.monotonic()
        self.status_changed.emit(robot, msg.data)

    def _on_battery(self, robot: str, msg: BatteryState):
        self.battery_changed.emit(robot, _norm_battery(msg.percentage))

    def _on_pose(self, msg: String):
        try:
            p = json.loads(msg.data)
            self.pose_changed.emit(str(p['robot']), float(p['x']), float(p['y']))
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            self._node.get_logger().warn(f'/traffic/pose 파싱 실패: {msg.data!r}')

    def _on_detection(self, kind: str, msg: String):
        """inbound={'product_name':…} / placed·cleared={'slot':…} — 필드가 다르다."""
        try:
            d = json.loads(msg.data)
        except json.JSONDecodeError:
            self._node.get_logger().warn(f'/detection/{kind} 파싱 실패: {msg.data!r}')
            return
        self.detection.emit(kind, d.get('product_name') or d.get('slot') or msg.data)

    # ── GUI 스레드에서 호출 ───────────────────────────────────

    def is_stale(self, robot: str) -> bool:
        """robot_status가 STALE_SEC 넘게 안 온 상태(= 브릿지/WiFi/로봇 다운)."""
        with self._lock:
            last = self._last_seen[robot]
        return last == 0.0 or (time.monotonic() - last) > STALE_SEC

    def publish_reclaim(self, order_id: int):
        """오배송 회수 위임 — task_manager가 구독해 reclaim task를 만든다.

        UI가 reclaim task를 직접 INSERT하지 않는 게 핵심. 어느 선반으로 되돌릴지는
        task_manager의 `_create_reclaim`이 outbound task를 보고 정한다(권한은 소유자에게).
        """
        self._reclaim_pub.publish(String(data=json.dumps({'order_id': order_id})))
        self._node.get_logger().info(f'/reclaim_request 발행: order_id={order_id}')

    def shutdown(self):
        self._exec.shutdown()
        self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
