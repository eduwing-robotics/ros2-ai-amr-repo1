"""카메라 구독 — 도메인이 다른 카메라를 한 프로세스에서 **필요할 때만** 받는다.

카메라가 두 도메인에 흩어져 있다:
  게이트 IN-1/OUT-1/OUT-2 = 도메인 12 (ai_detector가 YOLO 어노테이션까지 그려서 발행)
  로봇 전방 AMR_1/AMR_2   = 도메인 30/31 (camera_ros, 1024x768 10fps)

로봇 전방을 domain_bridge로 12에 넘기지 않는 이유:
  브릿지는 **아무도 안 보고 있어도 계속 중계**한다. 그 스트림은 이미 WiFi를 건너
  워크스테이션 aruco_estimator가 쓰고 있고(도킹 제어의 입력), 여기에 상시 사본을 하나 더
  얹으면 2026-07-09에 겪은 "Nav2 주행 시작 → 대역폭 포화 → estimator 구독 wedge"를 다시 부른다.
  (그때 10fps로 낮춰서 겨우 잡은 문제다. context/13 §8)

대신 rclpy 컨텍스트를 도메인별로 따로 열고, **화면이 열려 있는 동안만 구독**한다.
→ 관제 화면을 안 보는 동안 WiFi 비용 0. 브릿지로는 불가능한 이점.

QoS는 전부 sensor_data(best_effort). 카메라 발행자가 best_effort라 reliable로 구독하면
호환이 안 맞아 **한 프레임도 안 온다**. best_effort 구독은 reliable 발행자와도 호환되므로
게이트캠(기본 QoS=reliable)까지 이 하나로 커버된다.
"""

import threading
from dataclasses import dataclass

import rclpy
from PyQt5.QtCore import QObject, pyqtSignal
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


@dataclass(frozen=True)
class CamSpec:
    domain: int
    topic: str
    label: str


# 화면에 뜨는 카메라 전부. 도메인이 섞여 있는 게 이 표의 핵심.
CAMERAS = {
    'AMR_1': CamSpec(30, '/camera/image_raw/compressed', 'AMR_1 전방'),
    'AMR_2': CamSpec(31, '/camera/image_raw/compressed', 'AMR_2 전방'),
    'IN-1': CamSpec(12, '/detection/inbound/debug/compressed', '입고 IN-1'),
    'OUT-1': CamSpec(12, '/detection/out1/debug/compressed', '출고 OUT-1'),
    'OUT-2': CamSpec(12, '/detection/out2/debug/compressed', '출고 OUT-2'),
}

GATE_CAMS = ['IN-1', 'OUT-1', 'OUT-2']
ROBOT_CAMS = ['AMR_1', 'AMR_2']


class _DomainCtx:
    """도메인 하나당 컨텍스트+노드+스핀 스레드 한 벌."""

    def __init__(self, domain: int):
        self.domain = domain
        self.ctx = rclpy.Context()
        rclpy.init(context=self.ctx, domain_id=domain)
        self.node = Node(f'smart_gui_cam_d{domain}', context=self.ctx)
        self.executor = SingleThreadedExecutor(context=self.ctx)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()

    def _spin(self):
        try:
            self.executor.spin()
        except Exception:
            pass          # shutdown 중 정상 종료

    def close(self):
        """종료 순서를 지킨다: 스핀 정지 → 노드 파괴 → 컨텍스트 종료.

        이 순서를 어기고 스핀 중에 context를 내리면
        `terminate called without an active exception`으로 코어덤프 난다(실측).
        """
        self.executor.shutdown()
        self.thread.join(timeout=2.0)
        self.node.destroy_node()
        rclpy.shutdown(context=self.ctx)


class CameraHub(QObject):
    """카메라 구독을 켜고 끈다. 프레임은 JPEG 바이트 그대로 시그널로 넘긴다.

    디코드는 GUI 쪽(QPixmap)에서 — 여기서 이미지 라이브러리를 끌어들이지 않는다.
    """

    frame = pyqtSignal(str, bytes)      # cam 이름, JPEG 바이트

    def __init__(self):
        super().__init__()
        self._domains = {}      # domain → _DomainCtx  (첫 사용 시 생성)
        self._subs = {}         # cam 이름 → Subscription
        self._lock = threading.Lock()

    def _domain_ctx(self, domain: int) -> _DomainCtx:
        if domain not in self._domains:
            self._domains[domain] = _DomainCtx(domain)
        return self._domains[domain]

    def start(self, name: str):
        """구독 시작. 이미 구독 중이면 아무것도 안 한다."""
        with self._lock:
            if name in self._subs:
                return
            spec = CAMERAS[name]
            dc = self._domain_ctx(spec.domain)
            self._subs[name] = dc.node.create_subscription(
                CompressedImage, spec.topic,
                lambda msg, n=name: self.frame.emit(n, bytes(msg.data)),
                qos_profile_sensor_data)

    def stop(self, name: str):
        """구독 해제 = 이 카메라에 대한 네트워크 비용 중단."""
        with self._lock:
            sub = self._subs.pop(name, None)
            if sub is None:
                return
            dc = self._domains.get(CAMERAS[name].domain)
            if dc is not None:
                dc.node.destroy_subscription(sub)

    def stop_all(self):
        for name in list(self._subs):
            self.stop(name)

    def active(self) -> set:
        return set(self._subs)

    def shutdown(self):
        self.stop_all()
        for dc in self._domains.values():
            dc.close()
        self._domains.clear()
