#!/usr/bin/env python3
"""AI 입고 감지 노드 — IN-1 게이트 카메라를 YOLO(best.pt)로 감시.

물품이 게이트에 놓이면 종류를 판별해 /detection/inbound 로 1회 발행
→ task_manager가 storage_type 조회·빈 슬롯 선택 후 inbound task 생성.

    발행  /detection/inbound   {"product_name": "사과"}  (DB products.name)
    발행  /detection/inbound/debug  (선택) 어노테이션 프레임 — rviz 확인용

원샷 상태머신 (물건 1개 = task 1개 보장):
  ARMED    : 같은 클래스가 stable_frames 연속 감지되면 발행 → COOLDOWN
  COOLDOWN : 게이트가 clear_frames 연속 비어야 재무장 (로봇이 집어간 뒤)
             — 같은 물건이 계속 놓여 있는 동안 중복 발행 방지

서버(도메인 12)에서 실행:
    ros2 run ai_detector inbound_detector --ros-args \
        -p model_path:=$HOME/best.pt -p camera:=0
카메라 대신 동영상/이미지 파일 경로도 camera 파라미터로 지정 가능 (오프라인 테스트).
"""
import json
import os

import cv2
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# config/product_map.yaml — 패키지 share 기준, 소스트리 실행도 지원
_PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class InboundDetector(Node):
    def __init__(self):
        super().__init__('inbound_detector')
        self.declare_parameter('model_path', '')      # 기본: 패키지 내장 models/best.pt
        # 'topic:<이름>'=이미지 토픽 구독(기본 — 카메라 공유: 출고 감지·관제 뷰)
        #   '/compressed' 접미사면 JPEG 스트림 구독. 카메라 브링업(노트북)과 추론(데탑)을
        #   분리해 돌리므로 기본값이 압축 — raw는 프레임이 40배 커서 못 넘긴다.
        # 'realsense'=D435 직접 열기(단독 테스트) / 숫자=웹캠 / 경로=파일
        self.declare_parameter(
            'camera', 'topic:/camera/camera/color/image_raw/compressed')
        # 책상 테스트 기준 0.35 (apple ~0.39). 게이트 근접 마운트 후 0.5+로 재튜닝
        self.declare_parameter('conf', 0.35)
        self.declare_parameter('stable_frames', 5)     # 발행에 필요한 연속 동일 감지
        self.declare_parameter('clear_frames', 10)     # 재무장에 필요한 연속 빈 게이트
        self.declare_parameter('rate', 5.0)            # 처리 주기(Hz)
        self.declare_parameter('device', 'auto')       # auto/cpu/cuda
        self.declare_parameter('product_map', '')      # 매핑 yaml (기본: 패키지 내장)
        self.declare_parameter('publish_debug', True)  # 어노테이션 프레임 발행

        self._pub = self.create_publisher(String, '/detection/inbound', 10)
        self._dbg_pub = None
        if self.get_parameter('publish_debug').value:
            from sensor_msgs.msg import CompressedImage
            # 원격 관제 GUI(WiFi)로도 넘어가도록 raw 대신 JPEG 압축 발행
            self._dbg_pub = self.create_publisher(
                CompressedImage, '/detection/inbound/debug/compressed', 1)

        self._load_map()
        self._load_model()
        self._open_camera()

        # 원샷 상태머신
        self._state = 'ARMED'
        self._streak_cls = None     # 연속 감지 중인 클래스
        self._streak = 0
        self._clear = 0

        period = 1.0 / self.get_parameter('rate').value
        self.create_timer(period, self._tick)
        self.get_logger().info('입고 감지 시작 (ARMED)')

    # ── 초기화 ───────────────────────────────────────────

    def _load_map(self):
        path = self.get_parameter('product_map').value or os.path.join(
            _PKG_DIR, 'config', 'product_map.yaml')
        if not os.path.exists(path):                   # 설치 환경: share 폴더
            from ament_index_python.packages import get_package_share_directory
            path = os.path.join(get_package_share_directory('ai_detector'),
                                'config', 'product_map.yaml')
        with open(path) as f:
            self.product_map = yaml.safe_load(f)['product_map']
        active = {k: v for k, v in self.product_map.items() if v}
        self.get_logger().info(f'상품 매핑 {len(active)}종 로드: {active}')

    def _load_model(self):
        import torch
        from ultralytics import YOLO                   # 지연 import (기동 로그 먼저)
        # 시스템 cuDNN(/lib)과 pip 휠 cuDNN 혼재 → SUBLIBRARY_VERSION_MISMATCH.
        # cudnn 끄면 네이티브 CUDA 커널로 동작 (best.pt 기준 4ms/frame — 충분)
        torch.backends.cudnn.enabled = False
        path = self.get_parameter('model_path').value or self._default_model()
        self.model = YOLO(path)
        dev = self.get_parameter('device').value
        self.device = dev if dev != 'auto' else (0 if torch.cuda.is_available() else 'cpu')
        try:                                           # 워밍업 겸 장치 검증
            import numpy as np
            self.model.predict(np.zeros((64, 64, 3), dtype=np.uint8),
                               device=self.device, verbose=False)
        except Exception as e:
            self.get_logger().warn(f'{self.device} 추론 실패({e}) — CPU 폴백')
            self.device = 'cpu'
        self.get_logger().info(
            f'모델 로드: {path} (device={self.device}) — '
            f'클래스 {list(self.model.names.values())}')
        unknown = set(self.model.names.values()) - set(self.product_map)
        if unknown:
            self.get_logger().warn(f'매핑에 없는 클래스 (무시됨): {unknown}')

    def _default_model(self):
        """패키지 내장 models/best.pt — 소스트리 실행이면 소스 쪽 우선."""
        src = os.path.join(_PKG_DIR, 'models', 'best.pt')
        if os.path.exists(src):
            return src
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('ai_detector'),
                            'models', 'best.pt')

    def _open_camera(self):
        src = self.get_parameter('camera').value
        self._rs = None
        self._sub_frame = None
        if src.startswith('topic:'):      # 카메라 브링업(realsense2_camera 등) 구독
            self._img_topic = src[len('topic:'):]
            if self._img_topic.endswith('/compressed'):
                from sensor_msgs.msg import CompressedImage
                self.create_subscription(
                    CompressedImage, self._img_topic, self._on_compressed, 1)
                self.get_logger().info(f'압축 이미지 토픽 구독: {self._img_topic}')
            else:
                from sensor_msgs.msg import Image
                self.create_subscription(Image, self._img_topic, self._on_image, 1)
                self.get_logger().info(f'이미지 토픽 구독: {self._img_topic}')
            return
        if src == 'realsense':            # Intel RealSense (D435 등) — SDK 경유
            import pyrealsense2 as rs
            self._rs = rs.pipeline()
            cfg = rs.config()
            # USB2 연결도 커버하는 보수적 프로파일 (노드 처리율 5Hz면 충분)
            cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
            self._rs.start(cfg)
            self.get_logger().info('RealSense color 640x480@15 시작')
            return
        self._cap = cv2.VideoCapture(int(src) if src.isdigit() else src)
        if not self._cap.isOpened():
            raise RuntimeError(f'카메라/파일 열기 실패: {src}')

    def _on_image(self, msg):
        """구독 프레임 저장 — 처리(_tick 5Hz)는 최신 프레임만 사용."""
        import numpy as np
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding in ('rgb8', 'rgb'):
            img = img[:, :, ::-1].copy()               # RGB→BGR (모델 입력 규약)
        self._sub_frame = img

    def _on_compressed(self, msg):
        """JPEG 프레임 디코드 — imdecode 결과가 BGR이라 변환 불필요(모델 입력 규약)."""
        import numpy as np
        self._sub_frame = cv2.imdecode(
            np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)

    def _read_frame(self):
        if hasattr(self, '_img_topic'):
            if self._sub_frame is None:
                return False, None                     # 아직 첫 프레임 전
            return True, self._sub_frame
        if self._rs is not None:
            import numpy as np
            try:
                frames = self._rs.wait_for_frames(1000)
            except RuntimeError:          # 일시적 프레임 미도착 — tick 스킵
                return False, None
            return True, np.asanyarray(frames.get_color_frame().get_data())
        return self._cap.read()

    # ── 메인 루프 ─────────────────────────────────────────

    def _tick(self):
        ok, frame = self._read_frame()
        if not ok:
            self.get_logger().warn('프레임 없음', throttle_duration_sec=5.0)
            return

        conf = self.get_parameter('conf').value
        res = self.model.predict(frame, conf=conf, device=self.device,
                                 verbose=False)[0]

        # 게이트 위 물품 1개 전제 — 신뢰도순으로 훑되,
        # 화면 대부분을 덮는 박스(오감지)와 매핑 없는 클래스는 건너뜀
        best_cls = None
        h, w = frame.shape[:2]
        order = res.boxes.conf.argsort(descending=True) if len(res.boxes) else []
        for i in order:
            x1, y1, x2, y2 = res.boxes.xyxy[int(i)].tolist()
            if (x2 - x1) * (y2 - y1) > 0.7 * w * h:
                continue                               # 풀프레임 쓰레기 박스
            cls = self.model.names[int(res.boxes.cls[int(i)])]
            if self.product_map.get(cls) is None:
                continue                               # 미매핑 클래스 (other_square 등)
            best_cls = cls
            break

        if self._dbg_pub is not None:
            self._dbg_pub.publish(self._to_compressed(res.plot()))

        if self._state == 'ARMED':
            self._tick_armed(best_cls)
        else:
            self._tick_cooldown(best_cls)

    def _to_compressed(self, bgr):
        """cv2 BGR 프레임 → sensor_msgs/CompressedImage(JPEG). cv_bridge 미사용
        (pip opencv 5.x와 시스템 cv_bridge 4.x 바이너리 충돌 회피).
        JPEG ~40KB/frame — raw(~920KB)보다 20배↓라 원격 GUI로도 스트리밍 가능."""
        from sensor_msgs.msg import CompressedImage
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = 'jpeg'
        msg.data = cv2.imencode('.jpg', bgr)[1].tobytes()
        return msg

    def _tick_armed(self, cls):
        if cls is None or self.product_map.get(cls) is None:
            self._streak_cls, self._streak = None, 0   # 미감지/무시 클래스 → 리셋
            return
        if cls == self._streak_cls:
            self._streak += 1
        else:
            self._streak_cls, self._streak = cls, 1
        if self._streak >= self.get_parameter('stable_frames').value:
            product = self.product_map[cls]
            self._pub.publish(String(data=json.dumps(
                {'product_name': product}, ensure_ascii=False)))
            self.get_logger().info(
                f'입고 감지 발행: {cls} → "{product}" (COOLDOWN 진입)')
            self._state, self._clear = 'COOLDOWN', 0
            self._streak_cls, self._streak = None, 0

    def _tick_cooldown(self, cls):
        """게이트가 비워질 때까지(clear_frames 연속 미감지) 재발행 금지."""
        if cls is None:
            self._clear += 1
            if self._clear >= self.get_parameter('clear_frames').value:
                self._state = 'ARMED'
                self.get_logger().info('게이트 비움 확인 — 재무장 (ARMED)')
        else:
            self._clear = 0

    def destroy_node(self):
        if self._rs is not None:
            self._rs.stop()
        elif not hasattr(self, '_img_topic'):
            self._cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = InboundDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
