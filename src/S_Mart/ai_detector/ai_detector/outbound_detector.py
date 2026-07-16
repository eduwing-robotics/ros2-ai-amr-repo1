#!/usr/bin/env python3
"""AI 출고 감지 노드 — OUT 게이트 카메라를 YOLO(best.pt)로 감시.

게이트의 '점유 여부'만 보고한다. 미수령 타이머는 task_manager가 소유한다.

    발행  /detection/placed   {"slot": "OUT-1"}   — 빈 게이트에 물건이 놓임
    발행  /detection/cleared  {"slot": "OUT-1"}   — 있던 물건이 사라짐
    발행  /detection/out1/debug/compressed (선택) 어노테이션 프레임 — 관제/rqt 확인용

★ 타이머를 여기 두면 안 된다 (2026-07-16 실주행에서 터짐).
  감지 노드는 '물건이 보이는 순간' 타이머를 시작하는데, task_manager는 주문이
  awaiting_pickup이 된 뒤에만 신호를 받는다. 로봇이 물건을 내려놓고 완료 보고를
  하기까지 49초가 걸렸고, 그 사이 30초 타이머가 먼저 터져 no_pickup이 버려졌다.
  그 뒤 감지 노드는 COOLDOWN에 갇혀(물건이 그대로라 재무장 조건인 '빈 게이트'가
  영원히 성립 안 함) 다시는 신호를 못 보냈고, 주문은 awaiting_pickup에 영구 고착,
  게이트와 선반이 같이 죽었다.
  → 시계는 주문 상태를 아는 쪽(task_manager)이 들어야 한다. 여기는 눈만 담당.

상태머신 (전이할 때만 발행 — 같은 상태가 계속되면 조용):
  EMPTY    : 같은 클래스가 stable_frames 연속 감지 → OCCUPIED, placed 발행
  OCCUPIED : clear_frames 연속 비면               → EMPTY,    cleared 발행

reclaim 로봇이 미수령 물건을 집어가도 cleared가 나가지만, 그때 주문은 이미
cancelled라 task_manager가 자연히 무시한다 (DB 상태가 방어 — COOLDOWN 불필요).

서버(도메인 12)에서 실행 (웹캠 직접 열기 or usb_cam 토픽 구독):
    ros2 run ai_detector outbound_detector --ros-args \
        -p camera:=topic:/out1/image_raw/compressed -p slot:=OUT-1
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


class OutboundDetector(Node):
    def __init__(self):
        super().__init__('outbound_detector')
        self.declare_parameter('model_path', '')      # 기본: 패키지 내장 models/best.pt
        # 'topic:<이름>'=이미지 토픽 구독(usb_cam 등) / 숫자·경로=웹캠 직접 / 'realsense'=SDK
        self.declare_parameter('camera', 'topic:/out1/image_raw/compressed')
        self.declare_parameter('slot', 'OUT-1')        # 이 카메라가 전담하는 출고 슬롯
        self.declare_parameter('conf', 0.35)
        self.declare_parameter('stable_frames', 5)     # 놓임 확정에 필요한 연속 동일 감지
        self.declare_parameter('clear_frames', 10)     # 사라짐 확정에 필요한 연속 빈 게이트
        self.declare_parameter('rate', 5.0)            # 처리 주기(Hz)
        self.declare_parameter('device', 'auto')       # auto/cpu/cuda
        self.declare_parameter('product_map', '')      # 매핑 yaml (기본: 패키지 내장)
        self.declare_parameter('publish_debug', True)  # 어노테이션 프레임 발행
        # ── 웹캠 직접 열기(숫자/경로) 전용 포맷 강제 ──────────────
        #   저가 UVC 웹캠은 포맷/해상도를 명시해야 read()가 프레임을 뱉는 경우가 있음.
        #   topic:/realsense 모드는 이 값 무시. '' 또는 0 = 미설정(드라이버 기본).
        self.declare_parameter('cam_fourcc', 'YUYV')     # 강제 픽셀포맷 (예: YUYV, MJPG)
        self.declare_parameter('cam_width', 320)
        self.declare_parameter('cam_height', 240)
        self.declare_parameter('cam_fps', 15.0)

        self._slot = self.get_parameter('slot').value
        self._placed_pub = self.create_publisher(String, '/detection/placed', 10)
        self._cleared_pub = self.create_publisher(String, '/detection/cleared', 10)
        self._dbg_pub = None
        if self.get_parameter('publish_debug').value:
            from sensor_msgs.msg import CompressedImage
            # 슬롯별 압축(JPEG) debug 토픽 (OUT-1 → /detection/out1/debug/compressed).
            # 원격 관제 GUI(WiFi)로도 넘어가도록 raw 대신 CompressedImage 발행.
            topic = f'/detection/{self._slot.lower().replace("-", "")}/debug/compressed'
            self._dbg_pub = self.create_publisher(CompressedImage, topic, 1)

        self._load_map()
        self._load_model()
        self._open_camera()

        # 점유 상태머신 (EMPTY ↔ OCCUPIED) — 전이할 때만 발행
        self._state = 'EMPTY'
        self._streak_cls = None     # 놓임 확정용 연속 감지 클래스
        self._streak = 0
        self._clear = 0

        period = 1.0 / self.get_parameter('rate').value
        self.create_timer(period, self._tick)
        self.get_logger().info(f'출고 감지 시작 (slot={self._slot}, EMPTY)')

    # ── 초기화 ───────────────────────────────────────────

    def _load_map(self):
        path = self.get_parameter('product_map').value or os.path.join(
            _PKG_DIR, 'config', 'product_map.yaml')
        if not os.path.exists(path):                   # 설치 환경: share 폴더
            from ament_index_python.packages import get_package_share_directory
            path = os.path.join(get_package_share_directory('ai_detector'),
                                'config', 'product_map.yaml')
        with open(path) as f:
            doc = yaml.safe_load(f)
        # 클래스 → 상품명. 여기선 '물건이 있냐'만 판단하므로 값은 안 쓰고 매핑 여부만 본다
        # (미매핑 클래스는 게이트 위 물건으로 치지 않음). 저장타입은 task_manager가 DB에서 조회.
        self.product_map = doc['product_map']
        active = {k: v for k, v in self.product_map.items() if v}
        self.get_logger().info(f'상품 매핑 {len(active)}종 로드: {list(active)}')

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
        if src.startswith('topic:'):      # 카메라 브링업(usb_cam/realsense 등) 구독
            self._img_topic = src[len('topic:'):]
            self._sub_frame = None
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
        if src == 'realsense':            # Intel RealSense (SDK 경유)
            import pyrealsense2 as rs
            self._rs = rs.pipeline()
            cfg = rs.config()
            cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
            self._rs.start(cfg)
            self.get_logger().info('RealSense color 640x480@15 시작')
            return
        # ★ V4L2 백엔드 명시 — 기본 백엔드(GStreamer/FFMPEG)가 UVC 웹캠에서
        #   open hang / 무프레임을 내는 경우가 많음. V4L2로 강제하면 안정적.
        self._cap = cv2.VideoCapture(int(src) if src.isdigit() else src, cv2.CAP_V4L2)
        if not self._cap.isOpened():
            raise RuntimeError(f'카메라/파일 열기 실패: {src}')
        # 저가 UVC 웹캠: 포맷/해상도 명시해야 read()가 프레임을 뱉는 경우가 있음
        fourcc = self.get_parameter('cam_fourcc').value
        w = int(self.get_parameter('cam_width').value)
        h = int(self.get_parameter('cam_height').value)
        fps = float(self.get_parameter('cam_fps').value)
        if fourcc:
            self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        if w > 0:
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        if h > 0:
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        if fps > 0:
            self._cap.set(cv2.CAP_PROP_FPS, fps)
        self.get_logger().info(
            f'웹캠 열기 성공: {src} (fourcc={fourcc or "기본"} {w}x{h}@{fps:.0f})')

    def _on_compressed(self, msg):
        """JPEG 프레임 디코드 — imdecode 결과가 BGR이라 변환 불필요(모델 입력 규약)."""
        import numpy as np
        self._sub_frame = cv2.imdecode(
            np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)

    def _on_image(self, msg):
        """구독 프레임 저장 — 처리(_tick)는 최신 프레임만 사용."""
        import numpy as np
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding in ('rgb8', 'rgb'):
            img = img[:, :, ::-1].copy()               # RGB→BGR (모델 입력 규약)
        self._sub_frame = img

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

    def _detect_product(self, frame):
        """프레임에서 매핑된 상품 클래스 1개(최고 신뢰도) 반환, 없으면 None.
        입고와 동일 규칙 — 풀프레임 쓰레기 박스·미매핑 클래스는 건너뜀."""
        conf = self.get_parameter('conf').value
        res = self.model.predict(frame, conf=conf, device=self.device,
                                 verbose=False)[0]
        best_cls = None
        h, w = frame.shape[:2]
        order = res.boxes.conf.argsort(descending=True) if len(res.boxes) else []
        for i in order:
            x1, y1, x2, y2 = res.boxes.xyxy[int(i)].tolist()
            if (x2 - x1) * (y2 - y1) > 0.7 * w * h:
                continue                               # 풀프레임 쓰레기 박스
            cls = self.model.names[int(res.boxes.cls[int(i)])]
            if self.product_map.get(cls) is None:
                continue                               # 미매핑 클래스
            best_cls = cls
            break
        return best_cls, res

    def _tick(self):
        ok, frame = self._read_frame()
        if not ok:
            self.get_logger().warn('프레임 없음', throttle_duration_sec=5.0)
            return

        cls, res = self._detect_product(frame)
        present = cls is not None

        if self._dbg_pub is not None:
            self._dbg_pub.publish(self._to_compressed(res.plot()))

        if self._state == 'EMPTY':
            self._tick_empty(present, cls)
        else:
            self._tick_occupied(present)

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

    def _emit(self, pub):
        pub.publish(String(data=json.dumps({'slot': self._slot}, ensure_ascii=False)))

    def _tick_empty(self, present, cls):
        """같은 클래스가 stable_frames 연속 보이면 OCCUPIED 전이 + placed 발행."""
        if not present:
            self._streak_cls, self._streak = None, 0
            return
        if cls == self._streak_cls:
            self._streak += 1
        else:
            self._streak_cls, self._streak = cls, 1   # 클래스 바뀌면 처음부터
        if self._streak >= self.get_parameter('stable_frames').value:
            self._emit(self._placed_pub)
            self.get_logger().info(
                f'물건 놓임: {cls} → /detection/placed {{slot:{self._slot}}} (OCCUPIED)')
            self._state, self._clear = 'OCCUPIED', 0
            self._streak_cls, self._streak = None, 0

    def _tick_occupied(self, present):
        """clear_frames 연속 비면 EMPTY 전이 + cleared 발행.

        고객 수령인지 reclaim 로봇 회수인지 여기선 구분하지 않는다 — task_manager가
        주문 상태로 판단한다(cancelled면 무시). 그래서 COOLDOWN 같은 별도 상태가 없다.
        """
        if present:
            self._clear = 0                            # 물건 보이면 사라짐 카운트 리셋
            return
        self._clear += 1
        if self._clear >= self.get_parameter('clear_frames').value:
            self._emit(self._cleared_pub)
            self.get_logger().info(
                f'게이트 비움 → /detection/cleared {{slot:{self._slot}}} (EMPTY)')
            self._state, self._clear = 'EMPTY', 0

    def destroy_node(self):
        if self._rs is not None:
            self._rs.stop()
        elif not hasattr(self, '_img_topic'):
            self._cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = OutboundDetector()
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
