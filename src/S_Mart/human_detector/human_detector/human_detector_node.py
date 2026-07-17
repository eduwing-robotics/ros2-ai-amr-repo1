#!/usr/bin/env python3
"""사람 감지 노드 — 로봇 카메라를 YOLO(human_best.pt)로 감시해 정지 신호 발행.

로봇 주행 경로에 사람이 있으면 /human_stop=true 를 발행한다. nav2 BT의 사람 게이트
(조건노드)가 이 신호를 읽어 FollowPath를 halt한다 (사람 사라지면 재개).

    발행  /human_stop  (std_msgs/Bool)  true=사람있음(정지) / false=없음(재개)
    발행  /human_detector/debug/compressed (선택) 어노테이션 프레임 — 관제/rqt 확인용

★ 비대칭 디바운스 (안전 방향):
  - 정지(assert)는 빠르게: 사람이 assert_frames(기본 1) 연속 감지되면 즉시 true.
    안전상 사람 보이면 곧장 멈춰야 하므로 문턱을 낮게 둔다.
  - 해제(release)는 느리게: 마지막으로 사람이 보인 뒤 release_sec(기본 2.0s)
    연속으로 안 보여야 false. 바운딩박스 깜빡임에 로봇이 앞으로 튀는 것을 막는다.
  둘을 같은 값으로 두면 안 된다 — 멈춤은 즉각, 재개는 신중.

배치: 도킹(aruco)과 동일 — 카메라 브링업은 로봇 Pi, 이 추론 노드는 데탑에서
로봇 도메인으로 실행. 같은 카메라 토픽(/camera/image_raw/compressed)을 구독한다.
    ros2 run human_detector human_detector --ros-args \
        -r image_raw:=/camera/image_raw/compressed
※ 거리 조건은 1차 미구현 — 프레임에 사람이 보이면 정지 (전방 카메라 전제).
"""
import os

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

# models/human_best.pt — 패키지 share 기준, 소스트리 실행도 지원
_PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class HumanDetector(Node):
    def __init__(self):
        super().__init__('human_detector')
        self.declare_parameter('model_path', '')       # 기본: 패키지 내장 models/human_best.pt
        # 'topic:<이름>' 구독 / 'realsense' / 숫자·경로. 기본은 로봇 카메라 압축 토픽.
        self.declare_parameter('camera', 'topic:/camera/image_raw/compressed')
        self.declare_parameter('conf', 0.35)           # 사람 감지 신뢰도 문턱
        self.declare_parameter('rate', 10.0)           # 처리 주기(Hz) — 안전이라 입고보다 높게
        self.declare_parameter('device', 'auto')       # auto/cpu/cuda
        self.declare_parameter('publish_debug', True)  # 어노테이션 프레임 발행
        self.declare_parameter('assert_frames', 1)     # 정지 확정에 필요한 연속 감지(빠르게)
        self.declare_parameter('release_sec', 2.0)     # 해제에 필요한 연속 미감지 시간(느리게)
        # ── 거리 필터 (박스 높이 = 거리 대용, 단안 카메라라 depth 없음) ──
        #   박스높이/화면높이 >= min_height_ratio 인 사람만 정지 대상 = 가까운 사람만.
        #   멀리 있으면 박스가 작아 이 값 미만 → 무시(정지 안 함).
        #   0.0 = 필터 끔(사람 보이면 무조건 정지). A4 타겟은 크기 고정이라 잘 맞음.
        #   보정: A4를 멈추고 싶은 거리에 두고 debug 로그의 height_ratio 확인 → 살짝 아래로 설정.
        self.declare_parameter('min_height_ratio', 0.0)
        # human_best.pt에서 '사람'에 해당하는 클래스명(들). 모델에 따라 다름 — 기동 로그로 확인.
        self.declare_parameter('person_classes', ['person'])

        self._stop_pub = self.create_publisher(Bool, '/human_stop', 10)
        self._dbg_pub = None
        if self.get_parameter('publish_debug').value:
            from sensor_msgs.msg import CompressedImage
            self._dbg_pub = self.create_publisher(
                CompressedImage, '/human_detector/debug/compressed', 1)

        self._load_model()
        self._open_camera()

        # 디바운스 상태
        self._hold = False          # 현재 정지 신호
        self._streak = 0            # 사람 연속 감지 프레임 수
        self._last_seen = None      # 마지막으로 사람 본 시각(node clock)

        self._person_classes = set(self.get_parameter('person_classes').value)
        period = 1.0 / self.get_parameter('rate').value
        self.create_timer(period, self._tick)
        # 기동 직후 안전값(정지 아님) 1회 발행 — 구독자가 초기 상태 알도록
        self._publish_hold(False)
        self.get_logger().info('사람 감지 시작 (정지=false)')

    # ── 초기화 ───────────────────────────────────────────

    def _load_model(self):
        import torch
        from ultralytics import YOLO
        # 시스템 cuDNN과 pip 휠 cuDNN 혼재 → SUBLIBRARY_VERSION_MISMATCH 회피
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
        src = os.path.join(_PKG_DIR, 'models', 'human_best.pt')
        if os.path.exists(src):
            return src
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('human_detector'),
                            'models', 'human_best.pt')

    def _open_camera(self):
        src = self.get_parameter('camera').value
        self._rs = None
        self._sub_frame = None
        if src.startswith('topic:'):
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
        # 웹캠 직접 열기 (단독 테스트용) — V4L2 백엔드 명시
        self._cap = cv2.VideoCapture(int(src) if src.isdigit() else src, cv2.CAP_V4L2)
        if not self._cap.isOpened():
            raise RuntimeError(f'카메라 열기 실패: {src}')

    def _on_compressed(self, msg):
        import numpy as np
        self._sub_frame = cv2.imdecode(
            np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)

    def _on_image(self, msg):
        import numpy as np
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding in ('rgb8', 'rgb'):
            img = img[:, :, ::-1].copy()
        self._sub_frame = img

    def _read_frame(self):
        if hasattr(self, '_img_topic'):
            if self._sub_frame is None:
                return False, None
            return True, self._sub_frame
        return self._cap.read()

    # ── 메인 루프 ─────────────────────────────────────────

    def _detect_person(self, frame):
        """가까운 사람이 있으면 True. person 박스 중 높이비율이 min_height_ratio 이상이면
        '가까움'=정지 대상. 멀면(박스 작음) 무시. 보정 편의로 최대 높이비율을 로그에 남김."""
        conf = self.get_parameter('conf').value
        min_ratio = self.get_parameter('min_height_ratio').value
        res = self.model.predict(frame, conf=conf, device=self.device, verbose=False)[0]
        h = frame.shape[0]
        best_ratio = 0.0
        present = False
        for i in range(len(res.boxes)):
            cls = self.model.names[int(res.boxes.cls[int(i)])]
            if cls not in self._person_classes:
                continue
            x1, y1, x2, y2 = res.boxes.xyxy[int(i)].tolist()
            ratio = (y2 - y1) / h
            best_ratio = max(best_ratio, ratio)
            if ratio >= min_ratio:
                present = True
        # 보정용: 감지된 사람의 최대 높이비율을 주기적으로 로그 (min_height_ratio 잡을 때 참고)
        if best_ratio > 0.0:
            self.get_logger().info(
                f'사람 감지 height_ratio={best_ratio:.2f} '
                f'(문턱 {min_ratio:.2f}, {"정지" if present else "무시(멀다)"})',
                throttle_duration_sec=1.0)
        return present, res

    def _tick(self):
        ok, frame = self._read_frame()
        if not ok:
            # 프레임이 안 오면 안전하게 정지 유지 (카메라 끊김 = 위험). 경고만 throttle.
            self.get_logger().warn('프레임 없음 — 정지 유지', throttle_duration_sec=5.0)
            self._publish_hold(True)
            return

        present, res = self._detect_person(frame)

        if self._dbg_pub is not None:
            self._dbg_pub.publish(self._to_compressed(res.plot()))

        now = self.get_clock().now()
        if present:
            self._streak += 1
            self._last_seen = now
            if self._streak >= self.get_parameter('assert_frames').value:
                self._set_hold(True)
        else:
            self._streak = 0
            # 마지막 감지 후 release_sec 연속 미감지면 해제
            if self._hold and self._last_seen is not None:
                elapsed = (now - self._last_seen).nanoseconds / 1e9
                if elapsed >= self.get_parameter('release_sec').value:
                    self._set_hold(False)

        # 상태와 무관하게 매 tick 재발행 — 신호 유실/늦게 뜬 구독자 대비 (안전 신호)
        self._publish_hold(self._hold)

    def _set_hold(self, value):
        if value != self._hold:
            self._hold = value
            self.get_logger().info(f'사람 정지 → {value}')

    def _publish_hold(self, value):
        self._stop_pub.publish(Bool(data=value))

    def _to_compressed(self, bgr):
        """cv2 BGR → CompressedImage(JPEG). cv_bridge 미사용(opencv/cv_bridge ABI 충돌 회피)."""
        from sensor_msgs.msg import CompressedImage
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = 'jpeg'
        msg.data = cv2.imencode('.jpg', bgr)[1].tobytes()
        return msg

    def destroy_node(self):
        if not hasattr(self, '_img_topic') and hasattr(self, '_cap'):
            self._cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = HumanDetector()
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
