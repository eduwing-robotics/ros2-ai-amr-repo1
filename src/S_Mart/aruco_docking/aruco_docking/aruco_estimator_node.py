#!/usr/bin/env python3
"""Smart Mart ArUco Pose Estimator (순수 인식 노드 — work/home 도킹 공용).

/image_raw + /camera_info 를 구독해 ArUco 마커를 검출하고,
solvePnP(IPPE_SQUARE)로 마커의 6DOF pose(카메라 광학프레임 기준)를 계산해
/detected_dock_pose (PoseStamped) 로 발행한다.

경계: 이 노드는 인식만 한다. TF 변환/odom 변환/오프셋 보정/필터/주행제어는
전부 소비자(Docking 컨트롤러)의 몫이다. 여기선 TF를 조회하지도 발행하지도 않는다.
상세: docs_hub/context/11-도킹서버-ArUco-상세.md §10, §14.
"""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from geometry_msgs.msg import PoseStamped


def rotmat_to_quat(R):
    """3x3 회전행렬 → (x, y, z, w). tf 의존성 없이 numpy만 사용."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return float(x), float(y), float(z), float(w)


class ArucoEstimatorNode(Node):

    def __init__(self):
        super().__init__('aruco_estimator')

        # ── 파라미터 ───────────────────────────────────────────────
        self.declare_parameter('marker_dict', 'DICT_4X4_50')
        self.declare_parameter('marker_size_m', 0.05)
        self.declare_parameter('target_marker_id', -1)   # -1 = 가장 가까운 마커
        self.declare_parameter('process_rate_hz', 10.0)  # <=0 이면 게이트 없음
        self.declare_parameter('publish_debug', False)
        self.declare_parameter('camera_optical_frame', '')  # 폴백 frame_id
        self.declare_parameter('use_compressed', True)   # WiFi 대역폭 절약(워크스테이션 배치 = 기본).
                                                         # raw로 받으려면 launch/CLI에서 False로.
        # 진단: N초마다 수신/검출 카운트 로그 → 조용한 실패(구독 stale vs 검출0) 가시화. 0=끔.
        self.declare_parameter('heartbeat_sec', 2.0)

        dict_name = self.get_parameter('marker_dict').value
        self.marker_size = float(self.get_parameter('marker_size_m').value)
        self.target_id = int(self.get_parameter('target_marker_id').value)
        rate = float(self.get_parameter('process_rate_hz').value)
        self.min_period = (1.0 / rate) if rate > 0.0 else 0.0
        self.publish_debug = bool(self.get_parameter('publish_debug').value)
        self.fallback_frame = self.get_parameter('camera_optical_frame').value
        self.use_compressed = bool(self.get_parameter('use_compressed').value)
        self.heartbeat_sec = float(self.get_parameter('heartbeat_sec').value)

        # ── ArUco 사전 + 디텍터 (1회 생성, 재사용) ─────────────────
        # OpenCV 4.7+ = ArucoDetector 객체 API / 4.6 = 자유함수 API.
        # Ubuntu 24.04(ROS Jazzy) apt python3-opencv는 4.6이라 구 API가 기본.
        # 주의: 4.6에서 DetectorParameters() 직접 생성자는 세그폴트 →
        #       반드시 DetectorParameters_create() 사용.
        dict_id = getattr(cv2.aruco, dict_name, None)
        if dict_id is None:
            self.get_logger().fatal(f"Unknown marker_dict: {dict_name}")
            raise SystemExit(1)
        self._dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
        self._use_new_api = hasattr(cv2.aruco, 'ArucoDetector')
        if self._use_new_api:
            params = cv2.aruco.DetectorParameters()
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self._detector = cv2.aruco.ArucoDetector(self._dictionary, params)
        else:
            self._params = cv2.aruco.DetectorParameters_create()
            self._params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        # ── 마커 로컬 프레임 3D 코너 (ArUco 순서 TL,TR,BR,BL) ──────
        h = self.marker_size / 2.0
        self.obj_points = np.array([
            [-h,  h, 0.0],
            [ h,  h, 0.0],
            [ h, -h, 0.0],
            [-h, -h, 0.0],
        ], dtype=np.float32)

        self.bridge = CvBridge()
        self.K = None
        self.D = None
        self.last_process_ts = None
        self._warned_no_frame = False
        self._got_image = False
        self._got_caminfo = False
        self._rx_count = 0            # 진단: heartbeat 구간 수신 프레임 수
        self._detect_count = 0        # 진단: heartbeat 구간 마커 검출(pose 발행) 수

        # ── 발행 ───────────────────────────────────────────────────
        self.pose_pub = self.create_publisher(PoseStamped, 'detected_dock_pose', 10)
        self.debug_pub = None
        if self.publish_debug:
            self.debug_pub = self.create_publisher(Image, 'aruco/debug_image', 1)

        # ── 구독 (SensorDataQoS = best-effort, 카메라 드라이버와 호환) ─
        self.create_subscription(
            CameraInfo, 'camera_info', self.caminfo_cb, qos_profile_sensor_data)
        # use_compressed=True → 같은 'image_raw' 이름으로 CompressedImage 구독(JPEG 디코드).
        # 구독 이름은 그대로라 remap도 그대로: -r image_raw:=/camera/image_raw/compressed
        img_type = CompressedImage if self.use_compressed else Image
        self.create_subscription(
            img_type, 'image_raw', self.image_cb, qos_profile_sensor_data)

        # ── 진단 heartbeat: 수신/검출 카운트 주기 로그 ──
        if self.heartbeat_sec > 0.0:
            self.create_timer(self.heartbeat_sec, self._heartbeat)

        # ── 런타임 파라미터 콜백: target_marker_id를 검출 필터에 반영 (targeted 도킹) ──
        #   설계상 FSM이 도킹 전 목적지 마커(locations.yaml)를 세팅 → 그 마커만 검출.
        #   -1은 스탠드얼론 폴백(아무 마커). 콜백 없으면 세팅이 무시돼 항상 closest로 돔(버그).
        self.add_on_set_parameters_callback(self._on_set_params)

        self.get_logger().info(
            f"aruco_estimator up | dict={dict_name} size={self.marker_size}m "
            f"target_id={self.target_id} rate={rate}Hz debug={self.publish_debug} "
            f"compressed={self.use_compressed}")

    # ── camera_info: K, D 저장 ─────────────────────────────────────
    def caminfo_cb(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.D = np.array(msg.d, dtype=np.float64)
        if not self._got_caminfo:
            self._got_caminfo = True
            self.get_logger().info("camera_info 수신 — K 설정됨")

    # ── 이미지: 검출 → pose → 발행 ─────────────────────────────────
    def image_cb(self, msg):
        self._rx_count += 1
        if not self._got_image:
            self._got_image = True
            self.get_logger().info("첫 이미지 수신 — image_cb 진입")

        # 1. 레이트 게이트 (detection 전에 드롭 = 가장 쌈).
        #    벽시계 기준 — 이미지 stamp는 0이거나 안 움직일 수 있어(특히 compressed
        #    republish) 스로틀엔 부적합. pose stamp는 아래서 이미지 stamp 유지(TF용).
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.min_period > 0.0 and self.last_process_ts is not None:
            if (now - self.last_process_ts) < self.min_period:
                return

        # 2. K 없으면 solvePnP 불가
        if self.K is None:
            self.get_logger().warn(
                "이미지는 오는데 camera_info(K) 미수신 → 대기 중",
                throttle_duration_sec=2.0)
            return

        # 3. mono8 gray 획득. compressed면 cv_bridge 우회하고 cv2.imdecode로 직접
        #    JPEG→gray (포맷 문자열 의존 없어 견고). raw면 cv_bridge mono8.
        try:
            if self.use_compressed:
                buf = np.frombuffer(msg.data, dtype=np.uint8)
                gray = cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE)
                if gray is None:
                    raise ValueError("cv2.imdecode 실패 (빈/손상 JPEG)")
            else:
                gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"이미지 디코드 실패: {e}", throttle_duration_sec=2.0)
            return

        # 4. 마커 검출 (API 버전 적응)
        if self._use_new_api:
            corners, ids, _ = self._detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self._dictionary, parameters=self._params)
        self.last_process_ts = now  # 처리한 프레임 기준(벽시계, 마커 유무 무관)
        has_markers = ids is not None and len(ids) > 0
        if has_markers:
            ids = ids.flatten()

        # 5. 후보 선정 + pose 추정 → 가장 가까운 마커 선택
        best = None  # (dist, rvec, tvec)
        if has_markers:
            for corner, mid in zip(corners, ids):
                if self.target_id >= 0 and int(mid) != self.target_id:
                    continue
                est = self._estimate_pose(corner)
                if est is None:
                    continue
                rvec, tvec = est
                dist = float(np.linalg.norm(tvec))
                if best is None or dist < best[0]:
                    best = (dist, rvec, tvec)

        # 6~7. pose 발행 (검출 + solvePnP 성공 시에만)
        if best is not None:
            _, rvec, tvec = best
            R, _ = cv2.Rodrigues(rvec)
            qx, qy, qz, qw = rotmat_to_quat(R)
            pose = PoseStamped()
            pose.header.stamp = msg.header.stamp
            frame_id = msg.header.frame_id or self.fallback_frame
            if not frame_id and not self._warned_no_frame:
                self.get_logger().warn(
                    "이미지 frame_id 비어있고 camera_optical_frame 파라미터도 없음 "
                    "→ TF 변환 불가. 카메라 드라이버 frame_id 확인 필요.")
                self._warned_no_frame = True
            pose.header.frame_id = frame_id
            # tvec은 (3,1) → flatten. numpy 2.x는 1원소 1차원 배열의 float()도 TypeError.
            t = tvec.reshape(-1)
            pose.pose.position.x = float(t[0])
            pose.pose.position.y = float(t[1])
            pose.pose.position.z = float(t[2])
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            self.pose_pub.publish(pose)
            self._detect_count += 1

        # 8. 디버그 이미지 — 매 처리 프레임 발행(마커 있으면 외곽선·축 표시).
        #    검출 여부와 무관하게 라이브 피드를 보여줘 브링업/디버깅을 쉽게 함.
        if self.debug_pub is not None:
            dbg = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            if has_markers:
                cv2.aruco.drawDetectedMarkers(dbg, corners, ids.reshape(-1, 1))
            if best is not None:
                cv2.drawFrameAxes(dbg, self.K, self.D, best[1], best[2],
                                  self.marker_size * 0.5)
            dbg_msg = self.bridge.cv2_to_imgmsg(dbg, encoding='bgr8')
            dbg_msg.header = msg.header
            self.debug_pub.publish(dbg_msg)

    def _estimate_pose(self, corner):
        """단일 마커 solvePnP(IPPE_SQUARE).

        IPPE_SQUARE는 평면 정사각의 flip 모호성으로 내부적으로 2해를 산출하지만,
        solvePnP는 그중 재투영오차 작은 해 1개를 반환한다(= 우리가 원하던 argmin).
        2해/재투영오차 값을 쓸 계획이 없으므로 solvePnPGeneric 대신 solvePnP 사용.
        (flip이 실물 튜닝서 실제 문제로 관측되면 Generic+시간 disambiguation으로 승격.)
        """
        img_points = corner.reshape(4, 2).astype(np.float32)
        try:
            retval, rvec, tvec = cv2.solvePnP(
                self.obj_points, img_points, self.K, self.D,
                flags=cv2.SOLVEPNP_IPPE_SQUARE)
        except cv2.error as e:
            self.get_logger().warn(f"solvePnP 실패: {e}")
            return None
        if not retval:
            return None
        return rvec, tvec

    def _on_set_params(self, params):
        """target_marker_id 런타임 변경 반영 (-1=아무 마커, N=해당 ID만)."""
        for p in params:
            if p.name == 'target_marker_id':
                self.target_id = int(p.value)
                self.get_logger().info(f"target_marker_id → {self.target_id}"
                                       + (" (타겟 해제)" if self.target_id < 0 else ""))
        return SetParametersResult(successful=True)

    def _heartbeat(self):
        """진단: heartbeat 구간 수신/검출 카운트 로그. 수신 0이면 구독 stale 경고."""
        rx, det = self._rx_count, self._detect_count
        self._rx_count = 0
        self._detect_count = 0
        if rx == 0:
            self.get_logger().warn(
                f"최근 {self.heartbeat_sec:.0f}s 이미지 수신 0 → 카메라 구독 끊김 의심 "
                f"(카메라 재기동됐으면 estimator 재시작 필요)")
        else:
            self.get_logger().info(
                f"[hb] 최근 {self.heartbeat_sec:.0f}s: 수신 {rx} / 검출 {det} (target_id={self.target_id})")


def main(args=None):
    rclpy.init(args=args)
    node = ArucoEstimatorNode()
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
