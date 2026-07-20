#!/usr/bin/env python3
"""마커 법선 probe — /detected_dock_pose(ArUco 자세)에서 법선각 e_θ 추출.

라이다 probe(lidar_wall_normal_probe.py)와 **동일한 비교**를 위해:
  - dock_controller._extract_errors 와 똑같은 법선 공식 사용
  - 같은 e_θ 정의(수직이면 0), 같은 σ·odom_yaw 출력 포맷
목적: 마커 법선의 노이즈(σ)·회전추종이 라이다(σ0.17°, slope−1)와 얼마나 다른지.

실행 (estimator가 떠 있어야 함 = dock launch 켠 상태에서 별 터미널):
    ROS_DOMAIN_ID=31 python3 ~/marker_normal_probe.py
"""
import math
import statistics

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs  # noqa: F401  (PoseStamped TF 변환 등록)

BASE = 'base_link'


def _yaw_deg(q):
    return math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


class Probe(Node):
    def __init__(self):
        super().__init__('marker_normal_probe')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(PoseStamped, 'detected_dock_pose', self.cb, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_cb, 10)
        self._buf = []
        self._yaw = None
        self.get_logger().info('마커 법선 probe — /detected_dock_pose 구독 (estimator 필요)')

    def odom_cb(self, o: Odometry):
        self._yaw = _yaw_deg(o.pose.pose.orientation)

    def cb(self, msg: PoseStamped):
        if not msg.header.frame_id:
            return
        try:
            pb = self.tf_buffer.transform(
                msg, BASE, timeout=rclpy.duration.Duration(seconds=0.1))
        except TransformException as e:
            self.get_logger().warn(f'TF {msg.header.frame_id}→{BASE} 실패: {e}',
                                   throttle_duration_sec=2.0)
            return
        p, q = pb.pose.position, pb.pose.orientation
        m_x, m_y = p.x, p.y
        # dock_controller._extract_errors 와 동일: 마커 자세에서 수평 법선 추출
        nx = 2.0 * (q.x * q.z + q.w * q.y)
        ny = 2.0 * (q.y * q.z - q.w * q.x)
        n = math.hypot(nx, ny)
        if n < 1e-9:
            return
        nx, ny = nx / n, ny / n
        if nx * (-m_x) + ny * (-m_y) < 0.0:      # 로봇 쪽으로 부호 통일
            nx, ny = -nx, -ny
        eth = math.degrees(math.atan2(-ny, -nx))  # 수직이면 0 (라이다 e_θ와 동일 정의)

        self._buf.append(eth)
        if len(self._buf) > 200:
            self._buf.pop(0)
        m = statistics.fmean(self._buf)
        sd = statistics.pstdev(self._buf) if len(self._buf) > 1 else 0.0
        yaw_s = f'{self._yaw:+6.2f}°' if self._yaw is not None else '  --  '
        self.get_logger().info(
            f'[marker] e_θ={eth:+6.2f}°  평균={m:+6.2f}° σ={sd:4.2f}°  '
            f'odom_yaw={yaw_s}  depth={math.hypot(m_x, m_y) * 100:4.1f}cm',
            throttle_duration_sec=0.5)


def main():
    rclpy.init()
    n = Probe()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
