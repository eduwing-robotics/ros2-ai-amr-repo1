#!/usr/bin/env python3
"""라이다 벽 법선 측정 프로브.

전방 섹터의 /scan 점들을 직선피팅 → 벽 법선각 γ 산출.
  로봇이 벽에 정면 수직이면 γ ≈ 0.  γ ≠ 0 = 편향 or 로봇이 삐딱.
편향(bias)·노이즈(σ)·회전추종(scale) 검증용. 나중 dock_controller 벽피팅의 원형.

실행 (로봇 도메인에서, 워크스테이션):
    ROS_DOMAIN_ID=30 python3 ~/lidar_wall_normal_probe.py     # AMR_1
    ROS_DOMAIN_ID=31 python3 ~/lidar_wall_normal_probe.py     # AMR_2

γ 정의: 법선을 벽쪽(+x)으로 통일 → γ = atan2(n_y, n_x).
  수직이면 벽점이 (d, ±y분포) → 법선 (1,0) → γ=0.
"""
import math
import statistics

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

SECTOR_DEG = 30.0        # 전방 ±이 각도 섹터만 사용
R_MIN, R_MAX = 0.05, 1.5 # 유효 range(m). 벽 거리에 맞게 조정
EMA_A = 0.3             # 필터 계수 (필터 후 노이즈 확인용)
MIN_PTS = 8             # 이보다 적으면 스킵


class Probe(Node):
    def __init__(self):
        super().__init__('lidar_wall_normal_probe')
        # /scan은 BEST_EFFORT(센서 QoS)로 발행됨 → 반드시 맞춰야 수신됨
        self.create_subscription(LaserScan, '/scan', self.cb, qos_profile_sensor_data)
        # 회전 추종 테스트용 실제 회전각 기준
        self.create_subscription(Odometry, '/odometry/filtered', self._odom_cb, 10)
        self._buf = []
        self._ema = None
        self._yaw = None
        self.get_logger().info(
            f'구독 /scan — 로봇을 벽 정면에 세우고 관측. 전방 ±{SECTOR_DEG:.0f}° 섹터.')

    def _odom_cb(self, o: Odometry):
        q = o.pose.pose.orientation
        self._yaw = math.degrees(
            math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                       1.0 - 2.0 * (q.y * q.y + q.z * q.z)))

    def cb(self, s: LaserScan):
        half = math.radians(SECTOR_DEG)
        pts = []
        for i, r in enumerate(s.ranges):
            if math.isinf(r) or math.isnan(r) or not (R_MIN < r < R_MAX):
                continue
            ang = s.angle_min + i * s.angle_increment
            a = math.atan2(math.sin(ang), math.cos(ang))   # [-pi,pi] 랩 (전방 0 주변 wrap 처리)
            if abs(a) > half:
                continue
            pts.append((r * math.cos(ang), r * math.sin(ang)))

        if len(pts) < MIN_PTS:
            self.get_logger().warn(
                f'전방 섹터 점 {len(pts)}개뿐 — 전방 인덱스가 벽을 안 보거나 거리/섹터 조정 필요',
                throttle_duration_sec=1.0)
            return

        P = np.array(pts)
        c = P.mean(axis=0)
        w, v = np.linalg.eigh(np.cov((P - c).T))
        normal = v[:, 0]                       # 최소 고유값 고유벡터 = 법선
        if normal[0] < 0:                      # 벽쪽(+x)으로 부호 통일
            normal = -normal
        gamma = math.degrees(math.atan2(normal[1], normal[0]))
        resid_mm = float(np.sqrt(max(w[0], 0.0))) * 1000.0   # 법선방향 RMS(피팅 두께)
        dist = float(np.hypot(*c))             # 벽까지 대략 거리

        self._ema = gamma if self._ema is None else EMA_A * gamma + (1 - EMA_A) * self._ema
        self._buf.append(gamma)
        if len(self._buf) > 200:
            self._buf.pop(0)
        m = statistics.fmean(self._buf)
        sd = statistics.pstdev(self._buf) if len(self._buf) > 1 else 0.0

        yaw_s = f'{self._yaw:+6.2f}°' if self._yaw is not None else '  --  '
        self.get_logger().info(
            f'γ(EMA)={self._ema:+6.2f}° (raw{gamma:+6.2f}) σ={sd:4.2f}°  '
            f'odom_yaw={yaw_s}  pts={len(pts):3d} 벽{dist*100:4.1f}cm 잔차{resid_mm:4.1f}mm',
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
