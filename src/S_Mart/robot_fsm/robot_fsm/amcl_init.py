"""AMCL 초기화 믹스인 — 초기 위치 발행 + 360° 수렴 회전.

RobotFSM에 믹스인으로 결합. 로봇을 "임무 가능한 상태"로 만드는 초기화 전담:
기동 시 _auto_init이 호출하고, 위치가 어긋나면 CLI(fsm> init / spin)로 재실행.
"""
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped

from robot_fsm.states import S
from robot_fsm.travel import _DIR_YAW


class AmclInitMixin:

    SPIN_VEL = 0.4                       # 수렴 회전 각속도 (rad/s)

    def set_initial_pose(self, name, facing, spin=False):
        """AMCL 초기 위치 발행. spin=True면 발행 후 360° 수렴 회전.

        주의: 회전 '후에' 재발행하면 파티클이 리셋돼 수렴이 무효 —
        반드시 발행 → 회전 → current 세팅 순서 유지.
        """
        name = name.upper()
        nd = self.nodes[name]
        facing = facing.upper()
        # facing = 방향(E/W/N/S) 또는 노드명(그 노드를 바라봄)
        if facing in _DIR_YAW:
            yaw = _DIR_YAW[facing]
        elif facing in self.nodes:
            t = self.nodes[facing]
            yaw = math.atan2(t['y'] - nd['y'], t['x'] - nd['x'])
        else:
            self.get_logger().error(f'알 수 없는 방향/노드: {facing} (E/W/N/S 또는 노드명)')
            return
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(nd['x'])
        msg.pose.pose.position.y = float(nd['y'])
        msg.pose.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.pose.orientation.w = math.cos(yaw / 2)
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        for _ in range(3):
            self._init_pub.publish(msg)
            time.sleep(0.2)   # 백그라운드 executor가 spin 중 → sleep으로만 간격
        if spin:
            prev = self._state
            self._set_state(S.MANUAL)    # busy 발행 → 회전 중 fleet 배정 차단
            try:
                time.sleep(1.0)          # AMCL 파티클 리셋 반영 여유
                if self._spin_converge():
                    self.get_logger().info(f'[{self.robot}] 초기 수렴 회전(360°) 완료')
            finally:
                self._set_state(prev)
        self.current = name
        self.get_logger().info(f'[{self.robot}] 초기 위치 {name} 방향 {facing}')

    def _spin_converge(self):
        """제자리 360° 회전 — cmd_vel 직접 발행 (AMCL 파티클 수렴용).

        nav2 액션 없이 시간 기반 개루프: 2π/속도 동안 회전 명령 후 정지.
        정확히 360°일 필요 없음(수렴 목적) — 홈 위 제자리 회전이라 안전.
        """
        duration = 2 * math.pi / self.SPIN_VEL      # ≈ 15.7초
        msg = TwistStamped()
        msg.twist.angular.z = self.SPIN_VEL
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < duration:
            msg.header.stamp = self.get_clock().now().to_msg()
            self._cmd_pub.publish(msg)
            time.sleep(0.05)                        # 20Hz
        # 정지 (확실히 여러 번)
        msg.twist.angular.z = 0.0
        for _ in range(5):
            msg.header.stamp = self.get_clock().now().to_msg()
            self._cmd_pub.publish(msg)
            time.sleep(0.05)
        return True
