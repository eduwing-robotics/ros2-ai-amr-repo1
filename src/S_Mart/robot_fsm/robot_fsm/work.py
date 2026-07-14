"""작업 믹스인 — 픽/플레이스 시퀀스 (도킹 방향 회전 + 도킹 서비스 + 포크).

RobotFSM에 믹스인으로 결합.
담당 경계: 도킹(aruco_docking 서비스 호출부) = 도킹 팀원,
          _fork(/fork_cmd·/fork_state ESP32 핸드셰이크) = 포크 담당 동료.
"""
import time

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import Int32
from std_srvs.srv import Trigger

from robot_fsm import fork
from robot_fsm.travel import _DIR_YAW, YAW_DOCK, YAW_FREE

# 포크 핸드셰이크 상수 (/fork_state 값 — 0=미사용, IDLE 폐기)
FORK_TIMEOUT = 90.0                              # AT_POSITION 대기 상한(초).
# 최악 이동 2↔4=23400스텝. 28BYJ-48 @12RPM=409.6스텝/s → ~57s. 마진 포함 90s
# (RPM을 10으로 낮춰도 ~69s라 커버). 모터 속도/스텝수 바뀌면 재계산.
_FORK_MOVING, _FORK_AT, _FORK_ERROR = 1, 2, 3


class WorkMixin:

    # ── 작업 (도킹 방향 회전 + 포크/도킹 시퀀스) ──────────────

    def _work(self, loc, pick):
        """픽/플레이스 — fork.py 시퀀스 테이블 기반, 임무 타입 무관 단일 흐름.

        L1은 after_back이 빈 리스트라 명령 자체가 안 나감 (개수 차이 흡수).
        """
        seq = fork.pick_sequence(loc['level']) if pick else fork.place_sequence(loc['level'])

        # 도킹 방향으로 제자리 회전 — 이 goal만 yaw 정밀
        if not self._face_dock(loc['node'], loc['dock']):
            self.get_logger().warn(f'[{self.robot}] {loc["node"]} 도킹 방향 회전 실패')
            return False

        for h in seq['before_dock']:
            if not self._fork(h):
                return False
        if not self._work_dock(loc.get('marker')):
            return False
        for h in seq['after_dock']:
            if not self._fork(h):
                return False
        if not self._undock():
            return False
        for h in seq['after_back']:
            if not self._fork(h):
                return False
        return True

    def _face_dock(self, node_name, direction):
        """도킹 방향 제자리 회전 — 이 goal만 yaw 정밀(YAW_DOCK), 끝나면 복원."""
        nd = self.nodes[node_name]
        self._set_yaw_tolerance(YAW_DOCK)
        ok = self._nav_to(nd['x'], nd['y'], _DIR_YAW[direction])
        self._set_yaw_tolerance(YAW_FREE)
        return ok

    def _set_yaw_tolerance(self, tol):
        """controller_server goal_checker.yaw_goal_tolerance 런타임 변경."""
        if not self._param_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('controller_server 파라미터 서비스 없음 — 톨러런스 유지')
            return
        req = SetParameters.Request()
        req.parameters = [Parameter(
            name='goal_checker.yaw_goal_tolerance',
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                 double_value=float(tol)))]
        self._await_future(self._param_cli.call_async(req), timeout=3.0)

    # ── 포크 (ESP32 micro-ROS, /fork_cmd·/fork_state 핸드셰이크) ──

    def _fork(self, level):
        """포크 높이 level(1~4) 명령 → AT_POSITION까지 블로킹. 실패/타임아웃 시 False.

        핸드셰이크(= Action 생명주기를 토픽으로): /fork_cmd 발행 → MOVING 목격
        → AT_POSITION → True. ESP32는 명령마다 MOVING을 반드시 한 번 찍어(델타 0이어도)
        이전 명령의 낡은 AT_POSITION과 구분되게 한다. FSM은 절대 높이만 보내고,
        현재 스텝/델타 계산은 ESP32(cur_step)가 담당한다.

        스레드: mission_loop에서 블록, _on_fork_state는 executor에서 갱신
        (travel의 _transact/_on_response와 동일 패턴 — 락 불필요).
        """
        self._fork_moving_seen = False
        self._fork_error = False
        self._fork_last = None
        self._fork_ev.clear()
        self._fork_pub.publish(Int32(data=int(level)))
        self.get_logger().info(f'[{self.robot}] 포크 → 높이 {level} 명령, 완료 대기')
        deadline = time.time() + FORK_TIMEOUT
        while rclpy.ok() and time.time() < deadline:
            self._fork_ev.wait(timeout=0.5)
            self._fork_ev.clear()
            if self._fork_error:
                self.get_logger().error(f'[{self.robot}] 포크 ERROR 보고 (높이 {level})')
                return False
            if self._fork_moving_seen and self._fork_last == _FORK_AT:
                self.get_logger().info(f'[{self.robot}] 포크 높이 {level} 도달')
                return True
        self.get_logger().error(
            f'[{self.robot}] 포크 타임아웃 {FORK_TIMEOUT}s (높이 {level}) — ESP32/Agent 확인')
        return False

    def _on_fork_state(self, msg):
        """/fork_state 콜백 (executor 스레드) — 핸드셰이크 플래그 갱신.

        1=MOVING(목격 기록) / 2=AT_POSITION / 3=ERROR. _fork가 ev로 대기 중.
        """
        v = int(msg.data)
        self._fork_last = v
        if v == _FORK_MOVING:
            self._fork_moving_seen = True
        elif v == _FORK_ERROR:
            self._fork_error = True
        self._fork_ev.set()

    # ── 도킹 (aruco_docking dock_controller) ──────────────────

    def _work_dock(self, marker_id=None):
        """작업 도킹 — aruco_docking `/start_work_dock` (전진 PBVS 정밀, blocking→bool).

        marker_id 지정 시 estimator target_marker_id를 먼저 세팅(타겟 도킹 —
        엉뚱한 마커 방지). 실패 시 임무 실패로 전파.
        """
        if marker_id is not None:
            self._set_estimator_marker(int(marker_id))
        self.get_logger().info(f'[{self.robot}] 작업 도킹 시작 (marker={marker_id})')
        ok = self._call_trigger(self._work_dock_cli, '/start_work_dock', timeout=90.0)
        if marker_id is not None:
            self._set_estimator_marker(-1)   # 도킹 끝 → 타겟 해제 (평소=아무 마커)
        return ok

    def _home_dock(self):
        """홈 후진 도킹 — 도크 방향 회전 후 `/start_home_dock` (정렬→180°→후진 안착).

        홈은 주차라 정밀 비필요. 실패해도 ERROR 아닌 warn(복귀 자체는 완료).
        """
        if not self._face_dock(self.home['node'], self.home['dock']):
            self.get_logger().warn(f'[{self.robot}] 홈 도크 방향 회전 실패')
            return False
        marker = self.home.get('marker')
        if marker is not None:
            self._set_estimator_marker(int(marker))
        self.get_logger().info(f'[{self.robot}] 홈 도킹 시작 (marker={marker})')
        ok = self._call_trigger(self._home_dock_cli, '/start_home_dock', timeout=90.0)
        if marker is not None:
            self._set_estimator_marker(-1)   # 도킹 끝 → 타겟 해제
        return ok

    def _undock(self):
        """언도킹 — aruco_docking `/start_undock` (odom 후진 → 노드 복귀)."""
        self.get_logger().info(f'[{self.robot}] 언도킹 시작')
        return self._call_trigger(self._undock_cli, '/start_undock', timeout=40.0)

    def _call_trigger(self, cli, name, timeout):
        """Trigger 서비스 blocking 호출 → success bool. 없거나 타임아웃이면 False."""
        if not cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'[{self.robot}] {name} 서비스 없음 (dock_controller 미기동?)')
            return False
        res = self._await_future(cli.call_async(Trigger.Request()), timeout=timeout)
        if res is None:
            self.get_logger().error(f'[{self.robot}] {name} 응답 없음(타임아웃)')
            return False
        if not res.success:
            self.get_logger().warn(f'[{self.robot}] {name} 실패: {res.message}')
        return bool(res.success)

    def _set_estimator_marker(self, marker_id):
        """estimator target_marker_id 세팅(best-effort). 실패해도 진행(closest 마커 사용)."""
        if not self._estimator_param_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                f'[{self.robot}] estimator param 서비스 없음 → target_marker_id 미설정(closest)')
            return
        req = SetParameters.Request()
        req.parameters = [Parameter(
            name='target_marker_id',
            value=ParameterValue(type=ParameterType.PARAMETER_INTEGER,
                                 integer_value=int(marker_id)))]
        self._await_future(self._estimator_param_cli.call_async(req), timeout=3.0)
        self.get_logger().info(f'[{self.robot}] estimator target_marker_id={marker_id}')
