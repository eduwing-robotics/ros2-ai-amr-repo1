#!/usr/bin/env python3
"""로봇 FSM — 임무 실행기 (최종 형태).

/assignment(JSON) 수신 → locations.yaml 매핑 → Traffic Manager 경로 주행
→ 도킹/포크(스텁, 팀원 인터페이스 확정 시 교체) → /task_report 보고.

상태: IDLE → TO_SOURCE → PICK → TO_TARGET(적재) → PLACE → RETURNING → IDLE
  - PLACE 완료: target_done 보고 + 즉시 idle 발행 → fleet이 pending 배정하면
    홈 안 가고 현재 위치에서 TO_SOURCE 재진입 (임무 체이닝)
  - RETURNING 중에도 발행 상태는 idle(배정 가능) — 배정 오면 진행 중인
    세그먼트만 끝내고 경계(노드 위)에서 새 임무로 전환 (nav 취소 없음)
  - 외부 발행 상태는 busy/idle/error 3종만 (fleet 배정 판단용, 내부 상태와 분리)

토픽 (모두 로봇 도메인 — domain_bridge가 서버(12)와 중계):
  구독  /assignment            {"robot_id","source","target"} (location_id)
  발행  /task_report           {"robot_id","event": "source_arrived"|"target_done"}
  발행  /robot_status          "idle"|"busy"|"error" (1Hz, 브릿지가 /AMR_x/…로 remap)
  기존  /traffic/request|response|pose, /navigate_to_pose, /initialpose, /amcl_pose

실행 (로봇에서, 각자 도메인):
    ros2 run robot_fsm robot_fsm --ros-args -p robot:=AMR_1
사용:
    (기동 시 자동으로 자기 홈에 AMCL 초기화 — 로봇을 홈에 놓고 켜면 됨)
    fsm> init N13 E      # 홈이 아닌 곳에서 시작할 때만 수동 초기화
    fsm> init            # 홈으로 재초기화
    fsm> N22 [laden]     # (디버그) 노드 주행만 — fleet 없이 traffic 테스트용 (go 생략 가능)
    fsm> status / reset / q
※ 자동 초기화 끄기: --ros-args -p auto_init:=false
"""
import json
import math
import os
import queue
import sys
import threading
import time

import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

from robot_fsm import fork

_DIR_YAW = {'E': 0.0, 'N': math.pi / 2, 'W': math.pi, 'S': -math.pi / 2}

_DOMAIN_ROBOT = {30: 'AMR_1', 31: 'AMR_2'}   # ROS_DOMAIN_ID → 로봇 이름

# yaw 톨러런스: 통로 = don't-care (xy만 도착 판정, 다음 goal이 알아서 회전),
# dock 방향 회전 goal에서만 정밀. nav2_params 기본값도 YAW_FREE로 맞춰둠.
# YAW_DOCK 0.1 rad ≈ ±5.7° — 마커가 도킹 카메라 화각에 들어오면 충분,
# 이후 정밀 정렬은 도킹 서버(마커 기반)가 담당. 실기 확인값.
YAW_FREE = 3.14
YAW_DOCK = 0.1

# PLACE 완료 후 체이닝 배정 대기 시간(초) — 이 안에 /assignment 오면 홈 안 감
CHAIN_WAIT = 1.5


class RobotFSM(Node):
    def __init__(self):
        super().__init__('robot_fsm')
        # 로봇 이름 = ROS_DOMAIN_ID 자동 결정 (30→AMR_1, 31→AMR_2).
        # -p robot:=AMR_x 로 명시하면 그게 우선.
        self.declare_parameter('robot', '')
        param_robot = self.get_parameter('robot').get_parameter_value().string_value
        if param_robot:
            self.robot = param_robot
        else:
            domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
            self.robot = _DOMAIN_ROBOT.get(domain, 'AMR_1')

        # 그래프 노드 좌표 (traffic_manager share)
        nodes_file = os.path.join(
            get_package_share_directory('traffic_manager'), 'graph', 'nodes.yaml')
        with open(nodes_file) as f:
            self.nodes = yaml.safe_load(f)['nodes']

        # location_id → {node, level, dock} + 로봇별 홈 (robot_fsm share)
        loc_file = os.path.join(
            get_package_share_directory('robot_fsm'), 'config', 'locations.yaml')
        with open(loc_file) as f:
            loc = yaml.safe_load(f)
        self.locations = loc['locations']
        self.home = loc['homes'][self.robot]

        # ── nav2 / AMCL ─────────────────────────────────────
        self._ac = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)  # 수렴 회전용
        self._init_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, 10)
        self._param_cli = self.create_client(
            SetParameters, '/controller_server/set_parameters')

        # ── traffic ─────────────────────────────────────────
        self._req_pub = self.create_publisher(String, '/traffic/request', 10)
        self.create_subscription(
            String, f'/traffic/response/{self.robot}', self._on_response, 10)
        self._pose_pub = self.create_publisher(String, '/traffic/pose', 10)
        self._last_pose_pub = 0.0

        # ── fleet ───────────────────────────────────────────
        self.create_subscription(String, '/assignment', self._on_assignment, 10)
        self._report_pub = self.create_publisher(String, '/task_report', 10)
        self._status_pub = self.create_publisher(String, '/robot_status', 10)
        self.create_timer(1.0, self._publish_status)

        self._resp = None
        self._resp_ev = threading.Event()
        self.current = None              # 현재(마지막 도착) 노드
        self._queue = queue.Queue()      # 수신된 /assignment 대기열
        self._state = 'IDLE'             # IDLE/TO_SOURCE/PICK/TO_TARGET/PLACE/RETURNING/MANUAL/ERROR
        self.get_logger().info(f'[{self.robot}] FSM 시작 (홈: {self.home["node"]})')

        # 기본 자동 초기화: 도메인 ID → 로봇 이름 → locations.yaml 홈으로 AMCL 초기화
        # (30→AMR_1→N25, 31→AMR_2→N20). 홈이 아닌 곳에서 시작할 때만
        # -p auto_init:=false 로 끄고 수동 init.
        self.declare_parameter('auto_init', True)
        if self.get_parameter('auto_init').get_parameter_value().bool_value:
            threading.Thread(target=self._auto_init, daemon=True).start()

    def _auto_init(self):
        """AMCL이 /initialpose를 구독할 때까지 기다렸다가 홈 위치로 초기화.

        기한 없이 대기 — nav2를 FSM보다 늦게 켜도 됨 (기동 순서 무관).
        AMCL 미초기화 상태로 임무를 받으면 planner가 (0,0)에서 출발하려다
        costmap timeout으로 전부 실패하므로, 초기화가 안전의 전제.
        """
        waited = 0.0
        while rclpy.ok() and self.current is None:
            if self._init_pub.get_subscription_count() > 0:
                time.sleep(1.0)          # AMCL 준비 여유
                # 초기화 후 360° 수렴 회전 — 파티클을 시작부터 조여둠
                self.set_initial_pose(self.home['node'], self.home['dock'],
                                      spin=True)
                return
            time.sleep(0.5)
            waited += 0.5
            if waited % 15 < 0.5:        # 15초마다 안내
                self.get_logger().warn(
                    'auto_init 대기 중: AMCL(/initialpose 구독자) 미기동 — nav2를 켜면 자동 초기화됩니다')

    # ── 상태 발행 ─────────────────────────────────────────────

    def _ext_status(self):
        """외부 발행 상태 — fleet은 idle만 배정 대상으로 봄.
        RETURNING도 idle: 복귀 중 배정 가능 (세그먼트 경계에서 전환)."""
        if self._state == 'ERROR':
            return 'error'
        if self._state in ('IDLE', 'RETURNING'):
            return 'idle'
        return 'busy'

    def _publish_status(self):
        self._status_pub.publish(String(data=self._ext_status()))

    def _report(self, event):
        payload = json.dumps({'robot_id': self.robot, 'event': event})
        self._report_pub.publish(String(data=payload))
        self.get_logger().info(f'[{self.robot}] task_report: {event}')

    # ── 콜백 ─────────────────────────────────────────────────

    def _on_assignment(self, msg):
        try:
            a = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if a.get('robot_id') != self.robot:
            return
        if a.get('source') not in self.locations or a.get('target') not in self.locations:
            self.get_logger().error(f'알 수 없는 location_id: {msg.data}')
            return
        self._queue.put(a)
        self.get_logger().info(f'[{self.robot}] 배정 수신: {a["source"]} → {a["target"]}')

    def _on_amcl(self, msg):
        """amcl 위치 → traffic에 저주파(약 3Hz) 전달. WiFi 부하 최소화."""
        now = time.time()
        if now - self._last_pose_pub < 0.3:
            return
        self._last_pose_pub = now
        p = msg.pose.pose.position
        out = String()
        out.data = json.dumps({'robot': self.robot, 'x': p.x, 'y': p.y})
        self._pose_pub.publish(out)

    def _on_response(self, msg):
        self._resp = json.loads(msg.data)
        self._resp_ev.set()

    # ── traffic 요청/응답 ─────────────────────────────────────

    def _transact(self, payload, timeout=10.0):
        """요청 발행 → 응답 대기. 발행 전에 이벤트를 clear해 응답 유실 방지."""
        payload['robot'] = self.robot
        self._resp_ev.clear()
        msg = String()
        msg.data = json.dumps(payload)
        self._req_pub.publish(msg)
        if not self._resp_ev.wait(timeout):
            return None
        return self._resp

    # ── 임무 루프 (전용 스레드) ───────────────────────────────

    def mission_loop(self):
        """배정 대기 → 임무 실행 → 체이닝 or 홈 복귀. 데몬 스레드로 실행."""
        nxt = None
        while rclpy.ok():
            if self._state == 'ERROR':
                nxt = None
                time.sleep(0.5)          # reset 명령 대기
                continue
            if nxt is None:
                try:
                    nxt = self._queue.get(timeout=0.5)
                except queue.Empty:
                    continue
            if self.current is None:
                self.get_logger().error('init 전 임무 수신 — 무시 (init 후 재배정 필요)')
                nxt = None
                continue

            ok = self._run_mission(nxt)
            nxt = None
            if not ok:
                self._set_error('임무 실패')
                continue

            # 체이닝: PLACE 직후 idle 발행됨 → 짧게 배정 대기, 오면 홈 안 감
            try:
                nxt = self._queue.get(timeout=CHAIN_WAIT)
                continue
            except queue.Empty:
                pass
            nxt = self._return_home()    # 복귀 중 배정 오면 그 임무 반환

    def _run_mission(self, a):
        src, tgt = self.locations[a['source']], self.locations[a['target']]
        self.get_logger().info(f'[{self.robot}] 임무 시작: {a["source"]} → {a["target"]}')

        self._state = 'TO_SOURCE'
        if self._travel(src['node'], laden=False) != 'done':
            return False
        self._state = 'PICK'
        if not self._work(src, pick=True):
            return False
        self._report('source_arrived')

        self._state = 'TO_TARGET'
        if self._travel(tgt['node'], laden=True) != 'done':
            return False
        self._state = 'PLACE'
        if not self._work(tgt, pick=False):
            return False
        self._report('target_done')

        self._state = 'IDLE'
        self._publish_status()           # 1Hz 타이머 안 기다리고 즉시 idle 발행
                                         # → fleet이 CHAIN_WAIT(1.5s) 안에 배정 가능
        self.get_logger().info(f'[{self.robot}] 임무 완료: {a["source"]} → {a["target"]}')
        return True

    def _return_home(self):
        """홈 복귀. 배정 오면 세그먼트 경계에서 중단하고 그 임무를 반환."""
        if self.current == self.home['node']:
            self._state = 'IDLE'
            return None
        self._state = 'RETURNING'
        res = self._travel(self.home['node'], laden=False, preempt=True)
        if res == 'preempted':
            try:
                return self._queue.get_nowait()
            except queue.Empty:
                return None
        if res == 'failed':
            self._set_error('홈 복귀 실패')
            return None
        self._state = 'IDLE'
        self.get_logger().info(f'[{self.robot}] 홈({self.home["node"]}) 복귀 완료')
        return None

    def _set_error(self, why):
        self._state = 'ERROR'
        self.get_logger().error(
            f'[{self.robot}] ERROR: {why} — 배정 제외됨, 조치 후 fsm> reset')

    # ── 주행 ─────────────────────────────────────────────────

    def _travel(self, goal_node, laden=False, preempt=False):
        """현재 노드 → goal_node. 경로 요청(거절 시 재시도) + 세그먼트 주행.

        preempt=True(RETURNING): 배정 수신 시 세그먼트 경계에서 'preempted'.
        반환: 'done' | 'preempted' | 'failed'
        """
        while rclpy.ok():                # 경로 요청 — 시작 노드 점유 등이면 재시도
            resp = self._transact({'type': 'route', 'start': self.current,
                                   'goal': goal_node, 'laden': laden})
            if resp and resp.get('type') == 'route':
                self.get_logger().info(
                    f'[{self.robot}] 경로: {"→".join(resp["route"])}')
                break
            if preempt and not self._queue.empty():
                return 'preempted'
            self.get_logger().info(f'[{self.robot}] 경로 대기({goal_node}) — 재시도')
            time.sleep(1.0)

        while rclpy.ok():
            if preempt and not self._queue.empty():
                return 'preempted'       # 세그먼트 경계 = 노드 위에서만 전환
            resp = self._transact({'type': 'segment'})
            t = resp.get('type') if resp else None
            if t == 'segment':
                if not self._drive_segment(resp['nodes']):
                    return 'failed'
            elif t == 'wait':
                time.sleep(1.0)
            elif t == 'reroute':
                self.get_logger().info(
                    f'[{self.robot}] 우회: {"→".join(resp["route"])}')
            elif t == 'done':
                return 'done'
            else:
                self.get_logger().warn(f'[{self.robot}] traffic 응답 이상: {resp}')
                return 'failed'
        return 'failed'

    def _drive_segment(self, seg_nodes):
        if not self._ac.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('nav2 액션서버 없음')
            return False
        # 직선 run은 끝점 goal 하나로 쭉 (중간 노드 통과, 안 멈춤).
        # 중간 노드 release는 traffic이 amcl 위치로 처리 (로봇은 arrive 안 보냄).
        end = seg_nodes[-1]
        d = self._direction(self.current, seg_nodes[0])
        nd = self.nodes[end]
        if not self._nav_to(nd['x'], nd['y'], _DIR_YAW[d]):
            self.get_logger().warn(f'[{self.robot}] {end} 주행 실패')
            return False
        self.current = end
        return True

    def _direction(self, a, b):
        ax, ay = self.nodes[a]['x'], self.nodes[a]['y']
        bx, by = self.nodes[b]['x'], self.nodes[b]['y']
        dx, dy = bx - ax, by - ay
        if abs(dx) >= abs(dy):
            return 'E' if dx > 0 else 'W'
        return 'N' if dy > 0 else 'S'

    def _nav_to(self, x, y, yaw):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        gh = self._await_future(self._ac.send_goal_async(goal))
        if not gh or not gh.accepted:
            return False
        res = self._await_future(gh.get_result_async())
        return res is not None and res.status == GoalStatus.STATUS_SUCCEEDED

    @staticmethod
    def _await_future(fut, timeout=120.0):
        ev = threading.Event()
        fut.add_done_callback(lambda _f: ev.set())
        if not ev.wait(timeout):
            return None
        return fut.result()

    # ── 작업 (도킹 방향 회전 + 포크/도킹 시퀀스) ──────────────

    def _work(self, loc, pick):
        """픽/플레이스 — fork.py 시퀀스 테이블 기반, 임무 타입 무관 단일 흐름.

        L1은 after_back이 빈 리스트라 명령 자체가 안 나감 (개수 차이 흡수).
        """
        seq = fork.pick_sequence(loc['level']) if pick else fork.place_sequence(loc['level'])

        # 도킹 방향으로 제자리 회전 — 이 goal만 yaw 정밀(0.08)
        nd = self.nodes[loc['node']]
        self._set_yaw_tolerance(YAW_DOCK)
        ok = self._nav_to(nd['x'], nd['y'], _DIR_YAW[loc['dock']])
        self._set_yaw_tolerance(YAW_FREE)
        if not ok:
            self.get_logger().warn(f'[{self.robot}] {loc["node"]} 도킹 방향 회전 실패')
            return False

        for h in seq['before_dock']:
            self._fork(h)
        if not self._dock():
            return False
        for h in seq['after_dock']:
            self._fork(h)
        if not self._undock():
            return False
        for h in seq['after_back']:
            self._fork(h)
        return True

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

    # ── 스텁: 팀원 인터페이스 확정 시 교체 ─────────────────────

    def _fork(self, level):
        """[스텁] micro-ROS 포크 명령 — 절대 높이 level(0~4)로."""
        self.get_logger().info(
            f'[{self.robot}] [스텁] 포크 → 높이 {level} (스텝 {fork.steps(level)})')
        time.sleep(0.5)
        return True

    def _dock(self):
        """[스텁] 도킹 모듈 호출 (마커 기반 PBVS 정밀 접근).

        서비스/액션 여부는 도킹 모듈 팀원과 협의 후 확정 — 어느 쪽이든
        이 함수 몸통만 교체 (성공 bool 반환 계약은 동일).
        """
        self.get_logger().info(f'[{self.robot}] [스텁] 도킹')
        time.sleep(1.0)
        return True

    def _undock(self):
        """[스텁] 언도킹 모듈 호출 (후진 back → 노드 위 복귀)."""
        self.get_logger().info(f'[{self.robot}] [스텁] 언도킹')
        time.sleep(1.0)
        return True

    # ── 초기화 (CLI) ─────────────────────────────────────────

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
            self._state = 'MANUAL'       # busy 발행 → 회전 중 fleet 배정 차단
            try:
                time.sleep(1.0)          # AMCL 파티클 리셋 반영 여유
                if self._spin_converge():
                    self.get_logger().info(f'[{self.robot}] 초기 수렴 회전(360°) 완료')
            finally:
                self._state = prev
        self.current = name
        self.get_logger().info(f'[{self.robot}] 초기 위치 {name} 방향 {facing}')

    SPIN_VEL = 0.4                       # 수렴 회전 각속도 (rad/s)

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

    # ── 디버그: 수동 노드 주행 (fleet 없이 traffic 테스트) ─────

    def manual_go(self, goal, laden=False):
        if self.current is None:
            print('먼저 init 필요')
            return
        if self._state != 'IDLE':
            print(f'현재 {self._state} — IDLE에서만 수동 주행 가능')
            return
        self._state = 'MANUAL'           # busy 발행 → fleet 배정 차단
        try:
            res = self._travel(goal.upper(), laden=laden)
            print(f'수동 주행 결과: {res}')
        finally:
            self._state = 'IDLE'


def main():
    rclpy.init()
    node = RobotFSM()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    threading.Thread(target=node.mission_loop, daemon=True).start()

    # 터미널 없이 기동(launch/자동 기동 = 최종 운영)이면 CLI 없이 상주.
    # CLI는 개발·정비용 — 터미널에서 직접 실행했을 때만 열림.
    if not sys.stdin.isatty():
        node.get_logger().info('headless 모드 — CLI 없이 임무 대기')
        try:
            while rclpy.ok():
                time.sleep(1.0)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
        return

    try:
        print(f"[{node.robot}] 명령: init(=홈) / init N13 E / go N1 [laden] / status / reset / q")
        while rclpy.ok():
            try:
                cmd = input(f'{node.robot}> ').strip()
            except EOFError:
                break
            if cmd.lower() in ('q', 'quit', 'exit'):
                break
            if not cmd:
                continue
            p = cmd.split()
            c = p[0].lower()
            if c == 'init' and len(p) == 1:
                # 인자 없으면 자기 홈 (locations.yaml homes)
                node.set_initial_pose(node.home['node'], node.home['dock'])
            elif c == 'init' and len(p) >= 3:
                node.set_initial_pose(p[1], p[2])
            elif c == 'go' and len(p) >= 2:
                laden = len(p) >= 3 and p[2].lower() == 'laden'
                node.manual_go(p[1], laden)
            elif p[0].upper() in node.nodes:
                # 노드명만 입력해도 주행 (예: 'N22' / 'N22 laden')
                laden = len(p) >= 2 and p[1].lower() == 'laden'
                node.manual_go(p[0], laden)
            elif c == 'spin':
                # 수동 360° 수렴 회전 (재초기화 후 파티클 조이기용)
                if node._state != 'IDLE':
                    print(f'현재 {node._state} — IDLE에서만 가능')
                else:
                    node._state = 'MANUAL'
                    try:
                        print('수렴 회전 성공' if node._spin_converge() else '수렴 회전 실패')
                    finally:
                        node._state = 'IDLE'
            elif c == 'status':
                print(f'state={node._state} current={node.current} '
                      f'대기임무={node._queue.qsize()}')
            elif c == 'reset':
                if node._state == 'ERROR':
                    node._state = 'IDLE'
                    print('IDLE 복귀 (위치 어긋났으면 init 재실행)')
                else:
                    print(f'ERROR 아님 (현재 {node._state})')
            else:
                print('사용: init N25 N21 / go N1 [laden] / status / reset / q')
    finally:
        node.destroy_node()
        if rclpy.ok():                 # Ctrl-C 시그널이 이미 shutdown했으면 생략
            rclpy.shutdown()


if __name__ == '__main__':
    main()
