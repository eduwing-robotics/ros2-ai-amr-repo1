#!/usr/bin/env python3
"""로봇 FSM — 임무 실행기 (오케스트레이션).

/assignment(JSON) 수신 → locations.yaml 매핑 → Traffic Manager 경로 주행
→ 도킹/포크 → /task_report 보고.

모듈 구성 (노드는 하나, 파일은 관심사·담당자 경계로 분리):
  robot_fsm.py  이 파일 — 노드/콜백/임무 루프 + CLI·수동 주행 (오케스트레이션)
  states.py     상태 Enum(S)·허용 전이 테이블 — FSM 설계도의 단일 소스
  travel.py     TravelMixin: traffic 트랜잭션 + 세그먼트 주행 (nav2)
  work.py       WorkMixin: 픽/플레이스 (도킹 서비스 + 포크 스텁)
  amcl_init.py  AmclInitMixin: AMCL 초기 위치 발행 + 360° 수렴 회전
  fork.py       포크 높이·시퀀스 데이터

상태: IDLE → TO_SOURCE → PICK → TO_TARGET(적재) → PLACE → RETURNING → IDLE
  - PLACE 완료: target_done 보고 + 즉시 idle 발행 → fleet이 pending 배정하면
    홈 안 가고 현재 위치에서 TO_SOURCE 재진입 (임무 체이닝)
  - RETURNING 중에도 발행 상태는 idle(배정 가능) — 배정 오면 진행 중인
    세그먼트만 끝내고 경계(노드 위)에서 새 임무로 전환 (nav 취소 없음)
  - 외부 발행 상태는 busy/idle/error 3종만 (fleet 배정 판단용, 내부 상태와 분리)
  - 상태 변경은 _set_state() 한 곳으로만 — states.TRANSITIONS 밖이면 경고

토픽 (모두 로봇 도메인 — domain_bridge가 서버(12)와 중계):
  구독  /assignment            {"robot_id","source","target"} (location_id)
  발행  /task_report           {"robot_id","event": "source_arrived"|"target_done"}
  발행  /robot_status          "idle"|"busy"|"error" (1Hz, 브릿지가 /AMR_x/…로 remap)
  기존  /traffic/request|response|pose, /navigate_to_pose, /initialpose, /amcl_pose

실행 (로봇에서, 각자 도메인):
    ros2 run robot_fsm robot_fsm --ros-args -p robot:=AMR_1
사용:
    (기동 시 자동으로 자기 홈에 AMCL 초기화 — 로봇을 홈에 dock 방향 바라보게 놓고 켜면 됨)
    fsm> init N13 E      # 홈이 아닌 곳에서 시작할 때만 수동 초기화
    fsm> init            # 홈으로 재초기화
    fsm> N22 [laden]     # (디버그) 노드 주행만 — fleet 없이 traffic 테스트용 (go 생략 가능)
    fsm> status / reset / q
※ 자동 초기화 끄기: --ros-args -p auto_init:=false
"""
import json
import os
import queue
import sys
import threading
import time

import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import String, Int32
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ament_index_python.packages import get_package_share_directory

from robot_fsm.states import S, TRANSITIONS, EXT_IDLE
from robot_fsm.travel import TravelMixin
from robot_fsm.work import WorkMixin
from robot_fsm.amcl_init import AmclInitMixin

_DOMAIN_ROBOT = {30: 'AMR_1', 31: 'AMR_2'}   # ROS_DOMAIN_ID → 로봇 이름

# PLACE 완료 후 체이닝 배정 대기 시간(초) — 이 안에 /assignment 오면 홈 안 감
CHAIN_WAIT = 1.5


class RobotFSM(TravelMixin, WorkMixin, AmclInitMixin, Node):
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

        # ── 도킹 (aruco_docking dock_controller) ──────────────
        self._work_dock_cli = self.create_client(Trigger, '/start_work_dock')   # 작업(전진)
        self._home_dock_cli = self.create_client(Trigger, '/start_home_dock')   # 홈(후진 180°+안착)
        self._undock_cli = self.create_client(Trigger, '/start_undock')
        self._estimator_param_cli = self.create_client(
            SetParameters, '/aruco_estimator/set_parameters')

        # ── 포크 (ESP32 micro-ROS) — 절대 높이만 발행, cur_step 추적은 ESP32 ──
        #   ★ /fork_state는 best_effort 필수: micro-ROS(rclc) 발행자를 reliable 구독자가
        #     못 받는 Jazzy 인터롭 이슈(type_hash INVALID). best_effort면 수신됨. 실물 검증(HW).
        #     상태는 5Hz 하트비트 + 이동 중 반복 발행이라 유실 몇 개는 핸드셰이크에 무해.
        self._fork_pub = self.create_publisher(Int32, '/fork_cmd', 10)
        _fork_state_qos = QoSProfile(depth=10)
        _fork_state_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(Int32, '/fork_state', self._on_fork_state, _fork_state_qos)
        self._fork_last, self._fork_moving_seen, self._fork_error = None, False, False
        self._fork_ev = threading.Event()

        # ── traffic ─────────────────────────────────────────
        self._req_pub = self.create_publisher(String, '/traffic/request', 10)
        self.create_subscription(
            String, f'/traffic/response/{self.robot}', self._on_response, 10)
        self._pose_pub = self.create_publisher(String, '/traffic/pose', 10)
        self._last_pose_pub = 0.0

        # ── fleet ───────────────────────────────────────────
        self.create_subscription(String, '/assignment', self._on_assignment, 10)
        # 고객 주문취소 — {robot_id, order_id}. 자기 것만 처리(assignment와 동일 필터).
        self.create_subscription(String, '/cancel_mission', self._on_cancel_mission, 10)
        self._report_pub = self.create_publisher(String, '/task_report', 10)
        self._status_pub = self.create_publisher(String, '/robot_status', 10)
        self.create_timer(1.0, self._publish_status)

        self._resp = None
        self._resp_ev = threading.Event()
        self.current = None              # 현재(마지막 도착) 노드
        self._queue = queue.Queue()      # 수신된 /assignment 대기열
        # 취소 요청된 order_id (없으면 None). 콜백(executor 스레드)이 세팅,
        # 워커 스레드가 travel 세그먼트 경계에서 읽어 'cancelled' 반환 후 소비.
        # 파이썬 속성 대입은 GIL로 원자적이라 별도 락 불필요.
        self._cancel_order_id = None
        self._state = S.IDLE             # 상태 정의·전이 = states.py
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
                # 초기화 후 360° 수렴 회전 — 파티클을 시작부터 조여둠.
                # 홈 자세 = dock 방향을 바라봄(의도적 설계) — 로봇을 그 방향으로 놓고 켠다.
                self.set_initial_pose(self.home['node'], self.home['dock'], spin=True)
                return
            time.sleep(0.5)
            waited += 0.5
            if waited % 15 < 0.5:        # 15초마다 안내
                self.get_logger().warn(
                    'auto_init 대기 중: AMCL(/initialpose 구독자) 미기동 — nav2를 켜면 자동 초기화됩니다')

    # ── 상태 전이 (단일 통로) ─────────────────────────────────

    def _set_state(self, new):
        """모든 상태 변경은 여기로. states.TRANSITIONS 밖이면 경고(막지는 않음).

        실기 운용 중 FSM 정지보다 잘못된 전이를 로그로 표면화하는 쪽이 안전.
        """
        old = self._state
        if new is old:
            return
        if new not in TRANSITIONS.get(old, set()):
            self.get_logger().warn(
                f'[{self.robot}] 미정의 전이: {old.value} → {new.value} '
                f'(states.TRANSITIONS 확인 필요)')
        self._state = new
        self.get_logger().info(f'[{self.robot}] 상태: {old.value} → {new.value}')

    # ── 상태 발행 ─────────────────────────────────────────────

    def _ext_status(self):
        """외부 발행 상태 — fleet은 idle만 배정 대상으로 봄 (states.EXT_IDLE)."""
        if self._state is S.ERROR:
            return 'error'
        if self._state in EXT_IDLE:
            return 'idle'
        return 'busy'

    def _publish_status(self):
        self._status_pub.publish(String(data=self._ext_status()))

    def _report(self, event, order_id=None):
        payload = {'robot_id': self.robot, 'event': event}
        if order_id is not None:
            payload['order_id'] = order_id      # 취소 이벤트: fleet이 대상 task 특정용
        self._report_pub.publish(String(data=json.dumps(payload)))
        self.get_logger().info(f'[{self.robot}] task_report: {event}')

    def _on_cancel_mission(self, msg):
        """고객 주문취소 신호 — {robot_id, order_id}. 자기 것이면 플래그만 세운다.
        실제 중단은 워커 스레드가 travel 세그먼트 경계(노드 위)에서 처리(안전)."""
        try:
            d = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if d.get('robot_id') != self.robot:
            return
        self._cancel_order_id = d.get('order_id')
        self.get_logger().warn(
            f'[{self.robot}] 주문취소 신호 수신: order_id={self._cancel_order_id}')

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

    # ── 임무 루프 (전용 스레드) ───────────────────────────────

    def mission_loop(self):
        """배정 대기 → 임무 실행 → 체이닝 or 홈 복귀. 데몬 스레드로 실행."""
        nxt = None
        while rclpy.ok():
            if self._state is S.ERROR:
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

        self._set_state(S.TO_SOURCE)
        r = self._travel(src['node'], laden=False)
        if r == 'cancelled':
            return self._cancel_aborted()       # 빈손 중단 — 물건 안 건드림
        if r != 'done':
            return False

        self._set_state(S.PICK)
        if not self._work(src, pick=True):       # PICK은 원자구간 — 중간 중단 안 함
            return False
        # PICK 도중 취소가 왔으면 여기서 확인 → 물건 들었으니 반납 경로
        if self._cancel_order_id is not None:
            return self._cancel_return(src)
        self._report('source_arrived')

        self._set_state(S.TO_TARGET)
        r = self._travel(tgt['node'], laden=True)
        if r == 'cancelled':
            return self._cancel_return(src)      # 물건 들고 있음 → 원래 선반으로 반납
        if r != 'done':
            return False

        self._set_state(S.PLACE)
        if not self._work(tgt, pick=False):      # PLACE도 원자구간 — 취소 와도 완료
            return False
        # PLACE 중 취소는 여기서 안 다룸 — 게이트에 놓은 뒤 fleet이 target_done 시점에
        # order가 cancelled면 reclaim으로 자연수렴 처리(방식 확정). 플래그만 정리.
        self._cancel_order_id = None
        self._report('target_done')

        self._set_state(S.IDLE)
        self._publish_status()           # 1Hz 타이머 안 기다리고 즉시 idle 발행
                                         # → fleet이 CHAIN_WAIT(1.5s) 안에 배정 가능
        self.get_logger().info(f'[{self.robot}] 임무 완료: {a["source"]} → {a["target"]}')
        return True

    def _cancel_aborted(self):
        """TO_SOURCE(빈손) 중 고객취소 — 임무 폐기. 물건을 안 집었으니 되돌릴 것 없음.
        cancel_aborted 보고 → fleet이 outbound task cancelled + 선반 예약 해제.
        True 반환 = 임무 정상 종료(에러 아님) → 워커가 홈 복귀."""
        oid = self._cancel_order_id
        self._cancel_order_id = None
        self._set_state(S.IDLE)
        self._report('cancel_aborted', order_id=oid)
        self._publish_status()
        self.get_logger().info(f'[{self.robot}] 주문취소(빈손 중단): order_id={oid}')
        return True

    def _cancel_return(self, src):
        """물건 든 상태에서 고객취소 — 원래 선반(src)으로 되돌려 놓는다(자체 swap).
        target을 선반으로 바꿔 laden 이동 → PLACE(반납). cancel_returned 보고 →
        fleet이 outbound task cancelled + 선반 예약 해제(재고는 원래대로 유지)."""
        oid = self._cancel_order_id
        self._cancel_order_id = None      # 반납 이동 중 재취소 방지 위해 먼저 소비
        self.get_logger().info(f'[{self.robot}] 주문취소(선반 반납 시작): order_id={oid}')
        self._set_state(S.TO_TARGET)
        if self._travel(src['node'], laden=True) != 'done':
            return False                  # 반납 이동 실패 → ERROR
        self._set_state(S.PLACE)
        if not self._work(src, pick=False):
            return False
        self._set_state(S.IDLE)
        self._report('cancel_returned', order_id=oid)
        self._publish_status()
        self.get_logger().info(f'[{self.robot}] 주문취소(선반 반납 완료): order_id={oid}')
        return True

    def _return_home(self):
        """홈 복귀. 배정 오면 세그먼트 경계에서 중단하고 그 임무를 반환."""
        if self.current == self.home['node']:
            self._set_state(S.IDLE)
            return None
        self._set_state(S.RETURNING)
        res = self._travel(self.home['node'], laden=False, preempt=True)
        if res == 'preempted':
            try:
                return self._queue.get_nowait()
            except queue.Empty:
                return None
        if res == 'failed':
            self._set_error('홈 복귀 실패')
            return None
        # 홈 후진 정밀 도킹(주차). 실패해도 복귀는 완료로 처리(홈이라 ERROR까진 X).
        if not self._home_dock():
            self.get_logger().warn(f'[{self.robot}] 홈 도킹 실패 — 복귀는 완료 처리')
        self._set_state(S.IDLE)
        self.get_logger().info(f'[{self.robot}] 홈({self.home["node"]}) 복귀 완료')
        return None

    def _set_error(self, why):
        self._set_state(S.ERROR)
        self.get_logger().error(
            f'[{self.robot}] ERROR: {why} — 배정 제외됨, 조치 후 fsm> reset')

    # ── 디버그: 수동 노드 주행 (fleet 없이 traffic 테스트) ─────

    def manual_go(self, goal, laden=False):
        if self.current is None:
            print('먼저 init 필요')
            return
        if self._state is not S.IDLE:
            print(f'현재 {self._state.value} — IDLE에서만 수동 주행 가능')
            return
        self._set_state(S.MANUAL)        # busy 발행 → fleet 배정 차단
        try:
            res = self._travel(goal.upper(), laden=laden)
            print(f'수동 주행 결과: {res}')
        finally:
            self._set_state(S.IDLE)


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
                # 인자 없으면 자기 홈 (locations.yaml homes) — dock 방향 바라봄
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
                if node._state is not S.IDLE:
                    print(f'현재 {node._state.value} — IDLE에서만 가능')
                else:
                    node._set_state(S.MANUAL)
                    try:
                        print('수렴 회전 성공' if node._spin_converge() else '수렴 회전 실패')
                    finally:
                        node._set_state(S.IDLE)
            elif c == 'status':
                print(f'state={node._state.value} current={node.current} '
                      f'대기임무={node._queue.qsize()}')
            elif c == 'reset':
                if node._state is S.ERROR:
                    node._set_state(S.IDLE)
                    print('IDLE 복귀 (위치 어긋났으면 init 재실행)')
                else:
                    print(f'ERROR 아님 (현재 {node._state.value})')
            else:
                print('사용: init N25 N21 / go N1 [laden] / status / reset / q')
    finally:
        node.destroy_node()
        if rclpy.ok():                 # Ctrl-C 시그널이 이미 shutdown했으면 생략
            rclpy.shutdown()


if __name__ == '__main__':
    main()
