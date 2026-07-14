"""주행 믹스인 — traffic 요청/응답 + 세그먼트 주행 (nav2 액션).

RobotFSM에 믹스인으로 결합 — self.nodes/current/_queue 등 노드 상태 공유.
담당: 사용자 (traffic 계약 변경 시 이 파일).
"""
import json
import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String

_DIR_YAW = {'E': 0.0, 'N': math.pi / 2, 'W': math.pi, 'S': -math.pi / 2}

# yaw 톨러런스: 통로 = don't-care (xy만 도착 판정, 다음 goal이 알아서 회전),
# dock 방향 회전 goal에서만 정밀. nav2_params 기본값도 YAW_FREE로 맞춰둠.
# YAW_DOCK 0.1 rad ≈ ±5.7° — 마커가 도킹 카메라 화각에 들어오면 충분,
# 이후 정밀 정렬은 도킹 서버(마커 기반)가 담당. 실기 확인값.
YAW_FREE = 3.14
YAW_DOCK = 0.1

# 세그먼트 주행 중 nav2 recovery(Wait)가 이 횟수 도달 = 지속 장애물 확정 → 재경로.
# BT의 RecoveryNode number_of_retries=6보다 작아야 abort 전에 감지 (4 < 6).
RECOVERY_BLOCK = 4


class TravelMixin:

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
                outcome = self._drive_segment(resp['nodes'])
                if outcome == 'blocked':
                    # 지속 장애물 감지 → traffic에 보고 → 엣지 차단 + 재경로
                    r = self._transact({'type': 'blocked'})
                    if r and r.get('type') == 'reroute':
                        self.current = r['route'][0]   # traffic index로 동기화(엣지 중간→노드)
                        self.get_logger().warn(
                            f'[{self.robot}] 장애물 우회: {"→".join(r["route"])}')
                    else:                               # wait(목적지 막힘 or 우회로 없음) or None
                        self.get_logger().warn(
                            f'[{self.robot}] 장애물 — 대기 (사람이 치울 때까지)')
                        time.sleep(1.0)
                elif outcome == 'failed':
                    return 'failed'
                # 'done' → 계속
            elif t == 'wait':
                time.sleep(1.0)
            elif t == 'reroute':
                self.current = resp['route'][0]         # 로봇B 선제 우회 동기화
                self.get_logger().info(
                    f'[{self.robot}] 우회: {"→".join(resp["route"])}')
            elif t == 'done':
                return 'done'
            else:
                self.get_logger().warn(f'[{self.robot}] traffic 응답 이상: {resp}')
                return 'failed'
        return 'failed'

    def _drive_segment(self, seg_nodes):
        """직선 run 주행 (recovery 감시 포함). 반환: 'done' | 'failed' | 'blocked'."""
        if not self._ac.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('nav2 액션서버 없음')
            return 'failed'
        # 직선 run은 끝점 goal 하나로 쭉 (중간 노드 통과, 안 멈춤).
        # 중간 노드 release는 traffic이 amcl 위치로 처리 (로봇은 arrive 안 보냄).
        end = seg_nodes[-1]
        d = self._direction(self.current, seg_nodes[0])
        nd = self.nodes[end]
        outcome = self._nav_monitored(nd['x'], nd['y'], _DIR_YAW[d])
        if outcome == 'done':
            self.current = end
            return 'done'
        if outcome == 'blocked':
            self.get_logger().warn(
                f'[{self.robot}] {end} 방향 지속 장애물 (recovery {RECOVERY_BLOCK}회)')
            return 'blocked'
        self.get_logger().warn(f'[{self.robot}] {end} 주행 실패')
        return 'failed'

    def _nav_monitored(self, x, y, yaw):
        """NavigateToPose 주행 + recovery 감시. 반환: 'done'|'failed'|'blocked'.

        feedback.number_of_recoveries가 RECOVERY_BLOCK 도달 = wait-only recovery가
        그만큼 못 뚫음 = 지속 장애물 → goal 취소하고 'blocked' (→ 위상적 우회).
        """
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        fb = {'n': 0}
        gh = self._await_future(self._ac.send_goal_async(
            goal, feedback_callback=lambda m: fb.update(n=m.feedback.number_of_recoveries)))
        if not gh or not gh.accepted:
            return 'failed'
        result_fut = gh.get_result_async()
        ev = threading.Event()
        result_fut.add_done_callback(lambda _f: ev.set())
        while not ev.wait(0.2):                       # 0.2s마다 recovery 감시
            if fb['n'] >= RECOVERY_BLOCK:
                self._await_future(gh.cancel_goal_async(), timeout=5.0)
                return 'blocked'
        res = result_fut.result()
        return 'done' if (res and res.status == GoalStatus.STATUS_SUCCEEDED) else 'failed'

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
