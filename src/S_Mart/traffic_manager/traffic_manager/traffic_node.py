#!/usr/bin/env python3
"""Traffic Manager ROS 노드 (서버).

TrafficManager(경로/예약/교착)를 토픽 인터페이스로 감싼다.
domain_bridge 궁합을 위해 std_msgs/String(JSON) 사용.

[로봇 → 서버] /traffic/request (String JSON)
  {"robot":"AMR_1","type":"route","start":"N25","goal":"N1","laden":false}
  {"robot":"AMR_1","type":"segment"}

[서버 → 로봇] /traffic/response/<robot> (String JSON)
  {"type":"route","route":[...]}
  {"type":"segment","nodes":[...]}
  {"type":"wait"}
  {"type":"reroute","route":[...]}
  {"type":"done"}
"""
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from traffic_manager.graph import Graph
from traffic_manager.router import Router
from traffic_manager.traffic import TrafficManager


ROBOT_IDS = ['AMR_1', 'AMR_2']


class TrafficNode(Node):
    def __init__(self):
        super().__init__('traffic_node')
        self.tm = TrafficManager(Router(Graph()))
        self._pubs = {}
        # 응답 퍼블리셔 선생성 — lazy 생성 후 즉시 발행하면 DDS 디스커버리
        # 완료 전이라 로봇당 첫 응답이 유실됨 (로봇은 10초 재시도로 버팀)
        for r in ROBOT_IDS:
            self._pub(r)
        self.create_subscription(String, '/traffic/request', self._on_request, 10)
        self.create_subscription(String, '/traffic/pose', self._on_pose, 10)
        # 장애물 차단 해제 — 테스트/정비용 수동 훅 (나중에 관제 GUI가 이 토픽 발행).
        #   {"node":"N9"} 특정 노드 해제 / {"all":true} 전체 해제
        self.create_subscription(String, '/traffic/unblock', self._on_unblock, 10)
        # 관제 GUI 장애물 알람 — block/clear 발행 (GUI가 구독·렌더·제거버튼 제공).
        #   block: {"event":"block","kind":"reroute|goal_blocked|no_route","node":,"robot":}
        #   clear: {"event":"clear","node":}
        self._obstacle_pub = self.create_publisher(String, '/traffic/obstacle', 10)
        self._announced = {}          # node -> kind : GUI에 알린 차단 상태 미러
        self.create_timer(0.5, self._on_timer)
        self.get_logger().info('Traffic Manager 노드 시작 (토픽 인터페이스)')

    def _emit_obstacle(self, event, node, kind='', robot=''):
        self._obstacle_pub.publish(String(data=json.dumps(
            {'event': event, 'node': node, 'kind': kind, 'robot': robot})))

    def _on_unblock(self, msg):
        try:
            d = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if d.get('all'):
            self.tm.clear_blocks()
            self.get_logger().info('장애물 차단 전체 해제')
        elif 'node' in d:
            self.tm.unblock_node(d['node'])
            self.get_logger().info(f'장애물 차단 해제: {d["node"]}')

    def _on_pose(self, msg):
        """로봇 위치 → traffic이 통과 노드 감지 → 앞으로 지난 노드 release."""
        try:
            d = json.loads(msg.data)
            robot, x, y = d['robot'], float(d['x']), float(d['y'])
        except (json.JSONDecodeError, KeyError, ValueError):
            return
        passed = self.tm.update_position(robot, x, y)
        if passed:
            self.get_logger().info(f'{robot} 통과: {passed} (뒤 노드 release)')

    def _pub(self, robot):
        if robot not in self._pubs:
            self._pubs[robot] = self.create_publisher(
                String, f'/traffic/response/{robot}', 10)
        return self._pubs[robot]

    def _send(self, robot, payload):
        msg = String()
        msg.data = json.dumps(payload)
        self._pub(robot).publish(msg)

    def _on_request(self, msg):
        try:
            req = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f'잘못된 요청: {msg.data}')
            return
        robot, rtype = req.get('robot'), req.get('type')
        if not robot or not rtype:
            return

        if rtype == 'route':
            ok = self.tm.set_route(robot, req['start'], req['goal'],
                                   laden=req.get('laden', False))
            if ok:
                self._send(robot, {'type': 'route',
                                   'route': self.tm.robots[robot].route})
                self.get_logger().info(
                    f'{robot} 경로: {"→".join(self.tm.robots[robot].route)}')
            else:
                self._send(robot, {'type': 'wait'})

        elif rtype == 'segment':
            self._reply_segment(robot)

        elif rtype == 'blocked':
            # 로봇이 recovery로 지속 장애물 확정 → 노드 전역 차단 + 대응 결정
            kind, node, route = self.tm.report_obstacle(robot)
            if node is not None:
                # 처음 알리는 노드면 GUI에 block 발행 (report_obstacle은 kind 무관하게
                # block_node로 노드를 차단하므로 3종 다 발행). clear는 _on_timer reconcile.
                if node not in self._announced:
                    self._announced[node] = kind
                    self._emit_obstacle('block', node, kind, robot)
                if kind == 'goal_blocked':
                    self.get_logger().warn(
                        f'⚠️ {robot} 목적지 노드 막힘 ({node}) — 대기. '
                        f'사람이 치우고 통과하면 자동해제(or /traffic/unblock)')
                else:
                    self.get_logger().warn(
                        f'⚠️ {robot} 장애물 노드 차단 {node} ({kind}). '
                        f'사람이 치우면 통과 시 자동해제(or /traffic/unblock)')
            if kind == 'reroute':
                self._send(robot, {'type': 'reroute', 'route': route})
                self.get_logger().info(f'{robot} 우회: {"→".join(route)}')
            else:                                     # goal_blocked / no_route / None → 대기
                self._send(robot, {'type': 'wait'})

    def _reply_segment(self, robot):
        if robot not in self.tm.robots:
            self._send(robot, {'type': 'wait'})
            return
        seg = self.tm.next_segment(robot)
        if seg:
            self._send(robot, {'type': 'segment', 'nodes': seg})
        elif self.tm.is_done(robot):
            self._send(robot, {'type': 'done'})
        elif self.tm.next_node_blocked(robot):
            # 진행 방향 노드가 (다른 로봇이 감지한) 장애물 차단
            st = self.tm.robots[robot]
            nxt = st.route[st.index + 1]
            if nxt == st.route[-1]:
                # 막힌 다음 노드 = 내 목적지 → 우회 무의미, 대기 (빙빙 돌기 방지)
                self._send(robot, {'type': 'wait'})
            elif self.tm.set_route(robot, self.tm.current_node(robot),
                                   st.route[-1], laden=st.laden):
                # 로봇B: 경유 노드 차단 → 분기점에서 미리 우회
                self._send(robot, {'type': 'reroute',
                                   'route': self.tm.robots[robot].route})
                self.get_logger().info(
                    f'{robot} 장애물 차단 회피 우회: {"→".join(self.tm.robots[robot].route)}')
            else:
                self._send(robot, {'type': 'wait'})   # 우회로 없음
        else:
            self._send(robot, {'type': 'wait'})

    def _on_timer(self):
        # 차단 해제 reconcile — 미러(_announced) vs 실제 차단집합 비교해 사라진 노드는
        # GUI에 clear 발행. 자동 자가치유(통과)·GUI 제거버튼·전체해제 3경로를 한 곳에서 커버.
        blocked = self.tm.blocked_nodes()
        for node in list(self._announced):
            if node not in blocked:
                self._emit_obstacle('clear', node)
                del self._announced[node]

        actions = self.tm.resolve_deadlock()
        for robot, act in actions.items():
            if act == 'reroute':
                route = self.tm.robots[robot].route
                self._send(robot, {'type': 'reroute', 'route': route})
                self.get_logger().info(
                    f'교착 해결: {robot} 우회 → {"→".join(route)}')
            elif act == 'stuck':
                self.get_logger().warn(
                    f'교착 미해결: {robot} 우회 경로 없음 (수동 개입 필요)',
                    throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = TrafficNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():                 # Ctrl-C 시그널이 이미 shutdown했으면 생략
            rclpy.shutdown()


if __name__ == '__main__':
    main()
