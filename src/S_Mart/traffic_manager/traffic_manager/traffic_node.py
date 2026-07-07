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
        self.create_timer(0.5, self._on_timer)
        self.get_logger().info('Traffic Manager 노드 시작 (토픽 인터페이스)')

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

    def _reply_segment(self, robot):
        if robot not in self.tm.robots:
            self._send(robot, {'type': 'wait'})
            return
        seg = self.tm.next_segment(robot)
        if seg:
            self._send(robot, {'type': 'segment', 'nodes': seg})
        elif self.tm.is_done(robot):
            self._send(robot, {'type': 'done'})
        else:
            self._send(robot, {'type': 'wait'})

    def _on_timer(self):
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
