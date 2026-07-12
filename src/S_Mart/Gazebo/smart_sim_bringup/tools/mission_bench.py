#!/usr/bin/env python3
"""임무 벤치마크 러너 — A/B 전략 비교 측정 (fake_fleet의 자동화판).

서버 도메인(12)에서 실행. 임무 10개를 로봇이 빌 때마다 순서대로 배정하고
sim time 기준으로 측정:
  - 임무별 소요 (assignment 발행 → 해당 로봇 target_done)
  - 총 소요 (첫 배정 → 마지막 완료)  /  최장 임무

    export ROS_DOMAIN_ID=12
    python3 mission_bench.py --out results.csv            # 시뮬 (sim time)
    python3 mission_bench.py --real --set 2 --out results_real.csv          # 실기 (wall clock)

전제: sim + 로봇별 nav/fsm + traffic + domain_bridge 가동 중.
주의: 실행 순서(배정 정책) 고정 — 전략 비교 시 임무 목록/정책 동일 유지.
"""
import argparse
import csv
import json
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

ROBOT_IDS = ['AMR_1', 'AMR_2']

# 고정 임무 세트 (입고/출고/회수 혼합) — A/B 동일 목록으로 비교. --set N으로 선택
MISSION_SETS = {
    1: [                      # 세트1 (1차 실험, 2026-07-09)
        ('IN-1',  'A1-L1'),   # 입고
        ('IN-1',  'B1-L1'),   # 입고
        ('A1-L1', 'OUT-1'),   # 출고
        ('IN-1',  'A2-L1'),   # 입고
        ('B1-L1', 'OUT-2'),   # 출고
        ('OUT-1', 'B2-L1'),   # 회수
        ('IN-1',  'A1-L2'),   # 입고 (L2)
        ('A2-L1', 'OUT-1'),   # 출고
        ('OUT-2', 'B1-L1'),   # 회수
        ('A1-L2', 'OUT-1'),   # 출고
    ],
    2: [                      # 세트2 — 서쪽 랙 열 왕복 강화 (통로 공유·교차 압박 ↑)
        ('IN-1',  'B2-L1'),   # 입고 (남서 끝)
        ('IN-1',  'A1-L1'),   # 입고 (북서 끝)
        ('B2-L1', 'OUT-1'),   # 출고 (남서→북동 대각)
        ('IN-1',  'B1-L2'),   # 입고 (L2)
        ('A1-L1', 'OUT-2'),   # 출고 (북서→북동)
        ('OUT-1', 'A2-L1'),   # 회수
        ('IN-1',  'B2-L2'),   # 입고 (L2, 다시 남서)
        ('B1-L2', 'OUT-1'),   # 출고
        ('OUT-2', 'A1-L2'),   # 회수
        ('A2-L1', 'OUT-2'),   # 출고
    ],
}


class MissionBench(Node):
    def __init__(self, out_path, missions, real=False):
        super().__init__('mission_bench')
        if not real:                      # 실기(--real)는 wall clock 사용
            self.set_parameters([Parameter('use_sim_time', value=True)])
        self.out_path = out_path
        self._pub = self.create_publisher(String, '/assignment', 10)
        self.create_subscription(String, '/task_report', self._on_report, 10)
        for r in ROBOT_IDS:
            self.create_subscription(
                String, f'/{r}/robot_status',
                lambda m, r=r: self._on_status(r, m), 10)

        self.status = {}                  # robot -> idle/busy/error
        self.active = {}                  # robot -> mission dict (진행 중)
        self.queue = [{'i': i + 1, 'source': s, 'target': t}
                      for i, (s, t) in enumerate(missions)]
        self.done = []
        self.t_first = None
        self.create_timer(0.5, self._tick)
        self.get_logger().info(
            f'벤치 시작 — 임무 {len(self.queue)}개, 로봇 idle 대기 중')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_status(self, robot, msg):
        prev = self.status.get(robot)
        self.status[robot] = msg.data
        if prev != msg.data:
            self.get_logger().info(f'{robot}: {msg.data}')
        if msg.data == 'error' and robot in self.active:
            self.get_logger().error(f'{robot} ERROR — 임무 {self.active[robot]["i"]} 실패로 기록')
            m = self.active.pop(robot)
            m['t_done'] = self._now()
            m['result'] = 'error'
            self.done.append(m)

    def _tick(self):
        # sim time이 흘러야 시작 (clock 미수신 시 0)
        if self._now() <= 0.0:
            return
        for robot in ROBOT_IDS:
            if not self.queue:
                break
            if robot in self.active:
                continue
            if self.status.get(robot) != 'idle':
                continue
            m = self.queue.pop(0)
            m['robot'] = robot
            m['t_assign'] = self._now()
            m['result'] = 'ok'
            if self.t_first is None:
                self.t_first = m['t_assign']
            self.active[robot] = m
            payload = json.dumps({'robot_id': robot,
                                  'source': m['source'], 'target': m['target']})
            self._pub.publish(String(data=payload))
            self.get_logger().info(
                f"배정 #{m['i']}: {robot} {m['source']}→{m['target']} "
                f"(t={m['t_assign']:.1f})")

    def _on_report(self, msg):
        try:
            d = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        robot, event = d.get('robot_id'), d.get('event')
        if event != 'target_done' or robot not in self.active:
            return
        m = self.active.pop(robot)
        m['t_done'] = self._now()
        self.done.append(m)
        dur = m['t_done'] - m['t_assign']
        self.get_logger().info(
            f"완료 #{m['i']}: {robot} — {dur:.1f}s "
            f"(남은 임무 {len(self.queue)}, 진행 중 {len(self.active)})")
        if not self.queue and not self.active:
            self._finish()

    def _finish(self):
        t_end = max(m['t_done'] for m in self.done)
        total = t_end - self.t_first
        durs = [(m['t_done'] - m['t_assign'], m) for m in self.done]
        worst_d, worst = max(durs, key=lambda x: x[0])
        print('\n' + '=' * 52)
        print(f"  총 소요(첫 배정→마지막 완료): {total:.1f}s (sim time)")
        print(f"  최장 임무: #{worst['i']} {worst['robot']} "
              f"{worst['source']}→{worst['target']} = {worst_d:.1f}s")
        print(f"  임무별:")
        for m in sorted(self.done, key=lambda m: m['i']):
            print(f"    #{m['i']:2d} {m['robot']} {m['source']:6s}→{m['target']:6s}"
                  f"  {m['t_done'] - m['t_assign']:6.1f}s  {m['result']}")
        print('=' * 52)
        with open(self.out_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['i', 'robot', 'source', 'target',
                        't_assign', 't_done', 'duration', 'result'])
            for m in sorted(self.done, key=lambda m: m['i']):
                w.writerow([m['i'], m['robot'], m['source'], m['target'],
                            f"{m['t_assign']:.2f}", f"{m['t_done']:.2f}",
                            f"{m['t_done'] - m['t_assign']:.2f}", m['result']])
            w.writerow([])
            w.writerow(['total', f'{total:.2f}'])
            w.writerow(['max_single', f'{worst_d:.2f}', f"#{worst['i']}"])
        print(f'CSV 저장: {self.out_path}')
        rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='results.csv', help='CSV 출력 경로')
    ap.add_argument('--set', type=int, default=1, choices=sorted(MISSION_SETS),
                    help='임무 세트 번호 (기본 1)')
    ap.add_argument('--real', action='store_true',
                    help='실기 모드: sim time 대신 wall clock (미지정 시 /clock 없으면 시작 안 함)')
    args = ap.parse_args()
    if os.path.exists(args.out):                     # 덮어쓰기 사고 방지
        print(f'⚠️  {args.out} 이미 존재 — 다른 이름을 쓰세요 (덮어쓰기 방지)')
        sys.exit(1)
    rclpy.init()
    node = MissionBench(args.out, MISSION_SETS[args.set], real=args.real)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    sys.exit(0)


if __name__ == '__main__':
    main()
