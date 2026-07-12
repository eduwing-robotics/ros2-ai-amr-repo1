#!/usr/bin/env python3
"""map_gimp.pgm → Gazebo 월드(smart_arena.sdf) 생성기.

실기 AMCL 맵의 점유 셀을 그대로 박스로 변환 — 월드↔맵 정합 자동 보장.
점유 셀을 행 단위 run으로 묶고, 같은 x구간의 연속 행을 세로로 병합해
박스 수를 최소화한다.

사용: python3 gen_world.py   (smart_nav_bringup/maps 경로에서 맵을 읽음)
출력: ../worlds/smart_arena.sdf
"""
import os

RES = 0.05
ORIGIN = (-0.861, -0.939)
WALL_H = 0.3           # 벽 높이(m) — TB3 LDS(~0.17m)가 확실히 보도록
OCC_THRESH = 100       # 이보다 어두우면 점유

HERE = os.path.dirname(os.path.abspath(__file__))
MAP_PGM = os.path.normpath(os.path.join(
    HERE, '..', '..', 'smart_nav_bringup', 'maps', 'map_gimp.pgm'))
OUT_SDF = os.path.normpath(os.path.join(HERE, '..', 'worlds', 'smart_arena.sdf'))


def load_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()

    def tok(pos):
        while pos < len(data):
            if data[pos:pos + 1].isspace():
                pos += 1
            elif data[pos:pos + 1] == b'#':
                while pos < len(data) and data[pos:pos + 1] != b'\n':
                    pos += 1
            else:
                s = pos
                while pos < len(data) and not data[pos:pos + 1].isspace():
                    pos += 1
                return data[s:pos], pos
        return None, pos

    magic, p = tok(0)
    assert magic == b'P5', f'P5만 지원: {magic}'
    w, p = tok(p)
    h, p = tok(p)
    _, p = tok(p)
    w, h = int(w), int(h)
    return w, h, data[p + 1: p + 1 + w * h]


def occupied_boxes(w, h, px):
    """점유 셀 → (r0, r1, c0, c1) 박스 목록 (그리디 병합)."""
    occ = [[px[r * w + c] < OCC_THRESH for c in range(w)] for r in range(h)]
    # 1) 행별 run
    runs = {}                     # r -> [(c0, c1), ...]
    for r in range(h):
        row = []
        c = 0
        while c < w:
            if occ[r][c]:
                c0 = c
                while c < w and occ[r][c]:
                    c += 1
                row.append((c0, c - 1))
            else:
                c += 1
        runs[r] = row
    # 2) 같은 (c0,c1) run이 연속 행이면 세로 병합
    boxes = []
    active = {}                   # (c0,c1) -> r_start
    for r in range(h + 1):
        cur = set(runs.get(r, []))
        for key in list(active):
            if key not in cur:
                boxes.append((active.pop(key), r - 1, key[0], key[1]))
        for key in cur:
            if key not in active:
                active[key] = r
    return boxes


def box_pose_size(r0, r1, c0, c1, h):
    """셀 박스 → 월드 중심 좌표/크기. (row 0 = 맵 위 = +y 최대)"""
    ox, oy = ORIGIN
    x0, x1 = ox + c0 * RES, ox + (c1 + 1) * RES
    ytop, ybot = oy + (h - r0) * RES, oy + (h - r1 - 1) * RES
    cx, cy = (x0 + x1) / 2, (ytop + ybot) / 2
    sx, sy = x1 - x0, ytop - ybot
    return cx, cy, sx, sy


def main():
    w, h, px = load_pgm(MAP_PGM)
    boxes = occupied_boxes(w, h, px)
    print(f'맵 {w}x{h}, 벽 박스 {len(boxes)}개')

    walls = []
    for i, (r0, r1, c0, c1) in enumerate(boxes):
        cx, cy, sx, sy = box_pose_size(r0, r1, c0, c1, h)
        walls.append(f'''    <model name="wall_{i}">
      <static>true</static>
      <pose>{cx:.3f} {cy:.3f} {WALL_H / 2:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="col">
          <geometry><box><size>{sx:.3f} {sy:.3f} {WALL_H}</size></box></geometry>
        </collision>
        <visual name="vis">
          <geometry><box><size>{sx:.3f} {sy:.3f} {WALL_H}</size></box></geometry>
          <material><ambient>0.55 0.55 0.6 1</ambient><diffuse>0.55 0.55 0.6 1</diffuse></material>
        </visual>
      </link>
    </model>''')

    sdf = f'''<?xml version="1.0"?>
<!-- 자동 생성: tools/gen_world.py (원본: smart_nav_bringup/maps/map_gimp.pgm)
     수정하지 말 것 — 맵이 바뀌면 생성기를 다시 돌린다. -->
<sdf version="1.8">
  <world name="smart_arena">
    <physics name="default" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <direction>-0.3 0.2 -0.9</direction>
    </light>

    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="col">
          <geometry><plane><normal>0 0 1</normal><size>10 10</size></plane></geometry>
        </collision>
        <visual name="vis">
          <geometry><plane><normal>0 0 1</normal><size>10 10</size></plane></geometry>
          <material><ambient>0.85 0.85 0.85 1</ambient><diffuse>0.85 0.85 0.85 1</diffuse></material>
        </visual>
      </link>
    </model>

{chr(10).join(walls)}
  </world>
</sdf>
'''
    os.makedirs(os.path.dirname(OUT_SDF), exist_ok=True)
    with open(OUT_SDF, 'w') as f:
        f.write(sdf)
    print(f'생성: {OUT_SDF}')


if __name__ == '__main__':
    main()
