#!/usr/bin/env python3
"""Smart Mart dock_controller — 마커(ArUco) 정밀 도킹 컨트롤러 (work 전진 / home 후진).

구성 (실물 검증 — 상세 docs_hub/context/13):
  PREALIGN (수직 측면 재배치) 공통 → 분기:
    [work 전진] STAGE_GOTO (법선 위 점 go-to) → APPROACH (마커 위치 m_y 조향, 법선 무관)
                → CREEP (odom 전진 안착).   ← staging 방식 (2026-07-17 통합, ALIGN 없음)
    [home 후진] SERVO (법선 PBVS 폐루프) → ALIGN (제자리 yaw) → SPIN180 → 후진 안착.
  UNDOCK = odom 후진 고정거리 (포크 작업 후 이탈). 오차 로깅만 = `log_only:=true`.

★ work가 SERVO(법선 coupled)에서 staging으로 바뀐 이유: 조향을 법선(e_y)에서 유도하면
  도크별 법선오차(~1-2°)가 depth 레버암을 타고 e_y로 증폭(실측 ±8mm). m_y(위치)는 법선을
  안 타 도크별 트림 0. home은 후진·주차라 정밀 비필요 → 기존 SERVO 유지. 상세 context/13 §10.

제어법칙 (부호·게인 실물 확정 2026-07-08 — 상세 docs_hub/context/13):
  SERVO: ω = −k_y·e_y + k_θ·e_θ,  v = clamp(k_v·ρ, v_min, v_max)  (소프트 정렬게이트)
  CREEP: v = creep_speed, ω = odom yaw hold (직진). **정체 감지(막히면 정지)**.
검증 게인(기본값): k_y=6, k_θ=1, v_max=0.02 → 실전 오프셋(2~4cm) e_y<4mm·e_θ<2°.
  k_y는 거리 스케줄(원거리 k_y_far=4 → 근접 k_y=6): 초반 큰 e_y서 ω 포화·피루엣 방지하며
  v 안 올리고 캡처 확장. 정밀 성공은 시작 오프셋 ≲5cm(런웨이 천장, Nav2 도착 스펙).

★ 안전: CREEP은 odom이 안 늘면(충돌/막힘) 즉시 정지·STALL. 핸드오프는 **depth(마커거리)**
  기준이라 stop_dist와 분리. stop_dist=0.125 실물 확정(포크 기하 — 기체 바뀌면 재튜닝).

PREALIGN (enable_prealign=true, 기본): SERVO 앞단. 마커로 e_y 측정 → 마커 정면 → 90° 회전 →
  e_y만큼 odom 측면직진 → −90° 복귀. 측면이동이 법선에 수직이라 depth(런웨이) 소모 0으로
  큰 도착 오프셋을 축 위로 옮김. 회전 odom오차는 이어지는 SERVO가 폐루프로 청소. 상세 context/13.

오차(base_link, x=전방 y=좌): ρ=정지점(마커앞 stop_dist)까지 / e_y=cross-track /
  e_θ=마커정면까지 헤딩 / depth=마커까지. EMA 저역통과.
상태: IDLE → PREALIGN →[work] STAGE_GOTO → APPROACH → CREEP
                      →[home] SERVO → ALIGN → SPIN180 → 후진안착 → DONE
       UNDOCK(후진) 별도. (실패: LOST / TIMEOUT / STALL / MISALIGNED)
트리거(std_srvs/Trigger, blocking→success): /start_work_dock(전진) · /start_home_dock(후진 180°+안착) ·
  /start_undock(포크 후 이탈). work·home은 `_run_dock(reverse)` 공유(모드=서비스로 명시, param 아님).
  입력: /detected_dock_pose+TF+/odom. 출력: /cmd_vel(TwistStamped).
"""
import math
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs  # noqa: F401


def _norm(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


def _yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class DockControllerNode(Node):

    def __init__(self):
        super().__init__('dock_controller')

        # ── 프레임/기하/필터 ──────────────────────────────────────
        self.declare_parameter('base_frame', 'base_link')
        # ★ stop_dist = base_link가 마커 앞에서 멈출 거리. 실물 포크 삽입 기준 확정값
        #   (2026-07-08: 0.125 = 포크가 팔레트에 적정 깊이 삽입되는 지점). 기체/포크 바뀌면 재튜닝.
        self.declare_parameter('stop_dist', 0.125)
        self.declare_parameter('ema_alpha', 0.3)
        self.declare_parameter('tf_timeout_sec', 0.2)
        self.declare_parameter('log_period_sec', 0.5)
        self.declare_parameter('log_only', False)

        # ── SERVO 게인 (실물 검증값) ──────────────────────────────
        self.declare_parameter('k_v', 0.3)
        self.declare_parameter('k_y', 6.0)              # 근접 k_y(타이트 마무리 — 실물 검증값)
        # ── 거리 스케줄 k_y: 원거리=완만(무포화·마커유지) → 근접=타이트 ──
        #   피루엣 임계 e_y*=ω_max/k_y. 먼 데서 k_y 낮춰 초기 큰 e_y에도 ω 포화 안 시킴(회전 완만).
        #   v 안 올리고 캡처 확장. k_y_far_depth(원거리)~handoff_depth(근접) 선형보간.
        self.declare_parameter('k_y_far', 4.0)          # 원거리 k_y (실물 확정 — 강한 초기 선회, 무포화)
        self.declare_parameter('k_y_far_depth', 0.30)   # 이 거리 이상=k_y_far, handoff_depth서 k_y로 램프
        self.declare_parameter('k_theta', 2.0)          # 헤딩교정 게인. skip case서 e_y≈0이면 ω≈k_θ·e_θ뿐 → 1.0은 약해 핸드오프 e_θ가 −1.7°까지 샘. 2.0=핸드오프 e_θ≈0 확보(실물검증 2026-07-14) → ALIGN 무동작 → 드리프트 소멸.
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('v_max', 0.02)
        self.declare_parameter('v_min', 0.008)
        self.declare_parameter('omega_max', 0.3)
        self.declare_parameter('align_soft_deg', 25.0)
        self.declare_parameter('align_stop_deg', 40.0)

        # ── PREALIGN (odom 측면 재배치: 마커로 e_y 측정 → 법선 위로 수직이동 → SERVO) ──
        #   Nav2 도착이 노드 반경(~6cm) 안이라 SERVO 시작 e_y가 큼(런웨이 부족→MISALIGNED).
        #   해결: 마커로 e_y 재고 → 마커 정면(θ0) → 90° 회전 → e_y만큼 odom 측면직진 → −90° 복귀.
        #   측면 이동은 법선에 수직이라 depth(런웨이) 소모 0. 회전 odom오차는 SERVO가 폐루프로 청소.
        #   실패 시 enable_prealign:=false 로 즉시 옛 동작(바로 SERVO) 복귀.
        self.declare_parameter('enable_prealign', True)
        self.declare_parameter('prealign_min_ey', 0.005)    # e_y 이 이내면 재배치 스킵(이미 축 위). 1cm=재배치 오픈루프 정밀바닥(회전-직진-회전, odom회전2번+마커노이즈). 5mm 재시도(2026-07-18) 폐기: PREALIGN 정밀도는 STAGE_GOTO+APPROACH(m_y)가 덮어써 결과 무영향 → 이득 없이 재배치 빈발·리밋사이클 리스크만 추가
        self.declare_parameter('prealign_max_ey', 0.10)     # 측면직진 거리 상한(안전 캡)
        self.declare_parameter('prealign_omega', 0.3)       # 90° 회전 각속도
        self.declare_parameter('prealign_v', 0.03)          # 측면 직진 속도
        self.declare_parameter('prealign_ang_tol_deg', 1.5)  # 90° 회전 완료 허용오차
        # ※ 재앵커(측정→재배치 반복)·측면 coast 보정은 실물검증(2026-07-14)으로 폐기:
        #   재앵커 iter2는 오픈루프 바닥(~1cm)서 리밋사이클(1.4→−1.0cm, 축만 넘김)로 무익,
        #   측면 coast는 3cm/s에 관성이 없어 언더슛만 유발. 1패스 후 잔차는 SERVO가 mm로 청소.

        # ── 핸드오프 (depth 주도 — 블러존 진입 전 SERVO 종료) ──────
        #   depth ≤ handoff_depth 되면 SERVO를 항상 종료. e_y ≤ handoff_max_ey 면 ALIGN→CREEP,
        #   초과면 MISALIGNED 소프트 실패(도착 정밀 envelope 밖). e_θ는 ALIGN이 null하므로 게이트서 뺌.
        #   ★ e_y는 회전 불변 → 핸드오프 시점 e_y가 곧 최종 안착 오차. handoff_max_ey는 포크 clearance
        #     기준(슬롯27/포크15 → 편측 ~6mm)으로 조여야 하나, 포크 통합 전엔 전체사이클 검증용으로 완화.
        #   (구 버그: e_y<3mm AND e_θ<5° AND-게이트 → 못 만족 시 블러존까지 무한크롤→TIMEOUT, 개루프 미실행.)
        self.declare_parameter('handoff_depth', 0.18)     # 이 거리서 SERVO→ALIGN(블러존 0.16 진입 전)
        self.declare_parameter('handoff_max_ey', 0.02)    # 핸드오프 수용 e_y 상한(초과=MISALIGNED). 포크통합 전 전체사이클 테스트용 완화(0.012→0.02). 포크 붙일 땐 6mm로 조일 것

        # ── work(전진) 도킹 = staging 방식 (2026-07-17 통합) ─────────────────
        #   법선 SERVO(coupled) 폐기. PREALIGN→STAGE_GOTO(법선 위 점 go-to)→APPROACH(마커
        #   위치 m_y 조향, 법선 무관)→CREEP. home/undock 은 아래 SERVO/SPIN180 그대로.
        self.declare_parameter('stage_depth', 0.25)          # 마커 앞 법선 위 staging 거리 D
        self.declare_parameter('stage_ang_tol_deg', 2.0)     # staging 지점 조준/정렬 허용각
        self.declare_parameter('stage_dist_tol', 0.02)       # staging 지점 도달 허용거리
        self.declare_parameter('stage_v', 0.03)              # go-to-pose 직진 속도
        self.declare_parameter('stage_k', 0.8)               # bearing 조향 게인(과보정/진동 방지)
        self.declare_parameter('stage_deadband_deg', 2.0)    # |bearing| 이내면 조향 0
        self.declare_parameter('stage_freeze_range', 0.05)   # 점 근처면 조향 멈추고 직진(atan2 발산 방지)
        self.declare_parameter('approach_speed', 0.02)       # APPROACH 직진 속도
        self.declare_parameter('stage_handoff_depth', 0.20)  # work APPROACH 종료 depth(home handoff_depth와 분리)
        self.declare_parameter('approach_k', 1.0)            # m_y bearing 조향 게인
        self.declare_parameter('approach_deadband_deg', 1.5) # |bearing| 이내면 조향 0
        # ★ 도크별 lateral 목표 = 마커의 base_link y 목표(m). 0=마커 정중앙. FSM이 도킹 직전 param set.
        self.declare_parameter('target_marker_y', 0.0)
        # ★ 마커축 ↔ 포크슬롯 중심 계통 오프셋 보정 (실물서 포크가 슬롯 한쪽으로 치우치면 조정).
        #   e_y_target = 이 값. +면 로봇 좌(+y)로 치우쳐 정지. 부호는 실물서 밀리는 반대로.
        self.declare_parameter('target_lateral_offset', 0.0)

        # ★ 도크별 마커 법선 오프셋 보정 (deg). 로봇을 슬롯에 물리적으로 직각·정중앙으로
        #   세운 채 정지 관측한 e_θ 를 그대로 넣는다(= 추정 법선이 슬롯축에서 틀어진 각).
        #   법선 (nx,ny) 를 −β 회전 → ρ·e_y·e_θ 가 전 depth에서 일관되게 슬롯축 기준이 됨.
        #   0 = 보정 없음(기존 동작). 실측 예: 창고 +2.0, 입고 +0.75.
        self.declare_parameter('marker_yaw_offset_deg', 0.0)

        # ── ALIGN (핸드오프 후 제자리 yaw 정렬 → CREEP 직진) ──────
        #   e_y(cross-track)는 제자리 회전에 불변 → e_θ만 0으로 조여 정면 확보.
        self.declare_parameter('k_align', 1.5)
        self.declare_parameter('align_done_deg', 0.8)     # |e_θ| 이 이내면 정렬 완료
        self.declare_parameter('align_timeout_sec', 5.0)

        # ── CREEP (odom 맹행 + 정체감지) ──────────────────────────
        self.declare_parameter('creep_speed', 0.015)
        self.declare_parameter('creep_yaw_hold_gain', 1.5)
        self.declare_parameter('coast_comp', 0.010)
        self.declare_parameter('max_creep_dist', 0.15)    # 크립 거리 상한(안전 캡)
        self.declare_parameter('creep_stall_eps', 0.004)  # 이 이하로만 진행하면 정체
        self.declare_parameter('creep_stall_sec', 1.2)    # 정체가 이 시간 지속되면 정지
        self.declare_parameter('creep_timeout_sec', 20.0)

        # ── UNDOCK (마커 없이 odom 후진, 고정거리) ────────────────
        self.declare_parameter('undock_dist', 0.15)         # 후진 거리(고정 — 팔레트 벗어나 노드 복귀. 뒤 노드 대기로봇 충돌 회피로 0.20→0.15)
        self.declare_parameter('undock_speed', 0.02)
        self.declare_parameter('undock_yaw_hold_gain', 1.5) # 직진 유지
        self.declare_parameter('undock_timeout_sec', 20.0)
        # ★ 테스트용: /start_dock 성공 후 이 시간(초) 대기 → 자동 언도킹(포크 작업 시뮬).
        #   기본 0=끔(FSM이 dock/undock 분리 호출). 스탠드얼론 full-cycle 테스트 시 `:=5.0`.
        self.declare_parameter('auto_undock_delay', 0.0)

        # ── 홈 후진 도킹 (reverse=True): 마커 정렬 후 제자리 180° → 후진 안착 ──
        #   포크가 로봇 뒤라 정렬(카메라 앞)과 삽입/주차(뒤) 방향이 반대 → 180° 플립 필수.
        #   home_stop_dist = 안착 시 base_link가 벽(마커)서 멈출 거리(후면 기하로 벽 안 닿게 튜닝).
        #   180°+후진은 순수 odom(마커 시야 밖) — 홈은 주차라 정밀 비필요(mm drift 무관).
        self.declare_parameter('home_stop_dist', 0.12)
        self.declare_parameter('spin_omega', 0.6)          # 180° 회전 각속도 (홈이라 빠르게, ~5s)
        self.declare_parameter('spin_timeout_sec', 15.0)   # SPIN180 자체 타임아웃 (전체 dock_timeout과 분리)
        # 홈 전용 핸드오프 e_y 상한 — work 게이트(handoff_max_ey)와 분리.
        #   홈은 주차라 실패(중단·벽앞 어정쩡 정지)가 부정밀(1~2cm 삐딱 주차)보다 나쁨 →
        #   느슨한 sanity 상한만. PREALIGN 후 실측 잔차 ~1cm(odom 90°×2)라 12mm 공유 시 턱걸이,
        #   work를 6mm로 조이면 홈 상습 실패 → 디커플링 필수.
        self.declare_parameter('home_max_ey', 0.04)

        # ── 안전/타임아웃 ─────────────────────────────────────────
        self.declare_parameter('marker_timeout_sec', 0.7)
        self.declare_parameter('dock_timeout_sec', 40.0)     # PREALIGN 단일패스(~14s)+SERVO/ALIGN/CREEP(~18s) 수용.

        # ── 라이다 벽 법선 (coupled SERVO 헤딩 소스). false면 기존 마커/staging 경로 그대로 ──
        self.declare_parameter('use_lidar_normal', False)
        self.declare_parameter('lidar_scan_topic', 'scan')
        self.declare_parameter('lidar_sector_deg', 30.0)     # 전방 ±섹터 (벽 피팅용)
        self.declare_parameter('lidar_range_min', 0.05)
        self.declare_parameter('lidar_range_max', 1.5)
        self.declare_parameter('lidar_yaw_offset_deg', 0.0)  # base_scan 마운트 yaw + 편향 보정(캘리브)
        self.declare_parameter('lidar_max_stale_sec', 0.5)   # 벽피팅 신선도 한도

        g = self.get_parameter
        self.base_frame = g('base_frame').value
        self.stop_dist = float(g('stop_dist').value)
        self.ema_alpha = float(g('ema_alpha').value)
        self.tf_timeout = float(g('tf_timeout_sec').value)
        self.log_period = float(g('log_period_sec').value)
        self.log_only = bool(g('log_only').value)
        self.k_v = float(g('k_v').value)
        self.k_y = float(g('k_y').value)
        self.k_y_far = float(g('k_y_far').value)
        self.k_y_far_depth = float(g('k_y_far_depth').value)
        self.k_theta = float(g('k_theta').value)
        self.control_dt = 1.0 / float(g('control_rate_hz').value)
        self.v_max = float(g('v_max').value)
        self.v_min = float(g('v_min').value)
        self.omega_max = float(g('omega_max').value)
        self.align_soft = math.radians(float(g('align_soft_deg').value))
        self.align_stop = math.radians(float(g('align_stop_deg').value))
        self.enable_prealign = bool(g('enable_prealign').value)
        self.prealign_min_ey = float(g('prealign_min_ey').value)
        self.prealign_max_ey = float(g('prealign_max_ey').value)
        self.prealign_omega = float(g('prealign_omega').value)
        self.prealign_v = float(g('prealign_v').value)
        self.prealign_ang_tol = math.radians(float(g('prealign_ang_tol_deg').value))
        self.handoff_depth = float(g('handoff_depth').value)
        self.handoff_max_ey = float(g('handoff_max_ey').value)
        # work staging 파라미터 캐시
        self.stage_depth = float(g('stage_depth').value)
        self.stage_ang_tol = math.radians(float(g('stage_ang_tol_deg').value))
        self.stage_dist_tol = float(g('stage_dist_tol').value)
        self.stage_v = float(g('stage_v').value)
        self.stage_k = float(g('stage_k').value)
        self.stage_deadband = math.radians(float(g('stage_deadband_deg').value))
        self.stage_freeze_range = float(g('stage_freeze_range').value)
        self.approach_speed = float(g('approach_speed').value)
        self.stage_handoff_depth = float(g('stage_handoff_depth').value)
        self.approach_k = float(g('approach_k').value)
        self.approach_deadband = math.radians(float(g('approach_deadband_deg').value))
        self.lat_offset = float(g('target_lateral_offset').value)
        self.marker_yaw_off = math.radians(float(g('marker_yaw_offset_deg').value))
        self.k_align = float(g('k_align').value)
        self.align_done = math.radians(float(g('align_done_deg').value))
        self.align_timeout = float(g('align_timeout_sec').value)
        self.creep_speed = float(g('creep_speed').value)
        self.creep_yaw_gain = float(g('creep_yaw_hold_gain').value)
        self.coast_comp = float(g('coast_comp').value)
        self.max_creep_dist = float(g('max_creep_dist').value)
        self.creep_stall_eps = float(g('creep_stall_eps').value)
        self.creep_stall_sec = float(g('creep_stall_sec').value)
        self.creep_timeout = float(g('creep_timeout_sec').value)
        self.undock_dist = float(g('undock_dist').value)
        self.undock_speed = float(g('undock_speed').value)
        self.undock_yaw_gain = float(g('undock_yaw_hold_gain').value)
        self.undock_timeout = float(g('undock_timeout_sec').value)
        self.auto_undock_delay = float(g('auto_undock_delay').value)
        self.home_stop_dist = float(g('home_stop_dist').value)
        self.spin_omega = float(g('spin_omega').value)
        self.spin_timeout = float(g('spin_timeout_sec').value)
        self.home_max_ey = float(g('home_max_ey').value)
        self.marker_timeout = float(g('marker_timeout_sec').value)
        self.dock_timeout = float(g('dock_timeout_sec').value)
        self.use_lidar_normal = bool(g('use_lidar_normal').value)
        self.lidar_scan_topic = g('lidar_scan_topic').value
        self.lidar_sector = math.radians(float(g('lidar_sector_deg').value))
        self.lidar_rmin = float(g('lidar_range_min').value)
        self.lidar_rmax = float(g('lidar_range_max').value)
        self.lidar_yaw_off = math.radians(float(g('lidar_yaw_offset_deg').value))
        self.lidar_max_stale = float(g('lidar_max_stale_sec').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── 상태 (락 보호) ────────────────────────────────────────
        self._lock = threading.Lock()
        self._state = 'IDLE'          # IDLE/PREALIGN/[work]STAGE_GOTO/APPROACH/[home]SERVO/ALIGN/CREEP/SPIN180/UNDOCK/DONE/LOST/TIMEOUT/STALL/MISALIGNED
        self._align_t0 = None
        # PREALIGN 서브페이즈 상태 (control 스레드 단독 갱신)
        self._pa_phase = None         # FACE/TURN1/DRIVE/TURN2
        self._pa_ey_target = 0.0      # 측면 이동 목표 거리
        self._pa_dir = 1.0            # +1=좌(CCW) / −1=우(CW)
        self._pa_yaw0 = 0.0           # 회전 기준 odom yaw (PREALIGN 회전 + SPIN180 공용)
        self._pa_pos0 = None          # 측면직진 기준 odom pos
        self._reverse = False         # 홈 후진 도킹 플래그 (ALIGN 후 SPIN180→후진 안착)
        self._reverse_dist = 0.0      # 후진 안착 거리 (ALIGN서 마커 보며 캡처)
        self._target_y = 0.0          # work 도크별 lateral 목표 (도킹 시작 시 param에서 캡처)
        self._ema_rho = None
        self._ema_ey = None
        self._ema_eth = None
        self._ema_depth = None
        self._ema_mx = None           # work staging: 마커 위치/법선 EMA
        self._ema_my = None
        self._ema_nx = None
        self._ema_ny = None
        self._sg_phase = None         # STAGE_GOTO 서브페이즈: AIM/DRIVE/FACE
        self._last_pose_t = None
        self._start_t = None
        self._got_pose = False
        self._odom = None             # (x, y, yaw)
        self._creep_start = None      # (x0, y0, yaw0)
        self._creep_dist = None
        self._creep_t0 = None
        self._creep_max = 0.0         # 정체감지: 지금까지 최대 진행거리
        self._creep_prog_t = None     # 정체감지: 마지막 진행 시각
        self._wall_nx = None          # 라이다 벽 법선(base_link, 마커 규약=벽→로봇)
        self._wall_ny = None
        self._wall_eth = None         # 라이다 헤딩오차(비교/로그용)
        self._wall_t = None           # 최근 벽피팅 시각(신선도)

        cb = ReentrantCallbackGroup()
        self.cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.create_subscription(
            PoseStamped, 'detected_dock_pose', self.pose_cb, 10, callback_group=cb)
        self.create_subscription(Odometry, 'odom', self.odom_cb, 10, callback_group=cb)
        if self.use_lidar_normal:
            # /scan은 BEST_EFFORT(센서 QoS) → 반드시 맞춰야 수신됨
            self.create_subscription(LaserScan, self.lidar_scan_topic, self.scan_cb,
                                     qos_profile_sensor_data, callback_group=cb)
        self.create_timer(self.control_dt, self.control_step, callback_group=cb)
        if not self.log_only:
            self.create_service(Trigger, 'start_work_dock', self.on_start_work_dock, callback_group=cb)
            self.create_service(Trigger, 'start_home_dock', self.on_start_home_dock, callback_group=cb)
            self.create_service(Trigger, 'start_undock', self.on_start_undock, callback_group=cb)

        mode = '로깅 전용(제어 없음)' if self.log_only else '제어(work전진/home후진/undock)'
        self.get_logger().info(
            f"dock_controller [{mode}] up | base={self.base_frame} stop_dist={self.stop_dist}m "
            f"k=(v{self.k_v},y{self.k_y_far}~{self.k_y},θ{self.k_theta}) v_max={self.v_max} handoff_depth={self.handoff_depth} "
            f"creep={self.creep_speed} max_creep={self.max_creep_dist} "
            f"marker_yaw_off={math.degrees(self.marker_yaw_off):+.2f}° "
            f"prealign_min_ey={self.prealign_min_ey} lat_off={self.lat_offset:+.4f}m "
            f"| ω=−k_y·e_y+k_θ·e_θ "
            f"| lidar_normal={self.use_lidar_normal} "
            f"lidar_off={math.degrees(self.lidar_yaw_off):+.2f}°")
        # 라이다 켰는데 편향 미보정이면 헤딩 영점이 틀어져 크루키 도킹 위험 → 경고
        if self.use_lidar_normal and abs(self.lidar_yaw_off) < 1e-6:
            self.get_logger().warn(
                'use_lidar_normal=true 인데 lidar_yaw_offset_deg=0 (미보정) — '
                '로봇별 라이다 마운트 편향값을 넣어야 헤딩이 진짜 수직에 맞음')

    # ── EMA ────────────────────────────────────────────────────────
    def _ema(self, prev, new):
        if prev is None or self.ema_alpha >= 1.0:
            return new
        a = self.ema_alpha
        return a * new + (1.0 - a) * prev

    def _ema_angle(self, prev, new):
        if prev is None or self.ema_alpha >= 1.0:
            return new
        a = self.ema_alpha
        s = a * math.sin(new) + (1.0 - a) * math.sin(prev)
        c = a * math.cos(new) + (1.0 - a) * math.cos(prev)
        return math.atan2(s, c)

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        yaw = _yaw_from_quat(msg.pose.pose.orientation)
        with self._lock:
            self._odom = (float(p.x), float(p.y), yaw)

    def scan_cb(self, s: LaserScan):
        """전방 섹터 직선피팅 → 벽 법선(base_link) → e_θ. use_lidar_normal일 때만 구독됨.

        검증 완료(2026-07-20): 정지 σ0.17°·잔차1.3mm, 회전추종 기울기 −1.00.
        ※ base_scan 이 base_link 와 yaw 정렬됐다고 가정하고 lidar_yaw_off 로 마운트/편향 보정.
          (평행이동은 법선각에 무영향.) 스캔프레임이 yaw로 돌아있으면 그 값을 lidar_yaw_off 에.
        """
        half = self.lidar_sector
        pts = []
        for i, r in enumerate(s.ranges):
            if math.isinf(r) or math.isnan(r) or not (self.lidar_rmin < r < self.lidar_rmax):
                continue
            a = s.angle_min + i * s.angle_increment
            if abs(math.atan2(math.sin(a), math.cos(a))) > half:   # 전방 0 주변 wrap 처리
                continue
            pts.append((r * math.cos(a), r * math.sin(a)))
        if len(pts) < 8:
            self.get_logger().warn(f'라이다 전방 섹터 점 {len(pts)}개 — 벽 미검출',
                                   throttle_duration_sec=2.0)
            return
        P = np.array(pts)
        _, v = np.linalg.eigh(np.cov((P - P.mean(axis=0)).T))
        n = v[:, 0]                          # 최소 고유값 고유벡터 = 법선
        if n[0] < 0.0:
            n = -n                           # n = 로봇→벽 (+x)
        # base_scan yaw 마운트/편향 보정
        c, si = math.cos(self.lidar_yaw_off), math.sin(self.lidar_yaw_off)
        nx, ny = c * n[0] - si * n[1], si * n[0] + c * n[1]
        # 마커 법선 규약(벽→로봇)에 맞춰 반전 → _extract_errors 공식 그대로 재사용 가능
        # TODO(HW): 이 부호는 실물서 로그의 라이다 e_θ 가 마커 e_θ 와 같은 부호인지로 확정.
        nx, ny = -nx, -ny
        eth = _norm(math.atan2(-ny, -nx))    # 마커 e_θ 와 동일 정의(수직이면 0)
        with self._lock:
            self._wall_nx, self._wall_ny, self._wall_eth = nx, ny, eth
            self._wall_t = time.time()
        self.get_logger().info(f'[wall] e_θ={math.degrees(eth):+.2f}° pts={len(pts)}',
                               throttle_duration_sec=self.log_period)

    # ── pose 콜백: TF → 오차 → EMA → 저장 + 로그 ───────────────────
    def pose_cb(self, msg: PoseStamped):
        if not self._got_pose:
            self._got_pose = True
            self.get_logger().info(f"첫 detected_dock_pose 수신 (frame_id='{msg.header.frame_id}')")
        if not msg.header.frame_id:
            self.get_logger().warn("pose frame_id 비어있음 → TF 변환 불가", throttle_duration_sec=2.0)
            return

        query = PoseStamped()
        query.header.frame_id = msg.header.frame_id
        query.header.stamp = rclpy.time.Time().to_msg()
        query.pose = msg.pose
        try:
            pose_base = self.tf_buffer.transform(
                query, self.base_frame,
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout))
        except TransformException as e:
            self.get_logger().warn(
                f"TF {msg.header.frame_id}→{self.base_frame} 실패: {e} (static TF 확인)",
                throttle_duration_sec=2.0)
            return

        ext = self._extract_errors(pose_base)
        if ext is None:
            return
        rho, e_y, e_theta, depth, (nx, ny), (mx, my) = ext

        with self._lock:
            self._ema_rho = self._ema(self._ema_rho, rho)
            self._ema_ey = self._ema(self._ema_ey, e_y)
            self._ema_eth = self._ema_angle(self._ema_eth, e_theta)
            self._ema_depth = self._ema(self._ema_depth, depth)
            # work staging EMA(마커 위치/법선)는 work 도킹(reverse=False)에서만 소비 →
            #   home 도킹 중엔 갱신 생략. (work는 PREALIGN부터 갱신돼 STAGE_GOTO서 신선)
            if not self._reverse:
                self._ema_mx = self._ema(self._ema_mx, mx)
                self._ema_my = self._ema(self._ema_my, my)
                self._ema_nx = self._ema(self._ema_nx, nx)
                self._ema_ny = self._ema(self._ema_ny, ny)
            self._last_pose_t = time.time()
            fr, fy, ft, st = self._ema_rho, self._ema_ey, self._ema_eth, self._state

        self.get_logger().info(
            f"[{st}] ρ={rho:.3f} e_y={e_y:+.4f} e_θ={math.degrees(e_theta):+.1f}° "
            f"depth={depth:.3f} | filt ρ={fr:.3f} e_y={fy:+.4f} e_θ={math.degrees(ft):+.1f}°",
            throttle_duration_sec=self.log_period)

    def _extract_errors(self, pose_base: PoseStamped):
        p = pose_base.pose.position
        q = pose_base.pose.orientation
        m_x, m_y = float(p.x), float(p.y)
        depth = math.hypot(m_x, m_y)

        nx = 2.0 * (q.x * q.z + q.w * q.y)
        ny = 2.0 * (q.y * q.z - q.w * q.x)
        n = math.hypot(nx, ny)
        if n < 1e-9:
            return None
        nx, ny = nx / n, ny / n
        if nx * (-m_x) + ny * (-m_y) < 0.0:
            nx, ny = -nx, -ny
        # ★ raw 법선(트림 미적용) — work STAGE_GOTO 전용 반환. work는 marker_yaw_off/lat_offset
        #   (home 법선 기반 트림)에 오염되면 안 됨(staging 설계 = 트림 불필요, context/13 §10).
        nx_raw, ny_raw = nx, ny

        # 도크별 마커 오프셋 보정(home 전용): 법선을 −β 회전 → 아래 ρ·e_y·e_θ 가 슬롯축 기준.
        #   work는 위 raw 법선을 쓰므로 이 회전에 영향 안 받음.
        if self.marker_yaw_off != 0.0:
            c, s = math.cos(-self.marker_yaw_off), math.sin(-self.marker_yaw_off)
            nx, ny = c * nx - s * ny, s * nx + c * ny

        sx = m_x + self.stop_dist * nx
        sy = m_y + self.stop_dist * ny
        rho = math.hypot(sx, sy)
        # cross-track(부호) − 슬롯중심 계통 오프셋 보정 → 목표를 '슬롯축' 위로 (home SERVO/PREALIGN용)
        e_y = (m_y * nx - m_x * ny) - self.lat_offset
        e_theta = _norm(math.atan2(-ny, -nx))

        # ── 라이다 법선으로 e_y·e_θ 교체 (마커 위치 m_x·m_y·depth 는 유지) ──
        #   축(법선)만 정확한 라이다로 → 편향 3.7° 제거. 신선한 벽피팅 있을 때만.
        #   ★ work(전진)에만: 라이다 전방 섹터는 앞 벽 → home(후진, 뒤 벽 도킹)엔 부적합.
        if self.use_lidar_normal and not self._reverse:
            with self._lock:
                wnx, wny, wt = self._wall_nx, self._wall_ny, self._wall_t
            if wnx is not None and wt is not None and (time.time() - wt) < self.lidar_max_stale:
                nx, ny = wnx, wny
                e_y = (m_y * nx - m_x * ny) - self.lat_offset
                e_theta = _norm(math.atan2(-ny, -nx))

        # 반환 (nx,ny)=raw(work EMA 소비) / e_y·e_θ·ρ=트림 적용(home 소비)
        return rho, e_y, e_theta, depth, (nx_raw, ny_raw), (m_x, m_y)

    def control_step(self):
        with self._lock:
            state = self._state
        if state == 'PREALIGN':
            self._prealign_step()
        elif state == 'STAGE_GOTO':
            self._stage_goto_step()
        elif state == 'APPROACH':
            self._approach_step()
        elif state == 'SERVO':
            self._servo_step()
        elif state == 'ALIGN':
            self._align_step()
        elif state == 'CREEP':
            self._creep_step()
        elif state == 'SPIN180':
            self._spin180_step()
        elif state == 'UNDOCK':
            self._undock_step()

    def _sched_ky(self, depth):
        """거리 스케줄 k_y: 원거리=k_y_far(완만·무포화·마커유지) → 근접=k_y(타이트).
        handoff_depth(근접)~k_y_far_depth(원거리) 선형보간. 초반 큰 e_y에도 ω 포화 방지."""
        lo, hi = self.handoff_depth, self.k_y_far_depth
        if hi <= lo or depth <= lo:
            return self.k_y
        if depth >= hi:
            return self.k_y_far
        t = (depth - lo) / (hi - lo)              # 0(근접)→1(원거리)
        return self.k_y + t * (self.k_y_far - self.k_y)

    # ── PREALIGN: 마커로 e_y 측정 → 법선 위로 수직 측면이동 → SERVO ──
    #   FACE(마커정면 폐루프)→TURN1(90° odom)→DRIVE(e_y 측면 odom)→TURN2(−90° odom)→SERVO.
    #   TURN1~TURN2는 마커가 시야 밖이라 순수 odom(마커 체크 X). 측면이동=법선수직 → depth 소모 0.
    def _prealign_step(self):
        with self._lock:
            phase = self._pa_phase
            ey, eth = self._ema_ey, self._ema_eth
            last_t, odom, start_t = self._last_pose_t, self._odom, self._start_t
        now = time.time()
        if start_t is not None and (now - start_t) > self.dock_timeout:
            self._stop(); self._set_state('TIMEOUT')
            self.get_logger().warn("PREALIGN 타임아웃 → 정지 · TIMEOUT")
            return
        if odom is None:
            return

        if phase == 'FACE':
            # 마커 폐루프로 정면(e_θ→0) 확보 후 e_y 캡처. 마커 살아있어야 함.
            if last_t is None or (now - last_t) > self.marker_timeout:
                self._stop(); self._set_state('LOST')
                self.get_logger().warn("마커 소실(PREALIGN/FACE) → 정지 · LOST")
                return
            if ey is None or eth is None:
                return
            if abs(eth) >= self.align_done:
                omega = _clamp(self.k_align * eth, -self.omega_max, self.omega_max)
                self._publish(0.0, omega)
                return
            # 정면 확보 → 이미 축 위(min_ey 이내)면 재배치 스킵하고 SERVO 직행, 아니면 e_y 캡처 후 측면 재배치.
            if abs(ey) < self.prealign_min_ey:
                self.get_logger().info(
                    f"PREALIGN 종료 (e_y={ey:+.4f}, 축 위) → "
                    f"{'SERVO' if (self._reverse or self.use_lidar_normal) else 'STAGE_GOTO'}")
                self._enter_after_prealign()
                return
            eyt = _clamp(abs(ey), 0.0, self.prealign_max_ey)
            direction = 1.0 if ey < 0.0 else -1.0   # e_y<0=로봇 축 오른쪽 → 좌(+y,CCW)로 이동
            with self._lock:
                self._pa_ey_target = eyt
                self._pa_dir = direction
                self._pa_yaw0 = odom[2]
                self._pa_phase = 'TURN1'
            self._stop()
            self.get_logger().info(
                f"PREALIGN 정면확보(e_θ={math.degrees(eth):+.1f}°) → 측면 {eyt*100:.1f}cm "
                f"{'좌' if direction > 0 else '우'}이동 (수직 재배치, depth 보존)")
            return

        if phase == 'TURN1':
            turned = _norm(odom[2] - self._pa_yaw0)
            if turned * self._pa_dir >= (math.pi / 2 - self.prealign_ang_tol):
                with self._lock:
                    self._pa_pos0 = (odom[0], odom[1])
                    self._creep_max = 0.0
                    self._creep_prog_t = now
                    self._pa_phase = 'DRIVE'
                self._stop()
                self.get_logger().info("PREALIGN TURN1(90°) 완료 → 측면 직진")
                return
            self._publish(0.0, self._pa_dir * self.prealign_omega)
            return

        if phase == 'DRIVE':
            if self._pa_pos0 is None:
                return
            traveled = math.hypot(odom[0] - self._pa_pos0[0], odom[1] - self._pa_pos0[1])
            # 목표 e_y만큼 측면직진. 잔차(회전·정지 오차)는 이어지는 SERVO가 폐루프로 청소.
            if traveled >= self._pa_ey_target:
                with self._lock:
                    self._pa_yaw0 = odom[2]
                    self._pa_phase = 'TURN2'
                self._stop()
                self.get_logger().info(f"PREALIGN 측면 {traveled*100:.1f}cm 완료 → 복귀 회전")
                return
            # 정체 감지(측면 이동 중 충돌/막힘) — creep 파라미터 재사용.
            if self._stalled(traveled, self._creep_max, self._creep_prog_t, now):
                self._stop(); self._set_state('STALL')
                self.get_logger().error(
                    f"PREALIGN 측면 정체({traveled:.3f}/{self._pa_ey_target:.3f}m) → STALL (옆 막힘 의심)")
                return
            self._publish(self.prealign_v, 0.0)
            return

        if phase == 'TURN2':
            turned = _norm(odom[2] - self._pa_yaw0)
            if turned * (-self._pa_dir) >= (math.pi / 2 - self.prealign_ang_tol):
                self._stop()
                self.get_logger().info(
                    "PREALIGN TURN2(−90°) 완료 → "
                    f"{'SERVO' if (self._reverse or self.use_lidar_normal) else 'STAGE_GOTO'}")
                self._enter_after_prealign()
                return
            self._publish(0.0, -self._pa_dir * self.prealign_omega)
            return

    def _enter_servo(self):
        """PREALIGN 종료 → SERVO 인계. EMA 리셋(마커 재검출), 재획득 유예 부여(즉시 LOST 방지)."""
        now = time.time()
        with self._lock:
            self._ema_rho = self._ema_ey = self._ema_eth = self._ema_depth = None
            self._last_pose_t = now
            self._state = 'SERVO'

    # ── SERVO: 폐루프 접근 → 핸드오프(depth 기준)서 CREEP 전환 ─────
    def _servo_step(self):
        with self._lock:
            rho, ey, eth, depth = self._ema_rho, self._ema_ey, self._ema_eth, self._ema_depth
            last_t, start_t = self._last_pose_t, self._start_t
        if rho is None or depth is None:
            return
        now = time.time()
        if last_t is None or (now - last_t) > self.marker_timeout:
            self._stop(); self._set_state('LOST')
            self.get_logger().warn("마커 소실(pose stale) → 정지 · LOST")
            return
        if start_t is not None and (now - start_t) > self.dock_timeout:
            self._stop(); self._set_state('TIMEOUT')
            self.get_logger().warn("도킹 타임아웃 → 정지 · TIMEOUT")
            return

        # 핸드오프 = depth 주도(블러존 0.16 진입 전 SERVO 종료). depth 도달 시 항상 SERVO 탈출:
        #   e_y 수용 → ALIGN(제자리 yaw 정렬, e_y 불변)→CREEP / 초과 → MISALIGNED 소프트 실패.
        #   수용 한도: work=handoff_max_ey(포크 물리 요구) / home=home_max_ey(느슨한 sanity —
        #   홈은 실패가 부정밀보다 나쁨. work 게이트 조여도 홈 무영향).
        if depth <= self.handoff_depth:
            max_ey = self.home_max_ey if self._reverse else self.handoff_max_ey
            if abs(ey) <= max_ey:
                with self._lock:
                    self._align_t0 = now
                    self._state = 'ALIGN'
                self.get_logger().info(
                    f"핸드오프 (depth={depth:.3f} e_y={ey:+.4f} e_θ={math.degrees(eth):+.1f}°) "
                    f"→ ALIGN (제자리 yaw 정렬)")
            else:
                self._stop(); self._set_state('MISALIGNED')
                self.get_logger().warn(
                    f"핸드오프 지점(depth={depth:.3f}) e_y={ey:+.4f} > 수용 {max_ey:.3f} "
                    f"→ MISALIGNED (도착 정밀 envelope 밖 — 시작 오프셋 축소 필요)")
            return

        ky = self._sched_ky(depth)
        omega = -ky * ey + self.k_theta * eth
        v = _clamp(self.k_v * rho, self.v_min, self.v_max)
        ae = abs(eth)
        if ae >= self.align_stop:
            v = 0.0
        elif ae > self.align_soft:
            v *= (self.align_stop - ae) / (self.align_stop - self.align_soft)
        omega = _clamp(omega, -self.omega_max, self.omega_max)
        self._publish(v, omega)

    # ── ALIGN: 제자리 회전으로 e_θ→0 (e_y 불변) → CREEP ──────────
    def _align_step(self):
        with self._lock:
            eth, depth = self._ema_eth, self._ema_depth
            last_t, odom, t0 = self._last_pose_t, self._odom, self._align_t0
        if eth is None or depth is None:
            return
        now = time.time()
        if last_t is None or (now - last_t) > self.marker_timeout:
            self._stop(); self._set_state('LOST')
            self.get_logger().warn("마커 소실(ALIGN 중) → 정지 · LOST")
            return
        if odom is None:
            self.get_logger().warn("odom 없음 → 다음 단계 불가, 대기", throttle_duration_sec=2.0)
            return
        # 정렬 완료 or 타임아웃 → 전진 CREEP(work) / 후진 SPIN180(home) 진입
        timed_out = t0 is not None and (now - t0) > self.align_timeout
        if abs(eth) < self.align_done or timed_out:
            nxt = 'SPIN180(후진)' if self._reverse else 'CREEP(전진)'
            if timed_out and abs(eth) >= self.align_done:
                self.get_logger().warn(f"ALIGN 타임아웃 (e_θ={math.degrees(eth):+.1f}°) → {nxt}")
            else:
                self.get_logger().info(f"yaw 정렬 완료 (e_θ={math.degrees(eth):+.1f}°) → {nxt}")
            if self._reverse:
                self._enter_spin180(depth, odom)
            else:
                self._enter_creep(depth, odom)
            return
        # 제자리 회전 (v=0). ω=k_align·e_θ 로 e_θ→0. cross-track e_y는 회전 불변.
        omega = _clamp(self.k_align * eth, -self.omega_max, self.omega_max)
        self._publish(0.0, omega)

    def _enter_creep(self, depth, odom):
        creep_dist = _clamp(depth - self.stop_dist - self.coast_comp, 0.0, self.max_creep_dist)
        now = time.time()
        with self._lock:
            self._creep_start = odom
            self._creep_dist = creep_dist
            self._creep_t0 = now
            self._creep_max = 0.0
            self._creep_prog_t = now
            self._state = 'CREEP'
        self.get_logger().info(f"→ CREEP {creep_dist:.3f}m (odom 맹행, 정체감지 ON)")

    # ── SPIN180: 홈 후진 도킹 — 후진거리 캡처 → 제자리 180° → 후진 안착(UNDOCK 재사용) ──
    #   포크가 로봇 뒤라 정렬은 마커 보며(앞), 안착은 뒤로 → 중간에 180° 플립. 순수 odom.
    def _enter_spin180(self, depth, odom):
        rev_dist = _clamp(depth - self.home_stop_dist - self.coast_comp, 0.0, self.max_creep_dist)
        with self._lock:
            self._reverse_dist = rev_dist
            self._pa_yaw0 = odom[2]      # 180° 회전 기준 yaw
            self._pa_dir = 1.0           # CCW 180° (방향 무관 — 홈은 어느 쪽이든 벽 등짐)
            self._creep_t0 = time.time()  # SPIN180 자체 타임아웃 기준(전체 dock_timeout과 분리)
            self._state = 'SPIN180'
        self.get_logger().info(f"→ SPIN180 (제자리 180° → 후진 {rev_dist:.3f}m 홈 안착)")

    def _spin180_step(self):
        with self._lock:
            odom, spin_t0 = self._odom, self._creep_t0
            yaw0, rev_dist, direction = self._pa_yaw0, self._reverse_dist, self._pa_dir
        if odom is None:
            return
        now = time.time()
        if spin_t0 is not None and (now - spin_t0) > self.spin_timeout:
            self._stop(); self._set_state('TIMEOUT')
            self.get_logger().warn("SPIN180 타임아웃 → 정지 · TIMEOUT")
            return
        # |누적회전| ≈ 180° (odom yaw는 ±π wrap → abs(_norm)는 0→π 단조증가)
        if abs(_norm(odom[2] - yaw0)) >= (math.pi - self.prealign_ang_tol):
            with self._lock:                 # 후진 안착 = UNDOCK 프리미티브 재사용(캡처거리)
                self._creep_start = odom
                self._creep_dist = rev_dist
                self._creep_t0 = now
                self._creep_max = 0.0
                self._creep_prog_t = now
                self._state = 'UNDOCK'
            self._stop()
            self.get_logger().info(f"SPIN180 완료 → 후진 {rev_dist:.3f}m (홈 안착)")
            return
        self._publish(0.0, direction * self.spin_omega)

    def _stalled(self, traveled, cmax, prog_t, now):
        """odom 무진행 감지 (CREEP/UNDOCK/PREALIGN 공용). 진전 있으면 진행상태 갱신 후
        False, creep_stall_sec 동안 무진행이면 True(=충돌/막힘, 호출부가 STALL 처리)."""
        if traveled > cmax + self.creep_stall_eps:
            with self._lock:
                self._creep_max = traveled
                self._creep_prog_t = now
            return False
        return prog_t is not None and (now - prog_t) > self.creep_stall_sec

    # ── CREEP: odom 직진 + 정체감지 → 정지점서 DONE ───────────────
    def _creep_step(self):
        with self._lock:
            odom, start, target, t0 = self._odom, self._creep_start, self._creep_dist, self._creep_t0
            cmax, prog_t = self._creep_max, self._creep_prog_t
        if odom is None or start is None:
            return
        now = time.time()
        if t0 is not None and (now - t0) > self.creep_timeout:
            self._stop(); self._set_state('TIMEOUT')
            self.get_logger().warn("CREEP 타임아웃 → 정지 · TIMEOUT")
            return

        traveled = math.hypot(odom[0] - start[0], odom[1] - start[1])

        # 도달
        if traveled >= target:
            self._stop(); self._set_state('DONE')
            self.get_logger().info(
                f"CREEP 완료 ({traveled:.3f}/{target:.3f}m) → DONE (마커 앞 ~{self.stop_dist}m)")
            return

        if self._stalled(traveled, cmax, prog_t, now):
            self._stop(); self._set_state('STALL')
            self.get_logger().error(
                f"CREEP 정체 감지({traveled:.3f}/{target:.3f}m, {self.creep_stall_sec}s 무진행) "
                f"→ 정지 · STALL (충돌/막힘 의심 — stop_dist 확인)")
            return

        yaw_err = _norm(odom[2] - start[2])
        omega = _clamp(-self.creep_yaw_gain * yaw_err, -self.omega_max, self.omega_max)
        self._publish(self.creep_speed, omega)

    # ── UNDOCK: odom 후진 직진(고정거리) + 정체감지 → DONE ────────
    def _undock_step(self):
        with self._lock:
            odom, start, target, t0 = self._odom, self._creep_start, self._creep_dist, self._creep_t0
            cmax, prog_t = self._creep_max, self._creep_prog_t
        if odom is None or start is None:
            return
        now = time.time()
        if t0 is not None and (now - t0) > self.undock_timeout:
            self._stop(); self._set_state('TIMEOUT')
            self.get_logger().warn("UNDOCK 타임아웃 → 정지 · TIMEOUT")
            return

        traveled = math.hypot(odom[0] - start[0], odom[1] - start[1])
        if traveled >= target:
            self._stop(); self._set_state('DONE')
            self.get_logger().info(f"UNDOCK 완료 ({traveled:.3f}/{target:.3f}m 후진) → DONE")
            return

        if self._stalled(traveled, cmax, prog_t, now):
            self._stop(); self._set_state('STALL')
            self.get_logger().error(
                f"UNDOCK 정체({traveled:.3f}/{target:.3f}m) → 정지 · STALL (뒤 막힘 의심)")
            return

        # 후진(v 음수) + yaw hold로 곧게.
        yaw_err = _norm(odom[2] - start[2])
        omega = _clamp(-self.undock_yaw_gain * yaw_err, -self.omega_max, self.omega_max)
        self._publish(-self.undock_speed, omega)

    # ── 트리거 서비스 ──────────────────────────────────────────────
    #   work(전진)·home(후진) 공용 `_run_dock(reverse)`. 서비스는 얇은 래퍼(모드 명시 — 모드를
    #   param 상태로 나르지 않음). work=PREALIGN→STAGE_GOTO(staging) / home=PREALIGN→SERVO(_reverse).
    def on_start_work_dock(self, request, response):
        return self._run_dock(False, response)

    def on_start_home_dock(self, request, response):
        return self._run_dock(True, response)

    def _run_dock(self, reverse, response):
        """공용 도킹 드라이버: 상태머신 시작 → 종료까지 블록. reverse=True면 ALIGN 후 SPIN180+후진."""
        with self._lock:
            # busy 가드: 진행 중 하이재킹 방지 (수동 테스트 콜·타임아웃 후 재시도 등.
            # FSM 정상 흐름은 순차 호출이라 안 걸림 — 자기 불변식 방어).
            if self._state != 'IDLE':
                response.success = False
                response.message = f'busy ({self._state} 진행 중)'
                return response
            if self._last_pose_t is None or (time.time() - self._last_pose_t) > self.marker_timeout:
                response.success = False
                response.message = 'no fresh pose (마커 안 보임)'
                return response
            self._reverse = reverse
            self._ema_rho = self._ema_ey = self._ema_eth = self._ema_depth = None
            # work staging EMA도 리셋 — 직전 도크의 stale 법선/위치가 첫 STAGE_GOTO 지점에 새는 것 방지.
            self._ema_mx = self._ema_my = self._ema_nx = self._ema_ny = None
            # 도크별 lateral 목표는 도킹 시작 시 1회 캡처(FSM이 서비스 호출 직전 param set).
            #   파라미터 콜백 부재(§8.6) 우회 — APPROACH 핫루프서 매 틱 get_parameter 안 함.
            self._target_y = float(self.get_parameter('target_marker_y').value)
            self._start_t = time.time()
            if self.enable_prealign:
                self._pa_phase = 'FACE'
                self._state = 'PREALIGN'
            elif reverse:
                self._state = 'SERVO'          # home: 법선 SERVO
            else:
                self._sg_phase = 'AIM'          # work: staging
                self._state = 'STAGE_GOTO'
        kind = 'HOME(후진)' if reverse else 'WORK(전진/staging)'
        entry = 'PREALIGN' if self.enable_prealign else ('SERVO' if reverse else 'STAGE_GOTO')
        self.get_logger().info(f"{kind} 도킹 시작 ({entry})")

        while rclpy.ok():
            with self._lock:
                st = self._state
            if st in ('DONE', 'LOST', 'TIMEOUT', 'STALL', 'MISALIGNED'):
                break
            time.sleep(0.05)

        self._stop()
        with self._lock:
            st = self._state
            self._state = 'IDLE'
        response.success = (st == 'DONE')
        response.message = st
        self.get_logger().info(f"{kind} 도킹 종료: {st} (success={response.success})")

        # 테스트용 자동 언도킹 = work(전진)만. home(후진)은 그 자체가 종점(주차).
        if not reverse and response.success and self.auto_undock_delay > 0.0:
            self.get_logger().info(
                f"[테스트] 포크 작업 대기 {self.auto_undock_delay}s → 자동 언도킹")
            time.sleep(self.auto_undock_delay)
            ust = self._run_undock()
            self.get_logger().info(f"[테스트] 자동 언도킹 종료: {ust}")
        return response

    def _run_undock(self):
        """UNDOCK 실행 → 종료 상태(문자열) 반환. odom 없으면 None, 진행 중이면 'BUSY'."""
        now = time.time()
        with self._lock:
            if self._state != 'IDLE':     # busy 가드 (_run_dock과 동일)
                return 'BUSY'
            odom = self._odom
            if odom is None:
                return None
            self._creep_start = odom
            self._creep_dist = self.undock_dist
            self._creep_t0 = now
            self._creep_max = 0.0
            self._creep_prog_t = now
            self._state = 'UNDOCK'
        self.get_logger().info(f"UNDOCK 시작 ({self.undock_dist}m 후진)")
        while rclpy.ok():
            with self._lock:
                st = self._state
            if st in ('DONE', 'TIMEOUT', 'STALL'):
                break
            time.sleep(0.05)
        self._stop()
        with self._lock:
            st = self._state
            self._state = 'IDLE'
        return st

    def on_start_undock(self, request, response):
        st = self._run_undock()
        if st is None:
            response.success = False
            response.message = 'no odom'
            return response
        response.success = (st == 'DONE')
        response.message = st
        self.get_logger().info(f"언도킹 종료: {st} (success={response.success})")
        return response

    # ══ work(전진) staging 경로 (2026-07-17 통합) ═══════════════════════════
    #   PREALIGN 종료 → STAGE_GOTO(법선 위 점 go-to) → APPROACH(m_y 조향) → CREEP.
    #   home(reverse)은 _enter_servo 로 빠져 기존 법선 SERVO 유지.
    def _enter_after_prealign(self):
        if self._reverse:
            self._enter_servo()          # home: EMA 리셋 + SERVO
        elif self.use_lidar_normal:
            # work + 라이다: 구버전 coupled SERVO 복귀(법선=라이다). staging 우회 → 헤딩 연속 제어.
            self._enter_servo()
        else:
            self._enter_stage_goto()     # work 기본: EMA 유지 + STAGE_GOTO

    def _timed_out(self, now):
        return self._start_t is not None and (now - self._start_t) > self.dock_timeout

    def _marker_lost(self, now, last_t):
        return last_t is None or (now - last_t) > self.marker_timeout

    def _fail(self, state, reason):
        self._stop()
        self._set_state(state)
        self.get_logger().warn(f"{reason} → 정지 · {state}")

    def _enter_stage_goto(self):
        now = time.time()
        with self._lock:
            self._last_pose_t = now      # 마커 재검출 유예(PREALIGN 회전 후 EMA 신선화)
            self._sg_phase = 'AIM'
            self._state = 'STAGE_GOTO'
        self.get_logger().info(f"→ STAGE_GOTO (법선 위 {self.stage_depth}m 지점)")

    def _stage_point(self):
        """staging 지점(base_link) = 마커 + D·법선. 필터값 없으면 None."""
        with self._lock:
            mx, my = self._ema_mx, self._ema_my
            nx, ny = self._ema_nx, self._ema_ny
        if mx is None or nx is None:
            return None
        return mx + self.stage_depth * nx, my + self.stage_depth * ny

    def _stage_goto_step(self):
        with self._lock:
            phase = self._sg_phase
            eth, last_t, odom = self._ema_eth, self._last_pose_t, self._odom
        now = time.time()
        if self._timed_out(now):
            self._fail('TIMEOUT', 'STAGE_GOTO 타임아웃')
            return
        if self._marker_lost(now, last_t):
            self._fail('LOST', '마커 소실(STAGE_GOTO)')
            return
        if odom is None:
            return
        sp = self._stage_point()
        if sp is None:
            return
        sx, sy = sp
        rng = math.hypot(sx, sy)
        bearing = math.atan2(sy, sx)

        if phase == 'AIM':
            if abs(bearing) < self.stage_ang_tol or rng < self.stage_freeze_range:
                with self._lock:
                    self._sg_phase = 'DRIVE'
                self._stop()
                return
            self._publish(0.0, _clamp(self.stage_k * bearing, -self.omega_max, self.omega_max))
            return

        if phase == 'DRIVE':
            if rng < self.stage_dist_tol:
                with self._lock:
                    self._sg_phase = 'FACE'
                self._stop()
                self.get_logger().info(f"STAGE_GOTO 지점 도달(잔여 {rng*100:.1f}cm) → 마커 yaw 정렬")
                return
            if rng < self.stage_freeze_range or abs(bearing) < self.stage_deadband:
                omega = 0.0
            else:
                omega = _clamp(self.stage_k * bearing, -self.omega_max, self.omega_max)
            self._publish(self.stage_v, omega)
            return

        if phase == 'FACE':
            if eth is None:
                return
            if abs(eth) < self.align_done:
                self._stop()
                self.get_logger().info(
                    f"STAGE_GOTO yaw 정렬 완료(e_θ={math.degrees(eth):+.1f}°) → APPROACH")
                with self._lock:
                    self._state = 'APPROACH'
                return
            self._publish(0.0, _clamp(self.k_align * eth, -self.omega_max, self.omega_max))
            return

    def _approach_step(self):
        with self._lock:
            mx, my = self._ema_mx, self._ema_my
            depth, last_t = self._ema_depth, self._last_pose_t
        now = time.time()
        if self._timed_out(now):
            self._fail('TIMEOUT', 'APPROACH 타임아웃')
            return
        if self._marker_lost(now, last_t):
            self._fail('LOST', '마커 소실(APPROACH)')
            return
        if depth is None or mx is None:
            return
        tgt_y = self._target_y   # 도킹 시작 시 캡처(핫루프서 param 조회 안 함)
        if depth <= self.stage_handoff_depth:
            lat_err = my - tgt_y
            if abs(lat_err) > self.handoff_max_ey:
                self._fail('MISALIGNED',
                           f'마커 lateral err={lat_err:+.4f} > {self.handoff_max_ey:.3f} '
                           f'(m_y={my:+.4f}, 목표={tgt_y:+.4f})')
                return
            # ALIGN 폐지(2026-07-17): m_y는 이미 예산 안. e_θ 제자리회전은 편향 법선을 지우려다
            #   회전 레버암으로 그 좋은 m_y를 되레 민다(m_y는 회전 불변 아님). → 바로 CREEP.
            with self._lock:
                odom = self._odom
            if odom is None:
                return
            self.get_logger().info(
                f"핸드오프 (depth={depth:.3f}, m_y={my:+.4f}, 목표={tgt_y:+.4f}, e_θ 보정 안 함) → CREEP")
            self._enter_creep(depth, odom)
            return
        bearing = math.atan2(my - tgt_y, mx)
        if abs(bearing) < self.approach_deadband:
            omega = 0.0
        else:
            omega = _clamp(self.approach_k * bearing, -self.omega_max, self.omega_max)
        self._publish(self.approach_speed, omega)

    def _publish(self, v, omega):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.twist.linear.x = float(v)
        m.twist.angular.z = float(omega)
        self.cmd_pub.publish(m)

    def _stop(self):
        self._publish(0.0, 0.0)

    def _set_state(self, s):
        with self._lock:
            self._state = s


def main(args=None):
    rclpy.init(args=args)
    node = DockControllerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._stop()
        except Exception:     # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
