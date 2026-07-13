"""로봇 FSM 상태 정의 — 상태 목록·허용 전이의 단일 소스.

정상 흐름:
    IDLE → TO_SOURCE → PICK → TO_TARGET → PLACE ─┬→ TO_SOURCE (체이닝)
                                                  └→ IDLE → RETURNING → IDLE
예외 흐름:
    임무 단계 실패 → ERROR (배정 제외, fsm> reset으로 IDLE 복귀)
    RETURNING 중 배정 → 세그먼트 경계에서 TO_SOURCE (preempt)
    수동 주행/수렴 회전/init → MANUAL (busy 발행으로 fleet 배정 차단)

전이는 RobotFSM._set_state() 한 곳으로만 수행한다. 테이블에 없는 전이는
경고 로그만 남기고 통과 — 실기 운용 중 FSM을 세우는 것보다 버그를
표면화하는 쪽이 안전하다 (전이 누락 발견용 가드).
"""
from enum import Enum


class S(Enum):
    IDLE = 'IDLE'
    TO_SOURCE = 'TO_SOURCE'
    PICK = 'PICK'
    TO_TARGET = 'TO_TARGET'
    PLACE = 'PLACE'
    RETURNING = 'RETURNING'
    MANUAL = 'MANUAL'
    ERROR = 'ERROR'


# 허용 전이 테이블 (자기 자신으로의 전이는 항상 무시·허용)
TRANSITIONS = {
    S.IDLE:      {S.TO_SOURCE, S.RETURNING, S.MANUAL},
    S.TO_SOURCE: {S.PICK, S.ERROR},
    S.PICK:      {S.TO_TARGET, S.ERROR},
    S.TO_TARGET: {S.PLACE, S.ERROR},
    S.PLACE:     {S.IDLE, S.ERROR},
    S.RETURNING: {S.IDLE, S.TO_SOURCE, S.ERROR},   # TO_SOURCE = preempt 체이닝
    S.MANUAL:    {S.IDLE, S.ERROR},                # ERROR = init spin 중 원복
    S.ERROR:     {S.IDLE, S.MANUAL},               # reset / ERROR 중 init
}

# 외부 발행 상태 (fleet 배정 판단용 3종) — 내부 상태와 분리
# RETURNING도 idle: 복귀 중 배정 가능 (세그먼트 경계에서 전환)
EXT_IDLE = {S.IDLE, S.RETURNING}
