"""포크(리프트) 제어량 정의 — 4단 높이 모델.

우리 설계(포크 4단 높이):
  0 = 호밍 기준점 (리밋스위치 최저점)
  1 = 바닥 팔레트 안착 = 입/출고 팔레트, 랙 L1 안착 (포크 삽입 높이)
  2 = 운반 높이 ★ 모든 주행/대기의 디폴트
  3 = 랙 L2 안착 높이
  4 = L2 팔레트 든/내리기 직전 클리어런스 높이

★ 스텝값은 임의(placeholder) — 실물 캘리브레이션 후 교체.
  ESP32가 현재 높이 추적 + 절대 목표값("높이 N으로")으로 제어.
  부팅 시 호밍(0점) 후 2로 올림 = 디폴트.
"""

# 높이 단계 → 절대 스텝값 (임의값, 실측 후 교체)
FORK_STEPS = {
    0: 0,       # 호밍 기준 (최저)
    1: 500,     # 바닥 안착 (임의)
    2: 1500,    # 운반 (임의, 디폴트)
    3: 3000,    # L2 안착 (임의)
    4: 4000,    # L2 클리어런스 (임의)
}

DEFAULT_HEIGHT = 2          # 주행/대기 디폴트


def steps(level):
    """높이 단계(0~4) → 포크 스텝값. 없는 단계면 디폴트(2)."""
    return FORK_STEPS.get(level, FORK_STEPS[DEFAULT_HEIGHT])


def pick_sequence(level):
    """집기 시퀀스 (높이 단계 리스트). 도킹 전/후 포크 동작.

    하층(L1): 2->1 -> [도킹] -> 1->2 -> [back]
    상층(L2): 2->3 -> [도킹] -> 3->4 -> [back] -> 4->2
    반환: {'before_dock':[...], 'after_dock':[...], 'after_back':[...]}
    """
    if level == 1:
        return {'before_dock': [1], 'after_dock': [2], 'after_back': []}
    else:  # L2
        return {'before_dock': [3], 'after_dock': [4], 'after_back': [2]}


def place_sequence(level):
    """놓기 시퀀스.

    하층(L1): [도킹](운반2) -> 2->1 -> [back] -> 1->2
    상층(L2): 2->4 -> [도킹] -> 4->3 -> [back] -> 3->2
    """
    if level == 1:
        return {'before_dock': [], 'after_dock': [1], 'after_back': [2]}
    else:  # L2
        return {'before_dock': [4], 'after_dock': [3], 'after_back': [2]}
