"""포크(리프트) 동작 시퀀스 — 4단 높이 모델 (순수 로직, ROS 무관).

높이 정의 (2층 랙):
  1 = 1층 팔레트 삽입(꽂기) 높이          ← 창고랙 1층 + 입출고존
  2 = 1층 팔레트 든 높이 (1보다 ~2cm↑)    ← 디폴트·홈·주행 높이 (주행은 항상 2)
  3 = 2층 팔레트 삽입(꽂기) 높이          ← 창고랙 2층
  4 = 2층 팔레트 든 높이 (3보다 ~2cm↑)    ← 창고랙 2층

- 삽입(1,3): 랙 위 팔레트의 포크 포켓에 포크를 밀어넣는 높이.
- 든(2,4): 삽입 후 팔레트를 ~2cm 들어 랙에서 띄운 높이.
- 리미트 스위치 없음 — 부팅 시 포크를 물리적으로 높이 2에 놓고 켠다(= step 0 기준).
  현재 위치(step) 추적과 "절대높이→스텝" 변환은 전부 ESP32가 담당(단일 진실 원천).
  이 파일은 "몇 층이면 어떤 높이 순서로 움직여라"만 계산한다 (스텝값을 모른다).
  실측 스텝(ESP32 STEP 테이블, level2=0 기준): 1=-8000 / 3=+15400 / 4=+23400.

입출고존: 창고랙 1층과 같은 높이(2층 없음)라 항상 1층 취급 → locations.yaml에서 level=1.
"""


def pick_sequence(level):
    """집기 시퀀스 (포크 높이 순서). level = 랙 층(1=1층 / 2=2층).

    1층: 2(주행)→1(삽입)→[도킹]→2(들기)          → before[1] after[2] back[]
    2층: 2(주행)→3(삽입)→[도킹]→4(들기)→2(주행)   → before[3] after[4] back[2]
    반환: {'before_dock':[...], 'after_dock':[...], 'after_back':[...]}
    """
    if level == 1:
        return {'before_dock': [1], 'after_dock': [2], 'after_back': []}
    return {'before_dock': [3], 'after_dock': [4], 'after_back': [2]}


def place_sequence(level):
    """놓기 시퀀스. level = 랙 층(1=1층 / 2=2층).

    1층: 2(운반)→[도킹]→1(안착)→2(주행)            → before[]  after[1] back[2]
    2층: 2(운반)→4(들어올림)→[도킹]→3(안착)→2(주행) → before[4] after[3] back[2]
    """
    if level == 1:
        return {'before_dock': [], 'after_dock': [1], 'after_back': [2]}
    return {'before_dock': [4], 'after_dock': [3], 'after_back': [2]}
