# ai_detector — AI 입출고 감지

IN-1 게이트 카메라(RealSense D435)를 YOLO(best.pt)로 감시해 물품 종류를 판별하고,
`/detection/inbound`로 발행 → task_manager가 inbound task를 생성한다.
출고 감지(pickup/no_pickup)도 이 패키지에 추가 예정 (같은 카메라 토픽 공유).

## 필요 환경 (클론 후 이것만 설치하면 실행 가능)

**apt (ROS Jazzy):**
```bash
sudo apt install ros-jazzy-realsense2-camera   # RealSense 드라이버 (librealsense 포함)
```

**pip:**
```bash
pip3 install --user ultralytics                # YOLO + torch (GPU 자동 사용)
pip3 install --user pyrealsense2               # (선택) 드라이버 없이 카메라 직접 열 때만
pip3 install --user 'setuptools<80'            # ★ ultralytics가 올려놓은 setuptools 원복
                                               #   (80 이상이면 colcon build 깨짐 — 필수)
```

**모델:** `models/best.pt`가 패키지에 포함돼 있음 (YOLO detect, 10클래스:
Coke/Drinks/apple/fish/icecream/other_fish/other_fruit/other_square/pear/pizza_box).
다른 가중치를 쓰려면 `-p model_path:=경로`. 재학습본은 이 파일 교체 후 커밋.

## 빌드·실행

```bash
cd ~/ros2-ai-amr-repo1
colcon build --packages-select ai_detector && source install/setup.bash
export ROS_DOMAIN_ID=12                        # 서버 도메인

ros2 launch ai_detector ai_bringup.launch.py   # 카메라 드라이버 + 감지 노드 한 번에
ros2 launch ai_detector ai_bringup.launch.py conf:=0.5   # 신뢰도 문턱 조정
```

확인:
```bash
ros2 topic echo /detection/inbound             # {"product_name": "사과"} — 감지 시 1회
rqt                                            # Image View → /detection/inbound/debug (박스 영상)
```

## 토픽

| 방향 | 토픽 | 내용 |
|---|---|---|
| 발행 | `/detection/inbound` | `{"product_name": "사과"}` — task_manager 계약 |
| 발행 | `/detection/inbound/debug` | YOLO 박스 어노테이션 영상 (관제·디버그) |
| 구독 | `/camera/camera/color/image_raw` | realsense2_camera 원본 (기본 소스) |

카메라 원본은 드라이버가 발행하므로 출고 감지 노드·관제 뷰(`.../image_raw/compressed`)와
경합 없이 공유된다.

## 주요 파라미터 (inbound_detector)

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `camera` | `topic:/camera/camera/color/image_raw` | `realsense`=SDK 직접 열기(단독 테스트), 숫자=웹캠, 경로=파일 |
| `conf` | 0.35 | 감지 신뢰도 문턱. 게이트 근접 마운트 후 0.5+ 권장 |
| `stable_frames` | 5 | 발행에 필요한 연속 동일 감지 수 |
| `clear_frames` | 10 | 재무장에 필요한 연속 빈 게이트 수 |
| `model_path` | (패키지 내장 `models/best.pt`) | YOLO 가중치 교체 시 지정 |
| `device` | auto | auto/cpu/cuda |

동작: **원샷 상태머신** — 같은 클래스 `stable_frames` 연속 감지 → 1회 발행 →
게이트가 `clear_frames` 연속 비워질 때까지 재발행 금지 (물건 1개 = task 1개).

## ⚠️ 계약 사항

- `config/product_map.yaml`의 클래스→상품명 매핑은 **DB `products.name` 시드와
  반드시 일치**해야 함 (불일치 시 task_manager가 "상품 없음" 에러로 task 미생성).
- 현재 매핑 근거: `client/src/constants.ts` EMOJI_MAP의 상품 6종
  (사과·배·콜라·생선·아이스크림·냉동피자). Drinks·other_* 4클래스는 DB에
  해당 상품이 없어 `null`(무시) — 의도인지 팀원과 최종 확인 필요.

## 알려진 이슈

- pip ultralytics가 opencv 5.x를 설치해 시스템 cv_bridge와 충돌 → 이 패키지는
  cv_bridge를 쓰지 않음 (Image 메시지 직접 생성).
- 시스템 cuDNN과 pip 휠 cuDNN 혼재 시 GPU 추론 크래시 → 코드에서
  `torch.backends.cudnn.enabled=False`로 우회 (GPU 4ms/frame).
- RealSense는 USB 3 포트(파란색) 필수 — USB 2로 잡히면 프레임이 안 온다
  (`rs-enumerate-devices`나 노드 로그에서 USB 모드 확인).
