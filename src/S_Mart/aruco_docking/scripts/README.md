# aruco_docking / scripts

도킹 센서 진단·캘리브용 probe. 소스에서 바로 실행(설치 불필요), 로봇 도메인에서 실행.

## lidar_wall_normal_probe.py
`/scan` 전방 섹터를 직선피팅해 벽 법선각 `e_θ`(수직=0)를 출력.
```bash
ROS_DOMAIN_ID=31 python3 scripts/lidar_wall_normal_probe.py
```
출력: `γ(EMA)` 법선각 · `σ` 노이즈 · `odom_yaw` · `pts` 섹터 점 수 · 벽거리 · 잔차.
파라미터: `SECTOR_DEG`(전방 섹터 폭), `R_MIN/R_MAX`(유효 range).

## marker_normal_probe.py
`/detected_dock_pose`(ArUco 자세)에서 법선각 `e_θ`를 출력. estimator 실행 중이어야 함.
```bash
ROS_DOMAIN_ID=31 python3 scripts/marker_normal_probe.py
```
출력: `e_θ` 법선각 · `σ` 노이즈 · `odom_yaw` · `depth`. `lidar_wall_normal_probe`와 동일 정의라 비교 가능.
