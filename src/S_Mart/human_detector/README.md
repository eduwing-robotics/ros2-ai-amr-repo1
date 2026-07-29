# human_detector - 사람 감지
각 로봇의 카메라 Pi Camera v2 를 YOLO(human_best.pt)로 감시해 정지 신호 발행

감지된 사람이 화면 높이의 60%이상 크기일 시 정지

정지 후 2초동안 미감지시 동작 재개

# 토픽
|방향|토픽|내용|
|---|---|---|
|발행|/human_stop|true=사람있음(정지) / false=없음(재개)|
|발행|/human_detector/debug/compressed (선택) | 어노테이션 프레임(관제, rqt 확인)|

# 주요 파라미터
|파라미터|기본값|내용|
|---|---|---|
|conf|0.35|신뢰도|
|release_sec|2.0 |해제 대기 |
|min_height_ratio|0.6|근접 사람 판정 기준(박스 높이 / 화면 높이)|
|camera|topic:/camera/image_raw/compressed|카메라 소스|
