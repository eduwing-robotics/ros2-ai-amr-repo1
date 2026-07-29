# human_detector - 사람 감지
각 로봇의 카메라 Pi Camera v2 를 YOLO(human_best.pt)로 감시해 정지 신호 발행

# 토픽
|방향|토픽|내용|
|---|---|---|
|발행|/human_stop|true=사람있음(정지) / false=없음(재개)|
|발행|/human_detector/debug/compressed (선택) | 어노테이션 프레임(관제, rqt 확인)|
