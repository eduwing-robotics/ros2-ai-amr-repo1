/*
 * fork_esp32.ino — Smart Mart 포크리프트 ESP32 micro-ROS 펌웨어
 *
 * 역할: robot_fsm(Pi)과 /fork_cmd·/fork_state 토픽으로 핸드셰이크하며
 *       ULN2003 → 28BYJ-48 스텝모터로 포크 높이를 제어한다.
 *
 * 계약 (robot_fsm.work.py 와 일치):
 *   구독 /fork_cmd  (std_msgs/Int32): 목표 높이 1~4
 *   발행 /fork_state(std_msgs/Int32): 1=MOVING, 2=AT_POSITION, 3=ERROR
 *
 * 모터 구동: 팀원이 실물 검증한 방식 그대로 — Stepper.h 라이브러리로 연속 구동.
 *   Stepper(2048, 19, 5, 18, 17): 28BYJ-48 풀스텝(2048/rev), 항상 2코일=최대토크.
 *   ★핀은 라이브러리 규칙상 IN1,IN3,IN2,IN4 순서로 넣는다(가운데 교차). 배선=19/18/5/17.
 *   자작 인크리멘털 스테핑은 executor/ping 끼임으로 스텝 간격이 불규칙→진동 → 폐기.
 *   이동은 한 스텝씩 연속 호출(라이브러리가 setSpeed로 간격 유지) + 사이사이 상태 발행으로
 *   micro-ROS 세션 유지. (spin_some은 콜백 재진입이라 이동 중 호출 금지.)
 *
 * 높이 모델(리미트스위치 없음): 부팅 시 포크를 물리적으로 높이2에 놓고 켠다 → cur_step=0.
 *   STEP_ABS = 팀원 풀스텝 2048/rev 기준 실측 캘리브(스텝단위 일치 필수).
 *
 * 빌드: Arduino IDE + micro_ros_arduino(jazzy ZIP) + Stepper(내장). 보드=ESP32.
 *   실행: Pi에서 micro_ros_agent serial --dev /dev/ttyACM0 -b 115200.
 */
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <Stepper.h>

// ── ULN2003 입력 핀 (물리 배선: IN1=19, IN2=18, IN3=5, IN4=17) ──
#define PIN_IN1 19
#define PIN_IN2 18
#define PIN_IN3 5
#define PIN_IN4 17

// ── 28BYJ-48 Stepper (풀스텝 2048/rev). 라이브러리 규칙상 IN2·IN3 교차 배치 ──
const int STEPS_PER_REV = 2048;
Stepper myStepper(STEPS_PER_REV, PIN_IN1, PIN_IN3, PIN_IN2, PIN_IN4);  // = (2048,19,5,18,17)
#define MOTOR_RPM 12                 // 팀원 검증 속도(28BYJ-48이 토크 잘 내는 값)

// ── ROS 도메인 ★ ── micro-ROS는 클라이언트(ESP32)가 도메인을 정한다(agent는 ROS_DOMAIN_ID 무시).
//   기본 0이라 도메인31(AMR_2) 시스템과 안 만나 토픽이 안 보였음(2026-07-14 실기 규명).
//   AMR_1=30 / AMR_2=31. 로봇 바꾸면 이 값 변경 후 재플래시.
#define ROS_DOMAIN 30

// ── 포크 상태 코드 (/fork_state 값, robot_fsm 계약) ──
#define ST_MOVING 1
#define ST_AT     2
#define ST_ERROR  3

// ── STEP 테이블: level(1~4) → 절대 스텝 (부팅=level2=0). 풀스텝 실측 캘리브값 ──
const long STEP_ABS[5] = {0, -8000, 0, 15400, 23400};   // idx 0 미사용
#define HEARTBEAT_MS 200            // 상태 하트비트 주기(5Hz)
#define MOVE_PUB_EVERY 200          // 이동 중 이 스텝마다 MOVING 재발행(세션 유지, ~0.5s)

// ── 포크 상태 (단일 진실 원천) ──
long cur_step = 0;                 // 현재 절대 위치 (부팅 = level2 = 0)
long tgt_step = 0;                 // 목표 절대 위치
volatile int fork_state = ST_AT;   // 부팅부터 유효 위치(level2)

// ── micro-ROS 핸들 ──
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_publisher_t pub_state;
rcl_subscription_t sub_cmd;
rclc_executor_t executor;
std_msgs__msg__Int32 msg_cmd;
std_msgs__msg__Int32 msg_state;

// 연결 상태머신 (Agent 끊김 자동 복구)
enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
AgentState agent_state = WAITING_AGENT;

#define RCCHECK(fn)     { if ((fn) != RCL_RET_OK) { return false; } }
#define RCSOFTCHECK(fn) { (void)(fn); }
#define EVERY_MS(ms, X) do { static int64_t _t = -1; \
  if (_t == -1) { _t = uxr_millis(); } \
  if ((int32_t)(uxr_millis() - _t) > (ms)) { X; _t = uxr_millis(); } } while (0)

// ── 상태 발행 ──
void publish_state() {
  msg_state.data = fork_state;
  RCSOFTCHECK(rcl_publish(&pub_state, &msg_state, NULL));
}

// ── 구독 콜백: /fork_cmd 수신 (짧게 — 목표만 세팅, 이동은 loop에서) ──
void cmd_callback(const void *msgin) {
  const std_msgs__msg__Int32 *m = (const std_msgs__msg__Int32 *)msgin;
  int h = m->data;
  if (h < 1 || h > 4) {                 // 잘못된 높이 → ERROR
    fork_state = ST_ERROR;
    publish_state();
    return;
  }
  tgt_step = STEP_ABS[h];
  fork_state = ST_MOVING;
  publish_state();                       // MOVING 최소 1회 보장 (핸드셰이크 규칙)
}

// ── 이동 실행: cur_step → tgt_step 을 라이브러리로 연속 스텝 (blocking) ──
//   spin_some(콜백 재진입) 대신 사이사이 publish_state로만 세션 유지.
void run_move() {
  long remaining = tgt_step - cur_step;
  int dir = (remaining > 0) ? 1 : -1;
  long n = (remaining > 0) ? remaining : -remaining;
  for (long i = 0; i < n; i++) {
    myStepper.step(dir);               // 한 스텝(라이브러리가 setSpeed 간격 유지)
    cur_step += dir;
    if ((i % MOVE_PUB_EVERY) == 0) publish_state();   // 이동 중 MOVING 하트비트
  }
  fork_state = ST_AT;
  publish_state();                       // 도달 보고
}

// ── micro-ROS 엔티티 생성/파괴 (연결 복구용) ──
bool create_entities() {
  allocator = rcl_get_default_allocator();
  // ★ 도메인 명시(ROS_DOMAIN) — 안 하면 도메인 0으로 생성돼 시스템(31)과 안 만남.
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN));
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  RCCHECK(rclc_node_init_default(&node, "fork_esp32", "", &support));
  RCCHECK(rclc_publisher_init_default(
    &pub_state, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "fork_state"));
  RCCHECK(rclc_subscription_init_default(
    &sub_cmd, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "fork_cmd"));
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &sub_cmd, &msg_cmd, &cmd_callback, ON_NEW_DATA));
  return true;
}

void destroy_entities() {
  rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);
  rcl_publisher_fini(&pub_state, &node);
  rcl_subscription_fini(&sub_cmd, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// ── setup ──
void setup() {
  myStepper.setSpeed(MOTOR_RPM);   // 라이브러리가 스텝 간격 관리 (핀 OUTPUT도 여기서)
  set_microros_transports();       // USB 시리얼 트랜스포트 (기본 Serial 115200)

  cur_step = 0;            // 부팅 = level2 = step0 (리미트 스위치 없음)
  tgt_step = 0;
  fork_state = ST_AT;
  agent_state = WAITING_AGENT;
}

// ── loop: 연결 상태머신 + 제어 ──
void loop() {
  switch (agent_state) {
    case WAITING_AGENT:
      EVERY_MS(500, agent_state =
        (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) ? AGENT_AVAILABLE : WAITING_AGENT);
      break;

    case AGENT_AVAILABLE:
      agent_state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
      if (agent_state == WAITING_AGENT) destroy_entities();
      else publish_state();               // 접속 직후 현재 상태 1회 알림
      break;

    case AGENT_CONNECTED:
      EVERY_MS(1000, agent_state =
        (rmw_uros_ping_agent(100, 3) == RMW_RET_OK) ? AGENT_CONNECTED : AGENT_DISCONNECTED);
      if (agent_state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));   // 콜백 처리(→MOVING·tgt 세팅)
        if (cur_step != tgt_step) {
          run_move();                                          // 대기 중 이동을 연속 구동
        } else {
          EVERY_MS(HEARTBEAT_MS, publish_state());             // idle 하트비트
        }
      }
      break;

    case AGENT_DISCONNECTED:
      destroy_entities();
      agent_state = WAITING_AGENT;
      break;
  }
}
