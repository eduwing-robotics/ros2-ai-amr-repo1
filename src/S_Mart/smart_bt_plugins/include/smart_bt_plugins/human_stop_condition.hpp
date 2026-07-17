// human_stop_condition.hpp — 사람 감지(/human_stop) 시 FollowPath halt BT 조건 노드
//
// 동작:
//   /human_stop (std_msgs/Bool) 구독.
//   tick():
//     hold=false → SUCCESS → ReactiveSequence가 다음 자식(FollowPath) 실행
//     hold=true  → RUNNING → ReactiveSequence가 FollowPath를 halt() → 정지
//   FAILURE는 절대 반환하지 않음 → RecoveryNode의 Spin/BackUp 미발동(좁은 통로 안전).
//
//   /human_stop 은 human_detector 노드가 발행한다(로봇 카메라 YOLO → 사람 감지).
//
// ★ 버전 주의: BT.CPP v4(4.9.0, Jazzy)에서 ConditionNode는 원칙상 RUNNING을
//   반환하지 않도록 권고되나(헤더 주석), 예외를 던지지는 않는다(예외는 SyncActionNode만).
//   ReactiveSequence는 자식이 RUNNING이면 나머지 형제를 halt하고 RUNNING을 반환하므로
//   (controls/reactive_sequence.h) 이 게이트가 FollowPath를 멈추는 동작이 성립한다.
//   게이트가 RUNNING인 순간 FollowPath는 즉시 halt되어, 동시에 RUNNING인 자식은 항상
//   하나뿐 → ReactiveSequence "비동기 자식 1개" 제약도 위반하지 않는다.
//   울산 ulsan_bt_plugins(v3)를 v4 API로 포팅: 헤더 경로·패키지명만 교체, 로직 동일.
//
// Nav2 IsBatteryLowCondition 패턴(blackboard "node" + 전용 callback group executor) 채택.

#ifndef SMART_BT_PLUGINS__HUMAN_STOP_CONDITION_HPP_
#define SMART_BT_PLUGINS__HUMAN_STOP_CONDITION_HPP_

#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace smart_bt_plugins
{

class HumanStopCondition : public BT::ConditionNode
{
public:
  HumanStopCondition(
    const std::string & condition_name,
    const BT::NodeConfig & conf);

  HumanStopCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "topic", "/human_stop", "사람 정지 게이트 Bool 토픽명"),
    };
  }

private:
  void holdCallback(std_msgs::msg::Bool::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hold_sub_;

  std::string topic_;
  bool hold_active_;
};

}  // namespace smart_bt_plugins

#endif  // SMART_BT_PLUGINS__HUMAN_STOP_CONDITION_HPP_
