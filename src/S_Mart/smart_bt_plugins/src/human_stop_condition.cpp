// human_stop_condition.cpp — 사람 감지(/human_stop) 시 FollowPath halt BT 조건 노드
// 헤더 human_stop_condition.hpp 참고. (울산 MotionHoldCondition v3 → v4 포팅)

#include "smart_bt_plugins/human_stop_condition.hpp"

namespace smart_bt_plugins
{

HumanStopCondition::HumanStopCondition(
  const std::string & condition_name,
  const BT::NodeConfig & conf)
: BT::ConditionNode(condition_name, conf),
  hold_active_(false)
{
  // bt_navigator가 blackboard에 넣어주는 공유 rclcpp 노드 획득
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  getInput("topic", topic_);

  // 메인 실행기와 분리된 전용 callback group — tick()에서 spin_some으로 직접 수집
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  hold_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&HumanStopCondition::holdCallback, this, std::placeholders::_1),
    sub_option);

  RCLCPP_INFO(
    node_->get_logger(),
    "HumanStopCondition 초기화: topic=%s", topic_.c_str());
}

BT::NodeStatus HumanStopCondition::tick()
{
  // 최신 /human_stop 콜백 수집 (논블로킹)
  callback_group_executor_.spin_some();

  if (hold_active_) {
    // 사람 정지 ON → RUNNING → ReactiveSequence가 FollowPath halt → cmd_vel 정지
    return BT::NodeStatus::RUNNING;
  }
  // 게이트 OFF → SUCCESS → FollowPath 정상 진행
  return BT::NodeStatus::SUCCESS;
}

void HumanStopCondition::holdCallback(std_msgs::msg::Bool::SharedPtr msg)
{
  hold_active_ = msg->data;
}

}  // namespace smart_bt_plugins

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<smart_bt_plugins::HumanStopCondition>("HumanStopCondition");
}
