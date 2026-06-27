#include "nav2_bt_publish_goal/select_place_slot_action.hpp"

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace nav2_bt_publish_goal
{

SelectPlaceSlotAction::SelectPlaceSlotAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList SelectPlaceSlotAction::providedPorts()
{
  return {
    BT::InputPort<int>("start", 1, "起始 slot 序号(1-based, 对应 place_slot_priority 第几个)")
  };
}

bool SelectPlaceSlotAction::copyDouble(
  const std::string & prefix, const std::string & suffix)
{
  const std::string src = prefix + suffix;
  auto entry = config().blackboard->getEntry(src);
  if (!entry) {
    return false;
  }
  double value = 0.0;
  if (!config().blackboard->get<double>(src, value)) {
    return false;
  }
  // 工作键: 加 slot_ 前缀, 例如 x -> slot_x
  config().blackboard->set<double>("slot_" + suffix, value);
  return true;
}

BT::NodeStatus SelectPlaceSlotAction::tick()
{
  int start = 1;
  getInput("start", start);

  // 计数器跨实例共享: 黑板键 "place_slot_iter" (XML 里多处 <SelectPlaceSlot>
  // 必须按出现顺序递增, 否则不同实例各自从 0 数会撞同一个 slot)。
  int iter = 0;
  if (config().blackboard->getEntry("place_slot_iter")) {
    (void)config().blackboard->get<int>("place_slot_iter", iter);
  }

  const int index = start + iter;
  // simple_bt_runner.cpp 里写的键名是 "slot_<i>_x" 这种。
  const std::string prefix = "slot_" + std::to_string(index) + "_";

  const std::vector<std::string> suffixes = {"x", "y", "yaw"};

  for (const auto & suffix : suffixes) {
    if (!copyDouble(prefix, suffix)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("SelectPlaceSlot"),
        "找不到黑板键 [%s%s] (slot 序号 %d 越界或未加载)", prefix.c_str(),
        suffix.c_str(), index);
      return BT::NodeStatus::FAILURE;
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("SelectPlaceSlot"),
    "✓ 第 %d 次放置: 选定 slot 序号 %d (前缀 %s)", iter + 1, index, prefix.c_str());

  config().blackboard->set<int>("place_slot_iter", iter + 1);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_bt_publish_goal
