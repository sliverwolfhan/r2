#include "nav2_bt_publish_goal/select_kfs_params_action.hpp"

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace nav2_bt_publish_goal
{

SelectKfsParamsAction::SelectKfsParamsAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList SelectKfsParamsAction::providedPorts()
{
  return {
    BT::InputPort<int>("start", 1, "起始 KFS 序号(1-based, 对应 kfs_priority 第几个)")
  };
}

bool SelectKfsParamsAction::copyDouble(
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
  // 工作键: 给 suffix 加 kfs_ 前缀, 例如 prep_x -> kfs_prep_x
  config().blackboard->set<double>("kfs_" + suffix, value);
  return true;
}

BT::NodeStatus SelectKfsParamsAction::tick()
{
  int start = 1;
  getInput("start", start);

  const int index = start + iter_;
  // simple_bt_runner.cpp 里写的键名是 "kfs_<i>_prep_x" 这种
  // (用 yaml 里的 kfs_<i> 直接作为前缀, 末尾接 "_" 再加 suffix)。
  const std::string prefix = "kfs_" + std::to_string(index) + "_";

  const std::vector<std::string> suffixes = {
    "prep_x", "prep_y", "prep_yaw",
    "place_x", "place_y"};

  for (const auto & suffix : suffixes) {
    if (!copyDouble(prefix, suffix)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("SelectKfsParams"),
        "找不到黑板键 [%s%s] (KFS 序号 %d 越界或未加载)", prefix.c_str(),
        suffix.c_str(), index);
      return BT::NodeStatus::FAILURE;
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("SelectKfsParams"),
    "✓ 第 %d 次取块: 选定 KFS 序号 %d (前缀 %s)", iter_ + 1, index, prefix.c_str());

  ++iter_;
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_bt_publish_goal
