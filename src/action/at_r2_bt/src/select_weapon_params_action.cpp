#include "nav2_bt_publish_goal/select_weapon_params_action.hpp"

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace nav2_bt_publish_goal
{

SelectWeaponParamsAction::SelectWeaponParamsAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList SelectWeaponParamsAction::providedPorts()
{
  return {
    BT::InputPort<int>("start", 1, "起始武器序号(1-based, 对应 weapon_priority 第几个)")
  };
}

bool SelectWeaponParamsAction::copyDouble(
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
  config().blackboard->set<double>(suffix, value);
  return true;
}

BT::NodeStatus SelectWeaponParamsAction::tick()
{
  int start = 1;
  getInput("start", start);

  const int index = start + iter_;          // 本次选定的武器序号 (1-based)
  const std::string prefix = "w" + std::to_string(index) + "_";

  // 该武器的全部抓取参数键 (与 simple_bt_runner.cpp 写入的后缀保持一致)
  const std::vector<std::string> suffixes = {
    "grasp_prep_x", "grasp_prep_y", "grasp_prep_yaw",
    "grasp_prep_y_tolerance", "grasp_prep_yaw_tolerance",
    "grasp_laser_target_distance", "grasp_laser_distance_tolerance"};

  for (const auto & suffix : suffixes) {
    if (!copyDouble(prefix, suffix)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("SelectWeaponParams"),
        "找不到黑板键 [%s%s] (武器序号 %d 越界或未加载)", prefix.c_str(),
        suffix.c_str(), index);
      return BT::NodeStatus::FAILURE;
    }
  }

  // 选定目标名(可选, 仅用于日志); 没有也不算失败
  std::string target_name;
  (void)config().blackboard->get<std::string>(prefix + "grasp_target_name", target_name);
  config().blackboard->set<std::string>("grasp_target_name", target_name);

  RCLCPP_INFO(
    rclcpp::get_logger("SelectWeaponParams"),
    "✓ 第 %d 次抓取: 选定武器序号 %d (%s%s)", iter_ + 1, index, prefix.c_str(),
    target_name.empty() ? "" : target_name.c_str());

  ++iter_;
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_bt_publish_goal
