#include "nav2_bt_publish_goal/select_arm_prep_name_action.hpp"

#include <cmath>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

namespace nav2_bt_publish_goal
{

SelectArmPrepNameAction::SelectArmPrepNameAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList SelectArmPrepNameAction::providedPorts()
{
  return {
    BT::InputPort<int32_t>("next_from_id", "From-node id of the next step"),
    BT::InputPort<int32_t>("next_target_id", "Target-node id of the next step"),
    BT::InputPort<std::string>(
      "block_yaml", "",
      "Absolute path to block_*.yaml. Empty = default block_blue.yaml in nav2_bt_publish_goal/yaml."),
    BT::OutputPort<std::string>("arm_prep_name", "Selected named arm preparation pose")
  };
}

bool SelectArmPrepNameAction::ensureLoaded(const std::string & yaml_path)
{
  if (yaml_path == loaded_yaml_path_ && !blocks_.empty()) {
    return true;
  }

  blocks_.clear();
  loaded_yaml_path_.clear();

  try {
    YAML::Node root = YAML::LoadFile(yaml_path);
    YAML::Node node;
    if (root["blocks"]) {
      node = root["blocks"];
    } else {
      // 也容忍直接顶层就是 map
      node = root;
    }

    if (!node.IsMap()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("SelectArmPrepName"),
        "block_yaml [%s] root 'blocks' is not a map", yaml_path.c_str());
      return false;
    }

    for (const auto & entry : node) {
      const int32_t id = entry.first.as<int32_t>();
      const auto & v = entry.second;
      BlockInfo info;
      info.x = v["x"] ? v["x"].as<double>() : 0.0;
      info.y = v["y"] ? v["y"].as<double>() : 0.0;
      info.height = v["height"] ? v["height"].as<double>() : 0.0;
      blocks_[id] = info;
    }

    loaded_yaml_path_ = yaml_path;
    RCLCPP_INFO(
      rclcpp::get_logger("SelectArmPrepName"),
      "Loaded %zu blocks from %s", blocks_.size(), yaml_path.c_str());
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SelectArmPrepName"),
      "Failed to load block_yaml [%s]: %s", yaml_path.c_str(), e.what());
    return false;
  }
}

BT::NodeStatus SelectArmPrepNameAction::tick()
{
  int32_t from_id = 0;
  int32_t target_id = 0;
  if (!getInput("next_from_id", from_id) || !getInput("next_target_id", target_id)) {
    RCLCPP_ERROR(rclcpp::get_logger("SelectArmPrepName"),
      "Missing input next_from_id / next_target_id");
    setOutput<std::string>("arm_prep_name", "");
    return BT::NodeStatus::FAILURE;
  }

  std::string yaml_path;
  (void)getInput("block_yaml", yaml_path);
  if (yaml_path.empty()) {
    try {
      yaml_path = ament_index_cpp::get_package_share_directory("at_r2_bt") +
        "/yaml/block_blue.yaml";
    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger("SelectArmPrepName"),
        "Cannot locate at_r2_bt share dir: %s", e.what());
      setOutput<std::string>("arm_prep_name", "");
      return BT::NodeStatus::FAILURE;
    }
  }

  if (!ensureLoaded(yaml_path)) {
    setOutput<std::string>("arm_prep_name", "");
    return BT::NodeStatus::FAILURE;
  }

  const auto from_it = blocks_.find(from_id);
  const auto target_it = blocks_.find(target_id);
  if (from_it == blocks_.end() || target_it == blocks_.end()) {
    RCLCPP_ERROR(rclcpp::get_logger("SelectArmPrepName"),
      "Block id not found: from=%d target=%d", from_id, target_id);
    setOutput<std::string>("arm_prep_name", "");
    return BT::NodeStatus::FAILURE;
  }

  const double dy = target_it->second.y - from_it->second.y;
  const double dh = target_it->second.height - from_it->second.height;

  // 方向：target 在 from 的右侧（map y 更小） → "right"，否则 "front"
  const bool is_right = (dy < 0.0);

  // 高度桶：每级 0.2m，四舍五入到整数
  const int bucket = static_cast<int>(std::lround(dh / 0.2));

  std::string name;
  if (is_right) {
    if (bucket == 1) {
      name = "pick_right_up200";
    } else if (bucket == -1) {
      name = "pick_right_down200";
    }
  } else {
    if (bucket == 2) {
      name = "pick_front_up400";
    } else if (bucket == 1) {
      name = "pick_front_up200";
    } else if (bucket == -1) {
      name = "pick_front_down200";
    }
  }

  if (name.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("SelectArmPrepName"),
      "No matching prep name: from=%d target=%d dy=%.3f dh=%.3f bucket=%d direction=%s",
      from_id, target_id, dy, dh, bucket, is_right ? "right" : "front");
    setOutput<std::string>("arm_prep_name", "");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("SelectArmPrepName"),
    "Selected arm prep name=[%s]  from=%d target=%d dy=%.3f dh=%.3f bucket=%d",
    name.c_str(), from_id, target_id, dy, dh, bucket);
  setOutput<std::string>("arm_prep_name", name);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_bt_publish_goal
