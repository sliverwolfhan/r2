#include "nav2_bt_publish_goal/get_block_center_action.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

namespace nav2_bt_publish_goal
{

GetBlockCenterAction::GetBlockCenterAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList GetBlockCenterAction::providedPorts()
{
  return {
    BT::InputPort<int32_t>("block_id", "Node id whose center is queried"),
    BT::InputPort<std::string>(
      "block_yaml", "",
      "Absolute path to block_*.yaml. Empty = default at_r2_bt/yaml/block_red.yaml."),
    BT::OutputPort<double>("center_x", "Block center x (map frame)"),
    BT::OutputPort<double>("center_y", "Block center y (map frame)")
  };
}

bool GetBlockCenterAction::ensureBlocksLoaded(const std::string & yaml_path)
{
  if (yaml_path == loaded_block_yaml_path_ && !blocks_.empty()) {
    return true;
  }

  blocks_.clear();
  loaded_block_yaml_path_.clear();

  try {
    YAML::Node root = YAML::LoadFile(yaml_path);
    YAML::Node node;
    if (root["blocks"]) {
      node = root["blocks"];
    } else {
      node = root;
    }

    if (!node.IsMap()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("GetBlockCenter"),
        "block_yaml [%s] root 'blocks' is not a map", yaml_path.c_str());
      return false;
    }

    for (const auto & entry : node) {
      const int32_t id = entry.first.as<int32_t>();
      const auto & v = entry.second;
      BlockInfo info;
      info.x = v["x"] ? v["x"].as<double>() : 0.0;
      info.y = v["y"] ? v["y"].as<double>() : 0.0;
      blocks_[id] = info;
    }

    loaded_block_yaml_path_ = yaml_path;
    RCLCPP_INFO(
      rclcpp::get_logger("GetBlockCenter"),
      "Loaded %zu blocks from %s", blocks_.size(), yaml_path.c_str());
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("GetBlockCenter"),
      "Failed to load block_yaml [%s]: %s", yaml_path.c_str(), e.what());
    return false;
  }
}

BT::NodeStatus GetBlockCenterAction::tick()
{
  int32_t block_id = 0;
  if (!getInput("block_id", block_id)) {
    RCLCPP_ERROR(rclcpp::get_logger("GetBlockCenter"), "Missing input [block_id]");
    return BT::NodeStatus::FAILURE;
  }

  std::string block_yaml_path;
  (void)getInput("block_yaml", block_yaml_path);
  if (block_yaml_path.empty()) {
    try {
      block_yaml_path = ament_index_cpp::get_package_share_directory("at_r2_bt") +
        "/yaml/block_red.yaml";
    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger("GetBlockCenter"),
        "Cannot locate at_r2_bt share dir: %s", e.what());
      return BT::NodeStatus::FAILURE;
    }
  }

  if (!ensureBlocksLoaded(block_yaml_path)) {
    return BT::NodeStatus::FAILURE;
  }

  const auto it = blocks_.find(block_id);
  if (it == blocks_.end()) {
    RCLCPP_ERROR(rclcpp::get_logger("GetBlockCenter"),
      "block_id=%d not found in %s", block_id, block_yaml_path.c_str());
    return BT::NodeStatus::FAILURE;
  }

  setOutput<double>("center_x", it->second.x);
  setOutput<double>("center_y", it->second.y);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_bt_publish_goal
