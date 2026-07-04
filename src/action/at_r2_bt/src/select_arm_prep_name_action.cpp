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
      "Absolute path to block_*.yaml. Empty = default block_blue.yaml in at_r2_bt/yaml."),
    BT::InputPort<std::string>(
      "arm_yaml", "",
      "Absolute path to arm_ready_position.yaml. "
      "Empty = default at_r2_bt/yaml/arm_ready_position.yaml. "
      "Determines whether `pick_left_*` is available for a given height bucket."),
    BT::OutputPort<std::string>("arm_prep_name", "Selected named arm preparation pose"),
    BT::OutputPort<int>(
      "arm_prep_is_low",
      "1 if selected pose is down200/down400 (arm may scrape ground while chassis moves); "
      "0 otherwise. Always 0 on FAILURE.")
  };
}

bool SelectArmPrepNameAction::ensureBlocksLoaded(const std::string & yaml_path)
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

    loaded_block_yaml_path_ = yaml_path;
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

bool SelectArmPrepNameAction::ensureArmPosesLoaded(const std::string & yaml_path)
{
  if (yaml_path == loaded_arm_yaml_path_ && !arm_pose_names_.empty()) {
    return true;
  }

  arm_pose_names_.clear();
  loaded_arm_yaml_path_.clear();

  try {
    YAML::Node root = YAML::LoadFile(yaml_path);
    YAML::Node node;
    if (root["arm_positions"]) {
      node = root["arm_positions"];
    } else if (root["arm_position"]) {
      node = root["arm_position"];
    } else {
      RCLCPP_ERROR(
        rclcpp::get_logger("SelectArmPrepName"),
        "arm_yaml [%s] missing key 'arm_positions'", yaml_path.c_str());
      return false;
    }

    if (node.IsMap()) {
      for (const auto & entry : node) {
        arm_pose_names_.insert(entry.first.as<std::string>());
      }
    } else if (node.IsSequence()) {
      for (const auto & p : node) {
        if (p["name"]) {
          arm_pose_names_.insert(p["name"].as<std::string>());
        }
      }
    } else {
      RCLCPP_ERROR(
        rclcpp::get_logger("SelectArmPrepName"),
        "arm_yaml [%s] arm_positions is neither map nor sequence", yaml_path.c_str());
      return false;
    }

    loaded_arm_yaml_path_ = yaml_path;
    RCLCPP_INFO(
      rclcpp::get_logger("SelectArmPrepName"),
      "Loaded %zu arm pose names from %s",
      arm_pose_names_.size(), yaml_path.c_str());
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SelectArmPrepName"),
      "Failed to load arm_yaml [%s]: %s", yaml_path.c_str(), e.what());
    return false;
  }
}

BT::NodeStatus SelectArmPrepNameAction::tick()
{
  // 失败时统一兜底: arm_prep_is_low 始终先置 0, 任何 FAILURE 分支无需再单独写。
  setOutput<int>("arm_prep_is_low", 0);

  int32_t from_id = 0;
  int32_t target_id = 0;
  if (!getInput("next_from_id", from_id) || !getInput("next_target_id", target_id)) {
    RCLCPP_ERROR(rclcpp::get_logger("SelectArmPrepName"),
      "Missing input next_from_id / next_target_id");
    setOutput<std::string>("arm_prep_name", "");
    return BT::NodeStatus::FAILURE;
  }

  std::string block_yaml_path;
  (void)getInput("block_yaml", block_yaml_path);
  if (block_yaml_path.empty()) {
    try {
      block_yaml_path = ament_index_cpp::get_package_share_directory("at_r2_bt") +
        "/yaml/block_blue.yaml";
    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger("SelectArmPrepName"),
        "Cannot locate at_r2_bt share dir: %s", e.what());
      setOutput<std::string>("arm_prep_name", "");
      return BT::NodeStatus::FAILURE;
    }
  }

  std::string arm_yaml_path;
  (void)getInput("arm_yaml", arm_yaml_path);
  if (arm_yaml_path.empty()) {
    try {
      arm_yaml_path = ament_index_cpp::get_package_share_directory("at_r2_bt") +
        "/yaml/arm_ready_position.yaml";
    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger("SelectArmPrepName"),
        "Cannot locate at_r2_bt share dir: %s", e.what());
      setOutput<std::string>("arm_prep_name", "");
      return BT::NodeStatus::FAILURE;
    }
  }

  if (!ensureBlocksLoaded(block_yaml_path) || !ensureArmPosesLoaded(arm_yaml_path)) {
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

  // 高度桶：每级 0.2 m，四舍五入到整数 → up/down 后缀
  const int bucket = static_cast<int>(std::lround(dh / 0.2));
  std::string suffix;
  if (bucket == 1) {
    suffix = "up200";
  } else if (bucket == 2) {
    suffix = "up400";
  } else if (bucket == -1) {
    suffix = "down200";
  } else if (bucket == -2) {
    suffix = "down400";
  }

  if (suffix.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("SelectArmPrepName"),
      "Unsupported height bucket: from=%d target=%d dh=%.3f bucket=%d",
      from_id, target_id, dh, bucket);
    setOutput<std::string>("arm_prep_name", "");
    return BT::NodeStatus::FAILURE;
  }

  // 机械臂只能朝左抓：target 在 map +y 一侧（dy > 0）时优先尝试 pick_left_*；
  // 不可用（包括 target 不在左侧、或左抓姿态在 arm_yaml 里没定义）就回退 pick_front_*。
  const bool target_on_left = (dy > 0.0);
  std::string name;
  std::string chosen_dir;
  if (target_on_left) {
    const std::string left_name = "pick_left_" + suffix;
    if (arm_pose_names_.count(left_name)) {
      name = left_name;
      chosen_dir = "left";
    }
  }
  if (name.empty()) {
    const std::string front_name = "pick_front_" + suffix;
    if (arm_pose_names_.count(front_name)) {
      name = front_name;
      chosen_dir = "front";
    }
  }

  if (name.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("SelectArmPrepName"),
      "No arm pose available: from=%d target=%d dy=%.3f dh=%.3f bucket=%d "
      "(tried %s%s + pick_front_%s)",
      from_id, target_id, dy, dh, bucket,
      target_on_left ? "pick_left_" : "", target_on_left ? suffix.c_str() : "",
      suffix.c_str());
    setOutput<std::string>("arm_prep_name", "");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("SelectArmPrepName"),
    "Selected arm prep name=[%s] dir=%s from=%d target=%d dy=%.3f dh=%.3f bucket=%d",
    name.c_str(), chosen_dir.c_str(), from_id, target_id, dy, dh, bucket);
  setOutput<std::string>("arm_prep_name", name);
  // bucket < 0 (down200/down400) 意味着臂会下伸到底盘高度以下, 与导航并行会蹭地;
  // BT 侧据此把臂动作改成"先走完再下臂"。bucket > 0 / 平位则保持并行。
  setOutput<int>("arm_prep_is_low", bucket < 0 ? 1 : 0);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_bt_publish_goal
