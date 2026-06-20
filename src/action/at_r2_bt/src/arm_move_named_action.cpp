#include "nav2_bt_publish_goal/arm_move_named_action.hpp"

#include <chrono>
#include <functional>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "yaml-cpp/yaml.h"

namespace nav2_bt_publish_goal
{

using namespace std::chrono_literals;

ArmMoveNamedAction::ArmMoveNamedAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
}

BT::PortsList ArmMoveNamedAction::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "ROS node"),
    BT::InputPort<std::string>("pose_name", "Name of the named pose in arm_position.yaml"),
    BT::InputPort<std::string>("server_name", "robotic_task", "Action server name"),
    BT::InputPort<int32_t>("task_id", 1, "Arm task id (move_kfs = 1)"),
    BT::InputPort<double>("duration", 3.0, "Trajectory duration in seconds"),
    BT::InputPort<std::string>(
      "arm_yaml", "",
      "Absolute path to a yaml with `arm_positions:` map. "
      "Empty = nav2_bt_publish_goal/yaml/arm_ready_position.yaml."),
    BT::OutputPort<int32_t>("err_code", "ArmTask result error code"),
    BT::OutputPort<std::string>("reason", "ArmTask result reason")
  };
}

bool ArmMoveNamedAction::ensureLoaded(const std::string & yaml_path)
{
  if (yaml_path == loaded_yaml_path_ && !named_positions_.empty()) {
    return true;
  }

  named_positions_.clear();
  loaded_yaml_path_.clear();

  try {
    YAML::Node root = YAML::LoadFile(yaml_path);

    YAML::Node node;
    if (root["arm_positions"]) {
      node = root["arm_positions"];
    } else if (root["arm_position"]) {
      node = root["arm_position"];
    } else {
      RCLCPP_ERROR(node_->get_logger(),
        "arm_yaml [%s] missing key 'arm_positions'", yaml_path.c_str());
      return false;
    }

    if (node.IsMap()) {
      for (const auto & entry : node) {
        const std::string nm = entry.first.as<std::string>();
        const auto joints = entry.second.as<std::vector<double>>();
        named_positions_[nm] = joints;
      }
    } else if (node.IsSequence()) {
      for (const auto & p : node) {
        if (!p["name"] || !p["joints"]) {
          continue;
        }
        const std::string nm = p["name"].as<std::string>();
        const auto joints = p["joints"].as<std::vector<double>>();
        named_positions_[nm] = joints;
      }
    } else {
      RCLCPP_ERROR(node_->get_logger(),
        "arm_yaml [%s] arm_positions is neither map nor sequence", yaml_path.c_str());
      return false;
    }

    loaded_yaml_path_ = yaml_path;
    RCLCPP_INFO(node_->get_logger(),
      "ArmMoveNamed: loaded %zu named poses from %s",
      named_positions_.size(), yaml_path.c_str());
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(),
      "ArmMoveNamed: failed to load arm_yaml [%s]: %s",
      yaml_path.c_str(), e.what());
    return false;
  }
}

BT::NodeStatus ArmMoveNamedAction::onStart()
{
  if (!node_) {
    if (!getInput("node", node_) || !node_) {
      RCLCPP_ERROR(rclcpp::get_logger("ArmMoveNamed"),
        "Missing/null required input [node]");
      return BT::NodeStatus::FAILURE;
    }
  }

  std::string pose_name;
  if (!getInput("pose_name", pose_name) || pose_name.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "ArmMoveNamed: empty input [pose_name]");
    return BT::NodeStatus::FAILURE;
  }

  std::string yaml_path;
  (void)getInput("arm_yaml", yaml_path);
  if (yaml_path.empty()) {
    try {
      yaml_path = ament_index_cpp::get_package_share_directory("at_r2_bt") +
        "/yaml/arm_ready_position.yaml";
    } catch (const std::exception & e) {
      RCLCPP_ERROR(node_->get_logger(),
        "Cannot locate at_r2_bt share dir: %s", e.what());
      return BT::NodeStatus::FAILURE;
    }
  }

  if (!ensureLoaded(yaml_path)) {
    return BT::NodeStatus::FAILURE;
  }

  const auto it = named_positions_.find(pose_name);
  if (it == named_positions_.end()) {
    RCLCPP_ERROR(node_->get_logger(),
      "ArmMoveNamed: pose name [%s] not found in %s",
      pose_name.c_str(), yaml_path.c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (it->second.size() != 6) {
    RCLCPP_ERROR(node_->get_logger(),
      "ArmMoveNamed: pose [%s] has %zu joints, expected 6",
      pose_name.c_str(), it->second.size());
    return BT::NodeStatus::FAILURE;
  }

  int32_t task_id = 1;
  (void)getInput("task_id", task_id);

  double duration = 3.0;
  (void)getInput("duration", duration);

  std::string server_name{"robotic_task"};
  (void)getInput("server_name", server_name);

  action_client_ = rclcpp_action::create_client<ArmTask>(node_, server_name);
  if (!action_client_->wait_for_action_server(2s)) {
    RCLCPP_ERROR(node_->get_logger(),
      "ArmMoveNamed: action server [%s] not available", server_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  ArmTask::Goal goal_msg;
  goal_msg.task_id = task_id;
  goal_msg.data = it->second;  // 6 joints
  goal_msg.data.push_back(duration);  // 7th value: trajectory duration (seconds)

  goal_handle_.reset();
  goal_rejected_ = false;
  result_received_ = false;
  result_err_code_ = -1;
  result_reason_.clear();
  final_status_ = BT::NodeStatus::RUNNING;

  rclcpp_action::Client<ArmTask>::SendGoalOptions send_goal_options;
  send_goal_options.goal_response_callback =
    std::bind(&ArmMoveNamedAction::onGoalResponse, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(&ArmMoveNamedAction::onFeedback, this,
      std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&ArmMoveNamedAction::onResult, this, std::placeholders::_1);

  action_client_->async_send_goal(goal_msg, send_goal_options);
  RCLCPP_INFO(node_->get_logger(),
    "ArmMoveNamed goal sent: name=%s task_id=%d duration=%.2fs",
    pose_name.c_str(), task_id, duration);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ArmMoveNamedAction::onRunning()
{
  if (goal_rejected_) {
    return BT::NodeStatus::FAILURE;
  }
  if (!result_received_) {
    return BT::NodeStatus::RUNNING;
  }
  setOutput("err_code", result_err_code_);
  setOutput("reason", result_reason_);
  return final_status_;
}

void ArmMoveNamedAction::onHalted()
{
  if (action_client_ && goal_handle_) {
    (void)action_client_->async_cancel_goal(goal_handle_);
  }
  goal_handle_.reset();
  goal_rejected_ = false;
  result_received_ = false;
  final_status_ = BT::NodeStatus::RUNNING;
}

void ArmMoveNamedAction::onGoalResponse(const GoalHandleArmTask::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    goal_rejected_ = true;
    RCLCPP_ERROR(node_->get_logger(), "ArmMoveNamed goal rejected by server");
    return;
  }
  goal_handle_ = goal_handle;
  RCLCPP_INFO(node_->get_logger(), "ArmMoveNamed goal accepted");
}

void ArmMoveNamedAction::onFeedback(
  GoalHandleArmTask::SharedPtr,
  const std::shared_ptr<const ArmTask::Feedback> feedback)
{
  RCLCPP_INFO(node_->get_logger(),
    "ArmMoveNamed feedback: %s", feedback->describe.c_str());
}

void ArmMoveNamedAction::onResult(const GoalHandleArmTask::WrappedResult & result)
{
  result_received_ = true;
  result_err_code_ = result.result ? result.result->err_code : -1;
  result_reason_ = result.result ? result.result->reason : "empty result";

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      final_status_ = BT::NodeStatus::SUCCESS;
      break;
    case rclcpp_action::ResultCode::ABORTED:
    case rclcpp_action::ResultCode::CANCELED:
    default:
      final_status_ = BT::NodeStatus::FAILURE;
      break;
  }

  RCLCPP_INFO(node_->get_logger(),
    "ArmMoveNamed finished: code=%d, err_code=%d, reason=%s",
    static_cast<int>(result.code), result_err_code_, result_reason_.c_str());
}

}  // namespace nav2_bt_publish_goal

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_bt_publish_goal::ArmMoveNamedAction>("ArmMoveNamed");
}
