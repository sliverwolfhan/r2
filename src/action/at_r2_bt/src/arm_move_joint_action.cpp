#include "nav2_bt_publish_goal/arm_move_joint_action.hpp"

#include <functional>

#include "behaviortree_cpp/bt_factory.h"

namespace nav2_bt_publish_goal
{

using namespace std::chrono_literals;

ArmMoveJointAction::ArmMoveJointAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
}

BT::PortsList ArmMoveJointAction::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "ROS node"),
    BT::InputPort<int32_t>("task_id", 1, "Arm task id (move_kfs = 1)"),
    BT::InputPort<std::string>("server_name", "robotic_task", "Action server name"),
    BT::InputPort<double>("j1", "Joint 1 angle (rad), required"),
    BT::InputPort<double>("j2", "Joint 2 angle (rad), required"),
    BT::InputPort<double>("j3", "Joint 3 angle (rad), required"),
    BT::InputPort<double>("j4", "Joint 4 angle (rad), required"),
    BT::InputPort<double>("j5", "Joint 5 angle (rad), required"),
    BT::InputPort<double>("j6", "Joint 6 angle (rad), required"),
    BT::InputPort<double>(
      "duration_sec", -1.0,
      "Optional trajectory duration (s). If < 0, send 6 joints only (server uses trajectory_duration); "
      "if >= 0, append as 7th value."),
    BT::OutputPort<std::vector<double>>("goal_data", "goal.data sent to ArmTask"),
    BT::OutputPort<int32_t>("err_code", "ArmTask result error code"),
    BT::OutputPort<std::string>("reason", "ArmTask result reason")
  };
}

BT::NodeStatus ArmMoveJointAction::onStart()
{
  if (!node_) {
    if (!getInput("node", node_)) {
      RCLCPP_ERROR(rclcpp::get_logger("ArmMoveJointAction"), "Missing required input [node]");
      return BT::NodeStatus::FAILURE;
    }
    if (!node_) {
      RCLCPP_ERROR(rclcpp::get_logger("ArmMoveJointAction"), "Input [node] is null");
      return BT::NodeStatus::FAILURE;
    }
  }

  int32_t task_id = 1;
  (void)getInput("task_id", task_id);

  std::string server_name{"robotic_task"};
  (void)getInput("server_name", server_name);

  double j1 = 0.0, j2 = 0.0, j3 = 0.0, j4 = 0.0, j5 = 0.0, j6 = 0.0;
  if (!getInput("j1", j1) || !getInput("j2", j2) || !getInput("j3", j3) ||
    !getInput("j4", j4) || !getInput("j5", j5) || !getInput("j6", j6))
  {
    RCLCPP_ERROR(node_->get_logger(), "ArmMoveJoint: missing one of [j1..j6]");
    return BT::NodeStatus::FAILURE;
  }

  double duration_sec = -1.0;
  (void)getInput("duration_sec", duration_sec);

  std::vector<double> goal_data{j1, j2, j3, j4, j5, j6};
  if (duration_sec >= 0.0) {
    goal_data.push_back(duration_sec);
  }

  action_client_ = rclcpp_action::create_client<ArmTask>(node_, server_name);
  if (!action_client_->wait_for_action_server(2s)) {
    RCLCPP_ERROR(node_->get_logger(), "ArmMoveJoint: action server [%s] not available", server_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  ArmTask::Goal goal_msg;
  goal_msg.task_id = task_id;
  goal_msg.data = goal_data;
  setOutput("goal_data", goal_data);

  goal_handle_.reset();
  goal_rejected_ = false;
  result_received_ = false;
  result_err_code_ = -1;
  result_reason_.clear();
  final_status_ = BT::NodeStatus::RUNNING;

  rclcpp_action::Client<ArmTask>::SendGoalOptions send_goal_options;
  send_goal_options.goal_response_callback =
    std::bind(&ArmMoveJointAction::onGoalResponse, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(&ArmMoveJointAction::onFeedback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&ArmMoveJointAction::onResult, this, std::placeholders::_1);

  action_client_->async_send_goal(goal_msg, send_goal_options);
  RCLCPP_INFO(
    node_->get_logger(),
    "ArmMoveJoint goal sent: task_id=%d, data_size=%zu",
    task_id, goal_data.size());

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ArmMoveJointAction::onRunning()
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

void ArmMoveJointAction::onHalted()
{
  if (action_client_ && goal_handle_) {
    (void)action_client_->async_cancel_goal(goal_handle_);
  }
  goal_handle_.reset();
  goal_rejected_ = false;
  result_received_ = false;
  final_status_ = BT::NodeStatus::RUNNING;
}

void ArmMoveJointAction::onGoalResponse(const GoalHandleArmTask::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    goal_rejected_ = true;
    RCLCPP_ERROR(node_->get_logger(), "ArmMoveJoint goal rejected by server");
    return;
  }
  goal_handle_ = goal_handle;
  RCLCPP_INFO(node_->get_logger(), "ArmMoveJoint goal accepted");
}

void ArmMoveJointAction::onFeedback(
  GoalHandleArmTask::SharedPtr,
  const std::shared_ptr<const ArmTask::Feedback> feedback)
{
  RCLCPP_INFO(node_->get_logger(), "ArmMoveJoint feedback: %s", feedback->describe.c_str());
}

void ArmMoveJointAction::onResult(const GoalHandleArmTask::WrappedResult & result)
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

  RCLCPP_INFO(
    node_->get_logger(),
    "ArmMoveJoint finished: code=%d, err_code=%d, reason=%s",
    static_cast<int>(result.code), result_err_code_, result_reason_.c_str());
}

}  // namespace nav2_bt_publish_goal

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_bt_publish_goal::ArmMoveJointAction>("ArmMoveJoint");
}
