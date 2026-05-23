// Copyright 2024
// Licensed under the Apache License, Version 2.0

#include "nav2_bt_publish_goal/publish_goal_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_bt_publish_goal
{

PublishGoalAction::PublishGoalAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config),
  goal_result_available_(false)
{
  // 不在构造函数中获取 node，而是在 onStart 中获取
  RCLCPP_DEBUG(rclcpp::get_logger("PublishGoalAction"), "PublishGoalAction constructed");
}

BT::PortsList PublishGoalAction::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "ROS node"),
    BT::InputPort<double>("x", "X coordinate in meters"),
    BT::InputPort<double>("y", "Y coordinate in meters"),
    BT::InputPort<double>("yaw", "Yaw angle in radians"),
    BT::InputPort<std::string>("frame_id", "map", "Reference frame"),
    BT::InputPort<std::string>("action_name", "/AT_R2/navigate_to_pose", "Action server name"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal", "{goal}", "Goal pose")
  };
}

BT::NodeStatus PublishGoalAction::onStart()
{
  // Get the ROS node from the blackboard (first time initialization)
  if (!node_) {
    if (!getInput("node", node_)) {
      RCLCPP_ERROR(rclcpp::get_logger("PublishGoalAction"), "Missing required input [node]");
      return BT::NodeStatus::FAILURE;
    }
    
    if (!node_) {
      RCLCPP_ERROR(rclcpp::get_logger("PublishGoalAction"), "Node pointer is null");
      return BT::NodeStatus::FAILURE;
    }
    
    // Get action server name (default to /AT_R2/navigate_to_pose)
    std::string action_name = "/AT_R2/navigate_to_pose";
    getInput("action_name", action_name);
    
    // Create action client
    action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, action_name);

    // Latched publisher for the current navigation goal (consumed by recovery
    // behaviors like BackUpFreeSpace).
    auto goal_qos = rclcpp::QoS(1).transient_local().reliable();
    current_goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/AT_R2/current_nav_goal", goal_qos);

    RCLCPP_INFO(node_->get_logger(), "PublishGoalAction initialized with action server: %s", action_name.c_str());
  }
  
  // Read input parameters
  double x, y, yaw;
  std::string frame_id;
  
  if (!getInput("x", x)) {
    RCLCPP_ERROR(node_->get_logger(), "Missing required input [x]");
    return BT::NodeStatus::FAILURE;
  }
  
  if (!getInput("y", y)) {
    RCLCPP_ERROR(node_->get_logger(), "Missing required input [y]");
    return BT::NodeStatus::FAILURE;
  }
  
  if (!getInput("yaw", yaw)) {
    RCLCPP_ERROR(node_->get_logger(), "Missing required input [yaw]");
    return BT::NodeStatus::FAILURE;
  }
  
  if (!getInput("frame_id", frame_id)) {
    frame_id = "map";
  }
  
  // Wait for action server
  RCLCPP_INFO(node_->get_logger(), "Waiting for action server...");
  if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting 10 seconds");
    return BT::NodeStatus::FAILURE;
  }
  
  RCLCPP_INFO(node_->get_logger(), "Action server available!");
  
  // Create goal message
  auto goal_msg = createGoalMessage(x, y, yaw, frame_id);
  
  // Create action goal
  auto action_goal = NavigateToPose::Goal();
  action_goal.pose = goal_msg;
  action_goal.behavior_tree = "";  // 使用默认行为树
  
  // Send action goal
  RCLCPP_INFO(node_->get_logger(), "Sending navigation goal: [%.2f, %.2f, %.2f] in frame '%s'",
    x, y, yaw, frame_id.c_str());
  
  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  
  // 目标响应回调
  send_goal_options.goal_response_callback =
    [this](const GoalHandleNavigateToPose::SharedPtr & goal_handle) {
      if (!goal_handle) {
        RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
        goal_handle_ = nullptr;
      } else {
        RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, navigating...");
        goal_handle_ = goal_handle;
      }
    };
  
  // 结果回调
  send_goal_options.result_callback =
    [this](const GoalHandleNavigateToPose::WrappedResult & result) {
      goal_result_available_ = true;
      result_ = result;
      
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(node_->get_logger(), "✓ Navigation succeeded!");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(node_->get_logger(), "✗ Navigation was aborted");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_WARN(node_->get_logger(), "Navigation was canceled");
          break;
        default:
          RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
          break;
      }
    };
  
  // 反馈回调（可选）
  send_goal_options.feedback_callback =
    [this](GoalHandleNavigateToPose::SharedPtr,
           const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
      RCLCPP_DEBUG(node_->get_logger(), 
        "Distance remaining: %.2f", 
        feedback->distance_remaining);
    };
  
  // 发送目标
  goal_result_available_ = false;
  action_client_->async_send_goal(action_goal, send_goal_options);
  
  // Write to blackboard
  setOutput("goal", goal_msg);

  // 同时把目标点 latched 发出去，给 BackUpFreeSpace 等行为使用
  if (current_goal_pub_) {
    current_goal_pub_->publish(goal_msg);
  }
  
  // 返回 RUNNING 状态，表示正在执行
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PublishGoalAction::onRunning()
{
  // 检查是否收到结果
  if (goal_result_available_) {
    if (result_.code == rclcpp_action::ResultCode::SUCCEEDED) {
      return BT::NodeStatus::SUCCESS;
    } else {
      return BT::NodeStatus::FAILURE;
    }
  }
  
  // 还在执行中
  return BT::NodeStatus::RUNNING;
}

void PublishGoalAction::onHalted()
{
  // 如果节点被中断，取消目标
  if (goal_handle_) {
    RCLCPP_INFO(node_->get_logger(), "Canceling navigation goal...");
    action_client_->async_cancel_goal(goal_handle_);
    goal_handle_ = nullptr;
  }
  goal_result_available_ = false;
}

geometry_msgs::msg::PoseStamped PublishGoalAction::createGoalMessage(
  double x, double y, double yaw, const std::string & frame_id)
{
  geometry_msgs::msg::PoseStamped goal_msg;
  
  // Set header
  goal_msg.header.stamp = node_->now();
  goal_msg.header.frame_id = frame_id;
  
  // Set position
  goal_msg.pose.position.x = x;
  goal_msg.pose.position.y = y;
  goal_msg.pose.position.z = 0.0;
  
  // Convert yaw to quaternion
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  goal_msg.pose.orientation = tf2::toMsg(q);
  
  return goal_msg;
}

}  // namespace nav2_bt_publish_goal

// Register the node with BehaviorTree.CPP v4
#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_bt_publish_goal::PublishGoalAction>("PublishGoal");
}
