// Copyright 2024
// Licensed under the Apache License, Version 2.0

#include "nav2_bt_publish_goal/climb_stair_action.hpp"

namespace nav2_bt_publish_goal
{

ClimbStairAction::ClimbStairAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config),
  current_status_(0),
  status_received_(false)
{
  // 不在构造函数中获取 node，而是在 onStart 中获取
}

BT::PortsList ClimbStairAction::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "ROS node"),
    BT::InputPort<double>("height", "Climb height in meters (must be positive)"),
    BT::OutputPort<double>("climb_height", "Actual published climb height value")
  };
}

BT::NodeStatus ClimbStairAction::onStart()
{
  // Get the ROS node from the blackboard (first time initialization)
  if (!node_) {
    if (!getInput("node", node_)) {
      RCLCPP_ERROR(rclcpp::get_logger("ClimbStairAction"), "Missing required input [node]");
      return BT::NodeStatus::FAILURE;
    }
    
    if (!node_) {
      RCLCPP_ERROR(rclcpp::get_logger("ClimbStairAction"), "Node pointer is null");
      return BT::NodeStatus::FAILURE;
    }
    
    // Create publisher for climb command
    climb_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
      "/AT_R2/climb_stair", 10);
    
    // Create subscriber for climber status
    status_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
      "/AT_R2/climber_status", 10,
      std::bind(&ClimbStairAction::statusCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(node_->get_logger(), "ClimbStairAction initialized");
  }
  
  // Read height parameter
  double height;
  if (!getInput("height", height)) {
    RCLCPP_ERROR(node_->get_logger(), "Missing required input [height]");
    return BT::NodeStatus::FAILURE;
  }
  
  // Validate height parameter
  if (height <= 0.0) {
    RCLCPP_ERROR(node_->get_logger(), 
      "Invalid height: %.2f (must be positive)", height);
    return BT::NodeStatus::FAILURE;
  }
  
  // Reset state
  current_status_ = 0;
  status_received_ = false;
  
  // Publish climb command
  auto msg = std_msgs::msg::Float64();
  msg.data = height;
  climb_pub_->publish(msg);
  
  RCLCPP_INFO(node_->get_logger(), 
    "Published climb command: %.2f meters", height);
  
  // Set output port
  setOutput("climb_height", height);
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ClimbStairAction::onRunning()
{
  // If no status received yet, keep waiting
  if (!status_received_) {
    return BT::NodeStatus::RUNNING;
  }
  
  // Return result based on status
  switch (current_status_) {
    case STATUS_CLIMBING:
      RCLCPP_DEBUG(node_->get_logger(), "Still climbing...");
      return BT::NodeStatus::RUNNING;
      
    case STATUS_SUCCESS:
      RCLCPP_INFO(node_->get_logger(), "✓ Climb succeeded!");
      return BT::NodeStatus::SUCCESS;
      
    case STATUS_FAILURE:
      RCLCPP_ERROR(node_->get_logger(), "✗ Climb failed!");
      return BT::NodeStatus::FAILURE;
      
    default:
      RCLCPP_WARN(node_->get_logger(), 
        "Unknown status: %d", current_status_);
      return BT::NodeStatus::RUNNING;
  }
}

void ClimbStairAction::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "ClimbStair action halted");
  
  // Reset state
  current_status_ = 0;
  status_received_ = false;
  
  // Note: publisher and subscriber will be automatically cleaned up
}

void ClimbStairAction::statusCallback(
  const std_msgs::msg::Int32::SharedPtr msg)
{
  current_status_ = msg->data;
  status_received_ = true;
  
  RCLCPP_DEBUG(node_->get_logger(), 
    "Received climber status: %d", current_status_);
}

}  // namespace nav2_bt_publish_goal

// Register the node with BehaviorTree.CPP v4
#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_bt_publish_goal::ClimbStairAction>("ClimbStair");
}
