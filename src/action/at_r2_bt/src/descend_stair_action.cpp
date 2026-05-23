// Copyright 2024
// Licensed under the Apache License, Version 2.0

#include "nav2_bt_publish_goal/descend_stair_action.hpp"

namespace nav2_bt_publish_goal
{

DescendStairAction::DescendStairAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config),
  current_status_(0),
  status_received_(false)
{
  // 不在构造函数中获取 node，而是在 onStart 中获取
}

BT::PortsList DescendStairAction::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "ROS node"),
    BT::InputPort<double>("height", "Descend height in meters (must be positive)"),
    BT::OutputPort<double>("descend_height", "Actual published descend height value")
  };
}

BT::NodeStatus DescendStairAction::onStart()
{
  // Get the ROS node from the blackboard (first time initialization)
  if (!node_) {
    if (!getInput("node", node_)) {
      RCLCPP_ERROR(rclcpp::get_logger("DescendStairAction"), "Missing required input [node]");
      return BT::NodeStatus::FAILURE;
    }
    
    if (!node_) {
      RCLCPP_ERROR(rclcpp::get_logger("DescendStairAction"), "Node pointer is null");
      return BT::NodeStatus::FAILURE;
    }
    
    // Create publisher for descend command
    descend_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
      "/AT_R2/descend_stair", 10);
    
    // Create subscriber for climber status
    status_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
      "/AT_R2/climber_status", 10,
      std::bind(&DescendStairAction::statusCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(node_->get_logger(), "DescendStairAction initialized");
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
  
  // Publish descend command
  auto msg = std_msgs::msg::Float64();
  msg.data = height;
  descend_pub_->publish(msg);
  
  RCLCPP_INFO(node_->get_logger(), 
    "Published descend command: %.2f meters", height);
  
  // Set output port
  setOutput("descend_height", height);
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DescendStairAction::onRunning()
{
  // If no status received yet, keep waiting
  if (!status_received_) {
    return BT::NodeStatus::RUNNING;
  }
  
  // Return result based on status
  switch (current_status_) {
    case STATUS_DESCENDING:
      RCLCPP_DEBUG(node_->get_logger(), "Still descending...");
      return BT::NodeStatus::RUNNING;
      
    case STATUS_SUCCESS:
      RCLCPP_INFO(node_->get_logger(), "✓ Descend succeeded!");
      return BT::NodeStatus::SUCCESS;
      
    case STATUS_FAILURE:
      RCLCPP_ERROR(node_->get_logger(), "✗ Descend failed!");
      return BT::NodeStatus::FAILURE;
      
    default:
      RCLCPP_WARN(node_->get_logger(), 
        "Unknown status: %d", current_status_);
      return BT::NodeStatus::RUNNING;
  }
}

void DescendStairAction::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "DescendStair action halted");
  
  // Reset state
  current_status_ = 0;
  status_received_ = false;
  
  // Note: publisher and subscriber will be automatically cleaned up
}

void DescendStairAction::statusCallback(
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
  factory.registerNodeType<nav2_bt_publish_goal::DescendStairAction>("DescendStair");
}
