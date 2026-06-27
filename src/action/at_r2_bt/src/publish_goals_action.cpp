// Copyright 2024
// Licensed under the Apache License, Version 2.0

#include "nav2_bt_publish_goal/publish_goals_action.hpp"

#include <sstream>
#include <vector>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_bt_publish_goal
{

PublishGoalsAction::PublishGoalsAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config),
  goal_result_available_(false)
{
  RCLCPP_DEBUG(rclcpp::get_logger("PublishGoalsAction"), "PublishGoalsAction constructed");
}

BT::PortsList PublishGoalsAction::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "ROS node"),
    BT::InputPort<std::string>("poses",
      "List of poses: 'x1,y1,yaw1; x2,y2,yaw2; ...' (meters, radians)"),
    BT::InputPort<std::string>("frame_id", "map", "Reference frame"),
    BT::InputPort<std::string>("action_name",
      "/AT_R2/navigate_through_poses", "Action server name"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal", "{goal}",
      "Final goal pose (last entry of poses)")
  };
}

BT::NodeStatus PublishGoalsAction::onStart()
{
  if (!node_) {
    if (!getInput("node", node_)) {
      RCLCPP_ERROR(rclcpp::get_logger("PublishGoalsAction"),
        "Missing required input [node]");
      return BT::NodeStatus::FAILURE;
    }
    if (!node_) {
      RCLCPP_ERROR(rclcpp::get_logger("PublishGoalsAction"),
        "Node pointer is null");
      return BT::NodeStatus::FAILURE;
    }

    std::string action_name = "/AT_R2/navigate_through_poses";
    getInput("action_name", action_name);

    action_client_ = rclcpp_action::create_client<NavigateThroughPoses>(node_, action_name);

    auto goal_qos = rclcpp::QoS(1).transient_local().reliable();
    current_goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/AT_R2/current_nav_goal", goal_qos);

    RCLCPP_INFO(node_->get_logger(),
      "PublishGoalsAction initialized with action server: %s", action_name.c_str());
  }

  std::string poses_str;
  std::string frame_id;

  if (!getInput("poses", poses_str)) {
    RCLCPP_ERROR(node_->get_logger(), "Missing required input [poses]");
    return BT::NodeStatus::FAILURE;
  }
  if (!getInput("frame_id", frame_id)) {
    frame_id = "map";
  }

  std::vector<geometry_msgs::msg::PoseStamped> poses;
  if (!parsePoses(poses_str, frame_id, poses) || poses.empty()) {
    RCLCPP_ERROR(node_->get_logger(),
      "Failed to parse poses string or empty list: '%s'", poses_str.c_str());
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(), "Waiting for action server...");
  if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node_->get_logger(),
      "Action server not available after waiting 10 seconds");
    return BT::NodeStatus::FAILURE;
  }
  RCLCPP_INFO(node_->get_logger(), "Action server available!");

  auto action_goal = NavigateThroughPoses::Goal();
  action_goal.poses = poses;
  action_goal.behavior_tree = "";  // 使用默认 navigate_through_poses BT

  std::ostringstream pose_log;
  for (size_t i = 0; i < poses.size(); ++i) {
    const auto & p = poses[i].pose;
    pose_log << "[" << p.position.x << "," << p.position.y << "]";
    if (i + 1 < poses.size()) {
      pose_log << " -> ";
    }
  }
  RCLCPP_INFO(node_->get_logger(),
    "Sending NavigateThroughPoses with %zu pose(s) in frame '%s': %s",
    poses.size(), frame_id.c_str(), pose_log.str().c_str());

  auto send_goal_options =
    rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    [this](const GoalHandleNavigateThroughPoses::SharedPtr & goal_handle) {
      if (!goal_handle) {
        RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
        goal_handle_ = nullptr;
      } else {
        RCLCPP_INFO(node_->get_logger(),
          "Goal accepted by server, navigating through poses...");
        goal_handle_ = goal_handle;
      }
    };

  send_goal_options.result_callback =
    [this](const GoalHandleNavigateThroughPoses::WrappedResult & result) {
      goal_result_available_ = true;
      result_ = result;
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(node_->get_logger(), "✓ NavigateThroughPoses succeeded!");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(node_->get_logger(), "✗ NavigateThroughPoses was aborted");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_WARN(node_->get_logger(), "NavigateThroughPoses was canceled");
          break;
        default:
          RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
          break;
      }
    };

  send_goal_options.feedback_callback =
    [this](GoalHandleNavigateThroughPoses::SharedPtr,
           const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback) {
      RCLCPP_DEBUG(node_->get_logger(),
        "Distance remaining: %.2f, poses remaining: %d",
        feedback->distance_remaining,
        feedback->number_of_poses_remaining);
    };

  goal_result_available_ = false;
  action_client_->async_send_goal(action_goal, send_goal_options);

  // 把最后一个 pose 写到 blackboard 和 latched 话题, 与 PublishGoal 行为对齐
  const auto & final_goal = poses.back();
  setOutput("goal", final_goal);
  if (current_goal_pub_) {
    current_goal_pub_->publish(final_goal);
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PublishGoalsAction::onRunning()
{
  if (goal_result_available_) {
    if (result_.code == rclcpp_action::ResultCode::SUCCEEDED) {
      return BT::NodeStatus::SUCCESS;
    } else {
      return BT::NodeStatus::FAILURE;
    }
  }
  return BT::NodeStatus::RUNNING;
}

void PublishGoalsAction::onHalted()
{
  if (goal_handle_) {
    RCLCPP_INFO(node_->get_logger(), "Canceling NavigateThroughPoses goal...");
    action_client_->async_cancel_goal(goal_handle_);
    goal_handle_ = nullptr;
  }
  goal_result_available_ = false;
}

bool PublishGoalsAction::parsePoses(
  const std::string & poses_str,
  const std::string & frame_id,
  std::vector<geometry_msgs::msg::PoseStamped> & out)
{
  out.clear();

  // 以 ';' 切成一个个 pose, 每个 pose 内部以 ',' 切成 (x, y, yaw)
  std::stringstream ss(poses_str);
  std::string item;
  while (std::getline(ss, item, ';')) {
    // 去前后空白
    size_t start = item.find_first_not_of(" \t\r\n");
    size_t end = item.find_last_not_of(" \t\r\n");
    if (start == std::string::npos) {
      continue;  // 空段, 跳过
    }
    std::string token = item.substr(start, end - start + 1);

    double x = 0.0, y = 0.0, yaw = 0.0;
    char c1 = 0, c2 = 0;
    std::stringstream ts(token);
    ts >> x >> c1 >> y >> c2 >> yaw;
    if (ts.fail() || c1 != ',' || c2 != ',') {
      RCLCPP_ERROR(node_->get_logger(),
        "Invalid pose token: '%s' (expected 'x,y,yaw')", token.c_str());
      return false;
    }
    out.push_back(createGoalMessage(x, y, yaw, frame_id));
  }
  return true;
}

geometry_msgs::msg::PoseStamped PublishGoalsAction::createGoalMessage(
  double x, double y, double yaw, const std::string & frame_id)
{
  geometry_msgs::msg::PoseStamped goal_msg;
  goal_msg.header.stamp = node_->now();
  goal_msg.header.frame_id = frame_id;
  goal_msg.pose.position.x = x;
  goal_msg.pose.position.y = y;
  goal_msg.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  goal_msg.pose.orientation = tf2::toMsg(q);
  return goal_msg;
}

}  // namespace nav2_bt_publish_goal

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_bt_publish_goal::PublishGoalsAction>("PublishGoals");
}
