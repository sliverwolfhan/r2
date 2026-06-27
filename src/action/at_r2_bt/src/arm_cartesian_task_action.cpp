#include "nav2_bt_publish_goal/arm_cartesian_task_action.hpp"

#include <chrono>
#include <functional>
#include <sstream>

#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_bt_publish_goal
{

using namespace std::chrono_literals;

ArmCartesianTaskAction::ArmCartesianTaskAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
}

BT::PortsList ArmCartesianTaskAction::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "ROS node"),
    BT::InputPort<int32_t>("task_id", 4, "Arm task id (默认 4 = 笛卡尔末端运动)"),
    BT::InputPort<std::string>("server_name", "robotic_task", "Action server name"),
    BT::InputPort<std::string>("base_frame", "base_link", "Robot base TF frame"),
    BT::InputPort<std::string>("map_frame", "map", "Map TF frame"),
    BT::InputPort<std::string>("tf_namespace", "AT_R2",
      "Namespace prefix for /tf and /tf_static topics; empty = global /tf"),
    BT::InputPort<double>("tf_timeout", 1.0, "TF lookup timeout seconds"),
    BT::InputPort<double>("cube_x", "Target x in map frame"),
    BT::InputPort<double>("cube_y", "Target y in map frame"),
    BT::InputPort<double>("cube_z", 0.0, "Target z in map frame (默认 0)"),
    BT::OutputPort<std::vector<double>>("goal_data", "Final goal data sent to ArmTask"),
    BT::OutputPort<int32_t>("err_code", "ArmTask result error code"),
    BT::OutputPort<std::string>("reason", "ArmTask result reason")
  };
}

BT::NodeStatus ArmCartesianTaskAction::onStart()
{
  if (!node_) {
    if (!getInput("node", node_)) {
      RCLCPP_ERROR(rclcpp::get_logger("ArmCartesianTask"), "Missing required input [node]");
      return BT::NodeStatus::FAILURE;
    }
    if (!node_) {
      RCLCPP_ERROR(rclcpp::get_logger("ArmCartesianTask"), "Input [node] is null");
      return BT::NodeStatus::FAILURE;
    }

    // 优先用 simple_bt_runner 注入到黑板的全局 tf_buffer。
    auto bb = config().blackboard;
    if (bb) {
      std::shared_ptr<tf2_ros::Buffer> shared_buf;
      if (bb->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", shared_buf) && shared_buf) {
        tf_buffer_ = shared_buf;
        RCLCPP_INFO(node_->get_logger(),
          "ArmCartesianTask: using shared tf_buffer from blackboard");
      }
    }

    // 黑板里没有就回退到自己 lazy 创建。
    if (!tf_buffer_) {
      std::string tf_ns = "AT_R2";
      (void)getInput("tf_namespace", tf_ns);

      rclcpp::NodeOptions tf_node_opts;
      bool use_sim_time = false;
      node_->get_parameter_or("use_sim_time", use_sim_time, false);
      tf_node_opts.parameter_overrides({rclcpp::Parameter("use_sim_time", use_sim_time)});

      std::vector<std::string> remap_args;
      if (!tf_ns.empty()) {
        const std::string ns_prefix = (tf_ns.front() == '/') ? tf_ns : ("/" + tf_ns);
        remap_args = {
          "--ros-args",
          "-r", "/tf:=" + ns_prefix + "/tf",
          "-r", "/tf_static:=" + ns_prefix + "/tf_static",
        };
      }
      tf_node_opts.arguments(remap_args);

      tf_node_ = rclcpp::Node::make_shared("arm_cartesian_task_tf", "", tf_node_opts);
      RCLCPP_WARN(node_->get_logger(),
        "ArmCartesianTask: blackboard has no tf_buffer, falling back to local listener "
        "(remap /tf -> %s/tf)",
        tf_ns.empty() ? "(global)" : ("/" + tf_ns).c_str());

      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, tf_node_, true);
    }
  }

  int32_t task_id = 4;
  (void)getInput("task_id", task_id);

  std::string server_name{"robotic_task"};
  (void)getInput("server_name", server_name);

  std::string base_frame{"base_link"};
  (void)getInput("base_frame", base_frame);

  std::string map_frame{"map"};
  (void)getInput("map_frame", map_frame);

  double tf_timeout = 1.0;
  (void)getInput("tf_timeout", tf_timeout);
  if (tf_timeout < 0.0) {
    RCLCPP_ERROR(node_->get_logger(), "Input [tf_timeout] must be >= 0.0, got %.3f", tf_timeout);
    return BT::NodeStatus::FAILURE;
  }

  double cube_x = 0.0;
  double cube_y = 0.0;
  double cube_z = 0.0;
  if (!getInput("cube_x", cube_x)) {
    RCLCPP_ERROR(node_->get_logger(), "Missing required input [cube_x]");
    return BT::NodeStatus::FAILURE;
  }
  if (!getInput("cube_y", cube_y)) {
    RCLCPP_ERROR(node_->get_logger(), "Missing required input [cube_y]");
    return BT::NodeStatus::FAILURE;
  }
  (void)getInput("cube_z", cube_z);

  action_client_ = rclcpp_action::create_client<ArmTask>(node_, server_name);

  if (!action_client_->wait_for_action_server(2s)) {
    RCLCPP_ERROR(node_->get_logger(), "Action server [%s] not available", server_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  std::vector<double> goal_data;
  if (!buildGoalDataFromMap(
      base_frame, map_frame, tf_timeout, cube_x, cube_y, cube_z, goal_data))
  {
    return BT::NodeStatus::FAILURE;
  }

  ArmTask::Goal goal_msg;
  goal_msg.task_id = task_id;
  goal_msg.data = goal_data;
  setOutput("goal_data", goal_data);

  std::ostringstream oss;
  oss << "[";
  for (size_t i = 0; i < goal_msg.data.size(); ++i) {
    oss << goal_msg.data[i];
    if (i + 1 < goal_msg.data.size()) {
      oss << ", ";
    }
  }
  oss << "]";
  RCLCPP_INFO(
    node_->get_logger(),
    "ArmCartesianTask request: server=%s task_id=%d data_size=%zu data=%s",
    server_name.c_str(),
    task_id,
    goal_msg.data.size(),
    oss.str().c_str());

  goal_handle_.reset();
  goal_rejected_ = false;
  result_received_ = false;
  result_err_code_ = -1;
  result_reason_.clear();
  final_status_ = BT::NodeStatus::RUNNING;

  rclcpp_action::Client<ArmTask>::SendGoalOptions send_goal_options;
  send_goal_options.goal_response_callback =
    std::bind(&ArmCartesianTaskAction::onGoalResponse, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(&ArmCartesianTaskAction::onFeedback, this,
      std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&ArmCartesianTaskAction::onResult, this, std::placeholders::_1);

  action_client_->async_send_goal(goal_msg, send_goal_options);
  RCLCPP_INFO(
    node_->get_logger(),
    "ArmCartesianTask goal sent: task_id=%d, data_size=%zu",
    task_id, goal_data.size());

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ArmCartesianTaskAction::onRunning()
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

void ArmCartesianTaskAction::onHalted()
{
  if (action_client_ && goal_handle_) {
    (void)action_client_->async_cancel_goal(goal_handle_);
  }
  goal_handle_.reset();
  goal_rejected_ = false;
  result_received_ = false;
  final_status_ = BT::NodeStatus::RUNNING;
}

void ArmCartesianTaskAction::onGoalResponse(const GoalHandleArmTask::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    goal_rejected_ = true;
    RCLCPP_ERROR(node_->get_logger(), "ArmCartesianTask goal rejected by server");
    return;
  }
  goal_handle_ = goal_handle;
  RCLCPP_INFO(node_->get_logger(), "ArmCartesianTask goal accepted");
}

void ArmCartesianTaskAction::onFeedback(
  GoalHandleArmTask::SharedPtr,
  const std::shared_ptr<const ArmTask::Feedback> feedback)
{
  RCLCPP_INFO(node_->get_logger(), "ArmCartesianTask feedback: %s", feedback->describe.c_str());
}

void ArmCartesianTaskAction::onResult(const GoalHandleArmTask::WrappedResult & result)
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
    "ArmCartesianTask finished: code=%d, err_code=%d, reason=%s",
    static_cast<int>(result.code), result_err_code_, result_reason_.c_str());
}

bool ArmCartesianTaskAction::buildGoalDataFromMap(
  const std::string & base_frame,
  const std::string & map_frame,
  double tf_timeout_sec,
  double cube_x,
  double cube_y,
  double cube_z,
  std::vector<double> & out_goal_data)
{
  try {
    const tf2::Duration tf_timeout = tf2::durationFromSec(tf_timeout_sec);
    if (!tf_buffer_->canTransform(
        base_frame, map_frame, tf2::TimePointZero, tf_timeout))
    {
      RCLCPP_ERROR(node_->get_logger(),
        "TF unavailable: transform from %s to %s",
        map_frame.c_str(), base_frame.c_str());
      return false;
    }
    const auto tf_mb = tf_buffer_->lookupTransform(
      base_frame, map_frame, tf2::TimePointZero);

    geometry_msgs::msg::PoseStamped target_in_map;
    target_in_map.header.frame_id = map_frame;
    target_in_map.pose.position.x = cube_x;
    target_in_map.pose.position.y = cube_y;
    target_in_map.pose.position.z = cube_z;
    target_in_map.pose.orientation.w = 1.0;

    geometry_msgs::msg::PoseStamped target_in_base;
    tf2::doTransform(target_in_map, target_in_base, tf_mb);

    // 7 维: [x, y, z, qx, qy, qz, qw] —— 机械臂端忽略四元数, 但保留以匹配约定。
    // 注意: x、y 用 TF 变换后的 base_link 系下的值 (来自 map 系 cube_x/cube_y),
    //       z 用输入 cube_z 固定值, 不做 TF 变换。
    out_goal_data = {
      target_in_base.pose.position.x,
      target_in_base.pose.position.y,
      cube_z,
      target_in_base.pose.orientation.x,
      target_in_base.pose.orientation.y,
      target_in_base.pose.orientation.z,
      target_in_base.pose.orientation.w
    };
    RCLCPP_INFO(
      node_->get_logger(),
      "Target: map(x=%.3f, y=%.3f) -> send(x=%.3f, y=%.3f, z=%.3f[fixed]) in %s",
      cube_x, cube_y,
      target_in_base.pose.position.x,
      target_in_base.pose.position.y,
      cube_z,
      base_frame.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "TF transform failed: %s", e.what());
    return false;
  }

  return true;
}

}  // namespace nav2_bt_publish_goal

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_bt_publish_goal::ArmCartesianTaskAction>("ArmCartesianTask");
}
