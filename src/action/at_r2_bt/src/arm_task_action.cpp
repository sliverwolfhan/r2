#include "nav2_bt_publish_goal/arm_task_action.hpp"
#include <sstream>

#include <chrono>
#include <functional>

#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_bt_publish_goal
{

using namespace std::chrono_literals;

ArmTaskAction::ArmTaskAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
}

BT::PortsList ArmTaskAction::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "ROS node"),
    BT::InputPort<int32_t>("task_id", "Arm task id"),
    BT::InputPort<std::string>("server_name", "robotic_task", "Action server name"),
    BT::InputPort<std::string>("base_frame", "base_link", "Robot base TF frame"),
    BT::InputPort<std::string>("target_frame", "target_object", "Cube TF frame (preferred)"),
    BT::InputPort<std::string>("map_frame", "map", "Map TF frame (fallback path)"),
    BT::InputPort<std::string>("tf_namespace", "AT_R2",
      "Namespace prefix for /tf and /tf_static topics; empty = global /tf"),
    BT::InputPort<double>("tf_timeout", 1.0, "TF lookup timeout seconds"),
    BT::InputPort<double>("cube_x", "Cube x in map frame (used when TF target_frame missing)"),
    BT::InputPort<double>("cube_y", "Cube y in map frame (used when TF target_frame missing)"),
    BT::InputPort<double>("block_height", "Cube z (block top height) in map frame"),
    BT::InputPort<double>("grasp_height", "Grasp height appended into goal data"),
    BT::OutputPort<std::vector<double>>("goal_data", "Final goal data sent to ArmTask"),
    BT::OutputPort<int32_t>("err_code", "ArmTask result error code"),
    BT::OutputPort<std::string>("reason", "ArmTask result reason")
  };
}

BT::NodeStatus ArmTaskAction::onStart()
{
  if (!node_) {
    if (!getInput("node", node_)) {
      RCLCPP_ERROR(rclcpp::get_logger("ArmTaskAction"), "Missing required input [node]");
      return BT::NodeStatus::FAILURE;
    }
    if (!node_) {
      RCLCPP_ERROR(rclcpp::get_logger("ArmTaskAction"), "Input [node] is null");
      return BT::NodeStatus::FAILURE;
    }

    // 1) 优先用 simple_bt_runner 注入到黑板上的全局 tf_buffer——
    //    它从程序启动那一刻就开始接 /AT_R2/tf，BT 跑到这里时已经有完整数据，
    //    避免每个 ArmTask 节点 lazy 自建 listener 撞上 discovery 延迟。
    auto bb = config().blackboard;
    if (bb) {
      std::shared_ptr<tf2_ros::Buffer> shared_buf;
      if (bb->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", shared_buf) && shared_buf) {
        tf_buffer_ = shared_buf;
        RCLCPP_INFO(node_->get_logger(),
          "ArmTaskAction: using shared tf_buffer from blackboard");
      }
    }

    // 2) 黑板里没有就回退到自己 lazy 创建（兼容旧 launch / 单元测试）。
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

      tf_node_ = rclcpp::Node::make_shared("arm_task_action_tf", "", tf_node_opts);
      RCLCPP_WARN(node_->get_logger(),
        "ArmTaskAction: blackboard has no tf_buffer, falling back to local listener "
        "(remap /tf -> %s/tf)",
        tf_ns.empty() ? "(global)" : ("/" + tf_ns).c_str());

      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, tf_node_, true);
    }
  }

  int32_t task_id = 0;
  if (!getInput("task_id", task_id)) {
    RCLCPP_ERROR(node_->get_logger(), "Missing required input [task_id]");
    return BT::NodeStatus::FAILURE;
  }

  std::string server_name{"robotic_task"};
  (void)getInput("server_name", server_name);

  std::string base_frame{"base_link"};
  (void)getInput("base_frame", base_frame);

  std::string target_frame{"target_object"};
  (void)getInput("target_frame", target_frame);

  std::string map_frame{"map"};
  (void)getInput("map_frame", map_frame);

  double tf_timeout = 1.0;
  (void)getInput("tf_timeout", tf_timeout);
  if (tf_timeout < 0.0) {
    RCLCPP_ERROR(node_->get_logger(), "Input [tf_timeout] must be >= 0.0, got %.3f", tf_timeout);
    return BT::NodeStatus::FAILURE;
  }

  double grasp_height = 0.0;
  if (!getInput("grasp_height", grasp_height)) {
    RCLCPP_ERROR(node_->get_logger(), "Missing required input [grasp_height]");
    return BT::NodeStatus::FAILURE;
  }

  double cube_x = 0.0;
  double cube_y = 0.0;
  double block_height = 0.0;
  (void)getInput("cube_x", cube_x);
  (void)getInput("cube_y", cube_y);
  (void)getInput("block_height", block_height);

  action_client_ = rclcpp_action::create_client<ArmTask>(node_, server_name);

  if (!action_client_->wait_for_action_server(2s)) {
    RCLCPP_ERROR(node_->get_logger(), "Action server [%s] not available", server_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  std::vector<double> goal_data;
  if (!loadGoalDataFromTfOrFallback(
      base_frame, target_frame, map_frame, tf_timeout, grasp_height,
      cube_x, cube_y, block_height, goal_data))
  {
    return BT::NodeStatus::FAILURE;
  }

  ArmTask::Goal goal_msg;
  goal_msg.task_id = task_id;
  goal_msg.data = goal_data;
  setOutput("goal_data", goal_data);
  // 打印发送给 ArmTask action server 的完整请求
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
    "ArmTask request: server=%s task_id=%d data_size=%zu data=%s",
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
    std::bind(&ArmTaskAction::onGoalResponse, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(&ArmTaskAction::onFeedback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&ArmTaskAction::onResult, this, std::placeholders::_1);

  action_client_->async_send_goal(goal_msg, send_goal_options);
  RCLCPP_INFO(
    node_->get_logger(),
    "ArmTask goal sent: task_id=%d, data_size=%zu",
    task_id, goal_data.size());

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ArmTaskAction::onRunning()
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

void ArmTaskAction::onHalted()
{
  if (action_client_ && goal_handle_) {
    (void)action_client_->async_cancel_goal(goal_handle_);
  }
  goal_handle_.reset();
  goal_rejected_ = false;
  result_received_ = false;
  final_status_ = BT::NodeStatus::RUNNING;
}

void ArmTaskAction::onGoalResponse(const GoalHandleArmTask::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    goal_rejected_ = true;
    RCLCPP_ERROR(node_->get_logger(), "ArmTask goal rejected by server");
    return;
  }
  goal_handle_ = goal_handle;
  RCLCPP_INFO(node_->get_logger(), "ArmTask goal accepted");
}

void ArmTaskAction::onFeedback(
  GoalHandleArmTask::SharedPtr,
  const std::shared_ptr<const ArmTask::Feedback> feedback)
{
  RCLCPP_INFO(node_->get_logger(), "ArmTask feedback: %s", feedback->describe.c_str());
}

void ArmTaskAction::onResult(const GoalHandleArmTask::WrappedResult & result)
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
    "ArmTask finished: code=%d, err_code=%d, reason=%s",
    static_cast<int>(result.code), result_err_code_, result_reason_.c_str());
}

bool ArmTaskAction::loadGoalDataFromTfOrFallback(
  const std::string & base_frame,
  const std::string & target_frame,
  const std::string & map_frame,
  double tf_timeout_sec,
  double grasp_height,
  double cube_x,
  double cube_y,
  double block_height,
  std::vector<double> & out_goal_data)
{
  (void)target_frame;

  // 仅用 map → base：lookupTransform(base, map) 将 map 系点变到 base，配合黑板上的 map 坐标。
  try {
    const tf2::Duration tf_timeout = tf2::durationFromSec(tf_timeout_sec);
    std::string tf_err;
    if (!tf_buffer_->canTransform(
        base_frame, map_frame, tf2::TimePointZero, tf_timeout, &tf_err))
    {
      RCLCPP_ERROR(node_->get_logger(),
        "TF unavailable: transform from %s to %s | tf2 reason: %s",
        map_frame.c_str(), base_frame.c_str(),
        tf_err.empty() ? "(no detail)" : tf_err.c_str());

      // 逐段探测，指出到底是链条里哪一环断了（默认经 odom 中转）。
      const char * odom_frame = "odom";
      std::string seg_err;
      const bool map_to_odom = tf_buffer_->canTransform(
        odom_frame, map_frame, tf2::TimePointZero,
        tf2::durationFromSec(0.0), &seg_err);
      RCLCPP_ERROR(node_->get_logger(),
        "  段 [%s -> %s]: %s%s%s",
        map_frame.c_str(), odom_frame,
        map_to_odom ? "OK" : "断",
        map_to_odom ? "" : " | ",
        map_to_odom ? "" : seg_err.c_str());

      seg_err.clear();
      const bool odom_to_base = tf_buffer_->canTransform(
        base_frame, odom_frame, tf2::TimePointZero,
        tf2::durationFromSec(0.0), &seg_err);
      RCLCPP_ERROR(node_->get_logger(),
        "  段 [%s -> %s]: %s%s%s",
        odom_frame, base_frame.c_str(),
        odom_to_base ? "OK" : "断",
        odom_to_base ? "" : " | ",
        odom_to_base ? "" : seg_err.c_str());

      return false;
    }
    const auto tf_mb = tf_buffer_->lookupTransform(
      base_frame, map_frame, tf2::TimePointZero);

    geometry_msgs::msg::PoseStamped cube_in_map;
    cube_in_map.header.frame_id = map_frame;
    cube_in_map.pose.position.x = cube_x;
    cube_in_map.pose.position.y = cube_y;
    cube_in_map.pose.position.z = block_height;
    cube_in_map.pose.orientation.w = 1.0;

    geometry_msgs::msg::PoseStamped cube_in_base;
    tf2::doTransform(cube_in_map, cube_in_base, tf_mb);

    out_goal_data = {
      cube_in_base.pose.position.x,
      cube_in_base.pose.position.y,
      cube_in_base.pose.position.z,
      cube_in_base.pose.orientation.x,
      cube_in_base.pose.orientation.y,
      cube_in_base.pose.orientation.z,
      cube_in_base.pose.orientation.w
    };
    RCLCPP_INFO(
      node_->get_logger(),
      "Cube pose from map+TF: map(%.3f, %.3f, %.3f) -> %s(%.3f, %.3f, %.3f)",
      cube_x, cube_y, block_height,
      base_frame.c_str(),
      cube_in_base.pose.position.x,
      cube_in_base.pose.position.y,
      cube_in_base.pose.position.z);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "TF transform failed: %s", e.what());
    return false;
  }

  out_goal_data.push_back(grasp_height);
  return true;
}

}  // namespace nav2_bt_publish_goal

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_bt_publish_goal::ArmTaskAction>("ArmTask");
}
