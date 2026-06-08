// Copyright 2024
// Licensed under the Apache License, Version 2.0

#include "nav2_bt_publish_goal/set_mppi_params_action.hpp"

#include <chrono>

namespace nav2_bt_publish_goal
{

SetMppiParamsAction::SetMppiParamsAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config),
  start_time_(0, 0, RCL_ROS_TIME),
  wait_after_set_(rclcpp::Duration::from_seconds(0.0))
{
}

BT::PortsList SetMppiParamsAction::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "ROS node"),
    BT::InputPort<std::string>(
      "controller_node", "/AT_R2/controller_server",
      "Fully-qualified controller_server node name"),
    BT::InputPort<std::string>(
      "plugin_prefix", "FollowPath",
      "MPPI plugin instance prefix (YAML key under controller_server)"),
    BT::InputPort<int>("time_steps", "MPPI time_steps"),
    BT::InputPort<double>("model_dt", "MPPI model_dt"),
    BT::InputPort<int>("batch_size", "MPPI batch_size"),
    BT::InputPort<double>("vx_std", "MPPI vx_std"),
    BT::InputPort<double>("vy_std", "MPPI vy_std"),
    BT::InputPort<double>("wz_std", "MPPI wz_std"),
    BT::InputPort<double>("vx_max", "MPPI vx_max"),
    BT::InputPort<double>("vx_min", "MPPI vx_min"),
    BT::InputPort<double>("vy_max", "MPPI vy_max"),
    BT::InputPort<double>("wz_max", "MPPI wz_max"),
    BT::InputPort<double>("ax_max", "MPPI ax_max"),
    BT::InputPort<double>("ax_min", "MPPI ax_min"),
    BT::InputPort<double>("ay_max", "MPPI ay_max"),
    BT::InputPort<double>("ay_min", "MPPI ay_min"),
    BT::InputPort<double>("az_max", "MPPI az_max"),
    BT::InputPort<int>("iteration_count", "MPPI iteration_count"),
    BT::InputPort<double>("temperature", "MPPI temperature"),
    BT::InputPort<double>("gamma", "MPPI gamma"),
    BT::InputPort<double>("reset_period", "MPPI reset_period"),
    BT::InputPort<std::string>("motion_model", "MPPI motion_model (e.g. Omni, DiffDrive)"),
    BT::InputPort<bool>("regenerate_noises", "MPPI regenerate_noises"),
    // Critic tuning ports (mapped to plugin_prefix.<Critic>.<key>)
    BT::InputPort<int>("constraint_cost_power", "ConstraintCritic.cost_power"),
    BT::InputPort<double>("constraint_weight", "ConstraintCritic.cost_weight"),
    BT::InputPort<int>("goal_cost_power", "GoalCritic.cost_power"),
    BT::InputPort<double>("goal_weight", "GoalCritic.cost_weight"),
    BT::InputPort<double>("goal_threshold", "GoalCritic.threshold_to_consider"),
    BT::InputPort<int>("goal_angle_cost_power", "GoalAngleCritic.cost_power"),
    BT::InputPort<double>("goal_angle_weight", "GoalAngleCritic.cost_weight"),
    BT::InputPort<double>("goal_angle_threshold", "GoalAngleCritic.threshold_to_consider"),
    BT::InputPort<int>("costcritic_cost_power", "CostCritic.cost_power"),
    BT::InputPort<double>("costcritic_weight", "CostCritic.cost_weight"),
    BT::InputPort<double>("costcritic_critical_cost", "CostCritic.critical_cost"),
    BT::InputPort<bool>("costcritic_consider_footprint", "CostCritic.consider_footprint"),
    BT::InputPort<double>("costcritic_collision_cost", "CostCritic.collision_cost"),
    BT::InputPort<double>("costcritic_near_goal_distance", "CostCritic.near_goal_distance"),
    BT::InputPort<int>("costcritic_trajectory_point_step", "CostCritic.trajectory_point_step"),
    BT::InputPort<int>("pathalign_cost_power", "PathAlignCritic.cost_power"),
    BT::InputPort<double>("pathalign_weight", "PathAlignCritic.cost_weight"),
    BT::InputPort<double>("pathalign_max_path_occupancy_ratio", "PathAlignCritic.max_path_occupancy_ratio"),
    BT::InputPort<int>("pathalign_trajectory_point_step", "PathAlignCritic.trajectory_point_step"),
    BT::InputPort<double>("pathalign_threshold", "PathAlignCritic.threshold_to_consider"),
    BT::InputPort<int>("pathalign_offset_from_furthest", "PathAlignCritic.offset_from_furthest"),
    BT::InputPort<bool>("pathalign_use_path_orientations", "PathAlignCritic.use_path_orientations"),
    BT::InputPort<int>("pathfollow_cost_power", "PathFollowCritic.cost_power"),
    BT::InputPort<double>("pathfollow_weight", "PathFollowCritic.cost_weight"),
    BT::InputPort<int>("pathfollow_offset_from_furthest", "PathFollowCritic.offset_from_furthest"),
    BT::InputPort<double>("pathfollow_threshold", "PathFollowCritic.threshold_to_consider"),
    BT::InputPort<bool>("pathangle_enabled", "PathAngleCritic.enabled"),
    BT::InputPort<int>("pathangle_cost_power", "PathAngleCritic.cost_power"),
    BT::InputPort<double>("pathangle_weight", "PathAngleCritic.cost_weight"),
    BT::InputPort<int>("pathangle_offset_from_furthest", "PathAngleCritic.offset_from_furthest"),
    BT::InputPort<double>("pathangle_threshold", "PathAngleCritic.threshold_to_consider"),
    BT::InputPort<double>("pathangle_max_angle_to_furthest", "PathAngleCritic.max_angle_to_furthest"),
    BT::InputPort<int>("pathangle_mode", "PathAngleCritic.mode"),
    BT::InputPort<bool>("preferforward_enabled", "PreferForwardCritic.enabled"),
    BT::InputPort<int>("preferforward_cost_power", "PreferForwardCritic.cost_power"),
    BT::InputPort<double>("preferforward_weight", "PreferForwardCritic.cost_weight"),
    BT::InputPort<double>("preferforward_threshold", "PreferForwardCritic.threshold_to_consider"),
    BT::InputPort<bool>("twirling_enabled", "TwirlingCritic.enabled"),
    BT::InputPort<int>("twirling_cost_power", "TwirlingCritic.twirling_cost_power"),
    BT::InputPort<double>("twirling_weight", "TwirlingCritic.twirling_cost_weight"),
    // Goal checker ports (mapped directly to general_goal_checker.<key>, no plugin prefix)
    BT::InputPort<double>("goal_x_tolerance", "PreciseGoalChecker.x_goal_tolerance"),
    BT::InputPort<double>("goal_y_tolerance", "PreciseGoalChecker.y_goal_tolerance"),
    BT::InputPort<double>("goal_yaw_tolerance", "PreciseGoalChecker.yaw_goal_tolerance"),
    BT::InputPort<double>("goal_stable_duration", "PreciseGoalChecker.stable_duration"),
    BT::InputPort<double>(
      "wait_after_set", 0.3, "Seconds to wait after set_parameters before SUCCESS"),
    BT::InputPort<double>("service_timeout", 2.0, "Seconds to wait for set_parameters service")
  };
}

void SetMppiParamsAction::buildParams(Target & target, const std::string & prefix)
{
  target.params.clear();

  auto pname = [&prefix](const char * key) {
      return prefix + "." + key;
    };

  auto add_i = [this, &target, &pname](const char * port, const char * key) {
      int v = 0;
      if (getInput(port, v)) {
        target.params.emplace_back(pname(key), v);
      }
    };

  auto add_d = [this, &target, &pname](const char * port, const char * key) {
      double v = 0.0;
      if (getInput(port, v)) {
        target.params.emplace_back(pname(key), v);
      }
    };

  auto add_s = [this, &target, &pname](const char * port, const char * key) {
      std::string v;
      if (getInput(port, v) && !v.empty()) {
        target.params.emplace_back(pname(key), v);
      }
    };

  auto add_b = [this, &target, &pname](const char * port, const char * key) {
      bool v = false;
      if (getInput(port, v)) {
        target.params.emplace_back(pname(key), v);
      }
    };

  auto add_raw_d = [this, &target](const char * port, const char * key) {
      double v = 0.0;
      if (getInput(port, v)) {
        target.params.emplace_back(key, v);
      }
    };

  add_i("time_steps", "time_steps");
  add_d("model_dt", "model_dt");
  add_i("batch_size", "batch_size");
  add_d("vx_std", "vx_std");
  add_d("vy_std", "vy_std");
  add_d("wz_std", "wz_std");
  add_d("vx_max", "vx_max");
  add_d("vx_min", "vx_min");
  add_d("vy_max", "vy_max");
  add_d("wz_max", "wz_max");
  add_d("ax_max", "ax_max");
  add_d("ax_min", "ax_min");
  add_d("ay_max", "ay_max");
  add_d("ay_min", "ay_min");
  add_d("az_max", "az_max");
  add_i("iteration_count", "iteration_count");
  add_d("temperature", "temperature");
  add_d("gamma", "gamma");
  add_d("reset_period", "reset_period");
  add_s("motion_model", "motion_model");
  add_b("regenerate_noises", "regenerate_noises");
  add_i("constraint_cost_power", "ConstraintCritic.cost_power");
  add_d("constraint_weight", "ConstraintCritic.cost_weight");
  add_i("goal_cost_power", "GoalCritic.cost_power");
  add_d("goal_weight", "GoalCritic.cost_weight");
  add_d("goal_threshold", "GoalCritic.threshold_to_consider");
  add_i("goal_angle_cost_power", "GoalAngleCritic.cost_power");
  add_d("goal_angle_weight", "GoalAngleCritic.cost_weight");
  add_d("goal_angle_threshold", "GoalAngleCritic.threshold_to_consider");
  add_i("costcritic_cost_power", "CostCritic.cost_power");
  add_d("costcritic_weight", "CostCritic.cost_weight");
  add_d("costcritic_critical_cost", "CostCritic.critical_cost");
  add_b("costcritic_consider_footprint", "CostCritic.consider_footprint");
  add_d("costcritic_collision_cost", "CostCritic.collision_cost");
  add_d("costcritic_near_goal_distance", "CostCritic.near_goal_distance");
  add_i("costcritic_trajectory_point_step", "CostCritic.trajectory_point_step");
  add_i("pathalign_cost_power", "PathAlignCritic.cost_power");
  add_d("pathalign_weight", "PathAlignCritic.cost_weight");
  add_d("pathalign_max_path_occupancy_ratio", "PathAlignCritic.max_path_occupancy_ratio");
  add_i("pathalign_trajectory_point_step", "PathAlignCritic.trajectory_point_step");
  add_d("pathalign_threshold", "PathAlignCritic.threshold_to_consider");
  add_i("pathalign_offset_from_furthest", "PathAlignCritic.offset_from_furthest");
  add_b("pathalign_use_path_orientations", "PathAlignCritic.use_path_orientations");
  add_i("pathfollow_cost_power", "PathFollowCritic.cost_power");
  add_d("pathfollow_weight", "PathFollowCritic.cost_weight");
  add_i("pathfollow_offset_from_furthest", "PathFollowCritic.offset_from_furthest");
  add_d("pathfollow_threshold", "PathFollowCritic.threshold_to_consider");
  add_b("pathangle_enabled", "PathAngleCritic.enabled");
  add_i("pathangle_cost_power", "PathAngleCritic.cost_power");
  add_d("pathangle_weight", "PathAngleCritic.cost_weight");
  add_i("pathangle_offset_from_furthest", "PathAngleCritic.offset_from_furthest");
  add_d("pathangle_threshold", "PathAngleCritic.threshold_to_consider");
  add_d("pathangle_max_angle_to_furthest", "PathAngleCritic.max_angle_to_furthest");
  add_i("pathangle_mode", "PathAngleCritic.mode");
  add_b("preferforward_enabled", "PreferForwardCritic.enabled");
  add_i("preferforward_cost_power", "PreferForwardCritic.cost_power");
  add_d("preferforward_weight", "PreferForwardCritic.cost_weight");
  add_d("preferforward_threshold", "PreferForwardCritic.threshold_to_consider");
  add_b("twirling_enabled", "TwirlingCritic.enabled");
  add_i("twirling_cost_power", "TwirlingCritic.twirling_cost_power");
  add_d("twirling_weight", "TwirlingCritic.twirling_cost_weight");

  add_raw_d("goal_x_tolerance", "general_goal_checker.x_goal_tolerance");
  add_raw_d("goal_y_tolerance", "general_goal_checker.y_goal_tolerance");
  add_raw_d("goal_yaw_tolerance", "general_goal_checker.yaw_goal_tolerance");
  add_raw_d("goal_stable_duration", "general_goal_checker.stable_duration");

  target.valid = !target.params.empty();
}

void SetMppiParamsAction::sendIfReady(Target & target, double service_timeout_s)
{
  if (!target.valid) {
    target.finished = true;
    return;
  }

  const auto timeout = std::chrono::milliseconds(
    static_cast<int>(service_timeout_s * 1000));
  if (!target.client->service_is_ready()) {
    if (!target.client->wait_for_service(timeout)) {
      RCLCPP_WARN(
        node_->get_logger(),
        "SetMppiParams: set_parameters service of '%s' not available within %.1fs, skipping",
        target.node_name.c_str(), service_timeout_s);
      target.valid = false;
      target.finished = true;
      return;
    }
  }

  target.future = target.client->set_parameters(target.params);
  target.finished = false;
}

void SetMppiParamsAction::evaluate(Target & target)
{
  if (target.finished || !target.valid) {
    return;
  }

  if (target.future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
    return;
  }

  try {
    auto results = target.future.get();
    for (size_t i = 0; i < results.size() && i < target.params.size(); ++i) {
      if (!results[i].successful) {
        RCLCPP_WARN(
          node_->get_logger(),
          "SetMppiParams: failed to set '%s' on '%s': %s",
          target.params[i].get_name().c_str(),
          target.node_name.c_str(),
          results[i].reason.c_str());
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      node_->get_logger(),
      "SetMppiParams: exception while setting params on '%s': %s",
      target.node_name.c_str(), e.what());
  }

  target.finished = true;
}

BT::NodeStatus SetMppiParamsAction::onStart()
{
  if (!node_) {
    if (!getInput("node", node_)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("SetMppiParamsAction"),
        "Missing required input [node]");
      return BT::NodeStatus::FAILURE;
    }
    if (!node_) {
      RCLCPP_ERROR(
        rclcpp::get_logger("SetMppiParamsAction"),
        "Node pointer is null");
      return BT::NodeStatus::FAILURE;
    }
  }

  std::string controller_node_name = "/AT_R2/controller_server";
  getInput("controller_node", controller_node_name);

  std::string plugin_prefix = "FollowPath";
  getInput("plugin_prefix", plugin_prefix);

  if (!controller_.client || controller_.node_name != controller_node_name) {
    controller_.client = std::make_shared<rclcpp::AsyncParametersClient>(
      node_, controller_node_name);
    controller_.node_name = controller_node_name;
  }

  buildParams(controller_, plugin_prefix);

  if (!controller_.valid) {
    RCLCPP_WARN(
      node_->get_logger(),
      "SetMppiParams: no MPPI parameters provided in any port, nothing to do");
    return BT::NodeStatus::SUCCESS;
  }

  double service_timeout_s = 2.0;
  getInput("service_timeout", service_timeout_s);

  sendIfReady(controller_, service_timeout_s);

  double wait_after_set_s = 0.3;
  getInput("wait_after_set", wait_after_set_s);
  wait_after_set_ = rclcpp::Duration::from_seconds(wait_after_set_s);
  start_time_ = node_->now();

  RCLCPP_INFO(
    node_->get_logger(),
    "SetMppiParams: requested %zu params on '%s' (prefix '%s'), waiting %.2fs after set",
    controller_.params.size(),
    controller_node_name.c_str(),
    plugin_prefix.c_str(),
    wait_after_set_s);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SetMppiParamsAction::onRunning()
{
  evaluate(controller_);

  if (!controller_.finished) {
    return BT::NodeStatus::RUNNING;
  }

  if ((node_->now() - start_time_) < wait_after_set_) {
    return BT::NodeStatus::RUNNING;
  }

  RCLCPP_INFO(node_->get_logger(), "SetMppiParams: done");
  return BT::NodeStatus::SUCCESS;
}

void SetMppiParamsAction::onHalted()
{
  controller_.finished = true;
}

}  // namespace nav2_bt_publish_goal

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_bt_publish_goal::SetMppiParamsAction>("SetMppiParams");
}
