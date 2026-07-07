// Copyright 2024
// Licensed under the Apache License, Version 2.0

#include "nav2_bt_publish_goal/wait_for_start_action.hpp"

#include <cerrno>
#include <cstring>
#include <iostream>

#include <sys/select.h>
#include <unistd.h>

#include "behaviortree_cpp/bt_factory.h"

namespace nav2_bt_publish_goal
{

WaitForStartAction::WaitForStartAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
}

BT::PortsList WaitForStartAction::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "ROS node"),
    BT::InputPort<std::string>("prompt", "参数已设置完成，按 Enter 或等待启动话题开始",
      "Prompt shown while waiting for the start signal"),
    BT::InputPort<std::string>("start_topic", "/AT_R2/start_cmd",
      "Topic subscribed for the start command (std_msgs/Int32)"),
    BT::InputPort<int32_t>("start_value", 1,
      "start_topic value that triggers start"),
    BT::InputPort<bool>("enable_topic", true,
      "Subscribe to start_topic; false disables the topic branch (Enter only)"),
    BT::InputPort<double>("timeout", 0.0,
      "Timeout in seconds; <=0 waits forever"),
    BT::InputPort<bool>("clear_buffer", true,
      "Drain stale stdin bytes before showing the prompt")
  };
}

BT::NodeStatus WaitForStartAction::onStart()
{
  if (!node_) {
    if (!getInput("node", node_) || !node_) {
      RCLCPP_ERROR(rclcpp::get_logger("WaitForStart"), "Missing/null required input [node]");
      return BT::NodeStatus::FAILURE;
    }
  }

  prompt_ = "参数已设置完成，按 Enter 或等待启动话题开始";
  start_topic_ = "/AT_R2/start_cmd";
  start_value_ = 1;
  timeout_ = 0.0;
  clear_buffer_ = true;
  enable_topic_ = true;

  getInput("prompt", prompt_);
  getInput("start_topic", start_topic_);
  getInput("start_value", start_value_);
  getInput("enable_topic", enable_topic_);
  getInput("timeout", timeout_);
  getInput("clear_buffer", clear_buffer_);

  // 话题一路: 订阅启动命令, 收到 start_value 即放行。
  // enable_topic=false 时不订阅, 退化为纯 Enter 门控(等同 WaitForEnter)。
  start_received_ = false;
  if (enable_topic_) {
    start_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
      start_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&WaitForStartAction::startCmdCallback, this, std::placeholders::_1));
  } else {
    RCLCPP_INFO(node_->get_logger(), "WaitForStart: enable_topic=false, 关闭话题启动, 仅等待 Enter");
  }

  // Enter 一路: 仅在 stdin 是交互终端时启用。无 TTY(如 ros2 launch)时
  // 只保留话题一路, 不因此失败。
  enter_enabled_ = isatty(STDIN_FILENO);
  if (enter_enabled_) {
    if (clear_buffer_) {
      drainPendingInput();
    }
  } else if (enable_topic_) {
    RCLCPP_WARN(node_->get_logger(),
      "WaitForStart: stdin 非交互终端, 仅监听话题 %s == %d", start_topic_.c_str(), start_value_);
  } else {
    RCLCPP_ERROR(node_->get_logger(),
      "WaitForStart: enable_topic=false 且 stdin 非交互终端, 两路启动均不可用, 将一直等待/超时");
  }

  start_time_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(node_->get_logger(), "%s (话题 %s == %d)",
    prompt_.c_str(), start_topic_.c_str(), start_value_);
  std::cout << "\n[WaitForStart] " << prompt_ << std::endl;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForStartAction::onRunning()
{
  // 分支A: 话题启动
  if (start_received_.load()) {
    RCLCPP_INFO(node_->get_logger(),
      "WaitForStart: 收到启动话题 %s == %d, 继续行为树", start_topic_.c_str(), start_value_);
    start_sub_.reset();
    return BT::NodeStatus::SUCCESS;
  }

  // 分支B: 操作员按 Enter
  if (enter_enabled_) {
    bool got_enter = false;
    if (!readPendingInput(got_enter, false)) {
      // stdin 读失败/EOF: 放弃 Enter 一路, 退化为只依赖话题, 不整体失败。
      RCLCPP_WARN(node_->get_logger(),
        "WaitForStart: stdin 读取异常, 关闭 Enter 一路, 仅等待话题");
      enter_enabled_ = false;
    } else if (got_enter) {
      RCLCPP_INFO(node_->get_logger(), "WaitForStart: 按下 Enter, 继续行为树");
      start_sub_.reset();
      return BT::NodeStatus::SUCCESS;
    }
  }

  if (timeout_ > 0.0) {
    const auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start_time_).count();
    if (elapsed >= timeout_) {
      RCLCPP_WARN(node_->get_logger(), "WaitForStart timed out after %.2fs", timeout_);
      start_sub_.reset();
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void WaitForStartAction::onHalted()
{
  start_sub_.reset();
  if (node_) {
    RCLCPP_INFO(node_->get_logger(), "WaitForStart halted");
  }
}

void WaitForStartAction::startCmdCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  if (msg->data == start_value_) {
    start_received_ = true;
  }
}

bool WaitForStartAction::stdinReady() const
{
  fd_set read_fds;
  FD_ZERO(&read_fds);
  FD_SET(STDIN_FILENO, &read_fds);

  timeval timeout;
  timeout.tv_sec = 0;
  timeout.tv_usec = 0;

  const int ret = select(STDIN_FILENO + 1, &read_fds, nullptr, nullptr, &timeout);
  return ret > 0 && FD_ISSET(STDIN_FILENO, &read_fds);
}

bool WaitForStartAction::readPendingInput(bool & got_enter, bool fail_on_eof)
{
  got_enter = false;

  while (stdinReady()) {
    char buffer[64];
    const ssize_t bytes_read = read(STDIN_FILENO, buffer, sizeof(buffer));

    if (bytes_read > 0) {
      for (ssize_t i = 0; i < bytes_read; ++i) {
        if (buffer[i] == '\n' || buffer[i] == '\r') {
          got_enter = true;
        }
      }
      continue;
    }

    if (bytes_read == 0) {
      if (fail_on_eof && node_) {
        RCLCPP_ERROR(node_->get_logger(), "WaitForStart stdin reached EOF");
      }
      return !fail_on_eof;
    }

    if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
      continue;
    }

    if (node_) {
      RCLCPP_ERROR(node_->get_logger(), "WaitForStart read failed: %s", std::strerror(errno));
    }
    return false;
  }

  return true;
}

void WaitForStartAction::drainPendingInput()
{
  bool ignored_enter = false;
  (void)readPendingInput(ignored_enter, false);
}

}  // namespace nav2_bt_publish_goal

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_bt_publish_goal::WaitForStartAction>("WaitForStart");
}
