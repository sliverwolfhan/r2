// Copyright 2024
// Licensed under the Apache License, Version 2.0

#include "nav2_bt_publish_goal/wait_for_enter_action.hpp"

#include <cerrno>
#include <cstring>
#include <iostream>

#include <sys/select.h>
#include <unistd.h>

#include "behaviortree_cpp/bt_factory.h"

namespace nav2_bt_publish_goal
{

WaitForEnterAction::WaitForEnterAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
}

BT::PortsList WaitForEnterAction::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "ROS node"),
    BT::InputPort<std::string>("prompt", "参数已设置完成，按 Enter 开始导航",
      "Prompt shown while waiting for operator Enter"),
    BT::InputPort<double>("timeout", 0.0,
      "Timeout in seconds; <=0 waits forever"),
    BT::InputPort<bool>("require_tty", true,
      "Fail if stdin is not an interactive terminal"),
    BT::InputPort<bool>("clear_buffer", true,
      "Drain stale stdin bytes before showing the prompt")
  };
}

BT::NodeStatus WaitForEnterAction::onStart()
{
  if (!node_) {
    if (!getInput("node", node_) || !node_) {
      RCLCPP_ERROR(rclcpp::get_logger("WaitForEnter"), "Missing/null required input [node]");
      return BT::NodeStatus::FAILURE;
    }
  }

  prompt_ = "参数已设置完成，按 Enter 开始导航";
  timeout_ = 0.0;
  require_tty_ = true;
  clear_buffer_ = true;

  getInput("prompt", prompt_);
  getInput("timeout", timeout_);
  getInput("require_tty", require_tty_);
  getInput("clear_buffer", clear_buffer_);

  if (!isatty(STDIN_FILENO)) {
    if (require_tty_) {
      RCLCPP_ERROR(node_->get_logger(),
        "WaitForEnter requires an interactive terminal, but stdin is not a TTY");
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_WARN(node_->get_logger(),
      "WaitForEnter: stdin is not a TTY (require_tty=false), 跳过 Enter 门控");
    return BT::NodeStatus::SUCCESS;
  }

  if (clear_buffer_) {
    drainPendingInput();
  }

  start_time_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(node_->get_logger(), "%s", prompt_.c_str());
  std::cout << "\n[WaitForEnter] " << prompt_ << std::endl;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForEnterAction::onRunning()
{
  bool got_enter = false;
  if (!readPendingInput(got_enter, true)) {
    return BT::NodeStatus::FAILURE;
  }

  if (got_enter) {
    RCLCPP_INFO(node_->get_logger(), "WaitForEnter: Enter pressed, continue behavior tree");
    return BT::NodeStatus::SUCCESS;
  }

  if (timeout_ > 0.0) {
    const auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start_time_).count();
    if (elapsed >= timeout_) {
      RCLCPP_WARN(node_->get_logger(), "WaitForEnter timed out after %.2fs", timeout_);
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void WaitForEnterAction::onHalted()
{
  if (node_) {
    RCLCPP_INFO(node_->get_logger(), "WaitForEnter halted");
  }
}

bool WaitForEnterAction::stdinReady() const
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

bool WaitForEnterAction::readPendingInput(bool & got_enter, bool fail_on_eof)
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
        RCLCPP_ERROR(node_->get_logger(), "WaitForEnter stdin reached EOF");
      }
      return !fail_on_eof;
    }

    if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
      continue;
    }

    if (node_) {
      RCLCPP_ERROR(node_->get_logger(), "WaitForEnter read failed: %s", std::strerror(errno));
    }
    return false;
  }

  return true;
}

void WaitForEnterAction::drainPendingInput()
{
  bool ignored_enter = false;
  (void)readPendingInput(ignored_enter, false);
}

}  // namespace nav2_bt_publish_goal

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_bt_publish_goal::WaitForEnterAction>("WaitForEnter");
}
