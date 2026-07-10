// Copyright 2024
// Licensed under the Apache License, Version 2.0

#include "nav2_bt_publish_goal/wait_slot_select_action.hpp"

#include <cerrno>
#include <cstring>
#include <iostream>

#include <sys/select.h>
#include <unistd.h>

#include "behaviortree_cpp/bt_factory.h"

namespace nav2_bt_publish_goal
{

WaitSlotSelectAction::WaitSlotSelectAction(
  const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
}

BT::PortsList WaitSlotSelectAction::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "ROS node"),
    BT::InputPort<std::string>("topic", "/AT_R2/place_slot_select",
      "放格子选择话题 (std_msgs/Int32): 1/2/3=放 slot_1/2/3, 4=大胜, 5=去等待位"),
    BT::InputPort<std::string>("prompt", "等待放格子选择 (话题 1/2/3=格子, 4=大胜, 5=等待位)",
      "等待时的提示语"),
    BT::InputPort<bool>("enable_enter", true,
      "允许操作员按 Enter 手动放行 (调试用, 视作收到 enter_value)"),
    BT::InputPort<int32_t>("enter_value", 1,
      "按 Enter 时视作收到的值"),
    BT::InputPort<double>("timeout", 0.0,
      "超时秒数; <=0 一直等"),
    BT::InputPort<bool>("clear_buffer", true,
      "显示提示前清空 stdin 残留"),
    BT::OutputPort<int>("slot_sel",
      "收到的选择值 (1/2/3=格子编号, 4=大胜, 5=等待位)"),
  };
}

bool WaitSlotSelectAction::copyDouble(
  const std::string & prefix, const std::string & suffix)
{
  const std::string src = prefix + suffix;
  auto entry = config().blackboard->getEntry(src);
  if (!entry) {
    return false;
  }
  double value = 0.0;
  if (!config().blackboard->get<double>(src, value)) {
    return false;
  }
  config().blackboard->set<double>("slot_" + suffix, value);
  return true;
}

bool WaitSlotSelectAction::selectSlotParams(int v)
{
  // 值直接当 slot 编号: slot_<v>_x/y/yaw -> slot_x/y/yaw
  const std::string prefix = "slot_" + std::to_string(v) + "_";
  for (const auto & suffix : {"x", "y", "yaw"}) {
    if (!copyDouble(prefix, suffix)) {
      RCLCPP_ERROR(node_->get_logger(),
        "WaitSlotSelect: 找不到黑板键 [%s%s] (slot 编号 %d 越界或未加载)",
        prefix.c_str(), suffix, v);
      return false;
    }
  }
  RCLCPP_INFO(node_->get_logger(),
    "WaitSlotSelect: 选定格子 %d (前缀 %s) -> slot_x/y/yaw", v, prefix.c_str());
  return true;
}

BT::NodeStatus WaitSlotSelectAction::onStart()
{
  if (!node_) {
    if (!getInput("node", node_) || !node_) {
      RCLCPP_ERROR(rclcpp::get_logger("WaitSlotSelect"), "Missing/null required input [node]");
      return BT::NodeStatus::FAILURE;
    }
  }

  topic_ = "/AT_R2/place_slot_select";
  prompt_ = "等待放格子选择 (话题 1/2/3=格子, 4=大胜, 5=等待位)";
  enable_enter_ = true;
  enter_value_ = 1;
  timeout_ = 0.0;
  clear_buffer_ = true;

  getInput("topic", topic_);
  getInput("prompt", prompt_);
  getInput("enable_enter", enable_enter_);
  getInput("enter_value", enter_value_);
  getInput("timeout", timeout_);
  getInput("clear_buffer", clear_buffer_);

  received_ = false;
  received_value_ = 0;

  sub_ = node_->create_subscription<std_msgs::msg::Int32>(
    topic_, rclcpp::QoS(10).reliable(),
    std::bind(&WaitSlotSelectAction::slotSelectCallback, this, std::placeholders::_1));

  // Enter 一路: 仅在交互终端启用
  enter_enabled_ = enable_enter_ && isatty(STDIN_FILENO);
  if (enter_enabled_ && clear_buffer_) {
    drainPendingInput();
  }

  start_time_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(node_->get_logger(), "%s (话题 %s)", prompt_.c_str(), topic_.c_str());
  std::cout << "\n[WaitSlotSelect] " << prompt_ << std::endl;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitSlotSelectAction::onRunning()
{
  int selected = 0;

  // 分支A: 话题
  if (received_.load()) {
    selected = received_value_.load();
  } else if (enter_enabled_) {
    // 分支B: 操作员按 Enter (调试)
    bool got_enter = false;
    if (!readPendingInput(got_enter, false)) {
      RCLCPP_WARN(node_->get_logger(),
        "WaitSlotSelect: stdin 读取异常, 关闭 Enter 一路, 仅等待话题");
      enter_enabled_ = false;
    } else if (got_enter) {
      selected = enter_value_;
      RCLCPP_INFO(node_->get_logger(), "WaitSlotSelect: 按下 Enter, 视作选择 %d", selected);
    }
  }

  if (selected != 0) {
    sub_.reset();
    setOutput<int>("slot_sel", selected);

    // 1/2/3 选格子参数; 4=大胜不选; 其它值当非法
    if (selected >= 1 && selected <= 3) {
      if (!selectSlotParams(selected)) {
        return BT::NodeStatus::FAILURE;  // slot 参数缺失
      }
    } else if (selected == 4) {
      RCLCPP_INFO(node_->get_logger(), "WaitSlotSelect: 收到 4 (大胜)");
    } else if (selected == 5) {
      RCLCPP_INFO(node_->get_logger(), "WaitSlotSelect: 收到 5 (去等待位, 不放块)");
    } else {
      RCLCPP_ERROR(node_->get_logger(),
        "WaitSlotSelect: 非法选择值 %d (仅支持 1/2/3/4/5)", selected);
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }

  if (timeout_ > 0.0) {
    const auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start_time_).count();
    if (elapsed >= timeout_) {
      RCLCPP_WARN(node_->get_logger(), "WaitSlotSelect timed out after %.2fs", timeout_);
      sub_.reset();
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void WaitSlotSelectAction::onHalted()
{
  sub_.reset();
  if (node_) {
    RCLCPP_INFO(node_->get_logger(), "WaitSlotSelect halted");
  }
}

void WaitSlotSelectAction::slotSelectCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  const int v = msg->data;
  // 只接受有效选择 1/2/3/4/5, 其它忽略继续等
  if (v >= 1 && v <= 5) {
    received_value_ = v;
    received_ = true;
  }
}

bool WaitSlotSelectAction::stdinReady() const
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

bool WaitSlotSelectAction::readPendingInput(bool & got_enter, bool fail_on_eof)
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
      return !fail_on_eof;
    }

    if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
      continue;
    }

    if (node_) {
      RCLCPP_ERROR(node_->get_logger(), "WaitSlotSelect read failed: %s", std::strerror(errno));
    }
    return false;
  }

  return true;
}

void WaitSlotSelectAction::drainPendingInput()
{
  bool ignored_enter = false;
  (void)readPendingInput(ignored_enter, false);
}

}  // namespace nav2_bt_publish_goal

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_bt_publish_goal::WaitSlotSelectAction>("WaitSlotSelect");
}
