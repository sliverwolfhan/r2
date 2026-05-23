// Copyright 2026
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// 移动任务测试节点
//
// 本文件实现了一个 ROS2 测试客户端节点，用于测试机械臂的关节空间移动任务功能。
// 该节点直接向动作服务器发送包含运动时长和 6 个关节角的请求，不依赖 TF。
//
// 主要功能：
//   - 读取目标运动时长与 6 个关节角参数
//   - 向 robotic_task 动作服务器发送移动任务请求
//   - 监控任务执行过程并输出反馈信息
//   - 输出最终执行结果（成功/失败/取消）
//
// 使用方法：
//   1. 确保 robotic_task 动作服务器已启动
//   2. 启动本节点：ros2 run arm_task move_kfs_test
//
// 典型用例：
//   - 验证关节空间移动任务功能是否正常工作
//   - 调试移动任务执行流程
//   - 在仿真或实际环境中进行端到端测试

#include <chrono>
#include <array>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_interfaces/action/arm_task.hpp>
#include <robot_interfaces/msg/arm.hpp>

using namespace std::chrono_literals;

namespace {
constexpr int32_t kMoveTaskId = 1;
constexpr size_t kJointCount = 6;
constexpr std::chrono::seconds kJointStateWaitTimeout(5);
}  // namespace

class MoveKfsTestNode : public rclcpp::Node {
public:
    using ArmTask = robot_interfaces::action::ArmTask;
    using GoalHandleArmTask = rclcpp_action::ClientGoalHandle<ArmTask>;

    MoveKfsTestNode()
        : Node("move_kfs_test_node") {
        action_client_ = rclcpp_action::create_client<ArmTask>(this, "robotic_task");
        joint_state_sub_ = this->create_subscription<robot_interfaces::msg::Arm>(
            "myjoints_state", rclcpp::SensorDataQoS(),
            std::bind(&MoveKfsTestNode::on_joint_state, this, std::placeholders::_1));
        startup_timer_ = this->create_wall_timer(500ms, std::bind(&MoveKfsTestNode::run_once, this));
    }

private:
    void run_once() {
        if (request_started_) {
            return;
        }
        if (!action_server_ready_) {
            if (!action_client_->wait_for_action_server(0s)) {
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000, "等待动作服务 robotic_task 就绪...");
                return;
            }

            action_server_ready_ = true;
            joint_wait_start_ = this->now();
            RCLCPP_INFO(this->get_logger(), "动作服务 robotic_task 已就绪，开始等待当前关节状态");
        }

        if (!has_joint_state_) {
            const bool wait_timeout = (this->now() - joint_wait_start_) >= rclcpp::Duration::from_seconds(kJointStateWaitTimeout.count());
            if (!wait_timeout) {
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000, "等待 myjoints_state 当前关节状态...");
                return;
            }

            if (!joint_state_timeout_logged_) {
                RCLCPP_WARN(this->get_logger(), "5秒内未收到 myjoints_state，默认关节角回退为 0.0");
                joint_state_timeout_logged_ = true;
            }
        }

        request_started_ = true;
        startup_timer_->cancel();

        std::array<double, kJointCount> default_joints{};
        if (has_joint_state_) {
            default_joints = current_joint_rads_;
        }

        double move_duration = 3.0;
        double joint_1 = default_joints[0];
        double joint_2 = default_joints[1];
        double joint_3 = default_joints[2];
        double joint_4 = default_joints[3];
        double joint_5 = default_joints[4];
        double joint_6 = default_joints[5];

        if (has_joint_state_) {
            RCLCPP_INFO(
                this->get_logger(),
                "默认关节角使用当前状态: joints=(%.3f, %.3f, %.3f, %.3f, %.3f, %.3f)",
                joint_1, joint_2, joint_3, joint_4, joint_5, joint_6);
        } else {
            RCLCPP_INFO(this->get_logger(), "默认关节角使用回退值: joints=(0.000, 0.000, 0.000, 0.000, 0.000, 0.000)");
        }

        auto read_or_default = [](const std::string& prompt, double default_value, double& output_value) -> bool {
            std::cout << prompt << " (默认 " << default_value << ", 直接回车使用默认): " << std::flush;

            std::string line;
            if (!std::getline(std::cin, line)) {
                return false;
            }

            if (line.empty()) {
                output_value = default_value;
                return true;
            }

            std::istringstream iss(line);
            double parsed_value = 0.0;
            char extra = '\0';
            if (!(iss >> parsed_value) || (iss >> extra)) {
                return false;
            }

            output_value = parsed_value;
            return true;
        };

        if (!read_or_default("请输入关节1角度(弧度)", joint_1, joint_1)) {
            RCLCPP_ERROR(this->get_logger(), "读取关节1角度失败，输入必须是数字或空行");
            rclcpp::shutdown();
            return;
        }

        if (!read_or_default("请输入关节2角度(弧度)", joint_2, joint_2)) {
            RCLCPP_ERROR(this->get_logger(), "读取关节2角度失败，输入必须是数字或空行");
            rclcpp::shutdown();
            return;
        }

        if (!read_or_default("请输入关节3角度(弧度)", joint_3, joint_3)) {
            RCLCPP_ERROR(this->get_logger(), "读取关节3角度失败，输入必须是数字或空行");
            rclcpp::shutdown();
            return;
        }

        if (!read_or_default("请输入关节4角度(弧度)", joint_4, joint_4)) {
            RCLCPP_ERROR(this->get_logger(), "读取关节4角度失败，输入必须是数字或空行");
            rclcpp::shutdown();
            return;
        }

        if (!read_or_default("请输入关节5角度(弧度)", joint_5, joint_5)) {
            RCLCPP_ERROR(this->get_logger(), "读取关节5角度失败，输入必须是数字或空行");
            rclcpp::shutdown();
            return;
        }

        if (!read_or_default("请输入关节6角度(弧度)", joint_6, joint_6)) {
            RCLCPP_ERROR(this->get_logger(), "读取关节6角度失败，输入必须是数字或空行");
            rclcpp::shutdown();
            return;
        }

        if (!read_or_default("请输入移动时长(秒)", 3.0, move_duration)) {
            RCLCPP_ERROR(this->get_logger(), "读取移动时长失败，输入必须是数字或空行");
            rclcpp::shutdown();
            return;
        }

        ArmTask::Goal goal_msg;
        goal_msg.task_id = kMoveTaskId;
        goal_msg.data = {
            joint_1,
            joint_2,
            joint_3,
            joint_4,
            joint_5,
            joint_6,
            move_duration,
        };

        RCLCPP_INFO(
            this->get_logger(),
            "发送移动请求: task_id=%d, duration=%.3f, joints=(%.3f, %.3f, %.3f, %.3f, %.3f, %.3f)",
            goal_msg.task_id,
            goal_msg.data[6],
            goal_msg.data[0],
            goal_msg.data[1],
            goal_msg.data[2],
            goal_msg.data[3],
            goal_msg.data[4],
            goal_msg.data[5]);

        rclcpp_action::Client<ArmTask>::SendGoalOptions send_goal_options;
        send_goal_options.goal_response_callback =
            std::bind(&MoveKfsTestNode::on_goal_response, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&MoveKfsTestNode::on_feedback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&MoveKfsTestNode::on_result, this, std::placeholders::_1);

        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void on_goal_response(const GoalHandleArmTask::SharedPtr& goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "移动目标被服务器拒绝");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "移动目标已被接受，等待执行结果");
    }

    void on_feedback(
        const GoalHandleArmTask::SharedPtr&,
        const std::shared_ptr<const ArmTask::Feedback>& feedback) {
        RCLCPP_INFO(this->get_logger(), "动作反馈: %s", feedback->describe.c_str());
    }

    void on_result(const GoalHandleArmTask::WrappedResult& result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(
                    this->get_logger(), "移动动作成功: err_code=%d, reason=%s",
                    result.result->err_code, result.result->reason.c_str());
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(
                    this->get_logger(), "移动动作失败: err_code=%d, reason=%s",
                    result.result->err_code, result.result->reason.c_str());
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(
                    this->get_logger(), "移动动作被取消: err_code=%d, reason=%s",
                    result.result->err_code, result.result->reason.c_str());
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "移动动作返回了未知结果码");
                break;
        }

        rclcpp::shutdown();
    }

    void on_joint_state(const std::shared_ptr<const robot_interfaces::msg::Arm>& msg) {
        for (size_t i = 0; i < kJointCount; ++i) {
            current_joint_rads_[i] = msg->motor[i].rad;
        }

        if (!has_joint_state_) {
            RCLCPP_INFO(this->get_logger(), "已收到当前关节状态，将作为默认关节角");
        }
        has_joint_state_ = true;
    }

    rclcpp_action::Client<ArmTask>::SharedPtr action_client_;
    rclcpp::Subscription<robot_interfaces::msg::Arm>::SharedPtr joint_state_sub_;
    rclcpp::TimerBase::SharedPtr startup_timer_;
    std::array<double, kJointCount> current_joint_rads_{};
    rclcpp::Time joint_wait_start_;
    bool action_server_ready_{false};
    bool has_joint_state_{false};
    bool joint_state_timeout_logged_{false};
    bool request_started_{false};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveKfsTestNode>();
    rclcpp::spin(node);
    return 0;
}