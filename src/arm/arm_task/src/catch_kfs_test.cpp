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

// 抓取任务测试节点
//
// 本文件实现了一个 ROS2 测试客户端节点，用于测试机械臂的抓取任务功能。
// 该节点通过 TF2 获取目标物体的位姿，并向动作服务器发送抓取请求。
//
// 主要功能：
//   - 通过 TF2 查询 target_object 相对于 base_link 的位姿
//   - 向 robotic_task 动作服务器发送抓取任务请求
//   - 监控任务执行过程并输出反馈信息
//   - 输出最终执行结果（成功/失败/取消）
//
// 使用方法：
//   1. 确保 robotic_task 动作服务器已启动
//   2. 确保 TF 树中存在 target_object -> base_link 的变换
//   3. 启动本节点：ros2 run arm_task catch_kfs_test
//
// 典型用例：
//   - 验证抓取任务功能是否正常工作
//   - 调试抓取任务执行流程
//   - 在仿真或实际环境中进行端到端测试

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_interfaces/action/arm_task.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

namespace {
// 抓取任务的 ID 标识符
constexpr int32_t kCatchTaskId = 2;
}  // namespace

// 抓取任务测试节点类
//
// 该类封装了抓取任务的测试逻辑，负责与动作服务器通信并监控任务执行状态。
// 节点启动后会延迟 500ms 执行，以确保系统初始化完成。
//
// 典型生命周期：
//   构造 -> 延迟启动 -> 查询 TF -> 发送目标 -> 等待结果 -> 退出
class CatchKfsTestNode : public rclcpp::Node {
public:
    using ArmTask = robot_interfaces::action::ArmTask;
    using GoalHandleArmTask = rclcpp_action::ClientGoalHandle<ArmTask>;

    // 构造函数
    //
    // 初始化节点、TF 缓冲区、监听器和动作客户端。
    // 创建一个 500ms 的定时器来延迟执行抓取任务。
    CatchKfsTestNode()
        : Node("catch_kfs_test_node"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_) {
        action_client_ = rclcpp_action::create_client<ArmTask>(this, "robotic_task");
        startup_timer_ = this->create_wall_timer(500ms, std::bind(&CatchKfsTestNode::run_once, this));
        this->declare_parameter("grasp_it", 0.0);
    }

private:
    // 执行一次性测试任务
    //
    // 该方法只会在节点启动后执行一次，完成以下步骤：
    //   1. 检查动作服务器是否可用
    //   2. 等待并获取 target_object 相对于 base_link 的 TF 变换
    //   3. 构造抓取任务目标消息并发送
    //   4. 注册回调函数以监控执行状态
    //
    // 如果任何步骤失败，节点将输出错误日志并关闭。
    void run_once() {
        if (request_started_) {
            return;
        }
        request_started_ = true;
        startup_timer_->cancel();

        if (!action_client_->wait_for_action_server(10s)) {
            RCLCPP_ERROR(this->get_logger(), "等待动作服务 robotic_task 超时");
            rclcpp::shutdown();
            return;
        }

        geometry_msgs::msg::TransformStamped target_tf;
        try {
            if (!tf_buffer_.canTransform("base_link", "target_object", tf2::TimePointZero, 10s)) {
                RCLCPP_ERROR(this->get_logger(), "等待 TF base_link -> target_object 超时");
                rclcpp::shutdown();
                return;
            }
            target_tf = tf_buffer_.lookupTransform("base_link", "target_object", tf2::TimePointZero);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "读取 TF 失败: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        double grasp_it = this->get_parameter("grasp_height").as_double();
        double grasp_height = 0.0;
        if (grasp_it == 0.0) {
            grasp_height = 0.02;  // 低位抓取
        } else if (grasp_it == 1.0) {
            grasp_height = 0.41;  // 中位抓取
        } else {
            grasp_height = 0.68;  // 高位抓取
        }

        ArmTask::Goal goal_msg;
        goal_msg.task_id = kCatchTaskId;
        goal_msg.data = {
            target_tf.transform.translation.x,
            target_tf.transform.translation.y,
            target_tf.transform.translation.z,
            target_tf.transform.rotation.x,
            target_tf.transform.rotation.y,
            target_tf.transform.rotation.z,
            target_tf.transform.rotation.w,
            grasp_height
        };

        RCLCPP_INFO(
            this->get_logger(),
            "发送抓取请求: task_id=%d, target=(%.3f, %.3f, %.3f), quat=(%.3f, %.3f, %.3f, %.3f)",
            goal_msg.task_id,
            goal_msg.data[0],
            goal_msg.data[1],
            goal_msg.data[2],
            goal_msg.data[3],
            goal_msg.data[4],
            goal_msg.data[5],
            goal_msg.data[6],
            goal_msg.data[7]);

        rclcpp_action::Client<ArmTask>::SendGoalOptions send_goal_options;
        send_goal_options.goal_response_callback =
            std::bind(&CatchKfsTestNode::on_goal_response, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&CatchKfsTestNode::on_feedback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&CatchKfsTestNode::on_result, this, std::placeholders::_1);

        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    // 目标响应回调函数
    //
    // 当动作服务器响应目标请求时调用。
    //
    // 参数：
    //   goal_handle: 目标句柄，如果为空表示请求被拒绝
    void on_goal_response(const GoalHandleArmTask::SharedPtr& goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "抓取目标被服务器拒绝");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "抓取目标已被接受，等待执行结果");
    }

    // 反馈回调函数
    //
    // 在任务执行过程中接收服务器的反馈信息。
    //
    // 参数：
    //   goal_handle: 目标句柄（未使用）
    //   feedback: 反馈消息，包含执行状态描述
    void on_feedback(
        GoalHandleArmTask::SharedPtr,
        const std::shared_ptr<const ArmTask::Feedback> feedback) {
        RCLCPP_INFO(this->get_logger(), "动作反馈: %s", feedback->describe.c_str());
    }

    // 结果回调函数
    //
    // 当动作执行完成（成功、失败或取消）时调用。
    // 根据结果码输出不同的日志信息，然后关闭节点。
    //
    // 参数：
    //   result: 包含执行结果的结构体，包括结果码、错误码和原因描述
    void on_result(const GoalHandleArmTask::WrappedResult& result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(
                    this->get_logger(), "抓取动作成功: err_code=%d, reason=%s",
                    result.result->err_code, result.result->reason.c_str());
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(
                    this->get_logger(), "抓取动作失败: err_code=%d, reason=%s",
                    result.result->err_code, result.result->reason.c_str());
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(
                    this->get_logger(), "抓取动作被取消: err_code=%d, reason=%s",
                    result.result->err_code, result.result->reason.c_str());
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "抓取动作返回了未知结果码");
                break;
        }

        rclcpp::shutdown();
    }

    // 动作客户端，用于与 robotic_task 服务器通信
    rclcpp_action::Client<ArmTask>::SharedPtr action_client_;
    
    // TF 缓冲区，用于存储和管理坐标变换
    tf2_ros::Buffer tf_buffer_;
    
    // TF 监听器，自动订阅 TF 话题并更新缓冲区
    tf2_ros::TransformListener tf_listener_;
    
    // 启动定时器，用于延迟执行测试任务
    rclcpp::TimerBase::SharedPtr startup_timer_;
    
    // 标记是否已开始发送请求，防止重复执行
    bool request_started_{false};
};

// 程序入口
//
// 初始化 ROS2，创建测试节点并开始执行。
// 节点执行完成后会自动退出。
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CatchKfsTestNode>();
    rclcpp::spin(node);
    return 0;
}
