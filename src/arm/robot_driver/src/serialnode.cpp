// Copyright 2026 RoboticArm Project Authors
// SPDX-License-Identifier: Apache-2.0
//
// SerialNode ROS2 节点的实现文件。
// 负责上位机与下位机之间的双向数据通信，通过 USB CDC 协议传输数据。

#include "robot_interfaces/msg/arm.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <chrono>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/time.hpp>
#include <serialnode.hpp>

using namespace std::chrono_literals;

SerialNode::SerialNode()
    : Node("driver_node") {

    // 初始化目标数据包的消息类型
    arm_target.pack_type = 1;

    // 声明参数：气泵使能开关
    this->declare_parameter<bool>("enable_air_pump", false);
    this->declare_parameter<int>("grasp_it", 0);
    this->declare_parameter<int>("grasp_state", 0);
    this->declare_parameter<std::string>("arm_task_node_name", "arm_task_node");
    this->get_parameter("arm_task_node_name", arm_task_node_name);
    arm_task_param_client = std::make_shared<rclcpp::AsyncParametersClient>(this, arm_task_node_name);

    // 注册参数变更回调，支持运行时动态修改气泵状态
    param_server_handle = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) {
            rcl_interfaces::msg::SetParametersResult result;

            for (const auto& param : params) {
                if (param.get_name() == "enable_air_pump") {
                    enable_air_pump = param.as_bool();
                    RCLCPP_INFO(this->get_logger(), 
                               "气泵状态变更: %d", enable_air_pump ? 1 : 0);
                } else if (param.get_name() == "grasp_it") {
                    grasp_it = (param.as_int() == 0) ? 0 : 1;
                    RCLCPP_INFO(this->get_logger(), "抓取状态变更: %d", grasp_it);
                } else if (param.get_name() == "grasp_state") {
                    const int new_grasp_state = (param.as_int() == 0) ? 0 : 1;
                    if (grasp_state == 0 && new_grasp_state == 1) {
                        grasp_state_send_once_pending = true;
                    }
                    grasp_state = new_grasp_state;
                    RCLCPP_INFO(this->get_logger(), "抓取完成状态变更: %d", grasp_state);
                }
            }
            result.successful = true;
            return result;
        });

    // 初始化线程退出标志
    exit_thread = false;

    // 创建机械臂状态发布者
    joint_publisher = this->create_publisher<robot_interfaces::msg::Arm>(
        "myjoints_state", 10);

    // 创建机械臂目标状态订阅者
    joint_subscriber = this->create_subscription<robot_interfaces::msg::Arm>(
        "myjoints_target", 10, 
        std::bind(&SerialNode::legsSubscribCb, this, std::placeholders::_1));

    // 创建 CDC 传输对象并注册接收回调
    cdc_trans = std::make_unique<CDCTrans>();
    cdc_trans->register_recv_cb([this](const uint8_t* data, int size) {
        // 验证数据包长度是否匹配 ArmState_t
        if (size == sizeof(ArmState_t)) {
            const ArmState_t* pack = reinterpret_cast<const ArmState_t*>(data);
            // 确认数据包类型正确（pack_type == 1）
            if (pack->pack_type == 1) {
                publishLegState(pack);
                handleGraspIt(pack);
            }
        }
    });

    // 打开 USB CDC 设备（VID: 0x0483, PID: 0x5741）
    if (!cdc_trans->open(0x0483, 0x5741)) {
        RCLCPP_ERROR(get_logger(), "串口打开失败，无法驱动物理机械臂！");
        exit_thread = true;
    }

    // 创建独立线程处理 USB CDC 事件循环
    usb_event_handle_thread = std::make_unique<std::thread>([this]() {
        do {
            cdc_trans->process_once();
        } while (!exit_thread);
    });
}

SerialNode::~SerialNode() {
    // 设置线程退出标志
    exit_thread = true;

    // 等待 USB 事件处理线程安全退出
    if (usb_event_handle_thread && usb_event_handle_thread->joinable()) {
        usb_event_handle_thread->join();
    }

    // 关闭 USB CDC 连接
    if (cdc_trans) {
        cdc_trans->close();
    }
}

void SerialNode::publishLegState(const ArmState_t* arm_state) {
    robot_interfaces::msg::Arm msg;

    // 将下位机上报的关节状态转换为 ROS2 消息
    for (int i = 0; i < 6; i++) {
        msg.motor[i].rad    = arm_state->joints[i].rad;
        msg.motor[i].omega  = arm_state->joints[i].omega;
        msg.motor[i].torque = arm_state->joints[i].torque;
    }

    // 欺骗 MoveIt 认为不存在的关节6到达目标位置
    msg.motor[5].rad = arm_target.joints[5].rad;

    // 发布机械臂状态
    joint_publisher->publish(msg);

    // 控制日志输出频率
    cur_pub_cnt++;
    if (cur_pub_cnt == publish_cnt) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 50, 
                            "发布电机状态");
        cur_pub_cnt = 0;
    }
}

void SerialNode::legsSubscribCb(const robot_interfaces::msg::Arm& msg) {
    // 将 ROS2 消息转换为目标数据包
    for (int i = 0; i < 6; i++) {
        arm_target.joints[i].rad    = msg.motor[i].rad;
        arm_target.joints[i].omega  = msg.motor[i].omega;
        arm_target.joints[i].torque = msg.motor[i].torque;
    }

    // 设置气泵状态
    arm_target.air_pump = enable_air_pump ? 1 : 0;
    arm_target.grasp_state = grasp_state_send_once_pending ? 1U : 0U;
    grasp_state_send_once_pending = false;

    // 通过 USB CDC 发送目标数据包到下位机
    cdc_trans->send_struct(arm_target);
}

void SerialNode::handleGraspIt(const ArmState_t* arm_state) {
    const int new_grasp_it = (arm_state->grasp_it == 0U) ? 0 : 1;
    if (new_grasp_it == grasp_it) {
        return;
    }

    grasp_it = new_grasp_it;
    this->set_parameter(rclcpp::Parameter("grasp_it", grasp_it));
    RCLCPP_INFO(this->get_logger(), "收到抓取命令 grasp_it=%d", grasp_it);

    if (!arm_task_param_client || !arm_task_param_client->wait_for_service(std::chrono::milliseconds(200))) {
        RCLCPP_WARN(this->get_logger(), "arm_task 参数服务不可用，无法同步 grasp_it");
        return;
    }

    arm_task_param_client->set_parameters({rclcpp::Parameter("grasp_it", grasp_it)});
}



// grasp_it 参数有上位机控制，当下位机上报的 grasp_it 发生变化时，通过参数服务器同步到 arm_task 节点，触发抓取任务的执行。