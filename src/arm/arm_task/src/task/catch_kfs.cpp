#include "task/catch_kfs.hpp"
#include "robot.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <string>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/utilities.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/exceptions.h>
#include <thread>

using namespace std::chrono_literals;

CatchKFS::CatchKFS(Robot* context, const std::string name)
    : BaseTask(context, name) {
}

CatchKFS::~CatchKFS() {}


std::string CatchKFS::process(const std::string last_task_name) {
    (void)last_task_name;

    if (!robot->set_grasp_state(false)) {
        RCLCPP_WARN(robot->node_->get_logger(), "设置 grasp_state=0 失败");
    }

    // if (robot->current_kfs_num_ == 1) {
    //     RCLCPP_WARN(robot->node_->get_logger(), "当前车上 KFS 数量为 1 ，不能再执行抓取任务。");
    //     return "idel";
    // }


    
    Robot::ActiveTaskContext context;
    const bool has_action_context = robot->get_active_task_context(context);
    const auto goal_handle = has_action_context ? context.goal_handle : nullptr;
    if (!has_action_context) {
        RCLCPP_WARN(robot->node_->get_logger(), "catch_kfs 未获取到活动任务上下文，使用 TF 目标执行遥控抓取");
    }

    // Helper lambda to safely terminate the current task on failure
    auto fail_task = [&](const std::string& error_msg) {
        if (goal_handle) {
            robot->finish_current_task(goal_handle, false, error_msg);
        }
        return "idel";
    };
    robot->set_air_pump(true);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 获取抓取高度 z：从 action 上下文获取覆盖值（如果有）
    // 获取目标位姿：从 TF 查询 base_link -> target_object
    double grasp_height_for_check = 0.0;
    double action_position_x_ = 0.0;
    double action_position_y_ = 0.0;
    double action_position_z_ = 0.0;
    double action_orientation_x_ = 0.0;
    double action_orientation_y_ = 0.0;
    double action_orientation_z_ = 0.0;
    double action_orientation_w_ = 0.0;

    if (has_action_context && context.data.size() >= 8) {
        grasp_height_for_check = context.data[7];
        action_position_x_ = context.data[0];
        action_position_y_ = context.data[1];
        action_position_z_ = context.data[2];
        action_orientation_x_ = context.data[3];
        action_orientation_y_ = context.data[4];
        action_orientation_z_ = context.data[5];
        action_orientation_w_ = context.data[6];
        RCLCPP_INFO(robot->node_->get_logger(), "grasp_height_for_check = %lf", grasp_height_for_check);
    }

    geometry_msgs::msg::TransformStamped target_tf;
    bool tf_ok = false;
    try {
        if (robot->tf_buffer_->canTransform(robot->base_frame_, robot->object_frame_, tf2::TimePointZero, 200ms)) {
            target_tf = robot->tf_buffer_->lookupTransform(robot->base_frame_, robot->object_frame_, tf2::TimePointZero);
            tf_ok = true;
        } else {
            RCLCPP_WARN(robot->node_->get_logger(), "等待 TF %s -> %s 超时，改用 action 通信数据",
                         robot->base_frame_.c_str(), robot->object_frame_.c_str());
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(robot->node_->get_logger(), "读取 TF 目标位姿失败: %s，改用 action 通信数据", e.what());
    }

    // TF 查询失败时的回退：必须有 action 数据才能继续，否则无目标可抓
    if (!tf_ok && !(has_action_context && context.data.size() >= 8)) {
        RCLCPP_ERROR(robot->node_->get_logger(), "TF 查询失败且无可用的 action 目标数据，无法抓取");
        return fail_task("TF 查询失败且无 action 目标数据");
    }

    double position_x_ = tf_ok ? target_tf.transform.translation.x : action_position_x_;
    double position_y_ = tf_ok ? target_tf.transform.translation.y : action_position_y_;
    double position_z_ = tf_ok ? target_tf.transform.translation.z : action_position_z_;
    double orientation_x_ = tf_ok ? target_tf.transform.rotation.x : action_orientation_x_;
    double orientation_y_ = tf_ok ? target_tf.transform.rotation.y : action_orientation_y_;
    double orientation_z_ = tf_ok ? target_tf.transform.rotation.z : action_orientation_z_;
    double orientation_w_ = tf_ok ? target_tf.transform.rotation.w : action_orientation_w_;

    // 比较 TF 数据与 action 数据的位置偏差
    double final_position_x_ = position_x_;
    double final_position_y_ = position_y_;
    double final_position_z_ = position_z_;
    double final_orientation_x_ = orientation_x_;
    double final_orientation_y_ = orientation_y_;
    double final_orientation_z_ = orientation_z_;
    double final_orientation_w_ = orientation_w_;
    bool using_tf_data = tf_ok;  // 标记最终目标位姿来源：true=TF，false=action

    // 仅在 TF 查询成功且有 action 数据时，才做 TF/action 偏差比较；
    // TF 失败时上面已直接采用 action 数据，无需再比较。
    if (tf_ok && has_action_context && context.data.size() >= 8) {
        double dx = position_x_ - action_position_x_;
        double dy = position_y_ - action_position_y_;
        double dz = position_z_ - action_position_z_;
        double pos_distance = std::sqrt(dx * dx + dy * dy + dz * dz);

        const double POSITION_THRESHOLD = 0.5; // 10cm = 0.1m

        if (pos_distance > POSITION_THRESHOLD) {
            RCLCPP_WARN(robot->node_->get_logger(),
                "TF 与 action 位置偏差 %.4f m 超过阈值 %.2f m，使用 action 数据作为目标位姿",
                pos_distance, POSITION_THRESHOLD);
            final_position_x_ = action_position_x_;
            final_position_y_ = action_position_y_;
            final_position_z_ = action_position_z_;
            final_orientation_x_ = action_orientation_x_;
            final_orientation_y_ = action_orientation_y_;
            final_orientation_z_ = action_orientation_z_;
            final_orientation_w_ = action_orientation_w_;
            using_tf_data = false;
        } else {
            RCLCPP_INFO(robot->node_->get_logger(),
                "TF 与 action 位置偏差 %.4f m 在阈值 %.2f m 内，使用 TF 数据作为目标位姿",
                pos_distance, POSITION_THRESHOLD);
        }
    }






    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    std::vector<double> ready_joint_angles;
    std::string ready_position_name = "ready";

    // 构建目标位姿：使用 TF 查询结果，z 用 action 抓取高度覆盖
    RCLCPP_INFO(robot->node_->get_logger(), "准备获取抓取目标位姿");
    geometry_msgs::msg::PoseStamped object_pose;

    double grasp_right_run_ = 0.15;
    double grasp_right_run_qian_ = 0.0;
    double grasp_duration_ = 0.8;
    double grasp_height = 0.0;

    object_pose.header.frame_id = robot->base_frame_;
    object_pose.header.stamp = robot->node_->now();
    object_pose.pose.position.x = final_position_x_;
    object_pose.pose.position.y = final_position_y_;
    object_pose.pose.position.z = grasp_height_for_check;  // z 用 action 抓取高度覆盖

    object_pose.pose.orientation.x = final_orientation_x_;
    object_pose.pose.orientation.y = final_orientation_y_;
    object_pose.pose.orientation.z = final_orientation_z_;
    object_pose.pose.orientation.w = final_orientation_w_;

    RCLCPP_INFO(robot->node_->get_logger(), "=====================================");
    RCLCPP_INFO(robot->node_->get_logger(), "position.x = %lf", object_pose.pose.position.x);
    RCLCPP_INFO(robot->node_->get_logger(), "position.y = %lf", object_pose.pose.position.y);
    RCLCPP_INFO(robot->node_->get_logger(), "position.z = %lf", object_pose.pose.position.z);
    RCLCPP_INFO(robot->node_->get_logger(), "orientation.x = %lf", object_pose.pose.orientation.x);
    RCLCPP_INFO(robot->node_->get_logger(), "orientation.y = %lf", object_pose.pose.orientation.y);
    RCLCPP_INFO(robot->node_->get_logger(), "orientation.z = %lf", object_pose.pose.orientation.z);
    RCLCPP_INFO(robot->node_->get_logger(), "orientation.w = %lf", object_pose.pose.orientation.w);
    RCLCPP_INFO(robot->node_->get_logger(), "=====================================");
    

    RCLCPP_INFO(
        robot->node_->get_logger(), "物体在坐标: [%.3f, %.3f, %.3f]", object_pose.pose.position.x, object_pose.pose.position.y,
        object_pose.pose.position.z);

    // 强制规定姿态：比较 |x| 和 |y|，较大的一方决定末端 Z 轴指向哪个坐标轴，
    // 再按该分量的正负决定指向正方向还是负方向。
    // 同时沿接近方向回退 grasp_right_run_，避免末端顶到目标。
    // 注：TF 数据来源时，认为坐标已经准确，不再做回退处理。
    // 回退采用"朝 0 收缩"的方式，保证 |new| ≤ |old|，不会跨过 0 反而变大。
    auto shrink_toward_zero = [](double v, double offset) {
        if (v >= 0.0) return std::max(0.0, v - offset);
        return std::min(0.0, v + offset);
    };

    tf2::Quaternion quat;
    quat.setRPY(0, 0, 0);  // 兜底单位四元数，避免 |x|==|y| 时未初始化
    if (std::abs(final_position_x_) >= std::abs(final_position_y_)) {
        // 末端 Z 轴指向 base_link 的 ±X：绕 Y 轴旋转
        const bool x_positive = final_position_x_ >= 0.0;
        quat.setRPY(0.0, x_positive ? M_PI / 2.0 : -M_PI / 2.0, 0.0);
        if (!using_tf_data) {
            object_pose.pose.position.x = shrink_toward_zero(object_pose.pose.position.x, grasp_right_run_);
        }
    } else {
        // 末端 Z 轴指向 base_link 的 ±Y：绕 X 轴旋转
        const bool y_positive = final_position_y_ >= 0.0;
        quat.setRPY(y_positive ? -M_PI / 2.0 : M_PI / 2.0, 0.0, 0.0);
        if (!using_tf_data) {
            object_pose.pose.position.y = shrink_toward_zero(object_pose.pose.position.y, grasp_right_run_);
        }
    }
    quat.normalize();


    object_pose.pose.orientation.w = quat.getW();
    object_pose.pose.orientation.x = quat.getX();
    object_pose.pose.orientation.y = quat.getY();
    object_pose.pose.orientation.z = quat.getZ();

    RCLCPP_INFO(robot->node_->get_logger(), "执行抓取动作");
    if (!robot->execute_cartesian_space_trajectory(object_pose, 0.7)) { // 0.8
        return fail_task("执行抓取轨迹失败");
    }
    std::this_thread::sleep_for(500ms);

    const double lift_up_z_ = 0.20;          // 垂直向上抬升量
    const double lift_retract_along_tcp_ = 0.15;  // 沿末端 -Z 方向回退量

    object_pose.pose.position.z += lift_up_z_;
    tf2::Vector3 retract_in_tcp(0.0, 0.0, -lift_retract_along_tcp_);
    tf2::Vector3 retract_in_base = tf2::quatRotate(quat, retract_in_tcp);
    object_pose.pose.position.x += retract_in_base.x();
    object_pose.pose.position.y += retract_in_base.y();
    object_pose.pose.position.z += retract_in_base.z();

    RCLCPP_INFO(robot->node_->get_logger(),
        "执行抬起动作: 垂直 +%.3f m, 沿末端-Z 回退 %.3f m (base 偏移 dx=%.3f dy=%.3f dz=%.3f)",
        lift_up_z_, lift_retract_along_tcp_,
        retract_in_base.x(), retract_in_base.y(), retract_in_base.z());
    if (!robot->execute_cartesian_space_trajectory(object_pose, 0.7)) { // 0.8
        return fail_task("执行抬起失败");
    }
    
    robot->current_kfs_num_ += 1;

    if (goal_handle) {
        robot->finish_current_task(goal_handle, true, "抓取流程执行完成");
    }



    RCLCPP_INFO(robot->node_->get_logger(), "抓取流程完成");
    if (!robot->set_grasp_state(true)) {
        RCLCPP_WARN(robot->node_->get_logger(), "设置 grasp_state=1 失败");
    }


    return "idel";
}