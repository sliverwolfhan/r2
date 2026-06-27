#include "task/move_cartesian.hpp"
#include "robot.hpp"
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

MoveCartesian::MoveCartesian(Robot* context, const std::string name)
    : BaseTask(context, name) {
}

MoveCartesian::~MoveCartesian() {}0

std::string MoveCartesian::process(const std::string last_task_name) {
    (void)last_task_name;

    Robot::ActiveTaskContext context;
    if (!robot->get_active_task_context(context)) {
        RCLCPP_WARN(robot->node_->get_logger(), "move_cartesian 未获取到活动任务上下文，返回 idel");
        return "idel";
    }

    const auto goal_handle = context.goal_handle;

    // data 维度仍按 xyz + 四元数（7）约定，但只读取 xyz，姿态强制朝向 z 轴负方向
    if (context.data.size() != (3 + 4)) {
        RCLCPP_ERROR(
            robot->node_->get_logger(),
            "move_cartesian 接收到的数据维度不正确，预期为 3+4，实际为 %zu",
            context.data.size());
        robot->finish_current_task(goal_handle, false, "move_cartesian 数据维度不正确");
        return "idel";
    }

    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "base_link";
    target_pose.header.stamp    = robot->node_->now();
    target_pose.pose.position.x = context.data[0];
    target_pose.pose.position.y = context.data[1];
    target_pose.pose.position.z = context.data[2];

    // 强制末端姿态朝向 z 轴负方向
    tf2::Quaternion quat;
    quat.setRPY(0, M_PI, 0);
    quat.normalize();
    target_pose.pose.orientation.w = quat.getW();
    target_pose.pose.orientation.x = quat.getX();
    target_pose.pose.orientation.y = quat.getY();
    target_pose.pose.orientation.z = quat.getZ();

    RCLCPP_INFO(
        robot->node_->get_logger(),
        "move_cartesian 目标位置: [%.3f, %.3f, %.3f]，姿态固定朝向 -z",
        target_pose.pose.position.x,
        target_pose.pose.position.y,
        target_pose.pose.position.z);

    if (!robot->execute_cartesian_space_trajectory(target_pose, 1.0)) {
        robot->finish_current_task(goal_handle, false, "move_cartesian 笛卡尔轨迹执行失败");
        return "idel";
    }

    robot->finish_current_task(goal_handle, true, "move_cartesian 执行完成");
    return "idel";
}
