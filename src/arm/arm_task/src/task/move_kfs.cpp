#include "task/move_kfs.hpp"
#include "robot.hpp"
#include <rclcpp/rclcpp.hpp>
#include <vector>

MoveKFS::MoveKFS(Robot* context, const std::string name)
    : BaseTask(context, name) {
}

MoveKFS::~MoveKFS() {}

std::string MoveKFS::process(const std::string last_task_name) {
    (void)last_task_name;

    Robot::ActiveTaskContext context;
    const bool has_action_context = robot->get_active_task_context(context);
    const auto goal_handle = has_action_context ? context.goal_handle : nullptr;
    if (!has_action_context) {
        RCLCPP_WARN(robot->node_->get_logger(), "move_kfs 未获取到活动任务上下文，返回 idel");
        return "idel";
    }

    auto fail_task = [&](const std::string& error_msg) {
        if (goal_handle) {
            robot->finish_current_task(goal_handle, false, error_msg);
        }
        return "idel";
    };

    if (context.data.size() != 7) {
        RCLCPP_ERROR(robot->node_->get_logger(), "接收到的移动任务数据维度不正确，预期为7，实际为%zu", context.data.size());
        return fail_task("接收到的移动任务数据维度不正确");
    }

    const double duration_ = context.data[6];
    const std::vector<double> joint_angles = {
        context.data[0],
        context.data[1],
        context.data[2],
        context.data[3],
        context.data[4],
        context.data[5],
    };

    RCLCPP_INFO(robot->node_->get_logger(), "执行关节空间移动任务");
    if (!robot->execute_joint_space_trajectory(joint_angles, duration_)) {
        return fail_task("执行移动轨迹失败");
    }

    RCLCPP_INFO(robot->node_->get_logger(), "移动流程完成");
    if (goal_handle) {
        robot->finish_current_task(goal_handle, true, "移动流程执行完成");
    }

    return "idel";
}