#include "task/place_kfs.hpp"
#include "robot.hpp"
#include <chrono>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

using namespace std::chrono_literals;

PlaceKFS::PlaceKFS(Robot* context, const std::string name)
    : BaseTask(context, name) {
}

PlaceKFS::~PlaceKFS() {}

std::string PlaceKFS::process(const std::string last_task_name) {
    (void)last_task_name;

    if (robot->current_kfs_num_ == 0) {
        RCLCPP_WARN(robot->node_->get_logger(), "当前车上 KFS 数量为 0 ，不能再执行放置任务。");
        return "idel";
    }

    Robot::ActiveTaskContext context;
    if (!robot->get_active_task_context(context)) {
        RCLCPP_WARN(robot->node_->get_logger(), "place_kfs 未获取到活动任务上下文，返回 idel");
        return "idel";
    };

    const auto goal_handle = context.goal_handle;

    // 从任务数据中获取目标货架位姿
    if (context.data.size() == (3 + 4)) {
        robot->target_shelf_.position.x = context.data[0];
        robot->target_shelf_.position.y = context.data[1];
        robot->target_shelf_.position.z = context.data[2];
        robot->target_shelf_.orientation.x = context.data[3];
        robot->target_shelf_.orientation.y = context.data[4];
        robot->target_shelf_.orientation.z = context.data[5];
        robot->target_shelf_.orientation.w = context.data[6];
        RCLCPP_INFO(
            robot->node_->get_logger(),
            "从任务数据中获取目标货架位姿: [%.3f, %.3f, %.3f]",
            robot->target_shelf_.position.x,
            robot->target_shelf_.position.y,
            robot->target_shelf_.position.z);
    } else if (context.data.size() != 0) {
        RCLCPP_ERROR(
            robot->node_->get_logger(),
            "接收到的目标货架位姿数据维度不正确，预期为3+4或0，实际为%zu",
            context.data.size());
        robot->finish_current_task(goal_handle, false, "接收到的目标货架位姿数据维度不正确");
        return "idel";
    } else {
        RCLCPP_WARN(
            robot->node_->get_logger(),
            "未收到目标货架位姿数据，使用默认的 target_shelf_");
    }


    // 1. 移动到 "kfs" + std::to_string(robot->current_kfs_num_) + "_touch_pos"
    std::string touch_pos_name = "kfs" + std::to_string(robot->current_kfs_num_) + "_touch_pos";
    std::vector<double> touch_pos;
    if (!robot->get_named_joint_position(touch_pos_name, touch_pos)) {
        RCLCPP_ERROR(robot->node_->get_logger(), "未找到命名位姿 [%s]", touch_pos_name.c_str());
        robot->finish_current_task(goal_handle, false, "未找到命名位姿 " + touch_pos_name);
        return "idel";
    }

    RCLCPP_INFO(robot->node_->get_logger(), "移动到触摸位置 [%s]", touch_pos_name.c_str());
    if (!robot->execute_joint_space_trajectory(touch_pos, 1.0)) {
        robot->finish_current_task(goal_handle, false, "移动到触摸位置失败");
        return "idel";
    }

    // 2. 等待 1s
    std::this_thread::sleep_for(1s);

    // 3. 打开气泵
    RCLCPP_INFO(robot->node_->get_logger(), "打开气泵");
    if (!robot->set_air_pump(true)) {
        robot->finish_current_task(goal_handle, false, "气泵开启失败");
        return "idel";
    }

    // 4. 等待 1s
    std::this_thread::sleep_for(1s);

    std::vector<double> ready_joint_angles_;
    if (!robot->get_named_joint_position("place_interim_pos_0", ready_joint_angles_)) {
        RCLCPP_ERROR(robot->node_->get_logger(), "未找到命名位姿 [place_interim_pos_0]");
        robot->finish_current_task(goal_handle, false, "未找到命名位姿 place_interim_pos_0");
        return "idel";
    }

    // 6. Move back to ready position
    RCLCPP_INFO(robot->node_->get_logger(), "移动到准备位置");
    if (!robot->execute_joint_space_trajectory(ready_joint_angles_, 0.9)) {
        robot->finish_current_task(goal_handle, false, "抓取完成后返回准备位失败");
        return "idel";
    }

    // 5. 移动到 target_shelf_ 的前方（x 方向减0.40m）
    geometry_msgs::msg::PoseStamped approach_pose;
    approach_pose.header.frame_id = "base_link";
    approach_pose.header.stamp = robot->node_->now();
    approach_pose.pose = robot->target_shelf_;
    approach_pose.pose.position.x -= 0.40;
    // 强制规定姿态
    tf2::Quaternion quat;
    quat.setRPY(0, (M_PI/2.5), 0);
    quat.normalize();
    approach_pose.pose.orientation.w = quat.getW();
    approach_pose.pose.orientation.x = quat.getX();
    approach_pose.pose.orientation.y = quat.getY();
    approach_pose.pose.orientation.z = quat.getZ();

    double duration = robot->calculate_duration("place_interim_pos_0");
    
    RCLCPP_INFO(robot->node_->get_logger(), "计算出来移动到place_interim_pos_0的时间: %lf", duration);

    RCLCPP_INFO(robot->node_->get_logger(), "移动到货架前方位置");
    if (!robot->execute_cartesian_space_trajectory(approach_pose, 0.8)) {
        robot->finish_current_task(goal_handle, false, "移动到货架前方失败");
        return "idel";
    }

    robot->current_kfs_num_ -= 1;

    // 6. 等待 1s
    std::this_thread::sleep_for(1s);

    // 7. 沿直线轨迹移动到 target_shelf_
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "base_link";
    target_pose.header.stamp = robot->node_->now();
    target_pose.pose = robot->target_shelf_;
        // 强制规定姿态
    tf2::Quaternion quat_;
    quat_.setRPY(0, (M_PI / 2), 0);
    quat_.normalize();
    target_pose.pose.orientation.w = quat_.getW();
    target_pose.pose.orientation.x = quat_.getX();
    target_pose.pose.orientation.y = quat_.getY();
    target_pose.pose.orientation.z = quat_.getZ();

    RCLCPP_INFO(robot->node_->get_logger(), "沿直线轨迹移动到货架");
    if (!robot->execute_cartesian_space_trajectory(target_pose, 1.0)) {
        robot->finish_current_task(goal_handle, false, "沿直线轨迹移动到货架失败");
        return "idel";
    }

    // 8. 等待 1s
    std::this_thread::sleep_for(1s);

    // 9. 关闭气泵
    RCLCPP_INFO(robot->node_->get_logger(), "关闭气泵");
    if (!robot->set_air_pump(false)) {
        robot->finish_current_task(goal_handle, false, "气泵关闭失败");
        return "idel";
    }


    std::vector<double> ready_joint_angles;
    if (!robot->get_named_joint_position("ready", ready_joint_angles)) {
        RCLCPP_ERROR(robot->node_->get_logger(), "未找到命名位姿 [ready]");
        robot->finish_current_task(goal_handle, false, "未找到命名位姿 ready");
        return "idel";
    }

    // 6. Move back to ready position
    RCLCPP_INFO(robot->node_->get_logger(), "移动到准备位置");
    if (!robot->execute_joint_space_trajectory(ready_joint_angles, 0.9)) {
        robot->finish_current_task(goal_handle, false, "抓取完成后返回准备位失败");
        return "idel";
    }

    RCLCPP_INFO(robot->node_->get_logger(), "放置流程完成");
    robot->finish_current_task(goal_handle, true, "放置流程执行完成");

    return "idel";
}