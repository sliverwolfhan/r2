#include "task/idel.hpp"
#include "robot.hpp"
#include <chrono>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>

namespace {
constexpr int32_t kTaskCatchTarget = 2;
constexpr int32_t kTaskPlaceTarget = 3;
constexpr int32_t kTaskMoveTarget = 1;
constexpr int32_t kTaskMoveCartesianTarget = 4;
}

IdelTask::IdelTask(Robot* context,const std::string name) : BaseTask(context,name)
{
    
}

IdelTask::~IdelTask()
{

}

std::string IdelTask::process(const std::string last_task_name)
{
    (void)last_task_name;

    int grasp_it = 0;
    if (robot->node_->get_parameter("grasp_it", grasp_it) && grasp_it == 1) {
        RCLCPP_INFO(robot->node_->get_logger(), "idel 检测到 grasp_it=1，切换到 catch_kfs");
        robot->node_->set_parameter(rclcpp::Parameter("grasp_it", 0));
        robot->current_kfs_num_ = 0;
        robot->set_air_pump(false);
        return "catch_kfs";
    }

    if (is_first_run) {
        // 等待 MuJoCo 和硬件接口完全初始化（约 2 秒）
        RCLCPP_INFO(robot->node_->get_logger(), "idel 首次运行，等待硬件初始化...");
        rclcpp::sleep_for(std::chrono::seconds(2));
        
        RCLCPP_INFO(robot->node_->get_logger(), "idel 首次运行");
        std::vector<double> ready_joint_angles;
        std::string ready_position_name = "ready";
        
        if (!robot->get_named_joint_position(ready_position_name, ready_joint_angles)) {
            RCLCPP_ERROR(robot->node_->get_logger(), "未找到命名位姿 [%s]", ready_position_name.c_str());
        }

        RCLCPP_INFO(robot->node_->get_logger(), "idel 首次运行，移动到准备位置");
        if (!robot->execute_joint_space_trajectory(ready_joint_angles, 5.0)) { // 1.0n
            RCLCPP_INFO(robot->node_->get_logger(), "idel 首次运行，移动到准备位置失败");
        }

        is_first_run = false;
    }

    // // TODO: 移动到ready位置
    // std::vector<double> ready_joint_angles;
    // robot->get_named_joint_position("ready", ready_joint_angles);
    // robot->execute_joint_space_trajectory(ready_joint_angles, 1.0);

    if (!robot->wait_for_idle_signal(std::chrono::milliseconds(1000))) {
        RCLCPP_INFO(robot->node_->get_logger(), "idel 等待新任务信号超时，继续等待");
        return "idel";
    }

    Robot::PendingTaskRequest request;
    if (!robot->take_pending_task(request)) {
        RCLCPP_INFO(robot->node_->get_logger(), "获取挂起的任务失败，继续等待");
        return "idel";
    }

    switch (request.task_id) {
        case kTaskCatchTarget:
            RCLCPP_INFO(robot->node_->get_logger(), "idel 收到抓取任务，切换到 catch_kfs");
            return "catch_kfs";
        case kTaskMoveTarget:
            RCLCPP_WARN(robot->node_->get_logger(), "idel 收到移动任务，切换到 move_kfs");
            return "move_kfs";
        case kTaskPlaceTarget:
            RCLCPP_WARN(robot->node_->get_logger(), "idel 收到放置任务，切换到 place_kfs");
            return "place_kfs";
        case kTaskMoveCartesianTarget:
            RCLCPP_INFO(robot->node_->get_logger(), "idel 收到笛卡尔移动任务，切换到 move_cartesian");
            return "move_cartesian";
        default:
            RCLCPP_WARN(robot->node_->get_logger(), "未知 task_id=%d，无法分发任务", request.task_id);
            robot->finish_current_task(request.goal_handle, false, "未知 task_id，无法分发任务");
            return "idel";
    }

    return "idel";
}