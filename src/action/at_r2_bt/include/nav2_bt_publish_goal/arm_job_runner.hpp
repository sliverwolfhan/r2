#ifndef NAV2_BT_PUBLISH_GOAL__ARM_JOB_RUNNER_HPP_
#define NAV2_BT_PUBLISH_GOAL__ARM_JOB_RUNNER_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/action/arm_task.hpp"

namespace nav2_bt_publish_goal
{

// ArmJobRunner：把"按名字发臂动作"做成后台串行队列。
//   - enqueueNamed(): 立即返回，活在后台跑
//   - busy() / waitIdle(): 给上层做同步点（PICK 前等空队列）
// 共享单例放进 BT 黑板（key 默认 "arm_runner"），所有 ArmMoveNamedAsync / WaitArmIdle 节点都用同一个实例。
class ArmJobRunner
{
public:
  using ArmTask = robot_interfaces::action::ArmTask;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ArmTask>;

  ArmJobRunner(
    const std::string & node_name,
    const std::string & server_name,
    const std::string & default_yaml_path);
  ~ArmJobRunner();

  ArmJobRunner(const ArmJobRunner &) = delete;
  ArmJobRunner & operator=(const ArmJobRunner &) = delete;

  // 把命名姿态推进队列。失败原因（找不到名字 / yaml 加载失败）只通过返回值告知。
  bool enqueueNamed(
    const std::string & pose_name,
    int32_t task_id,
    double duration,
    const std::string & yaml_path = "");

  // 当前是否仍有未完成作业（队列里 + 正在执行的）。
  bool busy() const;

  // 阻塞等待 idle。timeout<=0 表示永久等。返回 true=已 idle，false=超时。
  bool waitIdle(double timeout_seconds);

  // 取消当前在跑的 goal 并清空队列（BT halted 时调用）。
  void cancelAll();

  rclcpp::Node::SharedPtr node() const { return node_; }

private:
  struct Job
  {
    std::vector<double> joints;       // 6
    double duration;
    int32_t task_id;
    std::string pose_name;            // 仅用于日志
  };

  bool ensureLoaded(const std::string & yaml_path);
  bool lookupPose(
    const std::string & pose_name,
    const std::string & yaml_path,
    std::vector<double> & out_joints);

  void workerLoop();
  void sendGoalAndWait(const Job & job);

  // ROS 资源
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread executor_thread_;
  rclcpp_action::Client<ArmTask>::SharedPtr action_client_;
  std::string server_name_;
  std::string default_yaml_path_;

  // YAML 缓存
  mutable std::mutex yaml_mtx_;
  std::map<std::string, std::vector<double>> named_positions_;
  std::string loaded_yaml_path_;

  // 队列
  mutable std::mutex q_mtx_;
  std::condition_variable q_cv_;
  std::deque<Job> queue_;
  bool running_active_{false};       // 当前 worker 正在送某个 goal
  std::atomic<bool> stop_{false};
  std::condition_variable idle_cv_;  // 用于 waitIdle

  // 工作线程
  std::thread worker_;

  // 当前在跑的 goal handle，供 cancelAll 使用
  std::mutex handle_mtx_;
  GoalHandle::SharedPtr current_handle_;
};

}  // namespace nav2_bt_publish_goal

#endif  // NAV2_BT_PUBLISH_GOAL__ARM_JOB_RUNNER_HPP_
