#include "nav2_bt_publish_goal/arm_job_runner.hpp"

#include <functional>
#include <utility>

#include "yaml-cpp/yaml.h"

namespace nav2_bt_publish_goal
{

using namespace std::chrono_literals;

ArmJobRunner::ArmJobRunner(
  const std::string & node_name,
  const std::string & server_name,
  const std::string & default_yaml_path)
: server_name_(server_name),
  default_yaml_path_(default_yaml_path)
{
  node_ = std::make_shared<rclcpp::Node>(node_name);
  action_client_ = rclcpp_action::create_client<ArmTask>(node_, server_name_);
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  executor_thread_ = std::thread([this]() { executor_->spin(); });

  worker_ = std::thread(&ArmJobRunner::workerLoop, this);

  RCLCPP_INFO(node_->get_logger(),
    "ArmJobRunner ready: server=%s default_yaml=%s",
    server_name_.c_str(), default_yaml_path_.c_str());
}

ArmJobRunner::~ArmJobRunner()
{
  stop_.store(true);
  {
    std::lock_guard<std::mutex> lk(q_mtx_);
    queue_.clear();
  }
  q_cv_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
  if (executor_) {
    executor_->cancel();
  }
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }
}

bool ArmJobRunner::ensureLoaded(const std::string & yaml_path)
{
  // 调用方持有 yaml_mtx_
  if (yaml_path == loaded_yaml_path_ && !named_positions_.empty()) {
    return true;
  }
  named_positions_.clear();
  loaded_yaml_path_.clear();

  try {
    YAML::Node root = YAML::LoadFile(yaml_path);
    YAML::Node poses;
    if (root["arm_positions"]) {
      poses = root["arm_positions"];
    } else if (root["arm_position"]) {
      poses = root["arm_position"];
    } else {
      RCLCPP_ERROR(node_->get_logger(),
        "ArmJobRunner: arm_yaml [%s] missing 'arm_positions'", yaml_path.c_str());
      return false;
    }
    if (poses.IsMap()) {
      for (const auto & entry : poses) {
        named_positions_[entry.first.as<std::string>()] =
          entry.second.as<std::vector<double>>();
      }
    } else if (poses.IsSequence()) {
      for (const auto & p : poses) {
        if (!p["name"] || !p["joints"]) { continue; }
        named_positions_[p["name"].as<std::string>()] =
          p["joints"].as<std::vector<double>>();
      }
    } else {
      RCLCPP_ERROR(node_->get_logger(),
        "ArmJobRunner: arm_yaml [%s] arm_positions is neither map nor sequence",
        yaml_path.c_str());
      return false;
    }
    loaded_yaml_path_ = yaml_path;
    RCLCPP_INFO(node_->get_logger(),
      "ArmJobRunner: loaded %zu named poses from %s",
      named_positions_.size(), yaml_path.c_str());
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(),
      "ArmJobRunner: failed to load arm_yaml [%s]: %s",
      yaml_path.c_str(), e.what());
    return false;
  }
}

bool ArmJobRunner::lookupPose(
  const std::string & pose_name,
  const std::string & yaml_path,
  std::vector<double> & out_joints)
{
  std::lock_guard<std::mutex> lk(yaml_mtx_);
  const std::string path = yaml_path.empty() ? default_yaml_path_ : yaml_path;
  if (!ensureLoaded(path)) { return false; }
  auto it = named_positions_.find(pose_name);
  if (it == named_positions_.end()) {
    RCLCPP_ERROR(node_->get_logger(),
      "ArmJobRunner: pose name [%s] not found in %s",
      pose_name.c_str(), path.c_str());
    return false;
  }
  if (it->second.size() != 6) {
    RCLCPP_ERROR(node_->get_logger(),
      "ArmJobRunner: pose [%s] has %zu joints, expected 6",
      pose_name.c_str(), it->second.size());
    return false;
  }
  out_joints = it->second;
  return true;
}

bool ArmJobRunner::enqueueNamed(
  const std::string & pose_name,
  int32_t task_id,
  double duration,
  const std::string & yaml_path)
{
  Job job;
  if (!lookupPose(pose_name, yaml_path, job.joints)) {
    return false;
  }
  job.duration = duration;
  job.task_id = task_id;
  job.pose_name = pose_name;

  {
    std::lock_guard<std::mutex> lk(q_mtx_);
    queue_.push_back(std::move(job));
  }
  q_cv_.notify_all();
  RCLCPP_INFO(node_->get_logger(),
    "ArmJobRunner: enqueued pose=%s duration=%.2fs (queue size now %zu)",
    pose_name.c_str(), duration,
    [this]() { std::lock_guard<std::mutex> lk(q_mtx_); return queue_.size(); }());
  return true;
}

bool ArmJobRunner::busy() const
{
  std::lock_guard<std::mutex> lk(q_mtx_);
  return running_active_ || !queue_.empty();
}

bool ArmJobRunner::waitIdle(double timeout_seconds)
{
  std::unique_lock<std::mutex> lk(q_mtx_);
  auto idle = [this]() { return !running_active_ && queue_.empty(); };
  if (timeout_seconds <= 0.0) {
    idle_cv_.wait(lk, idle);
    return true;
  }
  const auto dur = std::chrono::duration<double>(timeout_seconds);
  return idle_cv_.wait_for(lk, dur, idle);
}

void ArmJobRunner::cancelAll()
{
  {
    std::lock_guard<std::mutex> lk(q_mtx_);
    queue_.clear();
  }
  GoalHandle::SharedPtr h;
  {
    std::lock_guard<std::mutex> lk(handle_mtx_);
    h = current_handle_;
  }
  if (h && action_client_) {
    (void)action_client_->async_cancel_goal(h);
  }
}

void ArmJobRunner::workerLoop()
{
  while (!stop_.load()) {
    Job job;
    {
      std::unique_lock<std::mutex> lk(q_mtx_);
      q_cv_.wait(lk, [this]() { return stop_.load() || !queue_.empty(); });
      if (stop_.load()) { break; }
      job = std::move(queue_.front());
      queue_.pop_front();
      running_active_ = true;
    }
    sendGoalAndWait(job);
    {
      std::lock_guard<std::mutex> lk(q_mtx_);
      running_active_ = false;
    }
    idle_cv_.notify_all();
  }
}

void ArmJobRunner::sendGoalAndWait(const Job & job)
{
  if (!action_client_) { return; }
  if (!action_client_->wait_for_action_server(2s)) {
    RCLCPP_ERROR(node_->get_logger(),
      "ArmJobRunner: action server [%s] not available, dropping pose=%s",
      server_name_.c_str(), job.pose_name.c_str());
    return;
  }

  ArmTask::Goal goal;
  goal.task_id = job.task_id;
  goal.data = job.joints;
  goal.data.push_back(job.duration);

  // 用 future 等响应 + 结果。executor_ 在另一个线程里 spin，所以这里 .get() 不会死锁。
  auto goal_future = action_client_->async_send_goal(goal);
  auto goal_handle = goal_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(),
      "ArmJobRunner: goal rejected (pose=%s)", job.pose_name.c_str());
    return;
  }
  {
    std::lock_guard<std::mutex> lk(handle_mtx_);
    current_handle_ = goal_handle;
  }

  auto result_future = action_client_->async_get_result(goal_handle);
  auto wrapped = result_future.get();
  {
    std::lock_guard<std::mutex> lk(handle_mtx_);
    current_handle_.reset();
  }

  const int code = static_cast<int>(wrapped.code);
  RCLCPP_INFO(node_->get_logger(),
    "ArmJobRunner: pose=%s finished code=%d err=%d reason=%s",
    job.pose_name.c_str(), code,
    wrapped.result ? wrapped.result->err_code : -1,
    wrapped.result ? wrapped.result->reason.c_str() : "(null)");
}

}  // namespace nav2_bt_publish_goal
