#include "robot.hpp"
#include "task/base_task.hpp"
#include "task/catch_kfs.hpp"
#include "task/idel.hpp"
#include "task/move_kfs.hpp"
#include "task/place_kfs.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <robot_interfaces/action/arm_task.hpp>
#include <chrono>
#include <cmath>
#include <memory>
#include <rclcpp/logging.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <thread>
#include <yaml-cpp/yaml.h>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::chrono_literals;

Robot::Robot(rclcpp::Node::SharedPtr node) {

    node_ = node;
    // Initialize TF2
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Declare parameters
    node_->declare_parameter<int32_t>("arm_task", 0);
    node_->declare_parameter<bool>("stop_visual_servo", false);
    node_->declare_parameter<double>("trajectory_duration", 3.0);
    node_->declare_parameter<double>("grasp_time", 5.0);
    node_->declare_parameter<double>("approach_distance", 0.1);
    node_->declare_parameter<double>("visual_servo_kp", 2.0);
    node_->declare_parameter<double>("visual_servo_max_linear_acc", 0.5);
    node_->declare_parameter<std::string>("base_frame", "base_link");
    node_->declare_parameter<std::string>("camera_frame", "camera_link");
    node_->declare_parameter<std::string>("object_frame", "target_object");
    node_->declare_parameter<std::string>("tip_frame", "link6");
    node_->declare_parameter<std::string>("arm_calc_node_name", "arm_calc_node");
    node_->declare_parameter<std::string>("driver_node_name", "driver_node");
    node_->declare_parameter<double>("max_linear_velocity", 0.1);
    node_->declare_parameter<double>("max_angular_velocity", 0.5);
    node_->declare_parameter<double>("max_joint_velocity", 3.0);
    node_->declare_parameter<double>("min_trajectory_duration", 0.1);
    node_->declare_parameter<double>("max_trajectory_duration", 10.0);
    node_->declare_parameter<int>("grasp_it", 0);
    node_->declare_parameter<double>("grasp_height", 1.0);
    node_->declare_parameter<double>("grasp_right_run", 0.10);
    node_->declare_parameter<double>("grasp_down_run", 0.15);
    node_->declare_parameter<double>("grasp_right_run_qian", 0.00);

    // Get parameters
    node_->get_parameter("trajectory_duration", trajectory_duration_);
    node_->get_parameter("grasp_time", grasp_time_);
    node_->get_parameter("approach_distance", approach_distance_);
    // node_->get_parameter("visual_servo_kp", visual_servo_kp_);
    node_->get_parameter("visual_servo_max_linear_acc", visual_servo_max_linear_acc_);
    node_->get_parameter("base_frame", base_frame_);
    node_->get_parameter("camera_frame", camera_frame_);
    node_->get_parameter("object_frame", object_frame_);
    node_->get_parameter("tip_frame", tip_frame_);
    node_->get_parameter("arm_calc_node_name", arm_calc_node_name_);
    node_->get_parameter("driver_node_name", driver_node_name_);
    node_->get_parameter("max_linear_velocity", max_linear_velocity_);
    node_->get_parameter("max_angular_velocity", max_angular_velocity_);
    node_->get_parameter("max_joint_velocity", max_joint_velocity_);
    node_->get_parameter("min_trajectory_duration", min_trajectory_duration_);
    node_->get_parameter("max_trajectory_duration", max_trajectory_duration_);

    // Load arm positions from YAML
    load_arm_positions_from_yaml();

    // Create publishers
    visual_target_pub_      = node_->create_publisher<geometry_msgs::msg::PoseStamped>("visual_target_pose", 10);
    joint_space_target_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("joint_space_target", 10);
    marker_pub_             = node_->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);

    task_handle_server = rclcpp_action::create_server<robot_interfaces::action::ArmTask>(
        node_, "robotic_task", std::bind(&Robot::on_handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&Robot::on_cancel_goal, this, std::placeholders::_1), std::bind(&Robot::on_handle_accepted, this, std::placeholders::_1));

    // Setup parameter callback
    param_callback_ = node_->add_on_set_parameters_callback(std::bind(&Robot::on_parameters_changed, this, std::placeholders::_1));

    // Create parameter client for arm_calc node
    arm_calc_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, arm_calc_node_name_);
    RCLCPP_INFO(node_->get_logger(), "Using arm_calc parameter client target: %s", arm_calc_node_name_.c_str());

    // Create parameter client for driver node
    driver_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, driver_node_name_);
    RCLCPP_INFO(node_->get_logger(), "Using driver parameter client target: %s", driver_node_name_.c_str());



    register_task(std::make_shared<IdelTask>(this, "idel"));
    register_task(std::make_shared<CatchKFS>(this, "catch_kfs"));
    register_task(std::make_shared<PlaceKFS>(this, "place_kfs"));
    register_task(std::make_shared<MoveKFS>(this, "move_kfs"));
    init_task_manager("idel");

    task_thread_ = std::make_shared<std::thread>([this]() { porcess_task(); });

    RCLCPP_INFO(node_->get_logger(), "ArmTaskNode初始化完成");
}

Robot::~Robot() {
    shutdown_requested_.store(true);
    task_manager_cv_.notify_all();
    idle_task_signal_.release();

    if (task_thread_ && task_thread_->joinable()) {
        task_thread_->join();
    }

    RCLCPP_INFO(node_->get_logger(), "Shutting down ArmTaskNode...");
}

rclcpp_action::GoalResponse Robot::on_handle_goal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const ArmTaskGoal> goal) {
    (void)uuid;

    std::lock_guard<std::mutex> lock(action_state_mutex_);
    if (task_executing_ || goal_pending_) {
        RCLCPP_WARN(node_->get_logger(), "当前已有任务正在执行或等待调度，拒绝新的目标");
        return rclcpp_action::GoalResponse::REJECT;
    }

    expected_task_id_ = goal->task_id;
    expected_task_data_ = goal->data;
    goal_pending_ = true;

    RCLCPP_INFO(
        node_->get_logger(), "接收到新动作请求: task_id=%d, data_size=%zu", expected_task_id_, expected_task_data_.size());

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Robot::on_cancel_goal(const std::shared_ptr<ArmTaskGoalHandle> goal_handle) {
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void Robot::on_handle_accepted(const std::shared_ptr<ArmTaskGoalHandle> goal_handle) {
    bool should_notify_idle = false;
    {
        std::lock_guard<std::mutex> lock(action_state_mutex_);
        pending_goal_handle_ = goal_handle;
        should_notify_idle = !task_executing_ && goal_pending_;
    }

    if (should_notify_idle) {
        RCLCPP_INFO(node_->get_logger(), "动作目标已接受，通知 idel 任务开始分发");
        idle_task_signal_.release();
    }
}

bool Robot::wait_for_idle_signal(const std::chrono::milliseconds timeout) {
    return idle_task_signal_.try_acquire_for(timeout);
}

bool Robot::take_pending_task(PendingTaskRequest& request) {
    std::lock_guard<std::mutex> lock(action_state_mutex_);
    if (!goal_pending_ || !pending_goal_handle_) {
        return false;
    }

    request.task_id = expected_task_id_;
    request.data = expected_task_data_;
    request.goal_handle = pending_goal_handle_;

    active_task_context_.task_id = request.task_id;
    active_task_context_.data = request.data;
    active_task_context_.goal_handle = request.goal_handle;
    has_active_task_context_ = true;

    goal_pending_ = false;
    task_executing_ = true;
    return true;
}

bool Robot::get_active_task_context(ActiveTaskContext& context) const {
    std::lock_guard<std::mutex> lock(action_state_mutex_);
    if (!has_active_task_context_) {
        return false;
    }

    context = active_task_context_;
    return true;
}

void Robot::finish_current_task(
    const std::shared_ptr<ArmTaskGoalHandle>& goal_handle, const bool success, const std::string& reason) {
    auto result = std::make_shared<ArmTaskResult>();
    result->err_code = success ? 0 : -1;
    result->reason = reason;

    {
        std::lock_guard<std::mutex> lock(action_state_mutex_);
        task_executing_ = false;
        has_active_task_context_ = false;
        active_task_context_.task_id = 0;
        active_task_context_.data.clear();
        active_task_context_.goal_handle.reset();

        if (pending_goal_handle_ == goal_handle) {
            pending_goal_handle_.reset();
        }
    }

    if (!goal_handle) {
        RCLCPP_WARN(node_->get_logger(), "任务结束时 goal_handle 为空");
        return;
    }

    if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        return;
    }

    if (success) {
        goal_handle->succeed(result);
    } else {
        goal_handle->abort(result);
    }
}

void Robot::load_arm_positions_from_yaml() {
    try {
        std::string package_share = ament_index_cpp::get_package_share_directory("arm_task");
        std::string yaml_path     = package_share + "/config/arm_position.yaml";
        load_named_joint_positions_from_yaml(yaml_path);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load arm positions: %s", e.what());
    }
}

void Robot::porcess_task() {
    while (rclcpp::ok() && !shutdown_requested_.load()) {
        std::shared_ptr<BaseTask> current_task;
        std::string current_task_name;
        std::string previous_task_name;

        {
            std::unique_lock<std::mutex> lock(task_manager_mutex_);
            task_manager_cv_.wait_for(lock, std::chrono::milliseconds(100), [this]() {
                return shutdown_requested_.load() || !rclcpp::ok() || (task_manager_initialized_ && !current_task_name_.empty());
            });

            if (!rclcpp::ok() || shutdown_requested_.load()) {
                return;
            }

            if (current_task_name_.empty()) {
                continue;
            }

            auto task_it = task_table_.find(current_task_name_);
            if (task_it == task_table_.end()) {
                RCLCPP_ERROR(node_->get_logger(), "当前任务 [%s] 未注册，任务调度暂停", current_task_name_.c_str());
                current_task_name_.clear();
                continue;
            }

            current_task       = task_it->second;
            current_task_name  = current_task_name_;
            previous_task_name = last_task_name_;
        }

        std::string next_task_name;
        try {
            next_task_name = current_task->process(previous_task_name);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "执行任务 [%s] 时发生异常: %s", current_task_name.c_str(), e.what());
            next_task_name.clear();
        } catch (...) {
            RCLCPP_ERROR(node_->get_logger(), "执行任务 [%s] 时发生未知异常", current_task_name.c_str());
            next_task_name.clear();
        }

        {
            std::lock_guard<std::mutex> lock(task_manager_mutex_);
            last_task_name_ = current_task_name;

            if (next_task_name.empty()) {
                RCLCPP_INFO(node_->get_logger(), "任务 [%s] 执行完成，当前没有后续任务", current_task_name.c_str());
                current_task_name_.clear();
                continue;
            }

            if (task_table_.find(next_task_name) == task_table_.end()) {
                RCLCPP_ERROR(
                    node_->get_logger(), "任务 [%s] 请求切换到未注册任务 [%s]，调度保持等待", current_task_name.c_str(),
                    next_task_name.c_str());
                current_task_name_.clear();
                continue;
            }

            if (next_task_name != current_task_name) {
                RCLCPP_INFO(node_->get_logger(), "任务切换: [%s] -> [%s]", current_task_name.c_str(), next_task_name.c_str());
            }
            current_task_name_ = std::move(next_task_name);
        }
    }
}

void Robot::register_task(std::shared_ptr<BaseTask> task_ptr) {
    if (!task_ptr) {
        RCLCPP_ERROR(node_->get_logger(), "注册任务失败: task_ptr 为空");
        return;
    }

    const std::string task_name = task_ptr->task_name;
    if (task_name.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "注册任务失败: 任务名为空");
        return;
    }

    {
        std::lock_guard<std::mutex> lock(task_manager_mutex_);
        auto [it, inserted] = task_table_.insert_or_assign(task_name, std::move(task_ptr));
        (void)it;
        if (inserted) {
            RCLCPP_INFO(node_->get_logger(), "注册任务成功: [%s]", task_name.c_str());
        } else {
            RCLCPP_WARN(node_->get_logger(), "任务 [%s] 已存在，已更新为新的任务实现", task_name.c_str());
        }
    }

    task_manager_cv_.notify_all();
}

void Robot::init_task_manager(const std::string first_task_name) {
    std::lock_guard<std::mutex> lock(task_manager_mutex_);
    if (task_table_.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "初始化任务调度器失败: 当前没有已注册任务");
        task_manager_initialized_ = false;
        current_task_name_.clear();
        last_task_name_.clear();
        return;
    }

    std::string initial_task_name = first_task_name;
    if (initial_task_name.empty()) {
        initial_task_name = task_table_.begin()->first;
        RCLCPP_WARN(node_->get_logger(), "未指定初始任务，默认使用已注册的首个任务 [%s]", initial_task_name.c_str());
    }

    if (task_table_.find(initial_task_name) == task_table_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "初始化任务调度器失败: 初始任务 [%s] 未注册", initial_task_name.c_str());
        task_manager_initialized_ = false;
        current_task_name_.clear();
        last_task_name_.clear();
        return;
    }

    current_task_name_ = std::move(initial_task_name);
    last_task_name_.clear();
    task_manager_initialized_ = true;

    RCLCPP_INFO(node_->get_logger(), "任务调度器初始化完成，初始任务为 [%s]", current_task_name_.c_str());
    task_manager_cv_.notify_all();
}


rcl_interfaces::msg::SetParametersResult Robot::on_parameters_changed(const std::vector<rclcpp::Parameter>& params) {
    (void)params;

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    return result;
}



void Robot::stop_arm_motion() {
    RCLCPP_INFO(node_->get_logger(), "Stopping arm motion");
    if (arm_calc_param_client_->wait_for_service(5s)) {
        arm_calc_param_client_->set_parameters({rclcpp::Parameter("execute_trajectory", false)});
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Parameter service for %s not available", arm_calc_node_name_.c_str());
    }
}

bool Robot::execute_joint_space_trajectory(const std::vector<double>& joint_angles, double duration) {
    RCLCPP_INFO(node_->get_logger(), "Executing joint space trajectory");

    // Publish joint target
    std_msgs::msg::Float64MultiArray msg;
    msg.data = joint_angles;
    joint_space_target_pub_->publish(msg);

    // std::this_thread::sleep_for(100ms);

    // Set parameters on arm_calc
    if (!arm_calc_param_client_->wait_for_service(5s)) {
        RCLCPP_ERROR(node_->get_logger(), "Parameter service for %s not available", arm_calc_node_name_.c_str());
        return false;
    }

    arm_calc_param_client_->set_parameters(
        {rclcpp::Parameter("trajectory_duration", duration), rclcpp::Parameter("motion_mode", 1),
         rclcpp::Parameter("execute_trajectory", true)});

    const auto timeout_sec = duration + 2.0;
    const auto start_time  = std::chrono::steady_clock::now();
    bool observed_running  = false;

    while (!shutdown_requested_) {
        const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
        if (elapsed > timeout_sec) {
            RCLCPP_WARN(
                node_->get_logger(), "Timed out waiting for %s joint trajectory completion after %.2f s", arm_calc_node_name_.c_str(),
                timeout_sec);
            return false;
        }

        auto future              = arm_calc_param_client_->get_parameters({"execute_trajectory"});
        const auto future_status = future.wait_for(200ms);
        if (future_status != std::future_status::ready) {
            std::this_thread::sleep_for(50ms);
            continue;
        }

        bool execute_trajectory = false;
        try {
            const auto result = future.get();
            if (!result.empty() && result.front().get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
                execute_trajectory = result.front().as_bool();
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(node_->get_logger(), "Failed to query %s execute_trajectory: %s", arm_calc_node_name_.c_str(), e.what());
            std::this_thread::sleep_for(50ms);
            continue;
        }

        if (execute_trajectory) {
            observed_running = true;
        } else if (observed_running) {
            return true;
        }

        std::this_thread::sleep_for(50ms);
    }

    return false;
}

bool Robot::execute_cartesian_space_trajectory(const geometry_msgs::msg::PoseStamped& target_pose, double duration) {

    RCLCPP_INFO(node_->get_logger(), "Executing Cartesian space trajectory");

    // Publish visual target
    visual_target_pub_->publish(target_pose);

    // std::this_thread::sleep_for(100ms);

    // Set parameters on arm_calc
    if (!arm_calc_param_client_->wait_for_service(5s)) {
        RCLCPP_ERROR(node_->get_logger(), "Parameter service for %s not available", arm_calc_node_name_.c_str());
        return false;
    }

    arm_calc_param_client_->set_parameters({rclcpp::Parameter("trajectory_duration", duration), rclcpp::Parameter("motion_mode", 2)});

    // std::this_thread::sleep_for(100ms);

    arm_calc_param_client_->set_parameters({rclcpp::Parameter("execute_trajectory", true)});

    const auto timeout_sec = duration + 2.0;
    const auto start_time  = std::chrono::steady_clock::now();
    bool observed_running  = false;

    while (!shutdown_requested_) {
        const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
        if (elapsed > timeout_sec) {
            RCLCPP_WARN(
                node_->get_logger(), "Timed out waiting for %s Cartesian trajectory completion after %.2f s", arm_calc_node_name_.c_str(),
                timeout_sec);
            return false;
        }

        auto future              = arm_calc_param_client_->get_parameters({"execute_trajectory"});
        const auto future_status = future.wait_for(200ms);
        if (future_status != std::future_status::ready) {
            std::this_thread::sleep_for(50ms);
            continue;
        }

        bool execute_trajectory = false;
        try {
            const auto result = future.get();
            if (!result.empty() && result.front().get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
                execute_trajectory = result.front().as_bool();
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(node_->get_logger(), "Failed to query %s execute_trajectory: %s", arm_calc_node_name_.c_str(), e.what());
            std::this_thread::sleep_for(50ms);
            continue;
        }

        if (execute_trajectory) {
            observed_running = true;
        } else if (observed_running) {
            return true;
        }

        std::this_thread::sleep_for(50ms);
    }

    return false;
}

void Robot::execute_visual_servo(const geometry_msgs::msg::Twist& velocity) { (void)velocity; }

bool Robot::set_air_pump(const bool& enable) {
    if (arm_calc_param_client_->wait_for_service(std::chrono::duration<double>(0.5))) {
        arm_calc_param_client_->set_parameters({rclcpp::Parameter("enable_air_pump", enable)});
        return true;
    } else {
        RCLCPP_WARN(node_->get_logger(), "%s 的参数服务不可用", arm_calc_node_name_.c_str());
        return false;
    }
}

bool Robot::set_grasp_state(const bool& finished) {
    if (!driver_param_client_ || !driver_param_client_->wait_for_service(std::chrono::duration<double>(0.5))) {
        RCLCPP_WARN(node_->get_logger(), "%s 的参数服务不可用", driver_node_name_.c_str());
        return false;
    }
    driver_param_client_->set_parameters({rclcpp::Parameter("grasp_state", finished ? 1 : 0)});
    return true;
}

bool Robot::get_named_joint_position(const std::string& name, std::vector<double>& joint_angles) const {
    const auto it = arm_positions_.find(name);
    if (it == arm_positions_.end()) {
        return false;
    }

    joint_angles = it->second;
    return true;
}

void Robot::load_named_joint_positions_from_yaml(const std::string& yaml_path) {
    YAML::Node config = YAML::LoadFile(yaml_path);

    arm_positions_.clear();
    ready_position_.clear();

    YAML::Node arm_positions_node;
    if (config["arm_positions"]) {
        arm_positions_node = config["arm_positions"];
    } else if (config["arm_position"]) {
        arm_positions_node = config["arm_position"];
        RCLCPP_WARN(
            node_->get_logger(), "配置文件 %s 使用了旧键名 [arm_position]，建议改为 [arm_positions]",
            yaml_path.c_str());
    } else {
        RCLCPP_WARN(node_->get_logger(), "配置文件 %s 中未找到 [arm_positions] 或 [arm_position]", yaml_path.c_str());
        return;
    }

    if (arm_positions_node.IsMap()) {
        for (const auto& entry : arm_positions_node) {
            const std::string name           = entry.first.as<std::string>();
            const std::vector<double> joints = entry.second.as<std::vector<double>>();
            arm_positions_[name]             = joints;
            RCLCPP_INFO(node_->get_logger(), "Loaded named position [%s] with %zu joints", name.c_str(), joints.size());
        }
    } else if (arm_positions_node.IsSequence()) {
        for (const auto& pos : arm_positions_node) {
            if (!pos["name"] || !pos["joints"]) {
                RCLCPP_WARN(node_->get_logger(), "Skip invalid arm position entry in %s", yaml_path.c_str());
                continue;
            }

            const std::string name           = pos["name"].as<std::string>();
            const std::vector<double> joints = pos["joints"].as<std::vector<double>>();
            arm_positions_[name]             = joints;
            RCLCPP_INFO(node_->get_logger(), "Loaded named position [%s] with %zu joints", name.c_str(), joints.size());
        }
    } else {
        RCLCPP_WARN(node_->get_logger(), "Node [arm_positions] in %s is neither a map nor a sequence", yaml_path.c_str());
    }

    const auto ready_it = arm_positions_.find("ready");
    if (ready_it != arm_positions_.end()) {
        ready_position_ = ready_it->second;
    } else {
        RCLCPP_WARN(node_->get_logger(), "已加载命名位姿，但未找到 [ready] 配置");
    }
}

bool Robot::get_current_end_pose(geometry_msgs::msg::PoseStamped& current_pose) {
    try {
        geometry_msgs::msg::TransformStamped transform;
        transform = tf_buffer_->lookupTransform(
            base_frame_, tip_frame_, tf2::TimePointZero, std::chrono::milliseconds(500));
        
        current_pose.header = transform.header;
        current_pose.pose.position.x = transform.transform.translation.x;
        current_pose.pose.position.y = transform.transform.translation.y;
        current_pose.pose.position.z = transform.transform.translation.z;
        current_pose.pose.orientation = transform.transform.rotation;
        
        return true;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(node_->get_logger(), "获取当前末端位姿失败: %s", ex.what());
        return false;
    }
}

double Robot::calculate_duration(const geometry_msgs::msg::PoseStamped& target_pose) {
    // 获取当前末端位姿
    geometry_msgs::msg::PoseStamped current_pose;
    if (!get_current_end_pose(current_pose)) {
        RCLCPP_WARN(node_->get_logger(), "无法获取当前位姿，使用默认时间 %.2f s", trajectory_duration_);
        return trajectory_duration_;
    }

    // 计算位置距离（欧氏距离）
    double dx = target_pose.pose.position.x - current_pose.pose.position.x;
    double dy = target_pose.pose.position.y - current_pose.pose.position.y;
    double dz = target_pose.pose.position.z - current_pose.pose.position.z;
    double linear_distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    // 计算姿态距离（四元数角度差）
    tf2::Quaternion q_current, q_target;
    tf2::fromMsg(current_pose.pose.orientation, q_current);
    tf2::fromMsg(target_pose.pose.orientation, q_target);
    
    // 计算四元数角度差
    double angle_diff = q_current.angleShortestPath(q_target);

    // 根据距离和速度计算时间
    double linear_time = linear_distance / max_linear_velocity_;
    double angular_time = angle_diff / max_angular_velocity_;
    
    // 取较大值作为总时间，并限制在合理范围内
    double duration = std::max(linear_time, angular_time);
    duration = std::clamp(duration, min_trajectory_duration_, max_trajectory_duration_);

    RCLCPP_INFO(
        node_->get_logger(),
        "计算轨迹时间: 线性距离=%.3f m, 角度差=%.3f rad, 时间=%.2f s",
        linear_distance, angle_diff, duration);

    return duration;
}

double Robot::calculate_duration(const std::vector<double>& target_joints) {
    if (target_joints.empty()) {
        RCLCPP_WARN(node_->get_logger(), "目标关节角度为空，使用默认时间 %.2f s", trajectory_duration_);
        return trajectory_duration_;
    }

    // 获取当前末端位姿
    geometry_msgs::msg::PoseStamped current_pose;
    if (!get_current_end_pose(current_pose)) {
        RCLCPP_WARN(node_->get_logger(), "无法获取当前位姿，使用默认时间 %.2f s", trajectory_duration_);
        return trajectory_duration_;
    }

    // 通过正运动学计算目标关节对应的笛卡尔位姿
    // 注意：这里需要调用 arm_calc 服务的正运动学功能
    // 由于当前架构限制，我们使用简化的方法：计算关节角度差
    
    // 获取当前关节角度（从TF或其他方式）
    // 这里暂时使用简化的距离计算方法
    // 实际应用中应该调用正运动学服务
    
    // 方法1：基于关节角度差估算（简化方法）
    // 假设从参数或其他途径获取当前关节角度
    // 这里我们先使用笛卡尔距离的近似方法
    
    // 由于没有直接获取当前关节角度的方法，我们退回到使用默认轨迹时间
    // 但可以基于目标关节角度的变化幅度做简单估算
    
    // 计算关节角度变化，找出最大变化量
    const double joint_change_threshold = 0.1;  // 0.1 rad ≈ 5.7度
    double max_joint_change = 0.0;
    int significant_change_count = 0;
    
    for (const auto& angle : target_joints) {
        double change = std::abs(angle);
        if (change > joint_change_threshold) {
            significant_change_count++;
            if (change > max_joint_change) {
                max_joint_change = change;
            }
        }
    }
    
    // 如果没有显著变化的关节，使用最小轨迹时间
    if (significant_change_count == 0) {
        RCLCPP_INFO(node_->get_logger(), "所有关节变化都很小，使用最小轨迹时间 %.2f s", min_trajectory_duration_);
        return min_trajectory_duration_;
    }
    
    // 根据最大关节变化估算时间（运动时间取决于最慢的关节）
    double estimated_time = max_joint_change / max_joint_velocity_;
    RCLCPP_INFO(node_->get_logger(), "max_joint_velocity = %lf", max_joint_velocity_);
    estimated_time = std::clamp(estimated_time, min_trajectory_duration_, max_trajectory_duration_);
    
    RCLCPP_INFO(
        node_->get_logger(),
        "计算轨迹时间（关节空间）: 显著变化关节数=%d, 最大关节变化=%.3f rad, 时间=%.2f s",
        significant_change_count, max_joint_change, estimated_time);
    
    return estimated_time;
}

double Robot::calculate_duration(const std::string& named_position) {
    // 从命名位置中查找关节角度
    std::vector<double> target_joints;
    if (!get_named_joint_position(named_position, target_joints)) {
        RCLCPP_WARN(
            node_->get_logger(),
            "未找到命名位置 [%s]，使用默认时间 %.2f s",
            named_position.c_str(), trajectory_duration_);
        return trajectory_duration_;
    }

    RCLCPP_INFO(
        node_->get_logger(),
        "计算到命名位置 [%s] 的轨迹时间",
        named_position.c_str());

    return calculate_duration(target_joints);
}










