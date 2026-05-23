#include "arm_calc/arm_ctrl.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace arm_calc {

namespace {

constexpr char kJointStateTopic[] = "myjoints_state";
constexpr char kJointTargetTopic[] = "myjoints_target";
constexpr char kVisualTargetTopic[] = "visual_target_pose";
constexpr char kJointSpaceTargetTopic[] = "joint_space_target";
constexpr char kRvizJointTopic[] = "joint_states";
constexpr char kMarkerTopic[] = "visualization_marker_array";

geometry_msgs::msg::Point ToPoint(const Eigen::Vector3d& value) {
    geometry_msgs::msg::Point point;
    point.x = value.x();
    point.y = value.y();
    point.z = value.z();
    return point;
}

geometry_msgs::msg::Quaternion ToMsgQuaternion(const Eigen::Quaterniond& q) {
    geometry_msgs::msg::Quaternion msg;
    msg.w = q.w();
    msg.x = q.x();
    msg.y = q.y();
    msg.z = q.z();
    return msg;
}

JointTrajectoryPoint BuildStaticJointTarget(const std::shared_ptr<ArmCalc>& arm_calc, const JointVector& position) {
    JointTrajectoryPoint point;
    point.position = position;
    point.velocity.setZero();
    point.acceleration.setZero();
    point.torque = arm_calc->joint_torque_inverse_dynamics(position, JointVector::Zero(), JointVector::Zero());
    return point;
}

}  // namespace

ArmCtrlNode::ArmCtrlNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("arm_calc_node", options) {
    declare_parameters();
    load_robot_description_and_build_solver();
    create_interfaces();
    param_callback_ = this->add_on_set_parameters_callback(
        std::bind(&ArmCtrlNode::on_parameters_changed, this, std::placeholders::_1));
}

void ArmCtrlNode::declare_parameters() {
    this->declare_parameter<int>("motion_mode", 0);
    this->declare_parameter<double>("trajectory_duration", 3.0);
    this->declare_parameter<double>("control_period", 0.02);
    this->declare_parameter<bool>("execute_trajectory", false);
    this->declare_parameter<double>("visual_servo_kp", 2.0);
    this->declare_parameter<double>("visual_servo_max_linear_acceleration", 0.5);
    this->declare_parameter<std::string>("base_link", "base_link");
    this->declare_parameter<std::string>("tip_link", "link6");
    this->declare_parameter<std::vector<double>>("joint_target", std::vector<double>(kJointDoF, 0.0));
    this->declare_parameter<std::vector<double>>("cartesian_target_position", std::vector<double>{0.7, 0.0, 0.15});
    this->declare_parameter<std::vector<double>>("cartesian_target_quaternion", std::vector<double>{1.0, 0.0, 0.0, 0.0});
}

void ArmCtrlNode::create_interfaces() {
    joint_state_sub_ = this->create_subscription<robot_interfaces::msg::Arm>(
        kJointStateTopic, rclcpp::SensorDataQoS(),
        std::bind(&ArmCtrlNode::on_joint_state, this, std::placeholders::_1));

    visual_target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        kVisualTargetTopic, 10, std::bind(&ArmCtrlNode::on_visual_target, this, std::placeholders::_1));
    joint_space_target_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        kJointSpaceTargetTopic, 10, std::bind(&ArmCtrlNode::on_joint_space_target, this, std::placeholders::_1));

    joint_target_pub_ = this->create_publisher<robot_interfaces::msg::Arm>(kJointTargetTopic, 10);
    rviz_joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(kRvizJointTopic, 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(kMarkerTopic, 10);

    control_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(control_period_sec_), std::bind(&ArmCtrlNode::publish_control_loop, this));
}

void ArmCtrlNode::load_robot_description_and_build_solver() {
    base_link_ = this->get_parameter("base_link").as_string();
    tip_link_ = this->get_parameter("tip_link").as_string();
    trajectory_duration_sec_ = this->get_parameter("trajectory_duration").as_double();
    control_period_sec_ = this->get_parameter("control_period").as_double();
    execute_trajectory_ = this->get_parameter("execute_trajectory").as_bool();
    //visual_servo_kp_ = std::max(this->get_parameter("visual_servo_kp").as_double(), 0.0);
    visual_servo_max_linear_acceleration_ =
        std::max(this->get_parameter("visual_servo_max_linear_acceleration").as_double(), 0.0);
    requested_motion_mode_ = parse_motion_mode(this->get_parameter("motion_mode").as_int());
    active_motion_mode_ = MotionMode::kIdle;

    const std::vector<double> joint_target = get_double_array_param(*this, "joint_target", kJointDoF);
    for (std::size_t i = 0; i < kJointDoF; ++i) {
        joint_target_state_.position[static_cast<int>(i)] = joint_target[i];
    }

    const std::vector<double> cartesian_position = get_double_array_param(*this, "cartesian_target_position", 3);
    const std::vector<double> cartesian_quaternion = get_double_array_param(*this, "cartesian_target_quaternion", 4);
    cartesian_target_.position = Eigen::Vector3d(cartesian_position[0], cartesian_position[1], cartesian_position[2]);
    cartesian_target_.orientation = NormalizeQuaternion(Eigen::Quaterniond(
        cartesian_quaternion[0], cartesian_quaternion[1], cartesian_quaternion[2], cartesian_quaternion[3]));
    visual_target_ = cartesian_target_;

    KDL::Tree tree;
    const std::string urdf_xml = fetch_robot_description();
    if (!kdl_parser::treeFromString(urdf_xml, tree)) {
        throw std::runtime_error("failed to parse arm URDF into KDL tree");
    }
    if (!tree.getChain(base_link_, tip_link_, arm_chain_)) {
        throw std::runtime_error("failed to build KDL chain from " + base_link_ + " to " + tip_link_);
    }

    arm_calc_ = std::make_shared<ArmCalc>(arm_chain_);
    joint_space_move_ = std::make_shared<arm_action::JointSpaceMove>(arm_calc_);
    cartesian_space_move_ = std::make_shared<arm_action::JCartesianSpaceMove>(arm_calc_);
    visual_servo_move_ = std::make_shared<arm_action::VisualServoMove>(arm_calc_);
    //visual_servo_move_->set_kp(visual_servo_kp_);
    visual_servo_move_->set_max_linear_acceleration(visual_servo_max_linear_acceleration_);
}

std::string ArmCtrlNode::fetch_robot_description() const {
    auto client = std::make_shared<rclcpp::SyncParametersClient>(
        const_cast<ArmCtrlNode*>(this), "/robot_state_publisher");

    if (client->wait_for_service(std::chrono::seconds(1))) {
        try {
            const auto params = client->get_parameters({"robot_description"});
            if (!params.empty()) {
                const std::string urdf_xml = params.front().as_string();
                if (!urdf_xml.empty()) {
                    RCLCPP_INFO(this->get_logger(), "Loaded robot_description from /robot_state_publisher");
                    return urdf_xml;
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to fetch robot_description from parameter service: %s", e.what());
        }
    }

    RCLCPP_WARN(this->get_logger(), "Falling back to local URDF file");
    return load_local_urdf();
}

std::string ArmCtrlNode::load_local_urdf() const {
    const std::string arm_share = ament_index_cpp::get_package_share_directory("arm");
    const std::string urdf_path = arm_share + "/model/robotic_arm.urdf";
    std::ifstream input(urdf_path);
    if (!input.is_open()) {
        throw std::runtime_error("unable to open local URDF: " + urdf_path);
    }

    std::ostringstream buffer;
    buffer << input.rdbuf();
    return buffer.str();
}

void ArmCtrlNode::refresh_plan(double now_sec) {
    if (!arm_calc_ || !has_joint_state_) {
        return;
    }

    switch (active_motion_mode_) {
        case MotionMode::kIdle:
            enter_idle_mode();
            RCLCPP_INFO(get_logger(),"进入IDEL模式");
            break;
        case MotionMode::kJointSpace:
            joint_space_move_->set_start_state(current_joint_state_);
            joint_space_move_->set_goal_state(joint_target_state_, trajectory_duration_sec_);
            if (execute_trajectory_) {
                joint_space_move_->start(now_sec);
            }
            RCLCPP_INFO(get_logger(),"进入关节空间轨迹执行模式");
            break;
        case MotionMode::kCartesianSpace:
            cartesian_space_move_->set_start_state(current_joint_state_);
            cartesian_space_move_->set_goal_state(cartesian_target_, trajectory_duration_sec_);
            if (execute_trajectory_) {
                cartesian_space_move_->start(now_sec);
            }
            RCLCPP_INFO(get_logger(),"进入笛卡尔空间轨迹执行模式");
            break;
        case MotionMode::kVisualServo:
            //visual_servo_move_->set_kp(visual_servo_kp_);
            visual_servo_move_->set_max_linear_acceleration(visual_servo_max_linear_acceleration_);
            visual_servo_move_->start(current_joint_state_, now_sec);
            visual_servo_move_->set_target_pose(visual_target_);
            RCLCPP_INFO(get_logger(),"进入视觉伺服模式");
            break;
    }
    planners_ready_ = true;
}

void ArmCtrlNode::capture_idle_hold_from_current_state() {
    idle_hold_point_.position = current_joint_state_.position;
    idle_hold_point_.velocity.setZero();
    idle_hold_point_.acceleration.setZero();
    idle_hold_point_.torque = arm_calc_->joint_torque_inverse_dynamics(
        idle_hold_point_.position, JointVector::Zero(), JointVector::Zero());
    idle_hold_initialized_ = true;
}

void ArmCtrlNode::set_idle_hold_point(const JointTrajectoryPoint& point) {
    idle_hold_point_ = point;
    idle_hold_point_.velocity.setZero();
    idle_hold_point_.acceleration.setZero();
    idle_hold_point_.torque = arm_calc_->joint_torque_inverse_dynamics(
        idle_hold_point_.position, JointVector::Zero(), JointVector::Zero());
    idle_hold_initialized_ = true;
}

void ArmCtrlNode::apply_requested_mode(double now_sec) {
    if (!has_joint_state_) {
        return;
    }

    if (!execute_trajectory_) {
        if (active_motion_mode_ != MotionMode::kIdle || !idle_hold_initialized_) {
            capture_idle_hold_from_current_state();
        }
        active_motion_mode_ = MotionMode::kIdle;
        enter_idle_mode();
        return;
    }

    if (active_motion_mode_ == requested_motion_mode_) {
        if (active_motion_mode_ == MotionMode::kIdle && requested_motion_mode_ != MotionMode::kIdle) {
            active_motion_mode_ = requested_motion_mode_;
            refresh_plan(now_sec);
        }
        return;
    }

    if (can_switch_mode_immediately() || !is_trajectory_running(now_sec)) {
        active_motion_mode_ = requested_motion_mode_;
        refresh_plan(now_sec);
    }
}

void ArmCtrlNode::enter_idle_mode() {
    if (!idle_hold_initialized_) {
        capture_idle_hold_from_current_state();
    }
}

bool ArmCtrlNode::is_trajectory_running(double now_sec) const {
    switch (active_motion_mode_) {
        case MotionMode::kJointSpace:
            return joint_space_move_ && joint_space_move_->started() && joint_space_move_->active(now_sec);
        case MotionMode::kCartesianSpace:
            return cartesian_space_move_ && cartesian_space_move_->started() && cartesian_space_move_->active(now_sec);
        case MotionMode::kVisualServo:
            return execute_trajectory_;
        case MotionMode::kIdle:
        default:
            return false;
    }
}

bool ArmCtrlNode::can_switch_mode_immediately() const {
    return active_motion_mode_ == MotionMode::kIdle || active_motion_mode_ == MotionMode::kVisualServo;
}

void ArmCtrlNode::set_execute_trajectory_flag(bool value) {
    if (execute_trajectory_ == value) {
        return;
    }

    execute_trajectory_ = value;
    updating_execute_parameter_ = true;
    this->set_parameter(rclcpp::Parameter("execute_trajectory", value));
    updating_execute_parameter_ = false;
}

JointTrajectoryPoint ArmCtrlNode::build_preview_target() const {
    switch (requested_motion_mode_) {
        case MotionMode::kJointSpace:
            return BuildStaticJointTarget(arm_calc_, joint_target_state_.position);
        case MotionMode::kCartesianSpace: {
            int result = -1;
            const JointVector seed = has_joint_state_ ? current_joint_state_.position : idle_hold_point_.position;
            const JointVector preview_position = arm_calc_->joint_pos(cartesian_target_, &result, seed);
            if (result < 0) {
                RCLCPP_WARN(this->get_logger(), "Failed to solve IK for Cartesian preview target, keeping current display");
                return idle_hold_point_;
            }
            return BuildStaticJointTarget(arm_calc_, preview_position);
        }
        case MotionMode::kVisualServo: {
            int result = -1;
            const JointVector seed = has_joint_state_ ? current_joint_state_.position : idle_hold_point_.position;

            tf2::Quaternion q;
            q.setRPY(0.0, 1.57, 0.0);
            CartesianPose visual_preview_target = visual_target_;
            visual_preview_target.orientation = NormalizeQuaternion(
                Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z()));

            const JointVector preview_position = arm_calc_->joint_pos(visual_preview_target, &result, seed);
            if (result < 0) {
                RCLCPP_WARN(this->get_logger(), "Failed to solve IK for visual target preview, keeping current display");
                return idle_hold_point_;
            }
            return BuildStaticJointTarget(arm_calc_, preview_position);
        }
        case MotionMode::kIdle:
        default:
            return idle_hold_point_;
    }
}

void ArmCtrlNode::publish_control_loop() {
    if (!has_joint_state_ || !planners_ready_) {
        return;
    }

    const double now_sec = this->get_clock()->now().seconds();
    if (last_ee_log_time_sec_ < 0.0 || (now_sec - last_ee_log_time_sec_) >= 1.0) {
        const CartesianPose ee_pose = arm_calc_->end_pose(current_joint_state_.position);
        const Eigen::Vector3d ee_rpy = ee_pose.orientation.toRotationMatrix().eulerAngles(0, 1, 2);
        RCLCPP_INFO(
            get_logger(),
            "EE pose pos=(%.4f, %.4f, %.4f) rpy=(%.4f, %.4f, %.4f)",
            ee_pose.position.x(),
            ee_pose.position.y(),
            ee_pose.position.z(),
            ee_rpy.x(),
            ee_rpy.y(),
            ee_rpy.z());
        last_ee_log_time_sec_ = now_sec;
    }

    apply_requested_mode(now_sec);
    JointTrajectoryPoint target_point;

    switch (active_motion_mode_) {
        case MotionMode::kIdle:
            target_point = idle_hold_point_;
            target_point.torque =
                arm_calc_->joint_torque_inverse_dynamics(target_point.position, JointVector::Zero(), JointVector::Zero());
            //RCLCPP_INFO(get_logger(),"target_point=(%lf,%lf,%lf)",target_point.position[0],target_point.position[1],target_point.position[2]);
            break;
        case MotionMode::kJointSpace:
            target_point = joint_space_move_->sample(now_sec);
            if (!joint_space_move_->active(now_sec) && joint_space_move_->started()) {
                set_idle_hold_point(target_point);
                set_execute_trajectory_flag(false);
                active_motion_mode_ = MotionMode::kIdle;
                requested_motion_mode_ = MotionMode::kIdle;
                enter_idle_mode();
                target_point = idle_hold_point_;
            }
            break;
        case MotionMode::kCartesianSpace:
            target_point = cartesian_space_move_->sample(now_sec);
            if (!cartesian_space_move_->active(now_sec) && cartesian_space_move_->started()) {
                set_idle_hold_point(target_point);
                set_execute_trajectory_flag(false);
                active_motion_mode_ = MotionMode::kIdle;
                requested_motion_mode_ = MotionMode::kIdle;
                enter_idle_mode();
                target_point = idle_hold_point_;
            }
            break;
        case MotionMode::kVisualServo:
            visual_servo_move_->set_current_joint_state(current_joint_state_);
            target_point = visual_servo_move_->sample(now_sec);
            break;
    }

    JointTrajectoryPoint rviz_point = execute_trajectory_ ? target_point : build_preview_target();
    publish_joint_target(target_point);
    const rclcpp::Time stamp = this->get_clock()->now();
    rviz_joint_pub_->publish(to_joint_state_msg(rviz_point, stamp));
    publish_visualization(rviz_point);
}

void ArmCtrlNode::publish_joint_target(const JointTrajectoryPoint& point) {
    joint_target_pub_->publish(to_arm_message(point));
    //RCLCPP_INFO(get_logger(),"joint2=(%lf,%lf,%lf)",point.position[1],point.velocity[1],point.torque[1]);
}

void ArmCtrlNode::publish_visualization(const JointTrajectoryPoint& target_point) {
    if (!arm_calc_) {
        return;
    }

    visualization_msgs::msg::MarkerArray markers;
    const rclcpp::Time stamp = this->now();

    visualization_msgs::msg::Marker current_marker;
    current_marker.header.frame_id = base_link_;
    current_marker.header.stamp = stamp;
    current_marker.ns = "arm_ctrl";
    current_marker.id = 0;
    current_marker.type = visualization_msgs::msg::Marker::SPHERE;
    current_marker.action = visualization_msgs::msg::Marker::ADD;
    current_marker.scale.x = 0.04;
    current_marker.scale.y = 0.04;
    current_marker.scale.z = 0.04;
    current_marker.color.r = 0.1F;
    current_marker.color.g = 0.8F;
    current_marker.color.b = 0.2F;
    current_marker.color.a = 1.0F;
    current_marker.pose.position = ToPoint(arm_calc_->end_pose(current_joint_state_.position).position);
    current_marker.pose.orientation.w = 1.0;

    visualization_msgs::msg::Marker target_marker = current_marker;
    target_marker.id = 1;
    target_marker.color.r = 0.95F;
    target_marker.color.g = 0.25F;
    target_marker.color.b = 0.15F;
    target_marker.pose.position = ToPoint(arm_calc_->end_pose(target_point.position).position);
    target_marker.pose.orientation = ToMsgQuaternion(arm_calc_->end_pose(target_point.position).orientation);

    visualization_msgs::msg::Marker line_marker = current_marker;
    line_marker.id = 2;
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.scale.x = 0.01;
    line_marker.color.r = 0.1F;
    line_marker.color.g = 0.4F;
    line_marker.color.b = 0.95F;
    line_marker.points.push_back(current_marker.pose.position);
    line_marker.points.push_back(target_marker.pose.position);

    markers.markers.push_back(current_marker);
    markers.markers.push_back(target_marker);
    markers.markers.push_back(line_marker);
    marker_pub_->publish(markers);
}

void ArmCtrlNode::on_joint_state(const robot_interfaces::msg::Arm& msg) {
    const bool was_initialized = has_joint_state_;
    current_joint_state_ = from_arm_message(msg);
    has_joint_state_ = true;
    if (!was_initialized) {
        capture_idle_hold_from_current_state();
    }

    if (!planners_ready_) {
        refresh_plan(this->get_clock()->now().seconds());
    }
}

void ArmCtrlNode::on_visual_target(const geometry_msgs::msg::PoseStamped& msg) {
    const CartesianPose target_pose{
        Eigen::Vector3d(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z),
        NormalizeQuaternion(Eigen::Quaterniond(
            msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z))};
    visual_target_ = target_pose;
    cartesian_target_ = target_pose;

    if (visual_servo_move_) {
        visual_servo_move_->set_target_pose(visual_target_);
    }

    if (requested_motion_mode_ == MotionMode::kCartesianSpace && has_joint_state_) {
        planners_ready_ = false;
        if (active_motion_mode_ == MotionMode::kCartesianSpace && execute_trajectory_) {
            refresh_plan(this->get_clock()->now().seconds());
        }
    }

    if (requested_motion_mode_ == MotionMode::kVisualServo && has_joint_state_) {
        planners_ready_ = true;
    }
}

void ArmCtrlNode::on_joint_space_target(const std_msgs::msg::Float64MultiArray& msg) {
    if (msg.data.size() < kJointDoF) {
        RCLCPP_WARN(this->get_logger(), "joint_space_target requires at least 6 elements");
        return;
    }

    for (std::size_t i = 0; i < kJointDoF; ++i) {
        joint_target_state_.position[static_cast<int>(i)] = msg.data[i];
    }

    if (has_joint_state_ && active_motion_mode_ == MotionMode::kIdle &&
        requested_motion_mode_ == MotionMode::kJointSpace && execute_trajectory_) {
        active_motion_mode_ = MotionMode::kJointSpace;
        refresh_plan(this->get_clock()->now().seconds());
    }
}

rcl_interfaces::msg::SetParametersResult ArmCtrlNode::on_parameters_changed(const std::vector<rclcpp::Parameter>& params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto& param : params) {
        if (param.get_name() == "motion_mode") {
            requested_motion_mode_ = parse_motion_mode(param.as_int());
        } else if (param.get_name() == "trajectory_duration") {
            trajectory_duration_sec_ = std::max(param.as_double(), 0.1);
        } else if (param.get_name() == "control_period") {
            control_period_sec_ = std::max(param.as_double(), 0.005);
        } else if (param.get_name() == "execute_trajectory") {
            if (!updating_execute_parameter_) {
                execute_trajectory_ = param.as_bool();
            }
            if (!execute_trajectory_) {
                requested_motion_mode_ = MotionMode::kIdle;
                active_motion_mode_ = MotionMode::kIdle;
                enter_idle_mode();
            }
        } else if (param.get_name() == "visual_servo_kp") {
            //visual_servo_kp_ = std::max(param.as_double(), 0.0);
            if (visual_servo_move_) {
                //visual_servo_move_->set_kp(visual_servo_kp_);
            }
        } else if (param.get_name() == "visual_servo_max_linear_acceleration") {
            visual_servo_max_linear_acceleration_ = std::max(param.as_double(), 0.0);
            if (visual_servo_move_) {
                visual_servo_move_->set_max_linear_acceleration(visual_servo_max_linear_acceleration_);
            }
        } else if (param.get_name() == "joint_target") {
            const auto values = param.as_double_array();
            if (values.size() != kJointDoF) {
                result.successful = false;
                result.reason = "joint_target must contain 6 values";
                return result;
            }
            for (std::size_t i = 0; i < kJointDoF; ++i) {
                joint_target_state_.position[static_cast<int>(i)] = values[i];
            }
        } else if (param.get_name() == "cartesian_target_position") {
            const auto values = param.as_double_array();
            if (values.size() != 3) {
                result.successful = false;
                result.reason = "cartesian_target_position must contain 3 values";
                return result;
            }
            cartesian_target_.position = Eigen::Vector3d(values[0], values[1], values[2]);
        } else if (param.get_name() == "cartesian_target_quaternion") {
            const auto values = param.as_double_array();
            if (values.size() != 4) {
                result.successful = false;
                result.reason = "cartesian_target_quaternion must contain 4 values";
                return result;
            }
            cartesian_target_.orientation = NormalizeQuaternion(Eigen::Quaterniond(values[0], values[1], values[2], values[3]));
        }
    }

    if (control_timer_) {
        control_timer_->cancel();
        control_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(control_period_sec_), std::bind(&ArmCtrlNode::publish_control_loop, this));
    }

    planners_ready_ = false;
    if (has_joint_state_) {
        apply_requested_mode(this->get_clock()->now().seconds());
        if (active_motion_mode_ != MotionMode::kIdle) {
            refresh_plan(this->get_clock()->now().seconds());
        } else {
            planners_ready_ = true;
        }
    }
    return result;
}

ArmCtrlNode::MotionMode ArmCtrlNode::parse_motion_mode(int mode_value) {
    if (mode_value == 0) {
        return MotionMode::kIdle;
    }
    if (mode_value == 1) {
        return MotionMode::kJointSpace;
    }
    if (mode_value == 2) {
        return MotionMode::kCartesianSpace;
    }
    if (mode_value == 3) {
        return MotionMode::kVisualServo;
    }
    throw std::invalid_argument("unsupported motion_mode value: " + std::to_string(mode_value));
}

JointState ArmCtrlNode::from_arm_message(const robot_interfaces::msg::Arm& msg) {
    JointState state;
    for (std::size_t i = 0; i < kJointDoF; ++i) {
        state.position[static_cast<int>(i)] = msg.motor[i].rad;
        state.velocity[static_cast<int>(i)] = msg.motor[i].omega;
        state.torque[static_cast<int>(i)] = msg.motor[i].torque;
    }
    return state;
}

robot_interfaces::msg::Arm ArmCtrlNode::to_arm_message(const JointTrajectoryPoint& point) {
    robot_interfaces::msg::Arm msg;
    for (std::size_t i = 0; i < kJointDoF; ++i) {
        msg.motor[i].rad = static_cast<float>(point.position[static_cast<int>(i)]);
        msg.motor[i].omega = static_cast<float>(point.velocity[static_cast<int>(i)]);
        msg.motor[i].torque = static_cast<float>(point.torque[static_cast<int>(i)]);
    }
    return msg;
}

sensor_msgs::msg::JointState ArmCtrlNode::to_joint_state_msg(const JointTrajectoryPoint& point, const rclcpp::Time& stamp) {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = stamp;
    msg.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    msg.position.resize(kJointDoF);
    msg.velocity.resize(kJointDoF);
    msg.effort.resize(kJointDoF);
    for (std::size_t i = 0; i < kJointDoF; ++i) {
        msg.position[i] = point.position[static_cast<int>(i)];
        msg.velocity[i] = point.velocity[static_cast<int>(i)];
        msg.effort[i] = point.torque[static_cast<int>(i)];
    }
    return msg;
}

std::vector<double> ArmCtrlNode::get_double_array_param(const rclcpp::Node& node,
                                                        const std::string& name,
                                                        std::size_t expected_size) {
    const auto values = node.get_parameter(name).as_double_array();
    if (values.size() != expected_size) {
        throw std::runtime_error("parameter " + name + " expected size " + std::to_string(expected_size));
    }
    return values;
}

}  // namespace arm_calc
