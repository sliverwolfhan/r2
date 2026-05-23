#include "arm_action/basic_moves.hpp"

#include <algorithm>
#include <cmath>
#include <rclcpp/logger.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <rclcpp/rclcpp.hpp>

namespace arm_action {

namespace {

constexpr double kVisualServoMinDistanceMeters = 0.1;
constexpr double kVisualServoMaxDistanceMeters = 1.0;
rclcpp::Logger VisualServoLogger() { return rclcpp::get_logger("arm_action.visual_servo_move"); }
rclcpp::Clock& VisualServoThrottleClock() {
    static rclcpp::Clock clock(RCL_STEADY_TIME);
    return clock;
}
Eigen::Vector3d QuaternionToRotationVectorWorld(const Eigen::Quaterniond& start, const Eigen::Quaterniond& goal) {
    Eigen::Quaterniond delta = arm_calc::NormalizeQuaternion(goal) * arm_calc::NormalizeQuaternion(start).conjugate();
    if (delta.w() < 0.0) {
        delta.coeffs() *= -1.0;
    }

    Eigen::AngleAxisd angle_axis(delta);
    if (std::abs(angle_axis.angle()) < 1e-9 || angle_axis.axis().squaredNorm() < 1e-12) {
        return Eigen::Vector3d::Zero();
    }
    return angle_axis.axis() * angle_axis.angle();
}

Eigen::Vector3d ClampVectorNorm(const Eigen::Vector3d& vector, double limit) {
    if (limit <= 0.0) {
        return Eigen::Vector3d::Zero();
    }
    const double norm = vector.norm();
    if (norm <= limit || norm < 1e-9) {
        return vector;
    }
    return vector * (limit / norm);
}

Eigen::Quaterniond FixedVisualServoOrientation() {
    tf2::Quaternion q;
    q.setRPY(0.0, 1.57, 0.0);
    return arm_calc::NormalizeQuaternion(Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z()));
}

JointTrajectoryPoint BuildJointPoint(
    std::shared_ptr<ArmCalc> arm_calc, const JointVector& position, const JointVector& velocity, const JointVector& acceleration) {
    JointTrajectoryPoint point;
    point.position     = position;
    point.velocity     = velocity;
    point.acceleration = acceleration;
    if (arm_calc) {
        point.torque = arm_calc->joint_torque_inverse_dynamics(position, velocity, acceleration);
    }
    return point;
}

} // namespace

JointSpaceMove::JointSpaceMove(std::shared_ptr<ArmCalc> arm_calc)
    : arm_calc_(std::move(arm_calc)) {}

void JointSpaceMove::set_start_state(const JointState& state) {
    start_state_ = state;
    trajectory_.set_start_state(state.position, state.velocity);
}

void JointSpaceMove::set_goal_state(const JointState& state, double duration) {
    goal_state_   = state;
    duration_sec_ = std::max(duration, 1e-3);
    trajectory_.set_goal_state(state.position, duration_sec_, state.velocity);
}

void JointSpaceMove::start(double start_time_sec) {
    start_time_sec_ = start_time_sec;
    started_        = true;
}

JointTrajectoryPoint JointSpaceMove::sample(double current_time_sec) const {
    if (!started_) {
        return BuildJointPoint(arm_calc_, start_state_.position, start_state_.velocity, JointVector::Zero());
    }

    JointTrajectoryPoint point = trajectory_.sample(current_time_sec - start_time_sec_);
    point.torque               = arm_calc_->joint_torque_inverse_dynamics(point.position, point.velocity, point.acceleration);
    return point;
}

bool JointSpaceMove::active(double current_time_sec) const { return started_ && (current_time_sec - start_time_sec_) <= duration_sec_; }

bool JointSpaceMove::started() const { return started_; }

JCartesianSpaceMove::JCartesianSpaceMove(std::shared_ptr<ArmCalc> arm_calc)
    : arm_calc_(std::move(arm_calc)) {}

void JCartesianSpaceMove::set_start_state(const JointState& state) {
    start_joint_state_ = state;
    if (arm_calc_) {
        start_cartesian_state_ = arm_calc_->end_state(state.position, state.velocity);
    }
    start_orientation_ = arm_calc::NormalizeQuaternion(start_cartesian_state_.pose.orientation);
    goal_orientation_ = start_orientation_;
    orientation_rotation_vector_world_.setZero();

    JointVector pos_start = JointVector::Zero();
    pos_start.head<3>()   = start_cartesian_state_.pose.position;
    JointVector pos_start_velocity = JointVector::Zero();
    pos_start_velocity.head<3>()   = start_cartesian_state_.linear_velocity;
    position_trajectory_.set_start_state(pos_start, pos_start_velocity);

    JointVector sigma_start = JointVector::Zero();
    JointVector sigma_start_velocity = JointVector::Zero();
    const double rotation_norm_sq = orientation_rotation_vector_world_.squaredNorm();
    if (rotation_norm_sq > 1e-12) {
        sigma_start_velocity[0] = start_cartesian_state_.angular_velocity.dot(orientation_rotation_vector_world_) / rotation_norm_sq;
    }
    orientation_trajectory_.set_start_state(sigma_start, sigma_start_velocity);
}

void JCartesianSpaceMove::set_goal_state(const CartesianPose& pose, double duration) {
    goal_pose_    = pose;
    duration_sec_ = std::max(duration, 1e-3);

    JointVector pos_goal = JointVector::Zero();
    pos_goal.head<3>()   = pose.position;
    position_trajectory_.set_goal_state(pos_goal, duration_sec_);

    goal_orientation_ = arm_calc::NormalizeQuaternion(goal_pose_.orientation);
    orientation_rotation_vector_world_ = QuaternionToRotationVectorWorld(start_orientation_, goal_orientation_);
    JointVector sigma_start = JointVector::Zero();
    JointVector sigma_goal = JointVector::Zero();
    sigma_goal[0] = 1.0;
    JointVector sigma_start_velocity = JointVector::Zero();
    const double rotation_norm_sq = orientation_rotation_vector_world_.squaredNorm();
    if (rotation_norm_sq > 1e-12) {
        sigma_start_velocity[0] = start_cartesian_state_.angular_velocity.dot(orientation_rotation_vector_world_) / rotation_norm_sq;
    }
    orientation_trajectory_.set_start_state(sigma_start, sigma_start_velocity);
    orientation_trajectory_.set_goal_state(sigma_goal, duration_sec_);
}

void JCartesianSpaceMove::start(double start_time_sec) {
    start_time_sec_ = start_time_sec;
    started_        = true;
}

JointTrajectoryPoint JCartesianSpaceMove::sample(double current_time_sec) {
    if (!started_ || !arm_calc_) {
        return BuildJointPoint(arm_calc_, start_joint_state_.position, start_joint_state_.velocity, JointVector::Zero());
    }
    return arm_calc_->signal_arm_calc(build_cartesian_target(current_time_sec));
}

bool JCartesianSpaceMove::active(double current_time_sec) const {
    return started_ && (current_time_sec - start_time_sec_) <= duration_sec_;
}

bool JCartesianSpaceMove::started() const { return started_; }

CartesianTrajectoryPoint JCartesianSpaceMove::build_cartesian_target(double current_time_sec) const {
    const double elapsed                  = std::clamp(current_time_sec - start_time_sec_, 0.0, duration_sec_);
    const JointTrajectoryPoint pos_sample = position_trajectory_.sample(elapsed);
    const JointTrajectoryPoint sigma_sample = orientation_trajectory_.sample(elapsed);

    CartesianTrajectoryPoint target;
    target.pose.position       = pos_sample.position.head<3>();
    target.linear_velocity     = pos_sample.velocity.head<3>();
    target.linear_acceleration = pos_sample.acceleration.head<3>();

    const double sigma = sigma_sample.position[0];
    const double sigma_dot = sigma_sample.velocity[0];
    const double sigma_ddot = sigma_sample.acceleration[0];
    target.pose.orientation = arm_calc::NormalizeQuaternion(start_orientation_.slerp(sigma, goal_orientation_));
    target.angular_velocity = orientation_rotation_vector_world_ * sigma_dot;
    target.angular_acceleration = orientation_rotation_vector_world_ * sigma_ddot;
    return target;
}

VisualServoMove::VisualServoMove(std::shared_ptr<ArmCalc> arm_calc)
    : arm_calc_(std::move(arm_calc)) {}

void VisualServoMove::start(const JointState& state, double start_time_sec) {
    current_joint_state_ = state;
    has_joint_state_     = true;

    if (!arm_calc_) {
        servo_initialized_ = false;
        return;
    }

    current_cartesian_state_ = arm_calc_->end_state(state.position, state.velocity);
    desired_position_        = current_cartesian_state_.pose.position;
    desired_velocity_        = current_cartesian_state_.linear_velocity;
    desired_acceleration_.setZero();
    desired_joint_seed_   = state.position;
    last_sample_time_sec_ = start_time_sec;
    servo_initialized_    = true;
}

void VisualServoMove::set_current_joint_state(const JointState& state) {
    current_joint_state_ = state;
    has_joint_state_     = true;
    if (arm_calc_) {
        current_cartesian_state_ = arm_calc_->end_state(state.position, state.velocity);
    }
}

void VisualServoMove::set_target_pose(const CartesianPose& pose) {
    if (!has_joint_state_) {
        target_pose_ = pose;
        latched_target_pose_ = pose;
        has_target_pose_ = true;
        has_latched_target_pose_ = true;
        return;
    }

    const double horizontal_distance = (pose.position.head<2>() - current_cartesian_state_.pose.position.head<2>()).norm();
    if (horizontal_distance < kVisualServoMinDistanceMeters) {
        if (has_latched_target_pose_) {
            target_pose_ = latched_target_pose_;
            has_target_pose_ = true;
        }
        return;
    }

    target_pose_ = pose;
    has_target_pose_ = true;
    if (horizontal_distance <= kVisualServoMaxDistanceMeters) {
        latched_target_pose_ = pose;
        has_latched_target_pose_ = true;
    }
}

void VisualServoMove::set_kp(double kp) { kp_ = std::max(kp, 0.0); }

void VisualServoMove::set_max_linear_acceleration(double max_linear_acceleration) {
    max_linear_acceleration_ = std::max(max_linear_acceleration, 0.0);
}

JointTrajectoryPoint VisualServoMove::sample(double current_time_sec) {
    if (!arm_calc_ || !has_joint_state_) {
        return BuildJointPoint(arm_calc_, current_joint_state_.position, current_joint_state_.velocity, JointVector::Zero());
    }

    current_cartesian_state_ = arm_calc_->end_state(current_joint_state_.position, current_joint_state_.velocity);
    Eigen::Vector3d commanded_velocity = Eigen::Vector3d::Zero();

    if (!servo_initialized_) {
        start(current_joint_state_, current_time_sec);
    }

    if (!has_target_pose_) {
        return BuildJointPoint(arm_calc_, current_joint_state_.position, current_joint_state_.velocity, JointVector::Zero());
    }

    const double horizontal_distance_to_target = (target_pose_.position.head<2>() - desired_position_.head<2>()).norm();
    if (horizontal_distance_to_target > kVisualServoMaxDistanceMeters) {
        desired_velocity_.setZero();
        desired_acceleration_.setZero();
        last_sample_time_sec_ = current_time_sec;

        CartesianTrajectoryPoint cartesian_target;
        cartesian_target.pose.position       = desired_position_;
        cartesian_target.linear_velocity     = desired_velocity_;
        cartesian_target.linear_acceleration = desired_acceleration_;
        cartesian_target.pose.orientation    = FixedVisualServoOrientation();
        cartesian_target.angular_velocity.setZero();
        cartesian_target.angular_acceleration.setZero();
        RCLCPP_INFO_THROTTLE(
            VisualServoLogger(),
            VisualServoThrottleClock(),
            1000,
            "commanded_velocity=(%.4f, %.4f, %.4f), desired_velocity=(%.4f, %.4f, %.4f), desired_position=(%.4f, %.4f, %.4f)",
            commanded_velocity.x(),
            commanded_velocity.y(),
            commanded_velocity.z(),
            cartesian_target.linear_velocity.x(),
            cartesian_target.linear_velocity.y(),
            cartesian_target.linear_velocity.z(),
            cartesian_target.pose.position.x(),
            cartesian_target.pose.position.y(),
            cartesian_target.pose.position.z());
        RCLCPP_WARN_THROTTLE(
            VisualServoLogger(),
            VisualServoThrottleClock(),
            1000,
            "visual target rejected as too far: relative_xy_distance=%.4f m, target=(%.4f, %.4f, %.4f), desired=(%.4f, %.4f, %.4f)",
            horizontal_distance_to_target,
            target_pose_.position.x(),
            target_pose_.position.y(),
            target_pose_.position.z(),
            desired_position_.x(),
            desired_position_.y(),
            desired_position_.z());
        JointTrajectoryPoint point = arm_calc_->signal_arm_calc(cartesian_target, desired_joint_seed_);
        desired_joint_seed_        = point.position;
        return point;
    }

    const double dt = std::clamp(current_time_sec - last_sample_time_sec_, 1e-4, 0.1);
    Eigen::Vector3d last_desired_velocity = desired_velocity_;
    commanded_velocity = (target_pose_.position - desired_position_) * kp_;
    desired_acceleration_              = ClampVectorNorm((commanded_velocity - last_desired_velocity) / dt, max_linear_acceleration_);
    desired_velocity_                  = last_desired_velocity + desired_acceleration_ * dt;
    desired_position_                  = desired_position_ + desired_velocity_ * dt;
    last_sample_time_sec_              = current_time_sec;

    CartesianTrajectoryPoint cartesian_target;
    cartesian_target.pose.position       = desired_position_;
    cartesian_target.linear_velocity     = desired_velocity_;
    cartesian_target.linear_acceleration = desired_acceleration_;
    cartesian_target.pose.orientation    = FixedVisualServoOrientation();
    cartesian_target.angular_velocity.setZero();
    cartesian_target.angular_acceleration.setZero();
    RCLCPP_INFO_THROTTLE(
        VisualServoLogger(),
        VisualServoThrottleClock(),
        1000,
        "commanded_velocity=(%.4f, %.4f, %.4f), desired_velocity=(%.4f, %.4f, %.4f), desired_position=(%.4f, %.4f, %.4f)",
        commanded_velocity.x(),
        commanded_velocity.y(),
        commanded_velocity.z(),
        cartesian_target.linear_velocity.x(),
        cartesian_target.linear_velocity.y(),
        cartesian_target.linear_velocity.z(),
        cartesian_target.pose.position.x(),
        cartesian_target.pose.position.y(),
        cartesian_target.pose.position.z());

    JointTrajectoryPoint point = arm_calc_->signal_arm_calc(cartesian_target, desired_joint_seed_);
    desired_joint_seed_        = point.position;
    return point;
}

} // namespace arm_action
