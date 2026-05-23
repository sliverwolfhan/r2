#pragma once

#include "arm_calc/arm_calc.hpp"
#include "arm_calc/trajectory_calc.hpp"

#include <memory>

namespace arm_action {

using arm_calc::ArmCalc;
using arm_calc::CartesianPose;
using arm_calc::CartesianState;
using arm_calc::CartesianTrajectoryPoint;
using arm_calc::CartesianVector;
using arm_calc::JointState;
using arm_calc::JointTrajectoryPoint;
using arm_calc::JointVector;

class JointSpaceMove {
public:
    explicit JointSpaceMove(std::shared_ptr<ArmCalc> arm_calc);

    void set_start_state(const JointState& state);
    void set_goal_state(const JointState& state, double duration);
    void start(double start_time_sec);
    JointTrajectoryPoint sample(double current_time_sec) const;
    bool active(double current_time_sec) const;
    bool started() const;

private:
    std::shared_ptr<ArmCalc> arm_calc_;
    arm_calc::TrajectoryCalc trajectory_;
    JointState start_state_{};
    JointState goal_state_{};
    double start_time_sec_{0.0};
    double duration_sec_{0.0};
    bool started_{false};
};

class JCartesianSpaceMove {
public:
    explicit JCartesianSpaceMove(std::shared_ptr<ArmCalc> arm_calc);

    void set_start_state(const JointState& state);
    void set_goal_state(const CartesianPose& pose, double duration);
    void start(double start_time_sec);
    JointTrajectoryPoint sample(double current_time_sec);
    bool active(double current_time_sec) const;
    bool started() const;

private:
    CartesianTrajectoryPoint build_cartesian_target(double current_time_sec) const;

    std::shared_ptr<ArmCalc> arm_calc_;
    JointState start_joint_state_{};
    CartesianState start_cartesian_state_{};
    CartesianPose goal_pose_{};
    Eigen::Quaterniond start_orientation_{Eigen::Quaterniond::Identity()};
    Eigen::Quaterniond goal_orientation_{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d orientation_rotation_vector_world_{Eigen::Vector3d::Zero()};
    double start_time_sec_{0.0};
    double duration_sec_{0.0};
    bool started_{false};

    arm_calc::TrajectoryCalc position_trajectory_;
    arm_calc::TrajectoryCalc orientation_trajectory_;
};

class VisualServoMove {
public:
    explicit VisualServoMove(std::shared_ptr<ArmCalc> arm_calc);

    void start(const JointState& state, double start_time_sec);
    void set_current_joint_state(const JointState& state);
    void set_target_pose(const CartesianPose& pose);
    void set_kp(double kp);
    void set_max_linear_acceleration(double max_linear_acceleration);
    JointTrajectoryPoint sample(double current_time_sec);

private:
    std::shared_ptr<ArmCalc> arm_calc_;
    JointState current_joint_state_{};
    CartesianState current_cartesian_state_{};
    CartesianPose target_pose_{};
    CartesianPose latched_target_pose_{};
    bool has_joint_state_{false};
    bool has_target_pose_{false};
    bool has_latched_target_pose_{false};
    bool servo_initialized_{false};
    double last_sample_time_sec_{0.0};
    double kp_{0.4};
    double max_linear_acceleration_{0.5};

    Eigen::Vector3d desired_position_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d desired_velocity_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d desired_acceleration_{Eigen::Vector3d::Zero()};
    JointVector desired_joint_seed_{JointVector::Zero()};
};

}  // namespace arm_action
