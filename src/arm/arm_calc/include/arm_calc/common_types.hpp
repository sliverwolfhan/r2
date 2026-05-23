#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>

namespace arm_calc {

constexpr std::size_t kJointDoF = 6;

using JointVector = Eigen::Matrix<double, static_cast<int>(kJointDoF), 1>;
using JointMatrix = Eigen::Matrix<double, static_cast<int>(kJointDoF), static_cast<int>(kJointDoF)>;
using CartesianVector = Eigen::Matrix<double, 6, 1>;

struct JointState {
    JointVector position{JointVector::Zero()};
    JointVector velocity{JointVector::Zero()};
    JointVector torque{JointVector::Zero()};
};

struct JointTrajectoryPoint {
    JointVector position{JointVector::Zero()};
    JointVector velocity{JointVector::Zero()};
    JointVector acceleration{JointVector::Zero()};
    JointVector torque{JointVector::Zero()};
};

struct CartesianPose {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
};

struct CartesianState {
    CartesianPose pose{};
    Eigen::Vector3d linear_velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
};

struct CartesianTrajectoryPoint {
    CartesianPose pose{};
    Eigen::Vector3d linear_velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d linear_acceleration{Eigen::Vector3d::Zero()};
    Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d angular_acceleration{Eigen::Vector3d::Zero()};
};

inline Eigen::Quaterniond NormalizeQuaternion(const Eigen::Quaterniond& q) {
    Eigen::Quaterniond normalized = q;
    if (normalized.norm() < 1e-9) {
        return Eigen::Quaterniond::Identity();
    }
    normalized.normalize();
    return normalized;
}

}  // namespace arm_calc
