#include "arm_calc/arm_calc.hpp"

#include <Eigen/Geometry>

namespace arm_calc {

namespace {

CartesianVector ToCartesianVector(const Eigen::Vector3d& linear, const Eigen::Vector3d& angular) {
    CartesianVector vector = CartesianVector::Zero();
    vector.head<3>() = linear;
    vector.tail<3>() = angular;
    return vector;
}

}  // namespace

ArmCalc::ArmCalc(const KDL::Chain& chain)
    : chain_(chain),
      fk_solver_(chain_),
      jacobian_solver_(chain_),
      jdot_solver_(chain_),
      vel_solver_(chain_),
      ik_solver_(chain_, Eigen::Matrix<double, 6, 1>::Ones(), 1e-6, 200, 1e-10),
      dynamic_solver_(chain_, KDL::Vector(0.0, 0.0, -9.81)),
      mass_matrix_(chain_.getNrOfJoints()),
      coriolis_(chain_.getNrOfJoints()),
      gravity_(chain_.getNrOfJoints()),
      jacobian_cache_(chain_.getNrOfJoints()),
      last_joint_solution_(chain_.getNrOfJoints()),
      joint_vel_cache_(chain_.getNrOfJoints()) {
    for (unsigned int i = 0; i < chain_.getNrOfJoints(); ++i) {
        last_joint_solution_(i) = 0.0;
    }
}

JointVector ArmCalc::joint_pos(const CartesianPose& pose, int* result) {
    return joint_pos(pose, result, from_kdl_joints(last_joint_solution_));
}

JointVector ArmCalc::joint_pos(const CartesianPose& pose, int* result, const JointVector& seed_joint_pos) {
    KDL::JntArray seed = to_kdl_joints(seed_joint_pos);
    KDL::Frame target_frame = to_kdl_frame(pose);
    *result = ik_solver_.CartToJnt(seed, target_frame, seed);
    if (*result >= 0) {
        last_joint_solution_ = seed;
    }
    return from_kdl_joints(seed);
}

JointVector ArmCalc::joint_vel(const JointVector& joint_pos, const CartesianVector& cartesian_twist) {
    KDL::JntArray joints = to_kdl_joints(joint_pos);
    jacobian_solver_.JntToJac(joints, jacobian_cache_);
    Eigen::Matrix<double, 6, 6> jacobian = jacobian_cache_.data;
    return jacobian.completeOrthogonalDecomposition().solve(cartesian_twist);
}

JointVector ArmCalc::joint_acc(const JointVector& joint_pos, const JointVector& joint_vel, const CartesianVector& cartesian_acc) {
    joint_vel_cache_.q = to_kdl_joints(joint_pos);
    joint_vel_cache_.qdot = to_kdl_joints(joint_vel);
    jacobian_solver_.JntToJac(joint_vel_cache_.q, jacobian_cache_);
    jdot_solver_.JntToJacDot(joint_vel_cache_, jdot_qdot_cache_);

    Eigen::Matrix<double, 6, 6> jacobian = jacobian_cache_.data;
    const CartesianVector jdot_qdot = twist_to_vector(jdot_qdot_cache_);
    return jacobian.completeOrthogonalDecomposition().solve(cartesian_acc - jdot_qdot);
}

JointVector ArmCalc::joint_torque_dynamic(const JointVector& joint_pos,
                                          const JointVector& joint_vel,
                                          const CartesianVector& cartesian_acc) {
    return joint_torque_inverse_dynamics(joint_pos, joint_vel, joint_acc(joint_pos, joint_vel, cartesian_acc));
}

JointVector ArmCalc::joint_torque_inverse_dynamics(const JointVector& joint_pos,
                                                   const JointVector& joint_vel,
                                                   const JointVector& joint_acc) {
    const KDL::JntArray kdl_joint_pos = to_kdl_joints(joint_pos);
    const KDL::JntArray kdl_joint_vel = to_kdl_joints(joint_vel);

    dynamic_solver_.JntToGravity(kdl_joint_pos, gravity_);
    dynamic_solver_.JntToCoriolis(kdl_joint_pos, kdl_joint_vel, coriolis_);
    dynamic_solver_.JntToMass(kdl_joint_pos, mass_matrix_);

    JointMatrix mass = mass_matrix_.data;
    JointVector coriolis = from_kdl_joints(coriolis_);
    JointVector gravity = from_kdl_joints(gravity_);
    return mass * joint_acc + coriolis + gravity;
}

JointVector ArmCalc::joint_torque_cartesian_wrench(const JointVector& joint_pos, const CartesianVector& cartesian_wrench) {
    KDL::JntArray joints = to_kdl_joints(joint_pos);
    jacobian_solver_.JntToJac(joints, jacobian_cache_);
    Eigen::Matrix<double, 6, 6> jacobian = jacobian_cache_.data;
    return jacobian.transpose() * cartesian_wrench;
}

CartesianPose ArmCalc::end_pose(const JointVector& joint_pos) {
    KDL::Frame frame;
    fk_solver_.JntToCart(to_kdl_joints(joint_pos), frame);
    return from_kdl_frame(frame);
}

CartesianState ArmCalc::end_state(const JointVector& joint_pos, const JointVector& joint_vel) {
    CartesianState state;
    state.pose = end_pose(joint_pos);
    KDL::JntArray joints = to_kdl_joints(joint_pos);
    jacobian_solver_.JntToJac(joints, jacobian_cache_);
    const CartesianVector twist = jacobian_cache_.data * joint_vel;
    state.linear_velocity = twist.head<3>();
    state.angular_velocity = twist.tail<3>();
    return state;
}

KDL::Jacobian ArmCalc::jacobian(const JointVector& joint_pos) {
    KDL::JntArray joints = to_kdl_joints(joint_pos);
    jacobian_solver_.JntToJac(joints, jacobian_cache_);
    return jacobian_cache_;
}

void ArmCalc::set_joint_pd(std::size_t index, double kp, double kd) {
    if (index >= kJointDoF) {
        return;
    }
    kp_[static_cast<int>(index)] = kp;
    kd_[static_cast<int>(index)] = kd;
}

void ArmCalc::get_joint_pd(std::size_t index, double& kp, double& kd) const {
    if (index >= kJointDoF) {
        kp = 0.0;
        kd = 0.0;
        return;
    }
    kp = kp_[static_cast<int>(index)];
    kd = kd_[static_cast<int>(index)];
}

JointTrajectoryPoint ArmCalc::signal_arm_calc(const CartesianTrajectoryPoint& cartesian_target) {
    return signal_arm_calc(cartesian_target, from_kdl_joints(last_joint_solution_));
}

JointTrajectoryPoint ArmCalc::signal_arm_calc(const CartesianTrajectoryPoint& cartesian_target, const JointVector& seed_joint_pos) {
    JointTrajectoryPoint point;
    int result = -1;
    point.position = joint_pos(cartesian_target.pose, &result, seed_joint_pos);
    if (result < 0) {
        point.velocity.setZero();
        point.acceleration.setZero();
        point.torque = joint_torque_inverse_dynamics(point.position, point.velocity, point.acceleration);
        return point;
    }

    const CartesianVector twist = ToCartesianVector(cartesian_target.linear_velocity, cartesian_target.angular_velocity);
    const CartesianVector acceleration =
        ToCartesianVector(cartesian_target.linear_acceleration, cartesian_target.angular_acceleration);

    point.velocity = joint_vel(point.position, twist);
    point.acceleration = joint_acc(point.position, point.velocity, acceleration);
    point.torque = joint_torque_dynamic(point.position, point.velocity, acceleration);
    return point;
}

KDL::Frame ArmCalc::to_kdl_frame(const CartesianPose& pose) {
    const Eigen::Quaterniond q = NormalizeQuaternion(pose.orientation);
    return KDL::Frame(KDL::Rotation::Quaternion(q.x(), q.y(), q.z(), q.w()),
                      KDL::Vector(pose.position.x(), pose.position.y(), pose.position.z()));
}

CartesianPose ArmCalc::from_kdl_frame(const KDL::Frame& frame) {
    CartesianPose pose;
    pose.position = Eigen::Vector3d(frame.p.x(), frame.p.y(), frame.p.z());

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 1.0;
    frame.M.GetQuaternion(x, y, z, w);
    pose.orientation = NormalizeQuaternion(Eigen::Quaterniond(w, x, y, z));
    return pose;
}

KDL::JntArray ArmCalc::to_kdl_joints(const JointVector& joints) {
    KDL::JntArray output(static_cast<unsigned int>(kJointDoF));
    for (std::size_t i = 0; i < kJointDoF; ++i) {
        output(static_cast<unsigned int>(i)) = joints[static_cast<int>(i)];
    }
    return output;
}

JointVector ArmCalc::from_kdl_joints(const KDL::JntArray& joints) {
    JointVector output = JointVector::Zero();
    for (std::size_t i = 0; i < kJointDoF; ++i) {
        output[static_cast<int>(i)] = joints(static_cast<unsigned int>(i));
    }
    return output;
}

CartesianVector ArmCalc::twist_to_vector(const KDL::Twist& twist) {
    CartesianVector output = CartesianVector::Zero();
    output << twist.vel.x(), twist.vel.y(), twist.vel.z(), twist.rot.x(), twist.rot.y(), twist.rot.z();
    return output;
}

}  // namespace arm_calc
