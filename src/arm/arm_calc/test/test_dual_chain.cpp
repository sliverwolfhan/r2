#include <arm_calc/arm_calc.hpp>
#include <kdl/chain.hpp>
#include <kdl/segment.hpp>
#include <kdl/joint.hpp>
#include <kdl/frames.hpp>
#include <cassert>
#include <cmath>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

// 建一条 6-DOF 简易链：joint1 绕 Z（竖直），joint2 绕 Y（水平），其余绕 Y。
// link6 惯量可变：nominal 0.15kg，payload 0.78kg。
KDL::Chain MakeChain(double link6_mass) {
    KDL::Chain chain;
    // joint1: RotZ (base)
    chain.addSegment(KDL::Segment(
        "link1",
        KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame(KDL::Vector(0, 0, 0.3)),
        KDL::RigidBodyInertia(0.1, KDL::Vector(0, 0, 0.15),
            KDL::RotationalInertia(0.001, 0.001, 0.001))));
    // joint2: RotY (horizontal)
    chain.addSegment(KDL::Segment(
        "link2",
        KDL::Joint(KDL::Joint::RotY),
        KDL::Frame(KDL::Vector(0.5, 0, 0)),
        KDL::RigidBodyInertia(0.5, KDL::Vector(0.25, 0, 0),
            KDL::RotationalInertia(0.001, 0.001, 0.001))));
    // joint3: RotY
    chain.addSegment(KDL::Segment(
        "link3",
        KDL::Joint(KDL::Joint::RotY),
        KDL::Frame(KDL::Vector(0.5, 0, 0)),
        KDL::RigidBodyInertia(0.5, KDL::Vector(0.25, 0, 0),
            KDL::RotationalInertia(0.001, 0.001, 0.001))));
    // joint4: RotY
    chain.addSegment(KDL::Segment(
        "link4",
        KDL::Joint(KDL::Joint::RotY),
        KDL::Frame(KDL::Vector(0.3, 0, 0)),
        KDL::RigidBodyInertia(0.3, KDL::Vector(0.15, 0, 0),
            KDL::RotationalInertia(0.001, 0.001, 0.001))));
    // joint5: RotY
    chain.addSegment(KDL::Segment(
        "link5",
        KDL::Joint(KDL::Joint::RotY),
        KDL::Frame(KDL::Vector(0.2, 0, 0)),
        KDL::RigidBodyInertia(0.2, KDL::Vector(0.1, 0, 0),
            KDL::RotationalInertia(0.001, 0.001, 0.001))));
    // joint6: RotY, link6 带可变质量
    chain.addSegment(KDL::Segment(
        "link6",
        KDL::Joint(KDL::Joint::RotY),
        KDL::Frame(KDL::Vector(0.1, 0, 0)),
        KDL::RigidBodyInertia(link6_mass, KDL::Vector(0.05, 0, 0),
            KDL::RotationalInertia(0.001, 0.001, 0.001))));
    return chain;
}

int main() {
    KDL::Chain nominal = MakeChain(0.15);
    KDL::Chain payload = MakeChain(0.78);
    arm_calc::ArmCalc calc(nominal, payload);

    // 测试默认模式是 nominal
    assert(calc.payload_mode() == false);

    // 在 q 全 0 静止时，重力补偿力矩（水平关节受重力影响）
    arm_calc::JointVector q = arm_calc::JointVector::Zero();

    arm_calc::JointVector tau_nominal = calc.joint_torque_inverse_dynamics(
        q, arm_calc::JointVector::Zero(), arm_calc::JointVector::Zero());

    calc.SetPayloadMode(true);
    assert(calc.payload_mode() == true);
    arm_calc::JointVector tau_payload = calc.joint_torque_inverse_dynamics(
        q, arm_calc::JointVector::Zero(), arm_calc::JointVector::Zero());

    // 载荷链重力力矩应严格大于空链（link6 质量更大）
    std::cout << "tau_nominal = " << tau_nominal.transpose() << std::endl;
    std::cout << "tau_payload = " << tau_payload.transpose() << std::endl;
    // joint 1 (index 1) 是水平关节，受重力影响最大
    assert(std::abs(tau_payload(1)) > std::abs(tau_nominal(1)));

    // 切回 nominal 应恢复
    calc.SetPayloadMode(false);
    assert(calc.payload_mode() == false);
    arm_calc::JointVector tau_back = calc.joint_torque_inverse_dynamics(
        q, arm_calc::JointVector::Zero(), arm_calc::JointVector::Zero());
    assert((tau_back - tau_nominal).norm() < 1e-12);

    // ---- Deep-path recursive lock test ----------------------------------
    // Recreate a fresh ArmCalc so last_joint_solution_ is at zero; that
    // guarantees the seed for signal_arm_calc is the zero config which we
    // will also use as the FK pose source.  Without this, the IK seed would
    // be the last 2-arg call's solution (still zero here, but explicit
    // rebuild is honest).
    arm_calc::ArmCalc calc2(nominal, payload);
    // Reachable Cartesian target = FK(q_zero) on the active (nominal) chain.
    arm_calc::CartesianPose target = calc2.end_pose(q);
    arm_calc::CartesianTrajectoryPoint cart_target;
    cart_target.pose = target;
    // Zero velocity/acceleration: round-trip should reproduce q_zero.
    cart_target.linear_velocity = Eigen::Vector3d::Zero();
    cart_target.angular_velocity = Eigen::Vector3d::Zero();
    cart_target.linear_acceleration = Eigen::Vector3d::Zero();
    cart_target.angular_acceleration = Eigen::Vector3d::Zero();

    // signal_arm_calc takes the lock, then calls joint_pos (lock again),
    // joint_vel (lock again), joint_acc (lock again), joint_torque_dynamic
    // (lock again -> joint_acc (lock again)).  Six nested acquisitions on
    // the same recursive_mutex.  A nonrecursive mutex would deadlock here.
    arm_calc::JointTrajectoryPoint jtp = calc2.signal_arm_calc(cart_target);
    double err = (jtp.position - q).norm();
    std::cout << "signal_arm_calc round-trip position error = " << err << std::endl;
    // The IK may not converge exactly back to q_zero for all chain configs;
    // per-joint tolerance ~1e-3 is well within the LMA solver's epsilon.
    assert(err < 1e-3 && "signal_arm_calc q round-trip regressed");
    std::cout << "signal_arm_calc round-trip OK\n";

    std::cout << "All dual-chain tests passed.\n";
    return 0;
}
