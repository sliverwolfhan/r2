#pragma once

#include "arm_calc/common_types.hpp"

#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

namespace arm_calc {

class ArmCalc {
public:
    explicit ArmCalc(const KDL::Chain& chain);
    ~ArmCalc() = default;

    JointVector joint_pos(const CartesianPose& pose, int* result);
    JointVector joint_pos(const CartesianPose& pose, int* result, const JointVector& seed_joint_pos);

    JointVector joint_vel(const JointVector& joint_pos, const CartesianVector& cartesian_twist);
    JointVector joint_acc(const JointVector& joint_pos, const JointVector& joint_vel, const CartesianVector& cartesian_acc);

    JointVector joint_torque_dynamic(const JointVector& joint_pos,
                                     const JointVector& joint_vel,
                                     const CartesianVector& cartesian_acc);
    JointVector joint_torque_inverse_dynamics(const JointVector& joint_pos,
                                              const JointVector& joint_vel,
                                              const JointVector& joint_acc);
    JointVector joint_torque_cartesian_wrench(const JointVector& joint_pos, const CartesianVector& cartesian_wrench);

    CartesianPose end_pose(const JointVector& joint_pos);
    CartesianState end_state(const JointVector& joint_pos, const JointVector& joint_vel);
    KDL::Jacobian jacobian(const JointVector& joint_pos);

    void set_joint_pd(std::size_t index, double kp, double kd);
    void get_joint_pd(std::size_t index, double& kp, double& kd) const;

    JointTrajectoryPoint signal_arm_calc(const CartesianTrajectoryPoint& cartesian_target);
    JointTrajectoryPoint signal_arm_calc(const CartesianTrajectoryPoint& cartesian_target, const JointVector& seed_joint_pos);

private:
    static KDL::Frame to_kdl_frame(const CartesianPose& pose);
    static CartesianPose from_kdl_frame(const KDL::Frame& frame);
    static KDL::JntArray to_kdl_joints(const JointVector& joints);
    static JointVector from_kdl_joints(const KDL::JntArray& joints);
    static CartesianVector twist_to_vector(const KDL::Twist& twist);

    KDL::Chain chain_;
    KDL::ChainFkSolverPos_recursive fk_solver_;
    KDL::ChainJntToJacSolver jacobian_solver_;
    KDL::ChainJntToJacDotSolver jdot_solver_;
    KDL::ChainIkSolverVel_pinv vel_solver_;
    KDL::ChainIkSolverPos_LMA ik_solver_;
    KDL::ChainDynParam dynamic_solver_;

    KDL::JntSpaceInertiaMatrix mass_matrix_;
    KDL::JntArray coriolis_;
    KDL::JntArray gravity_;
    KDL::Jacobian jacobian_cache_;
    KDL::JntArray last_joint_solution_;
    KDL::JntArrayVel joint_vel_cache_;
    KDL::Twist jdot_qdot_cache_;

    JointVector kp_{JointVector::Constant(50.0)};
    JointVector kd_{JointVector::Constant(3.0)};
};

}  // namespace arm_calc
