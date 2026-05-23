#pragma once

#include "arm_calc/common_types.hpp"

namespace arm_calc {

class TrajectoryCalc {
public:
    struct QuinticPolynomial {
        double a0{0.0};
        double a1{0.0};
        double a2{0.0};
        double a3{0.0};
        double a4{0.0};
        double a5{0.0};
    };

    TrajectoryCalc() = default;

    void set_start_state(const JointVector& position,
                         const JointVector& velocity = JointVector::Zero(),
                         const JointVector& acceleration = JointVector::Zero());

    void set_goal_state(const JointVector& position,
                        double duration,
                        const JointVector& velocity = JointVector::Zero(),
                        const JointVector& acceleration = JointVector::Zero());

    JointTrajectoryPoint sample(double time_from_start) const;

    double duration() const { return duration_; }
    bool is_ready() const { return ready_; }

private:
    static QuinticPolynomial build_segment(double p0, double v0, double a0,
                                           double pf, double vf, double af,
                                           double duration);
    static double eval_position(const QuinticPolynomial& poly, double t);
    static double eval_velocity(const QuinticPolynomial& poly, double t);
    static double eval_acceleration(const QuinticPolynomial& poly, double t);

    double duration_{0.0};
    bool ready_{false};
    JointVector start_position_{JointVector::Zero()};
    JointVector start_velocity_{JointVector::Zero()};
    JointVector start_acceleration_{JointVector::Zero()};
    JointVector goal_position_{JointVector::Zero()};
    JointVector goal_velocity_{JointVector::Zero()};
    JointVector goal_acceleration_{JointVector::Zero()};
    std::array<QuinticPolynomial, kJointDoF> segments_{};
};

}  // namespace arm_calc
