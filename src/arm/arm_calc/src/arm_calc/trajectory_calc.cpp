#include "arm_calc/trajectory_calc.hpp"

#include <algorithm>
#include <cmath>

namespace arm_calc {

void TrajectoryCalc::set_start_state(const JointVector& position,
                                     const JointVector& velocity,
                                     const JointVector& acceleration) {
    start_position_ = position;
    start_velocity_ = velocity;
    start_acceleration_ = acceleration;
}

void TrajectoryCalc::set_goal_state(const JointVector& position,
                                    double duration,
                                    const JointVector& velocity,
                                    const JointVector& acceleration) {
    duration_ = std::max(duration, 1e-3);
    goal_position_ = position;
    goal_velocity_ = velocity;
    goal_acceleration_ = acceleration;

    for (std::size_t i = 0; i < kJointDoF; ++i) {
        segments_[i] = build_segment(start_position_[static_cast<int>(i)],
                                     start_velocity_[static_cast<int>(i)],
                                     start_acceleration_[static_cast<int>(i)],
                                     goal_position_[static_cast<int>(i)],
                                     goal_velocity_[static_cast<int>(i)],
                                     goal_acceleration_[static_cast<int>(i)],
                                     duration_);
    }
    ready_ = true;
}

JointTrajectoryPoint TrajectoryCalc::sample(double time_from_start) const {
    JointTrajectoryPoint point;
    if (!ready_) {
        return point;
    }

    const double t = std::clamp(time_from_start, 0.0, duration_);
    for (std::size_t i = 0; i < kJointDoF; ++i) {
        const auto& poly = segments_[i];
        point.position[static_cast<int>(i)] = eval_position(poly, t);
        point.velocity[static_cast<int>(i)] = eval_velocity(poly, t);
        point.acceleration[static_cast<int>(i)] = eval_acceleration(poly, t);
    }
    return point;
}

TrajectoryCalc::QuinticPolynomial TrajectoryCalc::build_segment(double p0, double v0, double a0,
                                                                double pf, double vf, double af,
                                                                double duration) {
    const double t = std::max(duration, 1e-3);
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double t4 = t3 * t;
    const double t5 = t4 * t;

    QuinticPolynomial poly;
    poly.a0 = p0;
    poly.a1 = v0;
    poly.a2 = 0.5 * a0;
    poly.a3 = (20.0 * (pf - p0) - (8.0 * vf + 12.0 * v0) * t - (3.0 * a0 - af) * t2) / (2.0 * t3);
    poly.a4 = (30.0 * (p0 - pf) + (14.0 * vf + 16.0 * v0) * t + (3.0 * a0 - 2.0 * af) * t2) / (2.0 * t4);
    poly.a5 = (12.0 * (pf - p0) - (6.0 * vf + 6.0 * v0) * t - (a0 - af) * t2) / (2.0 * t5);
    return poly;
}

double TrajectoryCalc::eval_position(const QuinticPolynomial& poly, double t) {
    return poly.a0 + poly.a1 * t + poly.a2 * t * t + poly.a3 * t * t * t + poly.a4 * t * t * t * t +
           poly.a5 * t * t * t * t * t;
}

double TrajectoryCalc::eval_velocity(const QuinticPolynomial& poly, double t) {
    return poly.a1 + 2.0 * poly.a2 * t + 3.0 * poly.a3 * t * t + 4.0 * poly.a4 * t * t * t +
           5.0 * poly.a5 * t * t * t * t;
}

double TrajectoryCalc::eval_acceleration(const QuinticPolynomial& poly, double t) {
    return 2.0 * poly.a2 + 6.0 * poly.a3 * t + 12.0 * poly.a4 * t * t + 20.0 * poly.a5 * t * t * t;
}

}  // namespace arm_calc
