#pragma once

#include "arm_action/basic_moves.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <kdl/chain.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <robot_interfaces/msg/arm.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>
#include <vector>

namespace arm_calc {

class ArmCtrlNode : public rclcpp::Node {
public:
    explicit ArmCtrlNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    enum class MotionMode {
        kIdle = 0,
        kJointSpace = 1,
        kCartesianSpace = 2,
        kVisualServo = 3
    };

    void declare_parameters();
    void create_interfaces();
    void load_robot_description_and_build_solver();
    std::string fetch_robot_description() const;
    std::string load_local_urdf() const;
    void refresh_plan(double now_sec);
    void apply_requested_mode(double now_sec);
    void capture_idle_hold_from_current_state();
    void set_idle_hold_point(const JointTrajectoryPoint& point);
    void enter_idle_mode();
    bool is_trajectory_running(double now_sec) const;
    bool can_switch_mode_immediately() const;
    void set_execute_trajectory_flag(bool value);
    JointTrajectoryPoint build_preview_target() const;
    void publish_control_loop();
    void publish_joint_target(const JointTrajectoryPoint& point);
    void publish_visualization(const JointTrajectoryPoint& target_point);

    void on_joint_state(const robot_interfaces::msg::Arm& msg);
    void on_visual_target(const geometry_msgs::msg::PoseStamped& msg);
    void on_joint_space_target(const std_msgs::msg::Float64MultiArray& msg);
    rcl_interfaces::msg::SetParametersResult on_parameters_changed(const std::vector<rclcpp::Parameter>& params);

    static MotionMode parse_motion_mode(int mode_value);
    static JointState from_arm_message(const robot_interfaces::msg::Arm& msg);
    static robot_interfaces::msg::Arm to_arm_message(const JointTrajectoryPoint& point);
    static sensor_msgs::msg::JointState to_joint_state_msg(const JointTrajectoryPoint& point, const rclcpp::Time& stamp);
    static std::vector<double> get_double_array_param(const rclcpp::Node& node, const std::string& name, std::size_t expected_size);

    JointState current_joint_state_{};
    bool has_joint_state_{false};
    bool idle_hold_initialized_{false};
    bool planners_ready_{false};
    MotionMode active_motion_mode_{MotionMode::kIdle};
    MotionMode requested_motion_mode_{MotionMode::kIdle};
    double trajectory_duration_sec_{3.0};
    double control_period_sec_{0.02};
    bool execute_trajectory_{false};
    bool updating_execute_parameter_{false};
    double visual_servo_kp_{2.0};
    double visual_servo_max_linear_acceleration_{0.5};
    double last_ee_log_time_sec_{-1.0};

    std::vector<std::string> joint_names_{"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    std::string base_link_{"base_link"};
    std::string tip_link_{"link6"};
    CartesianPose cartesian_target_{};
    CartesianPose visual_target_{};
    JointState joint_target_state_{};
    JointTrajectoryPoint idle_hold_point_{};

    KDL::Chain arm_chain_;
    std::shared_ptr<ArmCalc> arm_calc_;
    std::shared_ptr<arm_action::JointSpaceMove> joint_space_move_;
    std::shared_ptr<arm_action::JCartesianSpaceMove> cartesian_space_move_;
    std::shared_ptr<arm_action::VisualServoMove> visual_servo_move_;

    rclcpp::Subscription<robot_interfaces::msg::Arm>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr visual_target_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_space_target_sub_;
    rclcpp::Publisher<robot_interfaces::msg::Arm>::SharedPtr joint_target_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr rviz_joint_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
};

}  // namespace arm_calc
