#pragma once

#include "robot_interfaces/msg/arm.hpp"

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <string>
#include <vector>

namespace dog_controller {

class DogController : public controller_interface::ControllerInterface {
public:
    DogController();

    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

private:
    static constexpr std::size_t kJointCount = 6;

    rclcpp::Publisher<robot_interfaces::msg::Arm>::SharedPtr state_publisher_;
    rclcpp::Subscription<robot_interfaces::msg::Arm>::SharedPtr target_subscriber_;
    rclcpp_lifecycle::LifecycleNode::OnSetParametersCallbackHandle::SharedPtr param_cb_;

    std::vector<std::string> joints_name_;
    std::vector<double> joint_kp_;
    std::vector<double> joint_kd_;

    robot_interfaces::msg::Arm joints_target_{};
    robot_interfaces::msg::Arm joints_state_{};

    double joint_torque_filter_gate_{0.8};
    double joint_omega_filter_gate_{0.8};
    double command_effort_limit_{80.0};
    rclcpp::Time last_target_log_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace dog_controller
