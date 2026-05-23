#include "arm_calc/arm_ctrl.hpp"

#include <memory>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<arm_calc::ArmCtrlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
