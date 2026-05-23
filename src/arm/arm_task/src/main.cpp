#include "robot.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node=std::make_shared<rclcpp::Node>("arm_task_node");
    Robot arm_task(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
