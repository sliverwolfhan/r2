#include <QApplication>
#include <QTimer>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "robot_interfaces/msg/plan.hpp"

#include "r2_meilin_planner/block_table.hpp"
#include "r2_meilin_planner/r2_planner_qt_test_window.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  QApplication app(argc, argv);
  QApplication::setApplicationName("r2_planner_qt_test");

  auto node = rclcpp::Node::make_shared("r2_planner_qt_test");
  RCLCPP_INFO(node->get_logger(), "Qt planner UI (12 cells, Plan button).");

  // 加载方块表（默认使用包内 config/blocks.yaml，支持 ROS 参数 blocks_yaml 覆盖）
  std::string blocks_yaml;
  try {
    blocks_yaml = ament_index_cpp::get_package_share_directory("r2_meilin_planner") +
                  "/config/block_red.yaml";
  } catch (const std::exception & ex) {
    RCLCPP_WARN(node->get_logger(), "ament_index lookup failed: %s", ex.what());
  }
  blocks_yaml = node->declare_parameter<std::string>("blocks_yaml", blocks_yaml);

  r2_planner::BlockTable blocks;
  std::string load_err;
  if (!blocks.loadFromYaml(blocks_yaml, &load_err)) {
    RCLCPP_FATAL(node->get_logger(), "Failed to load blocks yaml: %s", load_err.c_str());
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "Loaded %zu blocks from %s", blocks.size(), blocks_yaml.c_str());

  // latched 发布器（订阅者后接也能拿到最近一次计划）
  auto plan_pub = node->create_publisher<robot_interfaces::msg::Plan>(
    "/r2_planner/plan",
    rclcpp::QoS(1).transient_local().reliable());

  PlannerWindow window(node, std::move(blocks), plan_pub);
  window.show();

  QTimer ros_timer;
  QObject::connect(&ros_timer, &QTimer::timeout, [&]() { rclcpp::spin_some(node); });
  ros_timer.start(20);

  const int rc = app.exec();
  rclcpp::shutdown();
  return rc;
}
