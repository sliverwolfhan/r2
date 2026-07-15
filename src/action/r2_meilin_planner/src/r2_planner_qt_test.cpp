#include <QApplication>
#include <QTimer>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include "robot_interfaces/msg/plan.hpp"

#include "r2_meilin_planner/block_table.hpp"
#include "r2_meilin_planner/r2_planner_qt_test_window.hpp"

namespace {

// 从包内 config/planner_params.yaml 直接解析规划代价基线（不走 ROS 参数机制，
// 因为该 yaml 顶层键是 kfs_subscriber_node，节点名不匹配 Qt 节点）。
// 读取 kfs_subscriber_node.ros__parameters 子树，字段名/默认值与 kfs_subscriber_node
// 及 r2_planner::CostConfig 保持一致，保证离线规划与真车同源。
// 读不到文件时返回默认（loaded=false），行为与历史默认一致。
PlannerParams load_planner_params(const rclcpp::Logger & logger)
{
  PlannerParams p;
  std::string path;
  try {
    path = ament_index_cpp::get_package_share_directory("r2_meilin_planner") +
           "/config/planner_params.yaml";
  } catch (const std::exception & ex) {
    RCLCPP_WARN(logger, "ament_index 查找失败，使用默认代价: %s", ex.what());
    return p;
  }

  try {
    const YAML::Node root = YAML::LoadFile(path);
    const YAML::Node params = root["kfs_subscriber_node"]["ros__parameters"];
    if (!params) {
      RCLCPP_WARN(
        logger, "%s 缺少 kfs_subscriber_node.ros__parameters，使用默认代价", path.c_str());
      return p;
    }

    p.cost.move_cost        = params["move_cost"].as<double>(p.cost.move_cost);
    p.cost.descend_200_cost = params["descend_200_cost"].as<double>(p.cost.descend_200_cost);
    p.cost.descend_400_cost = params["descend_400_cost"].as<double>(p.cost.descend_400_cost);
    p.cost.pick_cost        = params["pick_cost"].as<double>(p.cost.pick_cost);
    p.cost.push_cost        = params["push_cost"].as<double>(p.cost.push_cost);
    p.cost.climb_200_cost   = params["climb_200_cost"].as<double>(p.cost.climb_200_cost);
    p.cost.climb_400_cost   = params["climb_400_cost"].as<double>(p.cost.climb_400_cost);
    p.cost.turn_cost        = params["turn_cost"].as<double>(p.cost.turn_cost);
    p.cost.can_climb_400    = params["can_climb_400"].as<bool>(p.cost.can_climb_400);
    p.cost.preferred_column_pick_bonus =
      params["preferred_column_pick_bonus"].as<double>(p.cost.preferred_column_pick_bonus);

    p.move_prep_offset  = params["move_prep_offset"].as<double>(p.move_prep_offset);
    p.grasp_prep_offset = params["grasp_prep_offset"].as<double>(p.grasp_prep_offset);
    p.block_height_offset = params["block_height_offset"].as<double>(p.block_height_offset);
    p.grasp_prep_theta_offset =
      params["grasp_prep_theta_offset"].as<double>(p.grasp_prep_theta_offset);
    p.move_prep_theta_offset =
      params["move_prep_theta_offset"].as<double>(p.move_prep_theta_offset);

    p.r1_timed_removal_enable =
      params["r1_timed_removal_enable"].as<bool>(p.r1_timed_removal_enable);
    p.r1_removal_steps = params["r1_removal_steps"].as<int>(p.r1_removal_steps);
    // wait_cost 默认取（已被 yaml 覆盖后的）move_cost，与 kfs_subscriber_node 一致。
    p.wait_cost = params["wait_cost"].as<double>(p.cost.move_cost);

    p.ignore_height = params["ignore_height"].as<bool>(p.ignore_height);

    p.loaded = true;
    RCLCPP_INFO(
      logger,
      "已读入代价参数 %s: pick=%.2f push=%.2f turn=%.2f climb200=%.2f climb400=%.2f "
      "can_climb_400=%s ignore_height=%s preferred_bonus=%.2f",
      path.c_str(), p.cost.pick_cost, p.cost.push_cost, p.cost.turn_cost,
      p.cost.climb_200_cost, p.cost.climb_400_cost,
      p.cost.can_climb_400 ? "true" : "false", p.ignore_height ? "true" : "false",
      p.cost.preferred_column_pick_bonus);
  } catch (const std::exception & ex) {
    RCLCPP_WARN(logger, "解析 %s 失败，使用默认代价: %s", path.c_str(), ex.what());
  }
  return p;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  QApplication app(argc, argv);
  QApplication::setApplicationName("r2_planner_qt_test");

  auto node = rclcpp::Node::make_shared("r2_planner_qt_test");
  RCLCPP_INFO(node->get_logger(), "Qt planner UI (12 cells, Plan button).");

  // 红/蓝区：必须与真车运行的区一致，否则监视到的 plan（蓝区坐标）会画到红区方块外
  // ——路线飞出场地、着色左右镜像错位。启动脚本从 zone.conf 读出后透传本参数。
  const std::string zone = node->declare_parameter<std::string>("zone", "red");
  const bool zone_blue = (zone == "blue");

  // 加载方块表（按 zone 选红/蓝 yaml，支持 ROS 参数 blocks_yaml 显式覆盖）
  std::string blocks_yaml;
  try {
    blocks_yaml = ament_index_cpp::get_package_share_directory("r2_meilin_planner") +
                  (zone_blue ? "/config/block_blue.yaml" : "/config/block_red.yaml");
  } catch (const std::exception & ex) {
    RCLCPP_WARN(node->get_logger(), "ament_index lookup failed: %s", ex.what());
  }
  blocks_yaml = node->declare_parameter<std::string>("blocks_yaml", blocks_yaml);
  RCLCPP_INFO(node->get_logger(), "zone=%s（%s区）", zone.c_str(), zone_blue ? "蓝" : "红");

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

  // 读入 planner_params.yaml 的代价基线（yaml 打底，界面控件在其上覆盖）。
  PlannerParams planner_params = load_planner_params(node->get_logger());

  PlannerWindow window(node, std::move(blocks), plan_pub, std::move(planner_params), zone_blue);
  window.show();

  QTimer ros_timer;
  QObject::connect(&ros_timer, &QTimer::timeout, [&]() { rclcpp::spin_some(node); });
  ros_timer.start(20);

  const int rc = app.exec();
  rclcpp::shutdown();
  return rc;
}
