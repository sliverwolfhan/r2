// 最简单的行为树运行器
// 用于测试 PublishGoal 节点

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/time.h>
#include <chrono>
#include <string>
#include <thread>
#include <vector>
#include "nav2_bt_publish_goal/publish_goal_action.hpp"
#include "nav2_bt_publish_goal/climb_stair_action.hpp"
#include "nav2_bt_publish_goal/descend_stair_action.hpp"
#include "nav2_bt_publish_goal/pre_steer_align_action.hpp"
#include "nav2_bt_publish_goal/set_costmap_inflation_action.hpp"
#include "nav2_bt_publish_goal/set_mppi_params_action.hpp"
#include "nav2_bt_publish_goal/arm_task_action.hpp"
#include "nav2_bt_publish_goal/arm_move_joint_action.hpp"
#include "nav2_bt_publish_goal/get_plan_action.hpp"
#include "nav2_bt_publish_goal/pop_next_step_action.hpp"
#include "nav2_bt_publish_goal/peek_next_step_action.hpp"
#include "nav2_bt_publish_goal/select_arm_prep_name_action.hpp"
#include "nav2_bt_publish_goal/arm_move_named_action.hpp"
#include "nav2_bt_publish_goal/is_prep_skippable_condition.hpp"
#include "nav2_bt_publish_goal/dock_to_wall_action.hpp"
#include "nav2_bt_publish_goal/delay_decorator.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("simple_bt_runner");

  RCLCPP_INFO(node->get_logger(), "=== 简单行为树测试 ===");

  // ---- 全局 TF listener ----
  // 把 TF buffer 提到这里创建并共享给所有 BT 节点（写入黑板），
  // 这样从程序启动就开始接收 /AT_R2/tf，BT 跑到 ArmTask 时
  // buffer 里早就有 map / odom / base_link 了，不会因为 discovery
  // 慢导致 canTransform 超时。
  std::string tf_ns = "AT_R2";
  node->declare_parameter<std::string>("tf_namespace", tf_ns);
  node->get_parameter("tf_namespace", tf_ns);

  rclcpp::NodeOptions tf_node_opts;
  bool use_sim_time = false;
  node->get_parameter_or("use_sim_time", use_sim_time, false);
  tf_node_opts.parameter_overrides({rclcpp::Parameter("use_sim_time", use_sim_time)});

  std::vector<std::string> remap_args;
  if (!tf_ns.empty()) {
    const std::string ns_prefix = (tf_ns.front() == '/') ? tf_ns : ("/" + tf_ns);
    remap_args = {
      "--ros-args",
      "-r", "/tf:=" + ns_prefix + "/tf",
      "-r", "/tf_static:=" + ns_prefix + "/tf_static",
    };
  }
  tf_node_opts.arguments(remap_args);

  auto tf_node = rclcpp::Node::make_shared("bt_tf_listener", "", tf_node_opts);
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  // spin_thread=true：tf_node 自带后台线程，订阅、回调与主循环解耦。
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, tf_node, true);
  RCLCPP_INFO(node->get_logger(),
    "✓ 全局 TF listener 已就绪: /tf -> %s/tf, /tf_static -> %s/tf_static",
    tf_ns.empty() ? "(global)" : ("/" + tf_ns).c_str(),
    tf_ns.empty() ? "(global)" : ("/" + tf_ns).c_str());

  // 创建行为树工厂
  BT::BehaviorTreeFactory factory;

  // 直接注册我们的节点（不通过插件）
  factory.registerNodeType<nav2_bt_publish_goal::PublishGoalAction>("PublishGoal");
  factory.registerNodeType<nav2_bt_publish_goal::ClimbStairAction>("ClimbStair");
  factory.registerNodeType<nav2_bt_publish_goal::DescendStairAction>("DescendStair");
  factory.registerNodeType<nav2_bt_publish_goal::PreSteerAlignAction>("PreSteerAlign");
  factory.registerNodeType<nav2_bt_publish_goal::SetCostmapInflationAction>("SetCostmapInflation");
  factory.registerNodeType<nav2_bt_publish_goal::SetMppiParamsAction>("SetMppiParams");
  factory.registerNodeType<nav2_bt_publish_goal::ArmTaskAction>("ArmTask");
  factory.registerNodeType<nav2_bt_publish_goal::ArmMoveJointAction>("ArmMoveJoint");
  factory.registerNodeType<nav2_bt_publish_goal::ArmMoveNamedAction>("ArmMoveNamed");
  factory.registerNodeType<nav2_bt_publish_goal::GetPlanAction>("GetPlan");
  factory.registerNodeType<nav2_bt_publish_goal::PopNextStepAction>("PopNextStep");
  factory.registerNodeType<nav2_bt_publish_goal::PeekNextStepAction>("PeekNextStep");
  factory.registerNodeType<nav2_bt_publish_goal::SelectArmPrepNameAction>("SelectArmPrepName");
  factory.registerNodeType<nav2_bt_publish_goal::IsPrepSkippableCondition>("IsPrepSkippable");
  factory.registerNodeType<nav2_bt_publish_goal::DockToWallAction>("DockToWall");
  factory.registerNodeType<nav2_bt_publish_goal::DelayDecorator>("DelayMs");
  RCLCPP_INFO(node->get_logger(), "✓ PublishGoal 节点已注册");
  RCLCPP_INFO(node->get_logger(), "✓ ClimbStair 节点已注册");
  RCLCPP_INFO(node->get_logger(), "✓ DescendStair 节点已注册");
  RCLCPP_INFO(node->get_logger(), "✓ PreSteerAlign 节点已注册");
  RCLCPP_INFO(node->get_logger(), "✓ SetCostmapInflation 节点已注册");
  RCLCPP_INFO(node->get_logger(), "✓ SetMppiParams 节点已注册");
  RCLCPP_INFO(node->get_logger(), "✓ ArmTask 节点已注册");
  RCLCPP_INFO(node->get_logger(), "✓ ArmMoveJoint 节点已注册");
  RCLCPP_INFO(node->get_logger(), "✓ ArmMoveNamed 节点已注册");
  RCLCPP_INFO(node->get_logger(), "✓ GetPlan 节点已注册");
  RCLCPP_INFO(node->get_logger(), "✓ PopNextStep 节点已注册");
  RCLCPP_INFO(node->get_logger(), "✓ PeekNextStep 节点已注册");
  RCLCPP_INFO(node->get_logger(), "✓ SelectArmPrepName 节点已注册");
  RCLCPP_INFO(node->get_logger(), "✓ IsPrepSkippable 节点已注册");

  // 获取行为树 XML 文件路径 - 支持命令行参数
  std::string bt_file = "grasp_head.xml";
  if (argc > 1) {
    bt_file = argv[1];
  }

  std::string pkg_dir = ament_index_cpp::get_package_share_directory("at_r2_bt");
  std::string bt_xml = pkg_dir + "/behavior_trees/" + bt_file;

  RCLCPP_INFO(node->get_logger(), "加载行为树: %s", bt_xml.c_str());

  // 将 ROS 节点和共享 TF buffer 放入黑板（全局）
  BT::Blackboard::Ptr blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  blackboard->set("tf_buffer", tf_buffer);

  RCLCPP_INFO(node->get_logger(), "✓ ROS 节点 / TF buffer 已添加到黑板");

  // 创建树 - 传递黑板作为根黑板
  auto tree = factory.createTreeFromFile(bt_xml, blackboard);

  RCLCPP_INFO(node->get_logger(), "✓ 行为树已创建");

  // ---- 启动前等 TF 就绪 ----
  // 在 tick 之前先把 map -> base_link 等出现在 buffer 里，
  // 这样后续 BT 节点里 canTransform 都是 buffer 命中（几 us 级），
  // 不会再卡 1s 等数据。
  std::string wait_target_frame = "map";
  std::string wait_source_frame = "base_link";
  double tf_wait_timeout_s = 10.0;
  node->declare_parameter<std::string>("tf_wait_target_frame", wait_target_frame);
  node->declare_parameter<std::string>("tf_wait_source_frame", wait_source_frame);
  node->declare_parameter<double>("tf_wait_timeout", tf_wait_timeout_s);
  node->get_parameter("tf_wait_target_frame", wait_target_frame);
  node->get_parameter("tf_wait_source_frame", wait_source_frame);
  node->get_parameter("tf_wait_timeout", tf_wait_timeout_s);

  if (tf_wait_timeout_s > 0.0 &&
      !wait_target_frame.empty() && !wait_source_frame.empty())
  {
    RCLCPP_INFO(node->get_logger(),
      "等待 TF 就绪: %s -> %s (timeout=%.1fs)",
      wait_target_frame.c_str(), wait_source_frame.c_str(), tf_wait_timeout_s);

    const auto wait_start = std::chrono::steady_clock::now();
    const auto wait_deadline =
      wait_start + std::chrono::duration<double>(tf_wait_timeout_s);
    bool tf_ready = false;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < wait_deadline) {
      // 用 0 超时单次试探，避免占用 buffer 内部锁太久
      if (tf_buffer->canTransform(
          wait_target_frame, wait_source_frame, tf2::TimePointZero,
          tf2::durationFromSec(0.0)))
      {
        tf_ready = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    const auto elapsed_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - wait_start).count();

    if (tf_ready) {
      RCLCPP_INFO(node->get_logger(),
        "✓ TF 就绪: %s -> %s (耗时 %ld ms)",
        wait_target_frame.c_str(), wait_source_frame.c_str(),
        static_cast<long>(elapsed_ms));
    } else {
      RCLCPP_WARN(node->get_logger(),
        "TF 等待超时 (%ld ms): %s -> %s 仍未就绪，仍将开始执行行为树",
        static_cast<long>(elapsed_ms),
        wait_target_frame.c_str(), wait_source_frame.c_str());
    }
  }

  RCLCPP_INFO(node->get_logger(), "执行行为树...");

  // 异步执行树 - 需要循环 tick 直到完成
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
    status = tree.tickOnce();
    rclcpp::spin_some(node);  // 处理回调
    std::this_thread::sleep_for(std::chrono::milliseconds(50));  // 50ms 循环 (20 Hz)
  }

  if (status == BT::NodeStatus::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "✓ 行为树执行成功!");
  } else {
    RCLCPP_ERROR(node->get_logger(), "✗ 行为树执行失败: %d", static_cast<int>(status));
  }

  rclcpp::shutdown();
  return 0;
}
