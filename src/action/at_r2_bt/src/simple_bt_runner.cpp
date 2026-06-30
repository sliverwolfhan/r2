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
#include "nav2_bt_publish_goal/publish_goals_action.hpp"
#include "nav2_bt_publish_goal/follow_spin_path_action.hpp"
#include "nav2_bt_publish_goal/climb_stair_action.hpp"
#include "nav2_bt_publish_goal/descend_stair_action.hpp"
#include "nav2_bt_publish_goal/pre_steer_align_action.hpp"
#include "nav2_bt_publish_goal/set_costmap_inflation_action.hpp"
#include "nav2_bt_publish_goal/set_mppi_params_action.hpp"
#include "nav2_bt_publish_goal/arm_task_action.hpp"
#include "nav2_bt_publish_goal/arm_cartesian_task_action.hpp"
#include "nav2_bt_publish_goal/arm_move_joint_action.hpp"
#include "nav2_bt_publish_goal/get_plan_action.hpp"
#include "nav2_bt_publish_goal/pop_next_step_action.hpp"
#include "nav2_bt_publish_goal/peek_next_step_action.hpp"
#include "nav2_bt_publish_goal/select_arm_prep_name_action.hpp"
#include "nav2_bt_publish_goal/select_weapon_params_action.hpp"
#include "nav2_bt_publish_goal/select_kfs_params_action.hpp"
#include "nav2_bt_publish_goal/select_place_slot_action.hpp"
#include "nav2_bt_publish_goal/arm_move_named_action.hpp"
#include "nav2_bt_publish_goal/arm_job_runner.hpp"
#include "nav2_bt_publish_goal/arm_move_named_async_action.hpp"
#include "nav2_bt_publish_goal/wait_arm_idle_action.hpp"
#include "nav2_bt_publish_goal/is_prep_skippable_condition.hpp"
#include "nav2_bt_publish_goal/grasp_ready_by_pose_distance_condition.hpp"
#include "nav2_bt_publish_goal/distance_servo_align_action.hpp"
#include "nav2_bt_publish_goal/dock_to_wall_action.hpp"
#include "nav2_bt_publish_goal/dock_to_tag_action.hpp"
#include "nav2_bt_publish_goal/delay_decorator.hpp"
#include "nav2_bt_publish_goal/publish_head_cmd_action.hpp"
#include "nav2_bt_publish_goal/publish_pump_cmd_action.hpp"
#include "nav2_bt_publish_goal/publish_lift_cmd_action.hpp"
#include "nav2_bt_publish_goal/set_zone_mode_action.hpp"
#include "nav2_bt_publish_goal/wait_docking_release_action.hpp"
#include "nav2_bt_publish_goal/wait_for_enter_action.hpp"

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
  factory.registerNodeType<nav2_bt_publish_goal::PublishGoalsAction>("PublishGoals");
  factory.registerNodeType<nav2_bt_publish_goal::FollowSpinPathAction>("FollowSpinPath");
  factory.registerNodeType<nav2_bt_publish_goal::ClimbStairAction>("ClimbStair");
  factory.registerNodeType<nav2_bt_publish_goal::DescendStairAction>("DescendStair");
  factory.registerNodeType<nav2_bt_publish_goal::PreSteerAlignAction>("PreSteerAlign");
  factory.registerNodeType<nav2_bt_publish_goal::SetCostmapInflationAction>("SetCostmapInflation");
  factory.registerNodeType<nav2_bt_publish_goal::SetMppiParamsAction>("SetMppiParams");
  factory.registerNodeType<nav2_bt_publish_goal::ArmTaskAction>("ArmTask");
  factory.registerNodeType<nav2_bt_publish_goal::ArmCartesianTaskAction>("ArmCartesianTask");
  factory.registerNodeType<nav2_bt_publish_goal::ArmMoveJointAction>("ArmMoveJoint");
  factory.registerNodeType<nav2_bt_publish_goal::ArmMoveNamedAction>("ArmMoveNamed");
  factory.registerNodeType<nav2_bt_publish_goal::ArmMoveNamedAsyncAction>("ArmMoveNamedAsync");
  factory.registerNodeType<nav2_bt_publish_goal::WaitArmIdleAction>("WaitArmIdle");
  factory.registerNodeType<nav2_bt_publish_goal::GetPlanAction>("GetPlan");
  factory.registerNodeType<nav2_bt_publish_goal::PopNextStepAction>("PopNextStep");
  factory.registerNodeType<nav2_bt_publish_goal::PeekNextStepAction>("PeekNextStep");
  factory.registerNodeType<nav2_bt_publish_goal::SelectArmPrepNameAction>("SelectArmPrepName");
  factory.registerNodeType<nav2_bt_publish_goal::SelectWeaponParamsAction>("SelectWeaponParams");
  factory.registerNodeType<nav2_bt_publish_goal::SelectKfsParamsAction>("SelectKfsParams");
  factory.registerNodeType<nav2_bt_publish_goal::SelectPlaceSlotAction>("SelectPlaceSlot");
  factory.registerNodeType<nav2_bt_publish_goal::IsPrepSkippableCondition>("IsPrepSkippable");
  factory.registerNodeType<nav2_bt_publish_goal::GraspReadyByPoseDistanceCondition>("GraspReadyByPoseDistance");
  factory.registerNodeType<nav2_bt_publish_goal::DistanceServoAlignAction>("DistanceServoAlign");
  factory.registerNodeType<nav2_bt_publish_goal::DockToWallAction>("DockToWall");
  factory.registerNodeType<nav2_bt_publish_goal::DockToTagAction>("DockToTag");
  factory.registerNodeType<nav2_bt_publish_goal::DelayDecorator>("DelayMs");
  factory.registerNodeType<nav2_bt_publish_goal::PublishHeadCmdAction>("PublishHeadCmd");
  factory.registerNodeType<nav2_bt_publish_goal::PublishPumpCmdAction>("PublishPumpCmd");
  factory.registerNodeType<nav2_bt_publish_goal::PublishLiftCmdAction>("PublishLiftCmd");
  factory.registerNodeType<nav2_bt_publish_goal::SetZoneModeAction>("SetZoneMode");
  factory.registerNodeType<nav2_bt_publish_goal::WaitDockingReleaseAction>("WaitDockingRelease");
  factory.registerNodeType<nav2_bt_publish_goal::WaitForEnterAction>("WaitForEnter");
  RCLCPP_INFO(node->get_logger(), "✓ PublishGoal 节点已注册");
  RCLCPP_INFO(node->get_logger(), "✓ ClimbStair 节点已注册");
  RCLCPP_INFO(node->get_logger(), "✓ DescendStair 节点已注册");
  RCLCPP_INFO(node->get_logger(), "✓ PreSteerAlign 节点已注册");
  RCLCPP_INFO(node->get_logger(), "✓ SetCostmapInflation 节点已注册");
  RCLCPP_INFO(node->get_logger(), "✓ SetMppiParams 节点已注册");
  RCLCPP_INFO(node->get_logger(), "✓ ArmTask 节点已注册");
  RCLCPP_INFO(node->get_logger(), "✓ ArmCartesianTask 节点已注册");
  RCLCPP_INFO(node->get_logger(), "✓ ArmMoveJoint 节点已注册");
  RCLCPP_INFO(node->get_logger(), "✓ ArmMoveNamed 节点已注册");
  RCLCPP_INFO(node->get_logger(), "✓ ArmMoveNamedAsync 节点已注册");
  RCLCPP_INFO(node->get_logger(), "✓ WaitArmIdle 节点已注册");
  RCLCPP_INFO(node->get_logger(), "✓ GetPlan 节点已注册");
  RCLCPP_INFO(node->get_logger(), "✓ PopNextStep 节点已注册");
  RCLCPP_INFO(node->get_logger(), "✓ PeekNextStep 节点已注册");
  RCLCPP_INFO(node->get_logger(), "✓ SelectArmPrepName 节点已注册");
  RCLCPP_INFO(node->get_logger(), "✓ SelectKfsParams 节点已注册");
  RCLCPP_INFO(node->get_logger(), "✓ SelectPlaceSlot 节点已注册");
  RCLCPP_INFO(node->get_logger(), "✓ IsPrepSkippable 节点已注册");
  RCLCPP_INFO(node->get_logger(), "✓ GraspReadyByPoseDistance 节点已注册");
  RCLCPP_INFO(node->get_logger(), "✓ DistanceServoAlign 节点已注册");
  RCLCPP_INFO(node->get_logger(), "✓ PublishHeadCmd 节点已注册");
  RCLCPP_INFO(node->get_logger(), "✓ PublishPumpCmd 节点已注册");
  RCLCPP_INFO(node->get_logger(), "✓ PublishLiftCmd 节点已注册");
  RCLCPP_INFO(node->get_logger(), "✓ SetZoneMode 节点已注册");
  RCLCPP_INFO(node->get_logger(), "✓ WaitDockingRelease 节点已注册");
  RCLCPP_INFO(node->get_logger(), "✓ WaitForEnter 节点已注册");

  // ===== 共享 ArmJobRunner：后台串行队列下发命名臂动作 =====
  // 同一份 runner 给 ArmMoveNamedAsync / WaitArmIdle 复用，
  // 自带独立 rclcpp::Node + executor + worker 线程，与主 BT tick 解耦。
  std::string arm_yaml_default;
  try {
    arm_yaml_default = ament_index_cpp::get_package_share_directory("at_r2_bt") +
      "/yaml/arm_ready_position.yaml";
  } catch (const std::exception & e) {
    RCLCPP_WARN(node->get_logger(),
      "无法定位 at_r2_bt share dir 用于 arm yaml 默认路径: %s", e.what());
  }
  auto arm_runner = std::make_shared<nav2_bt_publish_goal::ArmJobRunner>(
    "bt_arm_runner", "robotic_task", arm_yaml_default);
  RCLCPP_INFO(node->get_logger(), "✓ ArmJobRunner 后台队列已就绪");

  // 获取行为树 XML 文件路径 - 支持命令行参数
  std::string bt_file = "grasp_head_3.xml";
  if (argc > 1) {
    bt_file = argv[1];
  }

  std::string pkg_dir = ament_index_cpp::get_package_share_directory("at_r2_bt");
  std::string bt_xml = pkg_dir + "/behavior_trees/" + bt_file;

  RCLCPP_INFO(node->get_logger(), "加载行为树: %s", bt_xml.c_str());

  // 将 ROS 节点、共享 TF buffer、抓取参数放入黑板（全局）
  BT::Blackboard::Ptr blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  blackboard->set("tf_buffer", tf_buffer);
  blackboard->set("arm_runner", arm_runner);

  std::vector<std::string> weapon_priority = {
    "weapon_1", "weapon_2", "weapon_3", "weapon_4", "weapon_5", "weapon_6"};
  node->declare_parameter<std::vector<std::string>>("weapon_priority", weapon_priority);
  node->get_parameter("weapon_priority", weapon_priority);
  if (weapon_priority.empty()) {
    RCLCPP_WARN(node->get_logger(), "weapon_priority 为空，默认使用 weapon_1");
    weapon_priority.push_back("weapon_1");
  }

  // 读取某个武器(weapon_x)的某个参数(suffix), 形如参数名 "weapon_1.grasp_prep_x"
  // 同一武器可能被加载多次(无前缀 + w1_ 前缀), 故先判断是否已声明再 declare.
  const auto get_weapon_double =
    [&](const std::string & weapon, const std::string & suffix, double default_value) {
      const std::string param_name = weapon + "." + suffix;
      if (!node->has_parameter(param_name)) {
        node->declare_parameter<double>(param_name, default_value);
      }
      return node->get_parameter(param_name).as_double();
    };

  // 把一个武器的全部抓取参数, 以给定前缀(如 "w1_" / "w2_" / "w3_") 写进黑板。
  // grasp_head_2.xml 通过这些带前缀的键分别引用三套位置。
  const auto load_weapon_to_blackboard =
    [&](const std::string & weapon, const std::string & prefix) {
      const double prep_x = get_weapon_double(weapon, "grasp_prep_x", 0.0);
      const double prep_y = get_weapon_double(weapon, "grasp_prep_y", 0.0);
      const double prep_yaw = get_weapon_double(weapon, "grasp_prep_yaw", 0.0);
      const double prep_y_tol = get_weapon_double(weapon, "grasp_prep_y_tolerance", 0.08);
      const double prep_yaw_tol = get_weapon_double(weapon, "grasp_prep_yaw_tolerance", 0.20);
      const double laser_dist = get_weapon_double(weapon, "laser_target_distance", 0.615);
      const double laser_dist_tol = get_weapon_double(weapon, "laser_distance_tolerance", 0.005);

      blackboard->set(prefix + "grasp_target_name", weapon);
      blackboard->set(prefix + "grasp_prep_x", prep_x);
      blackboard->set(prefix + "grasp_prep_y", prep_y);
      blackboard->set(prefix + "grasp_prep_yaw", prep_yaw);
      blackboard->set(prefix + "grasp_prep_y_tolerance", prep_y_tol);
      blackboard->set(prefix + "grasp_prep_yaw_tolerance", prep_yaw_tol);
      blackboard->set(prefix + "grasp_laser_target_distance", laser_dist);
      blackboard->set(prefix + "grasp_laser_distance_tolerance", laser_dist_tol);

      RCLCPP_INFO(node->get_logger(),
        "✓ 抓取参数[%s%s -> %s]: prep=(%.3f, %.3f, %.3f) prep_tol=(y=%.3f, yaw=%.3f) "
        "laser=%.3f±%.3f",
        prefix.c_str(), "", weapon.c_str(), prep_x, prep_y, prep_yaw,
        prep_y_tol, prep_yaw_tol, laser_dist, laser_dist_tol);
    };

  // ---- 单目标(grasp_head.xml)向后兼容: 把 weapon_priority 第一个加载成无前缀的扁平键 ----
  const std::string selected_weapon = weapon_priority.front();
  load_weapon_to_blackboard(selected_weapon, "");

  // ---- 多目标(grasp_head_2.xml): 把 weapon_priority 全部加载成 w1_/w2_/.../wN_ 前缀键 ----
  // 位置仍来自 weapon_grasp_params.yaml; SelectWeaponParams 节点按序号选择并拷到工作键。
  for (size_t i = 0; i < weapon_priority.size(); ++i) {
    const std::string prefix = "w" + std::to_string(i + 1) + "_";  // 1-based
    load_weapon_to_blackboard(weapon_priority[i], prefix);
  }

  // ---- 连续抓取的循环控制参数 ----
  // grasp_count: 抓几个(Repeat 循环次数); grasp_start: 从第几个武器开始(1-based)。
  int grasp_count = 3;
  int grasp_start = 1;
  node->declare_parameter<int>("grasp_count", grasp_count);
  node->declare_parameter<int>("grasp_start", grasp_start);
  node->get_parameter("grasp_count", grasp_count);
  node->get_parameter("grasp_start", grasp_start);
  if (grasp_count < 1) {
    RCLCPP_WARN(node->get_logger(), "grasp_count=%d 非法, 重置为 1", grasp_count);
    grasp_count = 1;
  }
  if (grasp_start < 1) {
    RCLCPP_WARN(node->get_logger(), "grasp_start=%d 非法, 重置为 1", grasp_start);
    grasp_start = 1;
  }
  blackboard->set("grasp_count", grasp_count);
  blackboard->set("grasp_start", grasp_start);
  RCLCPP_INFO(node->get_logger(),
    "✓ 连续抓取参数: grasp_start=%d grasp_count=%d (将抓 w%d_ ~ w%d_)",
    grasp_start, grasp_count, grasp_start, grasp_start + grasp_count - 1);

  // ===== KFS 放置参数 (place_kfs_params.yaml) =====
  // 每个 KFS 一组扁平键写入黑板：<kfs_name>_prep_x / _prep_y / _prep_yaw / _place_x / _place_y
  // 例如 yaml 里 kfs_1.prep_x -> 黑板键 kfs_1_prep_x，XML 用 {kfs_1_prep_x} 引用。
  std::vector<std::string> kfs_priority = {"kfs_1", "kfs_2"};
  node->declare_parameter<std::vector<std::string>>("kfs_priority", kfs_priority);
  node->get_parameter("kfs_priority", kfs_priority);
  if (kfs_priority.empty()) {
    RCLCPP_WARN(node->get_logger(), "kfs_priority 为空，默认使用 kfs_1");
    kfs_priority.push_back("kfs_1");
  }

  // 同名 lambda 思路同 weapon: declare-on-first-touch + get.
  const auto get_kfs_double =
    [&](const std::string & kfs, const std::string & suffix, double default_value) {
      const std::string param_name = kfs + "." + suffix;
      if (!node->has_parameter(param_name)) {
        node->declare_parameter<double>(param_name, default_value);
      }
      return node->get_parameter(param_name).as_double();
    };

  for (const auto & kfs : kfs_priority) {
    const double prep_x = get_kfs_double(kfs, "prep_x", 0.0);
    const double prep_y = get_kfs_double(kfs, "prep_y", 0.0);
    const double prep_yaw = get_kfs_double(kfs, "prep_yaw", 0.0);
    const double place_x = get_kfs_double(kfs, "place_x", 0.0);
    const double place_y = get_kfs_double(kfs, "place_y", 0.0);

    blackboard->set(kfs + "_prep_x", prep_x);
    blackboard->set(kfs + "_prep_y", prep_y);
    blackboard->set(kfs + "_prep_yaw", prep_yaw);
    blackboard->set(kfs + "_place_x", place_x);
    blackboard->set(kfs + "_place_y", place_y);

    RCLCPP_INFO(node->get_logger(),
      "✓ KFS 参数[%s]: prep=(%.3f, %.3f, %.3f) place=(%.3f, %.3f)",
      kfs.c_str(), prep_x, prep_y, prep_yaw, place_x, place_y);
  }

  // ===== 放置目标 slot 参数 (place_slot_priority) =====
  // 每个 slot: <slot>_x / _y / _yaw (机器人导航预备点位姿, map 系)。
  // 数量 = 地上 KFS 数 + 1 (预装那个 KFS 也要一个 slot)。
  std::vector<std::string> place_slot_priority = {"slot_1", "slot_2", "slot_3"};
  node->declare_parameter<std::vector<std::string>>(
    "place_slot_priority", place_slot_priority);
  node->get_parameter("place_slot_priority", place_slot_priority);
  if (place_slot_priority.empty()) {
    RCLCPP_WARN(node->get_logger(), "place_slot_priority 为空，默认使用 slot_1");
    place_slot_priority.push_back("slot_1");
  }

  const auto get_slot_double =
    [&](const std::string & slot, const std::string & suffix, double default_value) {
      const std::string param_name = slot + "." + suffix;
      if (!node->has_parameter(param_name)) {
        node->declare_parameter<double>(param_name, default_value);
      }
      return node->get_parameter(param_name).as_double();
    };

  for (size_t i = 0; i < place_slot_priority.size(); ++i) {
    const std::string & slot = place_slot_priority[i];
    const double sx = get_slot_double(slot, "x", 0.0);
    const double sy = get_slot_double(slot, "y", 0.0);
    const double syaw = get_slot_double(slot, "yaw", 0.0);

    // 黑板键: slot_<index>_x, slot_<index>_y, slot_<index>_yaw
    // (index 是 1-based, 对应 place_slot_priority 第几个; SelectPlaceSlot 用这个前缀)
    const std::string prefix = "slot_" + std::to_string(i + 1) + "_";
    blackboard->set(prefix + "x", sx);
    blackboard->set(prefix + "y", sy);
    blackboard->set(prefix + "yaw", syaw);

    RCLCPP_INFO(node->get_logger(),
      "✓ 放置 slot[%s -> %sN]: (x=%.3f, y=%.3f, yaw=%.3f)",
      slot.c_str(), prefix.c_str(), sx, sy, syaw);
  }

  // ---- 连续放置的循环控制参数 ----
  // place_count: 放几个; place_start: 从第几个 KFS 开始(1-based)。
  int place_count = static_cast<int>(kfs_priority.size());
  int place_start = 1;
  node->declare_parameter<int>("place_count", place_count);
  node->declare_parameter<int>("place_start", place_start);
  node->get_parameter("place_count", place_count);
  node->get_parameter("place_start", place_start);
  if (place_count < 1) {
    RCLCPP_WARN(node->get_logger(), "place_count=%d 非法, 重置为 1", place_count);
    place_count = 1;
  }
  if (place_start < 1) {
    RCLCPP_WARN(node->get_logger(), "place_start=%d 非法, 重置为 1", place_start);
    place_start = 1;
  }
  blackboard->set("place_count", place_count);
  blackboard->set("place_start", place_start);
  RCLCPP_INFO(node->get_logger(),
    "✓ 连续放置参数: place_start=%d place_count=%d (将放 kfs_%d ~ kfs_%d)",
    place_start, place_count, place_start, place_start + place_count - 1);

  // 笛卡尔抓取(C 段)发送 data 里的 z 固定值, 不做 TF 变换。
  double kfs_grab_z = 0.0;
  node->declare_parameter<double>("kfs_grab_z", kfs_grab_z);
  node->get_parameter("kfs_grab_z", kfs_grab_z);
  blackboard->set("kfs_grab_z", kfs_grab_z);
  RCLCPP_INFO(node->get_logger(), "✓ 笛卡尔抓取固定 z: kfs_grab_z=%.3f", kfs_grab_z);

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
