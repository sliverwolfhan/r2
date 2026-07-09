#include <array>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "robot_interfaces/msg/plan.hpp"

#include "r2_meilin_planner/planner.hpp"
#include "r2_meilin_planner/forest_defaults.hpp"
#include "r2_meilin_planner/block_table.hpp"

namespace {

// 下位机 kfs_data 取值约定（见 at_r2_serial_bridge）：
//   0:空  1:R1要取走的  2:R2  3:Fake  4:R1留下未取的KFS
// 对 R2 而言：1(R1已取走)与 0 一样是空的、无 KFS、可通行；
//             4(R1未取，实物还在) 当障碍处理，归为 R1_KFS。
r2_planner::BlockState state_from_code(int32_t code)
{
  using r2_planner::BlockState;
  switch (code) {
    case 0:
      return BlockState::EMPTY;
    case 1:
      return BlockState::EMPTY;    // R1 要取走，等 R2 上场已是空
    case 2:
      return BlockState::R2_KFS;
    case 3:
      return BlockState::FAKE_KFS;
    case 4:
      return BlockState::R1_KFS;  // R1未取，实物仍在，当障碍
    default:
      return BlockState::EMPTY;   // 未知值兜底为空
  }
}

const char * state_name(r2_planner::BlockState s)
{
  using r2_planner::BlockState;
  switch (s) {
    case BlockState::EMPTY:      return "EMPTY";
    case BlockState::R1_KFS:     return "R1_KFS";
    case BlockState::R2_KFS:     return "R2_KFS";
    case BlockState::FAKE_KFS:   return "FAKE_KFS";
    case BlockState::R1_PENDING: return "R1_PENDING";
  }
  return "?";
}

}  // namespace

class KfsSubscriberNode : public rclcpp::Node
{
public:
  KfsSubscriberNode()
  : Node("kfs_subscriber_node")
  {
    // 红蓝区：red → block_red.yaml，blue → block_blue.yaml
    const std::string zone = this->declare_parameter<std::string>("zone", "red");
    zone_blue_ = (zone == "blue");
    const std::string share =
      ament_index_cpp::get_package_share_directory("r2_meilin_planner");
    const std::string block_file =
      (zone == "blue") ? "block_blue.yaml" : "block_red.yaml";
    blocks_path_ = share + "/config/" + block_file;

    // 全局坐标偏移：补偿定位误差，加载时叠加到 yaml 原值上（不改 yaml 文件）
    r2_planner::BlockOffsets block_offsets;
    block_offsets.map_x  = this->declare_parameter<double>("map_x_offset", 0.0);
    block_offsets.map_y  = this->declare_parameter<double>("map_y_offset", 0.0);
    block_offsets.cube_x = this->declare_parameter<double>("cube_x_offset", 0.0);
    block_offsets.cube_y = this->declare_parameter<double>("cube_y_offset", 0.0);

    std::string err;
    if (!blocks_.loadFromYaml(blocks_path_, &err, block_offsets)) {
      RCLCPP_FATAL(this->get_logger(), "加载 %s 失败: %s", blocks_path_.c_str(), err.c_str());
      throw std::runtime_error(err);
    }

    // 代价参数（默认值与 CostConfig 默认一致）
    cost_.move_cost        = this->declare_parameter<double>("move_cost", cost_.move_cost);
    cost_.descend_200_cost = this->declare_parameter<double>("descend_200_cost", cost_.descend_200_cost);
    cost_.descend_400_cost = this->declare_parameter<double>("descend_400_cost", cost_.descend_400_cost);
    cost_.pick_cost        = this->declare_parameter<double>("pick_cost", cost_.pick_cost);
    cost_.push_cost        = this->declare_parameter<double>("push_cost", cost_.push_cost);
    cost_.climb_200_cost   = this->declare_parameter<double>("climb_200_cost", cost_.climb_200_cost);
    cost_.climb_400_cost   = this->declare_parameter<double>("climb_400_cost", cost_.climb_400_cost);
    cost_.turn_cost        = this->declare_parameter<double>("turn_cost", cost_.turn_cost);
    cost_.can_climb_400    = this->declare_parameter<bool>("can_climb_400", cost_.can_climb_400);
    cost_.preferred_column_pick_bonus = this->declare_parameter<double>(
      "preferred_column_pick_bonus", cost_.preferred_column_pick_bonus);

    // 准备位姿偏移量（米，越小越靠近目标），MOVE 与 PICK/PUSH 分开设
    move_prep_offset_  = this->declare_parameter<double>("move_prep_offset", 0.6);
    grasp_prep_offset_ = this->declare_parameter<double>("grasp_prep_offset", 0.6);

    // PICK/PUSH 发布的 block_height = (target.height - from.height) + 此偏移
    // 仅影响发布字段，不进 A* 代价
    block_height_offset_ = this->declare_parameter<double>("block_height_offset", 0.0);

    // PICK/PUSH 发布的 prep_pose.theta 偏移（弧度），左右两档共用
    grasp_prep_theta_offset_ = this->declare_parameter<double>("grasp_prep_theta_offset", 0.0);

    // MOVE 发布的 prep_pose.theta 偏移（弧度），不影响 turn_deg
    move_prep_theta_offset_ = this->declare_parameter<double>("move_prep_theta_offset", 0.0);

    // R1 块定时消失：把码1的 R1 块建模为"会随时间让开的硬障碍"。
    //   enable=false（默认）时码1仍当空地，行为与历史一致。
    r1_timed_removal_enable_ = this->declare_parameter<bool>("r1_timed_removal_enable", false);
    r1_removal_steps_ = this->declare_parameter<int>("r1_removal_steps", 3);
    // 原地等待一步的代价，默认取已被 yaml 覆盖后的 move_cost（故放在 move_cost 之后声明）。
    wait_cost_ = this->declare_parameter<double>("wait_cost", cost_.move_cost);

    // 高度开关：true 时把升/降代价清零并强制可上 400，使路径规划完全忽略高度
    const bool ignore_height = this->declare_parameter<bool>("ignore_height", false);
    if (ignore_height) {
      cost_.climb_200_cost   = 0.0;
      cost_.climb_400_cost   = 0.0;
      cost_.descend_200_cost = 0.0;
      cost_.descend_400_cost = 0.0;
      cost_.can_climb_400    = true;
    }

    // 与 plan_yaml_publisher_node 一致：同话题、同 QoS（latched），下游 BT 无需改动
    plan_pub_ = this->create_publisher<robot_interfaces::msg::Plan>(
      "/r2_planner/plan", rclcpp::QoS(1).transient_local().reliable());

    sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
      "/AT_R2/kfs_positions", 10,
      std::bind(&KfsSubscriberNode::on_kfs, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "zone=%s, 方块表 %s，can_climb_400=%s，ignore_height=%s，等待 KFS 数据后规划并发布 /r2_planner/plan",
      zone.c_str(), blocks_path_.c_str(),
      cost_.can_climb_400 ? "true" : "false", ignore_height ? "true" : "false");
  }

private:
  void on_kfs(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != 12) {
      RCLCPP_WARN(
        this->get_logger(), "kfs_positions 长度异常: 期望 12, 收到 %zu", msg->data.size());
      return;
    }

    // 布局变化时才重规划：与上次成功规划的 KFS 布局逐位相同则跳过，
    // 内容一变（KFS 更新）立即重新规划并发布，避免同一布局被 10Hz 重复刷路径。
    if (msg->data == last_kfs_) {
      return;
    }

    // 1) 解析：data[i] 对应方块 id = i + 1（1..12）
    r2_planner::ForestConfig config;
    r2_planner::fill_default_forest_topology(config);
    config.cost = cost_;  // 应用可调代价
    config.move_prep_offset  = move_prep_offset_;
    config.grasp_prep_offset = grasp_prep_offset_;
    config.block_height_offset = block_height_offset_;
    config.grasp_prep_theta_offset = grasp_prep_theta_offset_;
    config.move_prep_theta_offset = move_prep_theta_offset_;
    config.r1_timed_removal_enable = r1_timed_removal_enable_;
    config.r1_removal_steps = r1_removal_steps_;
    config.wait_cost = wait_cost_;
    // 抓取偏好：机器人物理左手列（+y 那一列）。红区 {3,6,9,12}，蓝区镜像 {1,4,7,10}。
    config.preferred_pick_nodes = zone_blue_
      ? std::unordered_set<int>{1, 4, 7, 10}
      : std::unordered_set<int>{3, 6, 9, 12};

    std::string line;
    for (int i = 0; i < 12; ++i) {
      const int32_t code = msg->data[static_cast<size_t>(i)];
      r2_planner::BlockState s = state_from_code(code);
      // 启用定时消失时，码1（R1 正在收取的目标）从"空地"改判为 R1_PENDING（定时消失障碍）。
      // 关闭时保持 state_from_code 的历史映射（码1→EMPTY），行为逐位一致。
      if (r1_timed_removal_enable_ && code == 1) {
        s = r2_planner::BlockState::R1_PENDING;
      }
      config.initial_items[i + 1] = s;
      line += " [" + std::to_string(i + 1) + "]" + state_name(s);
    }
    RCLCPP_INFO(this->get_logger(), "解析 KFS 布局:%s", line.c_str());

    // 用 block yaml 的 height（米）覆写节点高度，使代价与坐标用同一份数据
    for (int id = 0; id <= 13; ++id) {
      if (blocks_.has(id)) {
        config.node_heights[id] = blocks_.at(id).height;
      }
    }

    // 2) 规划：注入 blocks 后产出带 map 系坐标的结构化步骤
    r2_planner::R2MeilinPlanner planner(config, blocks_);
    std::vector<robot_interfaces::msg::PlanStep> steps;
    try {
      steps = planner.planPathStruct();
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(this->get_logger(), "规划失败: %s", ex.what());
      return;
    }

    if (steps.empty()) {
      RCLCPP_WARN(this->get_logger(), "规划无可行路径（KFS 布局可能不可解），等待下一帧");
      return;
    }

    // 3) 发布
    robot_interfaces::msg::Plan plan;
    plan.header.stamp = this->get_clock()->now();
    plan.notes = "plan from /AT_R2/kfs_positions";
    plan.steps = std::move(steps);
    plan_pub_->publish(plan);
    last_kfs_ = msg->data;  // 记录本次成功规划的布局，下次相同则跳过

    RCLCPP_INFO(
      this->get_logger(), "已规划 %zu 步并发布到 /r2_planner/plan", plan.steps.size());
  }

  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_;
  rclcpp::Publisher<robot_interfaces::msg::Plan>::SharedPtr plan_pub_;
  r2_planner::BlockTable blocks_;
  r2_planner::CostConfig cost_;
  double move_prep_offset_ = 0.6;
  double grasp_prep_offset_ = 0.6;
  double block_height_offset_ = 0.0;
  double grasp_prep_theta_offset_ = 0.0;
  double move_prep_theta_offset_ = 0.0;
  bool r1_timed_removal_enable_ = false;
  int r1_removal_steps_ = 3;
  double wait_cost_ = 1.0;
  std::string blocks_path_;
  bool zone_blue_ = false;
  std::vector<int32_t> last_kfs_;  // 上次成功规划的 KFS 布局（12 元），用于布局去重
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KfsSubscriberNode>());
  rclcpp::shutdown();
  return 0;
}
