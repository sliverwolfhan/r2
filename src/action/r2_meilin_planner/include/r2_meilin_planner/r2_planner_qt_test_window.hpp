#ifndef R2_PLANNER_QT_TEST_WINDOW_HPP_
#define R2_PLANNER_QT_TEST_WINDOW_HPP_

#include <QMainWindow>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

#include "robot_interfaces/msg/plan.hpp"

#include "r2_meilin_planner/planner.hpp"
#include "r2_meilin_planner/block_table.hpp"

class QGraphicsScene;
class QGraphicsView;
class QPlainTextEdit;
class QPushButton;
class QSpinBox;
class QDoubleSpinBox;
class QCheckBox;

/// planner_params.yaml 读来的规划基线（代价 + 偏移 + R1 消失 + 高度开关）。
/// 由 main() 从包内 config/planner_params.yaml 解析后传入窗口，作为「yaml 打底」，
/// 界面控件在其之上覆盖对应项。字段默认值与 r2_planner::CostConfig / ForestConfig 一致，
/// 保证未读到 yaml 时行为与历史默认相同。
struct PlannerParams
{
  r2_planner::CostConfig cost;            // 全部 A* 代价（move/pick/push/turn/climb/descend/preferred_bonus...）
  double move_prep_offset  = 0.15;
  double grasp_prep_offset = 0.12;
  double block_height_offset = -0.08;
  double grasp_prep_theta_offset = 0.0;
  double move_prep_theta_offset = 0.0;
  double wait_cost = 1.0;
  bool   r1_timed_removal_enable = false;
  int    r1_removal_steps = 3;
  bool   ignore_height = false;           // true → 升/降代价清零、强制可上 400
  bool   loaded = false;                  // 成功解析到 yaml 时为 true（仅用于日志）
};

class PlannerWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit PlannerWindow(
    rclcpp::Node::SharedPtr node,
    r2_planner::BlockTable blocks,
    rclcpp::Publisher<robot_interfaces::msg::Plan>::SharedPtr plan_pub,
    PlannerParams params,
    QWidget * parent = nullptr);

private slots:
  void on_cell_clicked();
  void on_plan_clicked();
  void on_zone_toggle_clicked();

private:
  // 监视话题回调（在 Qt 线程内经 spin_some 触发，可直接改控件）。
  void on_kfs_msg(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
  void on_plan_msg(const robot_interfaces::msg::Plan::SharedPtr msg);
  // 下位机码 -> 界面相位（0空/1R1/2R2/3假/4R1待）。
  static int phase_from_code(int code);

private:
  rclcpp::Node::SharedPtr node_;
  r2_planner::BlockTable blocks_;
  rclcpp::Publisher<robot_interfaces::msg::Plan>::SharedPtr plan_pub_;
  PlannerParams params_base_;   // planner_params.yaml 读来的代价基线（build_config_from_ui 先套用，UI 再覆盖）
  std::vector<QPushButton *> cells_;
  std::vector<int> cell_phase_;
  std::unordered_map<int, double> node_heights_;  // 与算法同源的各方块高度（node id -> 米）
  bool zone_blue_ = false;          // false=红区(block_red.yaml)，true=蓝区(block_blue.yaml)
  QPushButton * zone_btn_ = nullptr;
  QCheckBox * ignore_height_chk_ = nullptr;  // 勾选=忽略高度（升/降代价清零、强制可上 400）
  QCheckBox * can_climb_400_chk_ = nullptr;  // 勾选=可上/下 400 台阶；取消=400 档不可通行
  QCheckBox * r1_timed_removal_chk_ = nullptr;   // 勾选=R1待块(码1)定时消失，可原地 WAIT 等它让开
  QSpinBox * r1_removal_steps_spin_ = nullptr;   // R2 每走几步 R1 消失一个
  QDoubleSpinBox * wait_cost_spin_ = nullptr;    // 原地等待一步的代价
  QGraphicsScene * scene_;
  QGraphicsView * view_;
  QPlainTextEdit * log_;
  QSpinBox * r1_preclear_display_a_;
  QSpinBox * r1_preclear_display_b_;
  std::vector<std::string> last_path_;
  std::vector<robot_interfaces::msg::PlanStep> last_steps_;  // 最近一次规划的完整步骤（含真实 map 坐标 prep_pose），供真实坐标绘图

  // 监视模式：订阅真车话题，收到布局/路径就画出来（不本地重规划）。
  QCheckBox * monitor_chk_ = nullptr;   // 勾选=监视话题自动填充布局与路径
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr kfs_sub_;
  rclcpp::Subscription<robot_interfaces::msg::Plan>::SharedPtr plan_monitor_sub_;
  // 上一次已应用的 kfs 布局（12 个下位机码）。kfs 话题会周期/latched 反复发同一布局，
  // 若每条都 apply+redraw 会导致梅林格子一闪一闪；仅在布局真正变化时才刷新。
  std::vector<int> last_kfs_codes_;

  static r2_planner::BlockState block_from_phase(int phase);
  /** 显示序号 <-> 按钮下标。红区：第一行 3/2/1…最后一行 12/11/10。
      蓝区（zone_blue_）：每行左右翻转（1/2/3…10/11/12），对应 y=0 物理镜像。 */
  int display_number_from_cell_index(int index) const;
  static int planner_node_from_display_number(int display_number);
  int cell_index_from_display_number(int display_number) const;
  static int phase_from_block_state(r2_planner::BlockState state);
  /** Updates button phases from config so the UI matches exactly what is passed to the planner. */
  void sync_cell_phase_from_config(const r2_planner::ForestConfig & config);
  void apply_phase_to_button(int index);
  void build_config_from_ui(r2_planner::ForestConfig & config) const;
  /** R1 removes up to two R1_KFS cells before planning (simulates the pre-game R1 pick). */
  void apply_r1_preclear_selection(r2_planner::ForestConfig & config, std::string * notes_out) const;
  void redraw_scene();
  /** 重新加载红/蓝区方块表并刷新界面（序号镜像 + 按钮重绘）。 */
  void reload_zone_blocks();
};

#endif
