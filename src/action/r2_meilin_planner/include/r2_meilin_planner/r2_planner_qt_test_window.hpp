#ifndef R2_PLANNER_QT_TEST_WINDOW_HPP_
#define R2_PLANNER_QT_TEST_WINDOW_HPP_

#include <QMainWindow>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "robot_interfaces/msg/plan.hpp"

#include "r2_meilin_planner/planner.hpp"
#include "r2_meilin_planner/block_table.hpp"

class QGraphicsScene;
class QGraphicsView;
class QPlainTextEdit;
class QPushButton;
class QSpinBox;
class QCheckBox;

class PlannerWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit PlannerWindow(
    rclcpp::Node::SharedPtr node,
    r2_planner::BlockTable blocks,
    rclcpp::Publisher<robot_interfaces::msg::Plan>::SharedPtr plan_pub,
    QWidget * parent = nullptr);

private slots:
  void on_cell_clicked();
  void on_plan_clicked();
  void on_zone_toggle_clicked();

private:
  rclcpp::Node::SharedPtr node_;
  r2_planner::BlockTable blocks_;
  rclcpp::Publisher<robot_interfaces::msg::Plan>::SharedPtr plan_pub_;
  std::vector<QPushButton *> cells_;
  std::vector<int> cell_phase_;
  std::unordered_map<int, double> node_heights_;  // 与算法同源的各方块高度（node id -> 米）
  bool zone_blue_ = false;          // false=红区(block_red.yaml)，true=蓝区(block_blue.yaml)
  QPushButton * zone_btn_ = nullptr;
  QCheckBox * ignore_height_chk_ = nullptr;  // 勾选=忽略高度（升/降代价清零、强制可上 400）
  QCheckBox * can_climb_400_chk_ = nullptr;  // 勾选=可上/下 400 台阶；取消=400 档不可通行
  QGraphicsScene * scene_;
  QGraphicsView * view_;
  QPlainTextEdit * log_;
  QSpinBox * r1_preclear_display_a_;
  QSpinBox * r1_preclear_display_b_;
  std::vector<std::string> last_path_;

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
  static QPointF scene_pos_for_node(int node_id);
  /** 重新加载红/蓝区方块表并刷新界面（序号镜像 + 按钮重绘）。 */
  void reload_zone_blocks();
};

#endif
