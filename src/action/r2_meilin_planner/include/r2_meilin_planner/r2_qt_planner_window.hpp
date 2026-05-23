#ifndef R2_QT_PLANNER_WINDOW_HPP_
#define R2_QT_PLANNER_WINDOW_HPP_

#include <QMainWindow>
#include <memory>
#include <string>
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

private:
  rclcpp::Node::SharedPtr node_;
  r2_planner::BlockTable blocks_;
  rclcpp::Publisher<robot_interfaces::msg::Plan>::SharedPtr plan_pub_;
  std::vector<QPushButton *> cells_;
  std::vector<int> cell_phase_;
  QGraphicsScene * scene_;
  QGraphicsView * view_;
  QPlainTextEdit * log_;
  QSpinBox * r1_preclear_display_a_;
  QSpinBox * r1_preclear_display_b_;
  std::vector<std::string> last_path_;

  static r2_planner::BlockState block_from_phase(int phase);
  /** 与规划器方块 id 一致：右下=1 … 左上=12（按钮角标）。 */
  static int display_number_from_cell_index(int index);
  static int planner_node_from_display_number(int display_number);
  /** 按钮下标 0=左上 … 11=右下；方块 id = 12 - index。 */
  static int cell_index_from_display_number(int display_number);
  static int phase_from_block_state(r2_planner::BlockState state);
  /** Updates button phases from config so the UI matches exactly what is passed to the planner. */
  void sync_cell_phase_from_config(const r2_planner::ForestConfig & config);
  void apply_phase_to_button(int index);
  void build_config_from_ui(r2_planner::ForestConfig & config) const;
  /** R1 removes up to two R1_KFS cells before planning (same idea as r2_planner_node Phase 2). */
  void apply_r1_preclear_selection(r2_planner::ForestConfig & config, std::string * notes_out) const;
  void redraw_scene();
  static QPointF scene_pos_for_node(int node_id);
};

#endif
