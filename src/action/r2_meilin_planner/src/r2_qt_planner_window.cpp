#include "r2_meilin_planner/r2_qt_planner_window.hpp"

#include "r2_meilin_planner/forest_defaults.hpp"

#include <QBrush>
#include <QFont>
#include <QGraphicsEllipseItem>
#include <QGraphicsPathItem>
#include <QGraphicsRectItem>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QPen>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSplitter>
#include <QSpinBox>
#include <QVBoxLayout>
#include <QWidget>

#include <algorithm>
#include <sstream>
#include <stdexcept>

PlannerWindow::PlannerWindow(
  rclcpp::Node::SharedPtr node,
  r2_planner::BlockTable blocks,
  rclcpp::Publisher<robot_interfaces::msg::Plan>::SharedPtr plan_pub,
  QWidget * parent)
: QMainWindow(parent),
  node_(std::move(node)),
  blocks_(std::move(blocks)),
  plan_pub_(std::move(plan_pub)),
  scene_(nullptr),
  view_(nullptr),
  log_(nullptr),
  r1_preclear_display_a_(nullptr),
  r1_preclear_display_b_(nullptr)
{
  setWindowTitle(QString::fromUtf8("R2 梅林规划 (Qt)"));
  resize(960, 520);

  auto * central = new QWidget(this);
  setCentralWidget(central);

  auto * root = new QHBoxLayout(central);
  auto * splitter = new QSplitter(Qt::Horizontal, central);
  root->addWidget(splitter);

  auto * left = new QWidget(splitter);
  auto * left_lay = new QVBoxLayout(left);
  left_lay->addWidget(new QLabel(
    QString::fromUtf8(
      "方块编号 1–12：右下为 1，向左再向上到左上为 12（与规划器内部一致）。点「规划」后界面与传入规划器的状态一致（含 R1 赛前拿走）。"),
    left));

  auto * grid = new QGridLayout();
  left_lay->addLayout(grid);

  cell_phase_.assign(12, 0);
  cells_.reserve(12);
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 3; ++c) {
      const int idx = r * 3 + c;
      auto * b = new QPushButton(left);
      b->setMinimumSize(72, 44);
      cells_.push_back(b);
      connect(b, &QPushButton::clicked, this, &PlannerWindow::on_cell_clicked);
      grid->addWidget(b, r, c);
      apply_phase_to_button(idx);
    }
  }

  left_lay->addWidget(new QLabel(
    QString::fromUtf8("R1 赛前先去哪两格（方块编号 1–12，0=无）：须为摆场中的红 R1；与旧节点 Phase2「随机拿走 2 红」同义，此处由你指定。"),
    left));
  auto * r1_row = new QHBoxLayout();
  r1_preclear_display_a_ = new QSpinBox(left);
  r1_preclear_display_b_ = new QSpinBox(left);
  for (QSpinBox * s : {r1_preclear_display_a_, r1_preclear_display_b_}) {
    s->setRange(0, 12);
    s->setValue(0);
    s->setMinimum(0);
    s->setSpecialValueText(QString::fromUtf8("无"));
  }
  r1_row->addWidget(new QLabel(QString::fromUtf8("第一格:"), left));
  r1_row->addWidget(r1_preclear_display_a_);
  r1_row->addWidget(new QLabel(QString::fromUtf8("第二格:"), left));
  r1_row->addWidget(r1_preclear_display_b_);
  r1_row->addStretch(1);
  left_lay->addLayout(r1_row);

  auto * plan_btn = new QPushButton(QString::fromUtf8("规划"), left);
  plan_btn->setMinimumHeight(40);
  connect(plan_btn, &QPushButton::clicked, this, &PlannerWindow::on_plan_clicked);
  left_lay->addWidget(plan_btn);
  left_lay->addStretch(1);

  auto * right = new QWidget(splitter);
  auto * right_lay = new QVBoxLayout(right);
  right_lay->addWidget(new QLabel(QString::fromUtf8("路径示意（黄线为 MOVE 折线）"), right));
  scene_ = new QGraphicsScene(this);
  scene_->setSceneRect(-40, -80, 360, 520);
  view_ = new QGraphicsView(scene_, right);
  view_->setMinimumSize(380, 360);
  view_->setRenderHint(QPainter::Antialiasing, true);
  right_lay->addWidget(view_);

  log_ = new QPlainTextEdit(right);
  log_->setReadOnly(true);
  log_->setMinimumHeight(140);
  QFont mono("Monospace");
  mono.setStyleHint(QFont::TypeWriter);
  log_->setFont(mono);
  right_lay->addWidget(new QLabel(QString::fromUtf8("规划输出"), right));
  right_lay->addWidget(log_);

  splitter->setStretchFactor(0, 0);
  splitter->setStretchFactor(1, 1);

  redraw_scene();
}

r2_planner::BlockState PlannerWindow::block_from_phase(int phase)
{
  switch (phase % 4) {
    case 0:
      return r2_planner::BlockState::EMPTY;
    case 1:
      return r2_planner::BlockState::R1_KFS;
    case 2:
      return r2_planner::BlockState::R2_KFS;
    default:
      return r2_planner::BlockState::FAKE_KFS;
  }
}

int PlannerWindow::display_number_from_cell_index(int index)
{
  return 12 - index;
}

int PlannerWindow::planner_node_from_display_number(int display_number)
{
  if (display_number < 1 || display_number > 12) {
    return -1;
  }
  return display_number;
}

int PlannerWindow::cell_index_from_display_number(int display_number)
{
  if (display_number < 1 || display_number > 12) {
    return -1;
  }
  return 12 - display_number;
}

int PlannerWindow::phase_from_block_state(r2_planner::BlockState state)
{
  switch (state) {
    case r2_planner::BlockState::EMPTY:
      return 0;
    case r2_planner::BlockState::R1_KFS:
      return 1;
    case r2_planner::BlockState::R2_KFS:
      return 2;
    case r2_planner::BlockState::FAKE_KFS:
      return 3;
    default:
      return 0;
  }
}

void PlannerWindow::sync_cell_phase_from_config(const r2_planner::ForestConfig & config)
{
  for (int nid = 1; nid <= 12; ++nid) {
    r2_planner::BlockState st = r2_planner::BlockState::EMPTY;
    const auto it = config.initial_items.find(nid);
    if (it != config.initial_items.end()) {
      st = it->second;
    }
    const int idx = cell_index_from_display_number(nid);
    if (idx >= 0) {
      cell_phase_[static_cast<size_t>(idx)] = phase_from_block_state(st);
    }
  }
  for (int i = 0; i < 12; ++i) {
    apply_phase_to_button(i);
  }
}

void PlannerWindow::apply_phase_to_button(int index)
{
  if (index < 0 || index >= static_cast<int>(cells_.size())) {
    return;
  }
  QPushButton * b = cells_[static_cast<size_t>(index)];
  const int ph = cell_phase_[static_cast<size_t>(index)] % 4;
  QString text;
  QString style;
  switch (ph) {
    case 0:
      text = QString::fromUtf8("空");
      style = "background:#e8e8e8;color:#333;";
      break;
    case 1:
      text = QString::fromUtf8("R1");
      style = "background:#f08080;color:#200;";
      break;
    case 2:
      text = QString::fromUtf8("R2");
      style = "background:#90ee90;color:#030;";
      break;
    default:
      text = QString::fromUtf8("假");
      style = "background:#87ceeb;color:#012;";
      break;
  }
  b->setText(QString::number(display_number_from_cell_index(index)) + QString::fromUtf8(" ") + text);
  b->setStyleSheet(QString("font-weight:bold;border:1px solid #555;%1").arg(style));
}

void PlannerWindow::on_cell_clicked()
{
  auto * snd = qobject_cast<QPushButton *>(sender());
  if (!snd) {
    return;
  }
  auto it = std::find(cells_.begin(), cells_.end(), snd);
  if (it == cells_.end()) {
    return;
  }
  const int idx = static_cast<int>(std::distance(cells_.begin(), it));
  cell_phase_[static_cast<size_t>(idx)] = (cell_phase_[static_cast<size_t>(idx)] + 1) % 4;
  apply_phase_to_button(idx);
  redraw_scene();
}

void PlannerWindow::build_config_from_ui(r2_planner::ForestConfig & config) const
{
  r2_planner::fill_default_forest_topology(config);
  for (int i = 1; i <= 12; ++i) {
    config.initial_items[i] = r2_planner::BlockState::EMPTY;
  }
  for (int display = 1; display <= 12; ++display) {
    const int nid = planner_node_from_display_number(display);
    const int idx = cell_index_from_display_number(display);
    if (nid < 1 || idx < 0) {
      continue;
    }
    config.initial_items[nid] = block_from_phase(cell_phase_[static_cast<size_t>(idx)]);
  }
}

void PlannerWindow::apply_r1_preclear_selection(r2_planner::ForestConfig & config, std::string * notes_out) const
{
  std::vector<int> displays;
  const int a = r1_preclear_display_a_->value();
  const int b = r1_preclear_display_b_->value();
  if (a >= 1 && a <= 12) {
    displays.push_back(a);
  }
  if (b >= 1 && b <= 12 && std::find(displays.begin(), displays.end(), b) == displays.end()) {
    displays.push_back(b);
  }

  std::ostringstream oss;
  if (displays.empty()) {
    oss << "# R1 赛前：未指定（规划仍可在路径中用 R1_CLEAR 清最多 2 个红格）\n";
    oss << "# （未指定时传入布局与当前摆场一致）\n";
    if (notes_out) {
      notes_out->append(oss.str());
    }
    return;
  }

  oss << "# R1 赛前拿走：下方规划将使用「拿走之后」的摆场；界面会同步为该摆场\n";
  for (int d : displays) {
    const int nid = planner_node_from_display_number(d);
    if (nid < 1) {
      continue;
    }
    r2_planner::BlockState & cell = config.initial_items[nid];
    if (cell == r2_planner::BlockState::R1_KFS) {
      cell = r2_planner::BlockState::EMPTY;
      oss << "#   R1 已拿走方块 " << d << "\n";
    } else {
      oss << "#   警告: 方块 " << d << " 当前不是 R1，已忽略\n";
    }
  }
  if (notes_out) {
    notes_out->append(oss.str());
  }
}

QPointF PlannerWindow::scene_pos_for_node(int node_id)
{
  if (node_id == 0) {
    return QPointF(120, 20 + 4 * 100 + 30);
  }
  if (node_id == 13) {
    return QPointF(120, -40);
  }
  if (node_id >= 1 && node_id <= 12) {
    const int legacy = 13 - node_id;
    const int r = (legacy - 1) / 3;
    const int c = (legacy - 1) % 3;
    return QPointF(20 + c * 110 + 45, 20 + r * 100 + 45);
  }
  return QPointF(0, 0);
}

void PlannerWindow::redraw_scene()
{
  scene_->clear();

  for (int id = 1; id <= 12; ++id) {
    const int legacy = 13 - id;
    const int r = (legacy - 1) / 3;
    const int c = (legacy - 1) % 3;
    const QRectF rect(20 + c * 110, 20 + r * 100, 100, 85);
    const int idx = cell_index_from_display_number(id);
    const int ph = (idx >= 0) ? (cell_phase_[static_cast<size_t>(idx)] % 4) : 0;
    QColor fill(230, 230, 230);
    if (ph == 1) {
      fill = QColor(240, 128, 128);
    } else if (ph == 2) {
      fill = QColor(144, 238, 144);
    } else if (ph == 3) {
      fill = QColor(135, 206, 235);
    }
    auto * cell = scene_->addRect(rect, QPen(Qt::darkGray, 1), QBrush(fill));
    cell->setZValue(0);
    auto * t = scene_->addSimpleText(QString::number(id));
    t->setBrush(Qt::black);
    t->setPos(rect.left() + 4, rect.top() + 4);
    t->setZValue(1);
  }

  auto * entry = scene_->addEllipse(
    QRectF(scene_pos_for_node(0).x() - 8, scene_pos_for_node(0).y() - 8, 16, 16), QPen(Qt::black),
    QBrush(QColor(255, 220, 0)));
  entry->setZValue(2);

  auto * exit = scene_->addEllipse(
    QRectF(scene_pos_for_node(13).x() - 8, scene_pos_for_node(13).y() - 8, 16, 16), QPen(Qt::black),
    QBrush(QColor(200, 200, 255)));
  exit->setZValue(2);

  if (last_path_.empty()) {
    view_->fitInView(scene_->sceneRect(), Qt::KeepAspectRatio);
    return;
  }

  QPainterPath road;
  QPointF robot_pt = scene_pos_for_node(0);
  road.moveTo(robot_pt);
  bool any_move_segment = false;
  for (const auto & s : last_path_) {
    if (s.rfind("MOVE to ", 0) == 0) {
      try {
        const int nid = std::stoi(s.substr(8));
        robot_pt = scene_pos_for_node(nid);
        road.lineTo(robot_pt);
        any_move_segment = true;
      } catch (const std::exception &) {
      }
    }
  }
  if (any_move_segment) {
    auto * path_item = scene_->addPath(road, QPen(QColor(255, 200, 0), 3));
    path_item->setZValue(1.5);
  }

  auto * bot = scene_->addEllipse(QRectF(robot_pt.x() - 10, robot_pt.y() - 10, 20, 20), QPen(Qt::black, 2),
    QBrush(QColor(255, 255, 0)));
  bot->setZValue(3);

  view_->fitInView(scene_->sceneRect(), Qt::KeepAspectRatio);
}

namespace {

QString format_step(const robot_interfaces::msg::PlanStep & s)
{
  using PS = robot_interfaces::msg::PlanStep;
  switch (s.type) {
    case PS::TYPE_MOVE: {
      const char * dir = "SAME";
      if (s.stair_dir == PS::STAIR_UP) {
        dir = "UP";
      } else if (s.stair_dir == PS::STAIR_DOWN) {
        dir = "DOWN";
      }
      return QString::asprintf(
        "MOVE %d->%d  prep=(%.2f, %.2f, %.2f)  |dh|=%.2fm %s",
        s.from_id, s.target_id,
        s.prep_pose.x, s.prep_pose.y, s.prep_pose.theta,
        s.abs_dh, dir);
    }
    case PS::TYPE_PICK:
      return QString::asprintf(
        "PICK at %d (robot@%d)  prep=(%.2f, %.2f, %.2f)  cube=(%.2f, %.2f)  block_h=%.2fm",
        s.target_id, s.from_id,
        s.prep_pose.x, s.prep_pose.y, s.prep_pose.theta,
        s.cube_x, s.cube_y, s.block_height);
    case PS::TYPE_PUSH:
      return QString::asprintf(
        "PUSH %d (robot@%d)  prep=(%.2f, %.2f, %.2f)  cube=(%.2f, %.2f)  block_h=%.2fm",
        s.target_id, s.from_id,
        s.prep_pose.x, s.prep_pose.y, s.prep_pose.theta,
        s.cube_x, s.cube_y, s.block_height);
    default:
      return QString::asprintf("UNKNOWN type=%d target=%d", s.type, s.target_id);
  }
}

}  // namespace

void PlannerWindow::on_plan_clicked()
{
  r2_planner::ForestConfig config;
  build_config_from_ui(config);

  std::string preamble;
  apply_r1_preclear_selection(config, &preamble);
  sync_cell_phase_from_config(config);
  redraw_scene();

  r2_planner::R2MeilinPlanner planner(config, blocks_);

  std::ostringstream oss;
  oss << preamble;

  std::vector<robot_interfaces::msg::PlanStep> steps;
  bool plan_ok = false;
  try {
    steps = planner.planPathStruct();
    plan_ok = !steps.empty();
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(node_->get_logger(), "planPathStruct failed: %s", ex.what());
    oss << "FAIL: " << ex.what() << '\n';
  }

  if (!plan_ok) {
    if (steps.empty() && oss.str().find("FAIL") == std::string::npos) {
      oss << "FAIL: No path found.\n";
    }
    log_->setPlainText(QString::fromStdString(oss.str()));
    last_path_.clear();
    redraw_scene();
    r1_preclear_display_a_->setValue(0);
    r1_preclear_display_b_->setValue(0);
    return;
  }

  // 发布结构化计划
  robot_interfaces::msg::Plan plan_msg;
  plan_msg.header.stamp = node_->now();
  plan_msg.header.frame_id = "map";
  plan_msg.steps = steps;

  // 同步生成人类可读字符串（GUI + 日志 + plan.notes）
  for (const auto & step : steps) {
    const QString line = format_step(step);
    oss << line.toStdString() << '\n';
    last_path_.push_back(line.toStdString());
  }
  plan_msg.notes = oss.str();
  log_->setPlainText(QString::fromStdString(plan_msg.notes));

  if (plan_pub_) {
    plan_pub_->publish(plan_msg);
  }

  if (!preamble.empty()) {
    std::istringstream ps(preamble);
    std::string pline;
    while (std::getline(ps, pline)) {
      if (!pline.empty()) {
        RCLCPP_INFO(node_->get_logger(), "%s", pline.c_str());
      }
    }
  }
  RCLCPP_INFO(node_->get_logger(), "Planned %zu steps.", steps.size());
  for (const auto & step : steps) {
    RCLCPP_INFO(node_->get_logger(), "%s", format_step(step).toStdString().c_str());
  }

  // 重新构造仅供 redraw_scene() 使用的 MOVE 字符串路径
  last_path_.clear();
  for (const auto & step : steps) {
    if (step.type == robot_interfaces::msg::PlanStep::TYPE_MOVE) {
      last_path_.push_back("MOVE to " + std::to_string(step.target_id));
    }
  }

  r1_preclear_display_a_->setValue(0);
  r1_preclear_display_b_->setValue(0);

  redraw_scene();
}
