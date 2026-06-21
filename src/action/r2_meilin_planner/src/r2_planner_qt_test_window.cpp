#include "r2_meilin_planner/r2_planner_qt_test_window.hpp"

#include "r2_meilin_planner/forest_defaults.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <QBrush>
#include <QFont>
#include <QCheckBox>
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
#include <iomanip>
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
  setFixedSize(950, 920);

  auto * central = new QWidget(this);
  setCentralWidget(central);

  auto * root = new QHBoxLayout(central);
  auto * splitter = new QSplitter(Qt::Horizontal, central);
  root->addWidget(splitter);

  auto * left = new QWidget(splitter);
  auto * left_lay = new QVBoxLayout(left);
  {
    auto * hint = new QLabel(
      QString::fromUtf8(
        "方块编号 1–12：第一行为 1/2/3（左起 1），向下到最后一行 10/11/12（红区）；切蓝区每行左右镜像。点「规划」后界面与传入规划器的状态一致（含 R1 赛前拿走）。"),
      left);
    hint->setWordWrap(true);
    left_lay->addWidget(hint);
  }

  auto * grid = new QGridLayout();
  left_lay->addLayout(grid);

  cell_phase_.assign(12, 0);
  cells_.reserve(12);
  // 缓存各方块高度（与算法同源：fill_default_forest_topology）
  {
    r2_planner::ForestConfig hcfg;
    r2_planner::fill_default_forest_topology(hcfg);
    node_heights_ = hcfg.node_heights;
  }
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 3; ++c) {
      const int idx = r * 3 + c;
      auto * b = new QPushButton(left);
      b->setMinimumSize(56, 40);
      cells_.push_back(b);
      connect(b, &QPushButton::clicked, this, &PlannerWindow::on_cell_clicked);
      grid->addWidget(b, r, c);
      apply_phase_to_button(idx);
    }
  }

  // 红/蓝区切换：换底层方块坐标表 + 界面序号每行左右镜像。
  zone_btn_ = new QPushButton(left);
  zone_btn_->setMinimumHeight(32);
  connect(zone_btn_, &QPushButton::clicked, this, &PlannerWindow::on_zone_toggle_clicked);
  left_lay->addWidget(zone_btn_);
  zone_btn_->setText(QString::fromUtf8(zone_blue_ ? "当前：蓝区（点击切到红区）" : "当前：红区（点击切到蓝区）"));

  // 忽略高度开关：勾选后升/降代价清零、强制可上 400（等价于启动参数 ignore_height:=true）。
  ignore_height_chk_ = new QCheckBox(QString::fromUtf8("忽略台阶高度"), left);
  ignore_height_chk_->setToolTip(
    QString::fromUtf8("勾选后：爬升/下降代价清零、强制可上 400（等价于 ignore_height:=true）"));
  left_lay->addWidget(ignore_height_chk_);

  // 能否上/下 400 开关：默认勾选（可上下）；取消勾选后 400 档不可通行，A* 自动绕开。
  can_climb_400_chk_ = new QCheckBox(QString::fromUtf8("可上/下 400 台阶"), left);
  can_climb_400_chk_->setChecked(true);
  can_climb_400_chk_->setToolTip(
    QString::fromUtf8("取消勾选后：400 档台阶视为不可通行，A* 绕开（等价于 can_climb_400:=false）"));
  left_lay->addWidget(can_climb_400_chk_);

  {
    auto * r1_hint = new QLabel(
      QString::fromUtf8("R1 赛前取走哪两个方块（填序号 1–12，0=不填）：填的格子在规划时按已取走处理。"),
      left);
    r1_hint->setWordWrap(true);
    left_lay->addWidget(r1_hint);
  }
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
  view_->setMinimumSize(240, 360);
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

int PlannerWindow::display_number_from_cell_index(int index) const
{
  // 行 r=index/3，列 c=index%3。
  //   红区：每行左右顺序 1/2/3…10/11/12 → 序号 = r*3 + 1 + c。
  //   蓝区：每行左右翻转（c→2-c），即 3/2/1…12/11/10 → 序号 = (r+1)*3 - c。
  const int r = index / 3;
  const int c = index % 3;
  return zone_blue_ ? ((r + 1) * 3 - c) : (r * 3 + 1 + c);
}

int PlannerWindow::planner_node_from_display_number(int display_number)
{
  if (display_number < 1 || display_number > 12) {
    return -1;
  }
  return display_number;
}

int PlannerWindow::cell_index_from_display_number(int display_number) const
{
  if (display_number < 1 || display_number > 12) {
    return -1;
  }
  // display_number_from_cell_index 的逆。
  const int d = display_number - 1;
  const int r = d / 3;
  // 红区：c=d%3；蓝区：c=(r+1)*3-display。
  const int c = zone_blue_ ? ((r + 1) * 3 - display_number) : (d % 3);
  return r * 3 + c;
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
  const int disp = display_number_from_cell_index(index);
  double h = 0.0;
  auto it = node_heights_.find(disp);
  if (it != node_heights_.end()) {
    h = it->second;
  }
  b->setText(
    QString::number(disp) + QString::fromUtf8(" ") + text +
    QString::asprintf("\nh=%.2f", h));
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

void PlannerWindow::on_zone_toggle_clicked()
{
  zone_blue_ = !zone_blue_;
  reload_zone_blocks();
}

void PlannerWindow::reload_zone_blocks()
{
  // 红/蓝区方块表（坐标关于 y=0 镜像；序号、高度两区一致）。
  std::string share;
  try {
    share = ament_index_cpp::get_package_share_directory("r2_meilin_planner");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(node_->get_logger(), "ament_index 查找失败：%s", ex.what());
    return;
  }
  const std::string path =
    share + (zone_blue_ ? "/config/block_blue.yaml" : "/config/block_red.yaml");

  r2_planner::BlockTable next;
  std::string err;
  if (!next.loadFromYaml(path, &err)) {
    RCLCPP_ERROR(node_->get_logger(), "加载 %s 失败：%s（保持原方块表）", path.c_str(), err.c_str());
    return;
  }
  blocks_ = std::move(next);
  RCLCPP_INFO(node_->get_logger(), "切到%s区，方块表 %s", zone_blue_ ? "蓝" : "红", path.c_str());

  // 序号镜像 + 坐标变了：刷新按钮标签与示意图（cell_phase_ 按物理按钮位置保持不变）。
  zone_btn_->setText(QString::fromUtf8(zone_blue_ ? "当前：蓝区（点击切到红区）" : "当前：红区（点击切到蓝区）"));
  for (int i = 0; i < 12; ++i) {
    apply_phase_to_button(i);
  }
  redraw_scene();
}

void PlannerWindow::build_config_from_ui(r2_planner::ForestConfig & config) const
{
  r2_planner::fill_default_forest_topology(config);

  // 忽略高度：升/降代价清零、强制可上 400（与 kfs_subscriber_node 的 ignore_height 一致）。
  if (ignore_height_chk_ && ignore_height_chk_->isChecked()) {
    config.cost.climb_200_cost   = 0.0;
    config.cost.climb_400_cost   = 0.0;
    config.cost.descend_200_cost = 0.0;
    config.cost.descend_400_cost = 0.0;
    config.cost.can_climb_400    = true;
  } else if (can_climb_400_chk_) {
    // 未忽略高度时，由开关决定 400 档能否上/下；取消勾选则 400 视为不可通行。
    config.cost.can_climb_400 = can_climb_400_chk_->isChecked();
  }

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
    oss << "# R1 赛前：未指定（R1 留下的未取块按障碍绕开）\n";
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

QString format_step(
  const robot_interfaces::msg::PlanStep & s,
  const std::unordered_map<int, double> & heights)
{
  using PS = robot_interfaces::msg::PlanStep;
  auto name_of = [](int nid) -> QString {
    if (nid == 0) return QStringLiteral("入口");
    if (nid == 13) return QStringLiteral("出口");
    return QString::number(nid);
  };
  auto h_of = [&](int nid) -> double {
    auto it = heights.find(nid);
    return it != heights.end() ? it->second : 0.0;
  };
  auto turn_txt = [](double deg) -> QString {
    if (deg > 1.0) return QString::asprintf("左转%.0f°", deg);
    if (deg < -1.0) return QString::asprintf("右转%.0f°", -deg);
    return QStringLiteral("直行");
  };
  switch (s.type) {
    case PS::TYPE_MOVE: {
      const char * dir = "持平";
      if (s.stair_dir == PS::STAIR_UP) {
        dir = "爬升";
      } else if (s.stair_dir == PS::STAIR_DOWN) {
        dir = "下降";
      }
      return QString::asprintf(
        "走 %s -> %s  %s  %s %.2fm",
        name_of(s.from_id).toUtf8().constData(),
        name_of(s.target_id).toUtf8().constData(),
        turn_txt(s.turn_deg).toUtf8().constData(),
        dir, s.abs_dh);
    }
    case PS::TYPE_PICK: {
      const double dh = s.block_height - h_of(s.from_id);  // 脚下 → 目标块高度差
      return QString::asprintf(
        "抓 %s  (人在 %s，高度差 %+.2fm)",
        name_of(s.target_id).toUtf8().constData(),
        name_of(s.from_id).toUtf8().constData(), dh);
    }
    case PS::TYPE_PUSH: {
      const double dh = s.block_height - h_of(s.from_id);
      return QString::asprintf(
        "推 %s  (人在 %s，高度差 %+.2fm)",
        name_of(s.target_id).toUtf8().constData(),
        name_of(s.from_id).toUtf8().constData(), dh);
    }
    default:
      return QString::asprintf("未知步骤 type=%d target=%d", s.type, s.target_id);
  }
}

// 由步骤序列重建“经过的节点序列”，输出总览：起止序号 + 路线 + 沿途高度。
// node id 即梅林方块序号（入口 0，出口 13）；高度取自与算法同源的 config.node_heights。
std::string format_summary(
  const std::vector<robot_interfaces::msg::PlanStep> & steps,
  const std::unordered_map<int, double> & heights)
{
  using PS = robot_interfaces::msg::PlanStep;
  auto name_of = [](int nid) -> std::string {
    if (nid == 0) return std::string("入口");
    if (nid == 13) return std::string("出口");
    return std::to_string(nid);
  };
  auto h_of = [&](int nid) -> double {
    auto it = heights.find(nid);
    return it != heights.end() ? it->second : 0.0;
  };

  std::vector<int> route;
  if (!steps.empty()) {
    route.push_back(steps.front().from_id);  // 起点 = 第一步出发点
  }
  for (const auto & s : steps) {
    if (s.type == PS::TYPE_MOVE) {
      route.push_back(s.target_id);
    }
  }
  if (route.empty()) {
    return std::string();
  }

  std::ostringstream so;
  so << std::fixed << std::setprecision(2);
  so << "===== 规划总览 =====\n";
  so << "起点 " << name_of(route.front()) << " -> 终点 " << name_of(route.back())
     << "  (共 " << steps.size() << " 步)\n";
  so << "路线: ";
  for (size_t i = 0; i < route.size(); ++i) {
    so << name_of(route[i]) << "(h=" << h_of(route[i]) << "m)";
    if (i + 1 < route.size()) {
      so << " -> ";
    }
  }
  so << "\n";
  return so.str();
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

  // 同步生成人类可读字符串（GUI + 日志 + plan.notes）：先总览，再逐步明细
  oss << format_summary(steps, config.node_heights);
  oss << "----- 每步明细 -----\n";
  for (size_t i = 0; i < steps.size(); ++i) {
    oss << "[" << (i + 1) << "] " << format_step(steps[i], config.node_heights).toStdString() << '\n';
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
  // 终端同样先总览后明细
  {
    std::istringstream ss(format_summary(steps, config.node_heights));
    std::string sline;
    while (std::getline(ss, sline)) {
      if (!sline.empty()) {
        RCLCPP_INFO(node_->get_logger(), "%s", sline.c_str());
      }
    }
  }
  for (size_t i = 0; i < steps.size(); ++i) {
    RCLCPP_INFO(
      node_->get_logger(), "[%zu] %s", i + 1, format_step(steps[i], config.node_heights).toStdString().c_str());
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
