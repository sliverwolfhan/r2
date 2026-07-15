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
#include <QDoubleSpinBox>
#include <QVBoxLayout>
#include <QWidget>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include <QLineF>

PlannerWindow::PlannerWindow(
  rclcpp::Node::SharedPtr node,
  r2_planner::BlockTable blocks,
  rclcpp::Publisher<robot_interfaces::msg::Plan>::SharedPtr plan_pub,
  PlannerParams params,
  bool initial_zone_blue,
  QWidget * parent)
: QMainWindow(parent),
  node_(std::move(node)),
  blocks_(std::move(blocks)),
  plan_pub_(std::move(plan_pub)),
  params_base_(std::move(params)),
  zone_blue_(initial_zone_blue),   // 必须先于建 UI 设定：zone_btn_ 文案、序号镜像、方块着色都读它。
                                   // 传入的 blocks 已由 main() 按同一 zone 加载（红/蓝坐标一致）。
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

  // R1 定时消失：勾选后「R1待」格(码1)按"会随时间让开的硬障碍"处理，可原地 WAIT 等它让开。
  r1_timed_removal_chk_ = new QCheckBox(QString::fromUtf8("R1 定时消失（R1待块随步数让开）"), left);
  r1_timed_removal_chk_->setToolTip(
    QString::fromUtf8("勾选后：橙色「R1待」格建模为定时消失障碍，未消失前不可踩、可 WAIT 等待；"
                      "取消勾选则「R1待」当空地（等价 r1_timed_removal_enable:=false）"));
  left_lay->addWidget(r1_timed_removal_chk_);
  {
    auto * r1t_row = new QHBoxLayout();
    r1t_row->addWidget(new QLabel(QString::fromUtf8("每走几步消失一个:"), left));
    r1_removal_steps_spin_ = new QSpinBox(left);
    r1_removal_steps_spin_->setRange(1, 99);
    r1_removal_steps_spin_->setValue(3);
    r1t_row->addWidget(r1_removal_steps_spin_);
    r1t_row->addWidget(new QLabel(QString::fromUtf8("等待代价:"), left));
    wait_cost_spin_ = new QDoubleSpinBox(left);
    wait_cost_spin_->setRange(0.0, 1000.0);
    wait_cost_spin_->setSingleStep(0.5);
    wait_cost_spin_->setValue(1.0);
    r1t_row->addWidget(wait_cost_spin_);
    r1t_row->addStretch(1);
    left_lay->addLayout(r1t_row);
  }

  // 监视模式：勾选后订阅真车话题，收到布局/路径就画出来（不本地重规划）。
  monitor_chk_ = new QCheckBox(QString::fromUtf8("监视话题（自动显示 /AT_R2/kfs_positions + /r2_planner/plan）"), left);
  monitor_chk_->setChecked(true);
  monitor_chk_->setToolTip(
    QString::fromUtf8("勾选后：格子按收到的 kfs 布局着色，路径按真车发布的 plan 用真实 map 坐标画出；"
                      "取消勾选可手动摆场离线规划"));
  left_lay->addWidget(monitor_chk_);

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

  // 监视订阅：kfs 布局用普通可靠 QoS；plan 用 latched(transient_local) 以便窗口后开也能拿到最近一条。
  kfs_sub_ = node_->create_subscription<std_msgs::msg::Int32MultiArray>(
    "/AT_R2/kfs_positions", 10,
    std::bind(&PlannerWindow::on_kfs_msg, this, std::placeholders::_1));
  plan_monitor_sub_ = node_->create_subscription<robot_interfaces::msg::Plan>(
    "/r2_planner/plan", rclcpp::QoS(1).transient_local().reliable(),
    std::bind(&PlannerWindow::on_plan_msg, this, std::placeholders::_1));

  // 界面初值与 yaml 基线同步：这些控件会覆盖 yaml 对应项，故初始状态须反映 yaml，
  // 否则默认勾选/默认数值会在首次规划时默默覆盖 planner_params.yaml。
  if (ignore_height_chk_) {
    ignore_height_chk_->setChecked(params_base_.ignore_height);
  }
  if (can_climb_400_chk_) {
    can_climb_400_chk_->setChecked(params_base_.cost.can_climb_400);
  }
  if (r1_timed_removal_chk_) {
    r1_timed_removal_chk_->setChecked(params_base_.r1_timed_removal_enable);
  }
  if (r1_removal_steps_spin_) {
    r1_removal_steps_spin_->setValue(params_base_.r1_removal_steps);
  }
  if (wait_cost_spin_) {
    wait_cost_spin_->setValue(params_base_.wait_cost);
  }

  redraw_scene();
}

r2_planner::BlockState PlannerWindow::block_from_phase(int phase)
{
  switch (phase % 5) {
    case 0:
      return r2_planner::BlockState::EMPTY;
    case 1:
      return r2_planner::BlockState::R1_KFS;
    case 2:
      return r2_planner::BlockState::R2_KFS;
    case 3:
      return r2_planner::BlockState::FAKE_KFS;
    default:
      return r2_planner::BlockState::R1_PENDING;
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
    case r2_planner::BlockState::R1_PENDING:
      return 4;
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
  const int ph = cell_phase_[static_cast<size_t>(index)] % 5;
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
    case 3:
      text = QString::fromUtf8("假");
      style = "background:#87ceeb;color:#012;";
      break;
    default:
      text = QString::fromUtf8("R1待");
      style = "background:#f0b060;color:#210;";
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
  cell_phase_[static_cast<size_t>(idx)] = (cell_phase_[static_cast<size_t>(idx)] + 1) % 5;
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

  // yaml 打底：套用 planner_params.yaml 读来的代价/偏移/等待/R1 消失基线，
  // 覆盖 fill_default_forest_topology 的写死默认（拓扑与高度仍来自 topology）。
  // 之后的界面控件在此基线之上覆盖对应项（yaml 打底 + UI 覆盖）。
  config.cost = params_base_.cost;
  config.move_prep_offset       = params_base_.move_prep_offset;
  config.grasp_prep_offset      = params_base_.grasp_prep_offset;
  config.block_height_offset    = params_base_.block_height_offset;
  config.grasp_prep_theta_offset = params_base_.grasp_prep_theta_offset;
  config.move_prep_theta_offset  = params_base_.move_prep_theta_offset;
  config.wait_cost              = params_base_.wait_cost;
  config.r1_timed_removal_enable = params_base_.r1_timed_removal_enable;
  config.r1_removal_steps        = params_base_.r1_removal_steps;
  // yaml 里的 ignore_height 先在基线上生效（界面「忽略台阶高度」勾选后会再次清零，等价）。
  if (params_base_.ignore_height) {
    config.cost.climb_200_cost   = 0.0;
    config.cost.climb_400_cost   = 0.0;
    config.cost.descend_200_cost = 0.0;
    config.cost.descend_400_cost = 0.0;
    config.cost.can_climb_400    = true;
  }

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

  // 蓝区场地关于 x 轴镜像，节点编号→物理左右相反，moveHeading 需据此翻转左右编码。
  config.zone_blue = zone_blue_;
  // 抓取偏好：机器人物理左手列（+y）。红区 {3,6,9,12}，蓝区 {1,4,7,10}。
  config.preferred_pick_nodes = zone_blue_
    ? std::unordered_set<int>{1, 4, 7, 10}
    : std::unordered_set<int>{3, 6, 9, 12};

  // R1 定时消失：把「R1待」格建模为随步数让开的硬障碍（与 kfs_subscriber_node 一致）。
  if (r1_timed_removal_chk_) {
    config.r1_timed_removal_enable = r1_timed_removal_chk_->isChecked();
  }
  if (r1_removal_steps_spin_) {
    config.r1_removal_steps = r1_removal_steps_spin_->value();
  }
  if (wait_cost_spin_) {
    config.wait_cost = wait_cost_spin_->value();
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

void PlannerWindow::redraw_scene()
{
  scene_->clear();

  // 收集所有已知节点(0..13)的真实 map 坐标，算边界用于缩放。
  double minx = 1e9, maxx = -1e9, miny = 1e9, maxy = -1e9;
  bool any = false;
  for (int id = 0; id <= 13; ++id) {
    if (!blocks_.has(id)) continue;
    const auto & b = blocks_.at(id);
    minx = std::min(minx, b.x); maxx = std::max(maxx, b.x);
    miny = std::min(miny, b.y); maxy = std::max(maxy, b.y);
    any = true;
  }
  if (!any) { return; }
  const double pad = 0.8;                 // m，四周留白
  minx -= pad; maxx += pad; miny -= pad; maxy += pad;

  const double scale = 90.0;              // px/m（fitInView 再自适应，仅决定相对字号）
  // map(x,y) -> scene（x、y 方向对调）：
  //   屏幕横轴 = map y（+y 机器人左手 → 屏幕左）；屏幕纵轴 = map x（+x 前进 → 屏幕上，入口在下出口在上）。
  auto to_scene = [&](double x, double y) {
    return QPointF((maxy - y) * scale, (maxx - x) * scale);
  };
  auto center = [&](int nid) { const auto & b = blocks_.at(nid); return to_scene(b.x, b.y); };

  // ---- 网格 + 米制坐标轴刻度（浅灰虚线，每 1m 一格）----
  QPen grid_pen(QColor(210, 210, 210), 1, Qt::DashLine);
  for (int xm = static_cast<int>(std::ceil(minx)); xm <= static_cast<int>(std::floor(maxx)); ++xm) {
    auto * l = scene_->addLine(QLineF(to_scene(xm, maxy), to_scene(xm, miny)), grid_pen);
    l->setZValue(-1);
    auto * t = scene_->addSimpleText(QString::asprintf("x=%d", xm));
    t->setBrush(QColor(150, 150, 150));
    t->setPos(to_scene(xm, maxy).x() + 1, to_scene(xm, maxy).y() - 14);
    t->setZValue(-1);
  }
  for (int ym = static_cast<int>(std::ceil(miny)); ym <= static_cast<int>(std::floor(maxy)); ++ym) {
    auto * l = scene_->addLine(QLineF(to_scene(minx, ym), to_scene(maxx, ym)), grid_pen);
    l->setZValue(-1);
    auto * t = scene_->addSimpleText(QString::asprintf("y=%d", ym));
    t->setBrush(QColor(150, 150, 150));
    t->setPos(to_scene(minx, ym).x() + 1, to_scene(minx, ym).y() + 1);
    t->setZValue(-1);
  }

  // ---- 12 个方块：按真实坐标居中，1.0m 见方，按状态着色，标注编号+真实(x,y)----
  const double half = 0.5 * scale;        // 1.0m 见方的半边（像素）
  for (int id = 1; id <= 12; ++id) {
    if (!blocks_.has(id)) continue;
    const auto & b = blocks_.at(id);
    const QPointF c = to_scene(b.x, b.y);
    const QRectF rect(c.x() - half, c.y() - half, 2 * half, 2 * half);
    const int idx = cell_index_from_display_number(id);
    const int ph = (idx >= 0) ? (cell_phase_[static_cast<size_t>(idx)] % 5) : 0;
    QColor fill(230, 230, 230);
    if (ph == 1) fill = QColor(240, 128, 128);
    else if (ph == 2) fill = QColor(144, 238, 144);
    else if (ph == 3) fill = QColor(135, 206, 235);
    else if (ph == 4) fill = QColor(240, 176, 96);
    auto * cell = scene_->addRect(rect, QPen(Qt::darkGray, 1), QBrush(fill));
    cell->setZValue(0);
    auto * t = scene_->addSimpleText(QString::asprintf("%d\n(%.2f,%.2f)", id, b.x, b.y));
    t->setBrush(Qt::black);
    t->setPos(rect.left() + 3, rect.top() + 2);
    t->setZValue(1);
  }

  // ---- 入口/出口标记 ----
  auto draw_marker = [&](int nid, QColor col, const QString & label) {
    if (!blocks_.has(nid)) return;
    const QPointF c = center(nid);
    auto * e = scene_->addEllipse(QRectF(c.x() - 8, c.y() - 8, 16, 16), QPen(Qt::black), QBrush(col));
    e->setZValue(2);
    auto * t = scene_->addSimpleText(label);
    t->setBrush(Qt::black); t->setPos(c.x() + 8, c.y() - 8); t->setZValue(2);
  };
  draw_marker(0, QColor(255, 220, 0), QString::fromUtf8("入口0"));
  draw_marker(13, QColor(200, 200, 255), QString::fromUtf8("出口13"));

  // ---- 路径：沿机器人每步真实站位（prep_pose 的 map 坐标）连线；抓/推/等就地标注 ----
  // 关键：抓取时机器人不是"走到目标块上"，而是停在相邻块用 prep_pose 站位伸臂取块。
  // 若只连 MOVE 目标中心，会漏掉这些取块站位（例如先取 3 号再上 1 号，连线会像是
  // "直接去了 1 号"）。因此路线改走每步的 prep_pose 真实坐标，抓/推标在目标块上，
  // 并从站位画一小段虚线连到目标块，直观呈现"停在旁边取块"。
  if (last_steps_.empty() || !blocks_.has(0)) {
    view_->fitInView(scene_->itemsBoundingRect().adjusted(-10, -10, 10, 10), Qt::KeepAspectRatio);
    return;
  }
  using PS = robot_interfaces::msg::PlanStep;
  QPainterPath road;
  QPointF robot_pt = center(0);
  road.moveTo(robot_pt);
  QPen reach_pen(QColor(120, 120, 120), 1, Qt::DashLine);
  for (const auto & s : last_steps_) {
    if (s.type == PS::TYPE_MOVE) {
      // 走到本步的 prep_pose（在 from 块上对齐 target 的真实站位），而非目标块中心。
      robot_pt = to_scene(s.prep_pose.x, s.prep_pose.y);
      road.lineTo(robot_pt);
    } else if (s.type == PS::TYPE_PICK || s.type == PS::TYPE_PUSH) {
      // 机器人先走到取块站位（prep_pose），再伸臂到目标块 (cube_x, cube_y)。
      robot_pt = to_scene(s.prep_pose.x, s.prep_pose.y);
      road.lineTo(robot_pt);
      const QColor dot = (s.type == PS::TYPE_PICK) ? QColor(0, 160, 0) : QColor(200, 80, 0);
      // 目标块中心：优先用 cube_x/cube_y（真实物块坐标），否则回退到块表中心。
      QPointF blk = (s.cube_x != 0.0 || s.cube_y != 0.0)
        ? to_scene(s.cube_x, s.cube_y)
        : (blocks_.has(s.target_id) ? center(s.target_id) : robot_pt);
      // 站位 -> 目标块的伸臂虚线，直观表示"停在旁边取块"。
      auto * reach = scene_->addLine(QLineF(robot_pt, blk), reach_pen);
      reach->setZValue(1.4);
      auto * d = scene_->addEllipse(QRectF(blk.x() - 5, blk.y() - 5, 10, 10), QPen(Qt::black, 1), QBrush(dot));
      d->setZValue(2.5);
      auto * t = scene_->addSimpleText(
        QString::fromUtf8(s.type == PS::TYPE_PICK ? "抓" : "推") + QString::number(s.target_id));
      t->setBrush(dot); t->setPos(blk.x() + 5, blk.y() - 16); t->setZValue(2.5);
    } else if (s.type == PS::TYPE_WAIT) {
      auto * t = scene_->addSimpleText(QString::fromUtf8("等"));
      t->setBrush(QColor(200, 120, 0)); t->setPos(robot_pt.x() + 5, robot_pt.y() + 2); t->setZValue(2.5);
    }
  }
  auto * path_item = scene_->addPath(road, QPen(QColor(255, 170, 0), 3));
  path_item->setZValue(1.5);
  auto * bot = scene_->addEllipse(QRectF(robot_pt.x() - 9, robot_pt.y() - 9, 18, 18),
    QPen(Qt::black, 2), QBrush(QColor(255, 255, 0)));
  bot->setZValue(3);

  view_->fitInView(scene_->itemsBoundingRect().adjusted(-10, -10, 10, 10), Qt::KeepAspectRatio);
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
    case PS::TYPE_WAIT: {
      return QString::asprintf(
        "等待一拍  (在 %s，等 R1 让开)", name_of(s.from_id).toUtf8().constData());
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
    last_steps_.clear();
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

  // 存完整步骤供真实坐标绘图（含每步 prep_pose 的 map 坐标）。
  last_steps_ = steps;

  r1_preclear_display_a_->setValue(0);
  r1_preclear_display_b_->setValue(0);

  redraw_scene();
}

int PlannerWindow::phase_from_code(int code)
{
  // 下位机码 -> 界面相位（与 kfs_subscriber_node::state_from_code 语义一致）。
  //   0 空 / 1 R1正在收取(R1待) / 2 R2 / 3 假 / 4 R1未取(永久障碍)
  switch (code) {
    case 1: return 4;   // R1待（橙）
    case 2: return 2;   // R2
    case 3: return 3;   // 假
    case 4: return 1;   // R1 永久障碍
    default: return 0;  // 0 及未知 → 空
  }
}

void PlannerWindow::on_kfs_msg(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
  if (monitor_chk_ && !monitor_chk_->isChecked()) {
    return;  // 未开监视：不覆盖手动摆场
  }
  if (msg->data.size() != 12) {
    RCLCPP_WARN(node_->get_logger(), "kfs_positions 长度异常: 期望 12, 收到 %zu", msg->data.size());
    return;
  }
  // kfs 话题会周期/latched 反复发同一布局；若每条都重画，scene_->clear()+全量重建
  // 会让梅林格子一闪一闪。仅在布局相对上一次真正变化时才 apply+redraw。
  const std::vector<int> codes(msg->data.begin(), msg->data.end());
  if (codes == last_kfs_codes_) {
    return;
  }
  last_kfs_codes_ = codes;
  // data[i] 对应节点 id = i+1；按当前红/蓝区映射回按钮格子。
  for (int i = 0; i < 12; ++i) {
    const int idx = cell_index_from_display_number(i + 1);
    if (idx >= 0) {
      cell_phase_[static_cast<size_t>(idx)] = phase_from_code(msg->data[static_cast<size_t>(i)]);
    }
  }
  for (int i = 0; i < 12; ++i) {
    apply_phase_to_button(i);
  }
  redraw_scene();
}

void PlannerWindow::on_plan_msg(const robot_interfaces::msg::Plan::SharedPtr msg)
{
  if (monitor_chk_ && !monitor_chk_->isChecked()) {
    return;
  }
  // /r2_planner/plan 是 latched + 会周期重发同一条计划；每条都 redraw_scene()（内含
  // scene_->clear() 全量重建）会让示意图一闪一闪。计划内容相对上次没变就跳过重画/重设文本。
  if (msg->steps == last_steps_) {
    return;
  }
  last_steps_ = msg->steps;

  // 文字明细：优先展示真车发来的 notes；再附上本地按同源高度重算的总览+每步。
  std::ostringstream oss;
  if (!msg->notes.empty()) {
    oss << msg->notes;
    if (msg->notes.back() != '\n') oss << '\n';
  }
  oss << format_summary(msg->steps, node_heights_);
  oss << "----- 每步明细 -----\n";
  for (size_t i = 0; i < msg->steps.size(); ++i) {
    oss << "[" << (i + 1) << "] " << format_step(msg->steps[i], node_heights_).toStdString() << '\n';
  }
  log_->setPlainText(QString::fromStdString(oss.str()));
  redraw_scene();
}
