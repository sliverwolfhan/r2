#include "r2_meilin_planner/planner.hpp"
#include <iostream>
#include <unordered_set>
#include <algorithm>
#include <stdexcept>
#include <cmath>

namespace r2_planner {

R2MeilinPlanner::R2MeilinPlanner(const ForestConfig& config) : config_(config) {}

R2MeilinPlanner::R2MeilinPlanner(const ForestConfig& config, const BlockTable & blocks)
: config_(config), blocks_(blocks), has_blocks_(true) {}

uint16_t R2MeilinPlanner::generateInitialMask() const {
    uint16_t mask = 0;
    for (int i = 1; i <= NUM_FOREST_BLOCKS; ++i) {
        if (config_.initial_items.count(i) && config_.initial_items.at(i) != BlockState::EMPTY) {
            mask |= (1 << (i - 1));
        }
    }
    return mask;
}

bool R2MeilinPlanner::isNodeOccupied(uint16_t mask, int node_id) const {
    if (node_id < 1 || node_id > NUM_FOREST_BLOCKS) return false;
    return (mask & (1 << (node_id - 1))) != 0;
}

uint16_t R2MeilinPlanner::clearNodeOccupied(uint16_t mask, int node_id) const {
    if (node_id < 1 || node_id > NUM_FOREST_BLOCKS) return mask;
    return mask & ~(1 << (node_id - 1));
}

double R2MeilinPlanner::calculateMoveCost(int from_node, int to_node) const {
    const CostConfig & c = config_.cost;
    double translation_cost = c.move_cost;
    double height_diff = 0.0;
    if (config_.node_heights.count(to_node) && config_.node_heights.count(from_node)) {
        height_diff = config_.node_heights.at(to_node) - config_.node_heights.at(from_node);
    }
    double climb_cost = 0.0;
    if (height_diff > 1e-3) {                         // 米制：高度差分两档
        if (height_diff <= 0.2 + 1e-3) {
            climb_cost = c.climb_200_cost;
        } else if (height_diff <= 0.4 + 1e-3) {
            climb_cost = c.can_climb_400 ? c.climb_400_cost : c.prohibitive_cost;
        } else {
            climb_cost = c.prohibitive_cost;          // >0.4（含 600）不可上
        }
    } else if (height_diff < -1e-3) {
        const double drop = -height_diff;            // 下台阶也分两档
        if (drop <= 0.2 + 1e-3) {
            climb_cost = c.descend_200_cost;
        } else {
            // 下 400：与上 400 用同一开关，can_climb_400=false 时下行也禁止（绕开）。
            climb_cost = c.can_climb_400 ? c.descend_400_cost : c.prohibitive_cost;
        }
    }
    return translation_cost + climb_cost;
}

double R2MeilinPlanner::calculatePickCost() const { return config_.cost.pick_cost; }
double R2MeilinPlanner::calculatePushCost() const { return config_.cost.push_cost; }

int R2MeilinPlanner::moveHeading(int from_node, int to_node) const {
    // 进出场（涉及入口 0 / 出口 13）一律视为 +x 前进，与 prep_pose 规则一致。
    if (from_node == ENTRY_NODE_ID || to_node == EXIT_NODE_ID) return 0;   // +x
    if (to_node == ENTRY_NODE_ID || from_node == EXIT_NODE_ID) return 2;   // -x（防御）
    const int diff = to_node - from_node;
    if (diff == 3)  return 0;   // 同列上一行 → +x 前进
    if (diff == -3) return 2;   // -x 后退
    if (diff == 1)  return 1;   // 同行下一列 → +y 左
    if (diff == -1) return 3;   // -y 右
    return 0;                   // 理论不会到这（相邻必差 ±1/±3）
}

int R2MeilinPlanner::turnQuarters(int from_heading, int to_heading) const {
    int d = std::abs(from_heading - to_heading) % 4;   // 0/1/2/3
    if (d == 3) d = 1;                                 // 270° 等价 90°
    return d;                                          // 0 直行 / 1 转 90° / 2 掉头
}

int R2MeilinPlanner::pickTurnQuarters(int from_heading, int block_heading) const {
    // 块相对车头方位 r：0=正前 / 1=左 / 2=正后 / 3=右（heading 编码 +x/+y/-x/-y）。
    const int r = ((block_heading - from_heading) % 4 + 4) % 4;
    if (r == 0 || r == 1) return 0;                    // 前 / 左：夹爪在左手，免转向
    if (r == 3) return 1;                              // 右：转 90°
    return 2;                                          // 后：掉头 180°
}

double R2MeilinPlanner::calculateHeuristic(const SearchState& state) const {
    int remaining = KFS_TARGET_COUNT - state.kfs_held_count;
    double h_exit = 0.0;
    if (state.current_node_id != EXIT_NODE_ID) {
        int row = 0;
        if (state.current_node_id >= 1 && state.current_node_id <= 12) {
            row = (state.current_node_id - 1) / 3 + 1; // Row 1-4, from 1/2/3 side to 10/11/12 side
        } else if (state.current_node_id == ENTRY_NODE_ID) {
            row = 0; // 起点在 1/2/3 侧外
        }
        // 目标是对侧的 Row 5 (Node 13)
        // 距离 = 5 - 当前行
        h_exit = (5.0 - row) * 1.0;
    }
    // 剩余 pick 的下界：可能抓到偏好列享受 bonus 减免，必须按减免后的下界估计
    // 才能保持 admissible（否则 A* 可能错过偏好列里更便宜的路径）。
    const double pick_lb = std::max(0.0, config_.cost.pick_cost - config_.cost.preferred_column_pick_bonus);
    return remaining * pick_lb + h_exit;
}

std::vector<std::string> R2MeilinPlanner::planPath() {
    // 进场侧是 1, 2, 3
    std::vector<int> front_row = {1, 2, 3};
    bool front_row_has_target = false;
    for(int i : front_row) {
        if(config_.initial_items.count(i) && config_.initial_items.at(i) == BlockState::R2_KFS) {
            front_row_has_target = true;
            break;
        }
    }

    std::priority_queue<std::shared_ptr<SearchState>, std::vector<std::shared_ptr<SearchState>>, StateComparator> open_list;
    std::unordered_set<SearchState, StateHasher> closed_set;

    auto start = std::make_shared<SearchState>();
    start->current_node_id = ENTRY_NODE_ID; start->kfs_held_count = 0;
    start->env_mask = generateInitialMask(); start->g_cost = 0.0;
    start->heading = 0;  // 车头朝 +x，对着 1/2/3
    start->f_cost = calculateHeuristic(*start); start->parent = nullptr; start->action_taken = "START";
    open_list.push(start);

    while (!open_list.empty()) {
        auto current = open_list.top(); open_list.pop();
        if (current->kfs_held_count >= KFS_TARGET_COUNT && current->current_node_id == EXIT_NODE_ID) {
            std::vector<std::string> path; auto temp = current;
            while (temp && temp->action_taken != "START") { path.push_back(temp->action_taken); temp = temp->parent; }
            std::reverse(path.begin(), path.end()); return path;
        }
        if (closed_set.count(*current)) continue;
        closed_set.insert(*current);

        if (config_.adjacency_list.count(current->current_node_id) == 0) continue;
        const auto& neighbors = config_.adjacency_list.at(current->current_node_id);

        // 强制态：前排(1/2/3)有 R2 目标且一个都还没抓时，必须先抓一个前排 R2 才能上台阶。
        // 此态下禁止移动(上台阶)与推走，唯一允许的动作是抓前排 R2。
        const bool is_forced_pick = (front_row_has_target && current->kfs_held_count == 0);

        // 1. MOVE 动作（强制态下禁止移动，必须先抓前排）
        for (int next : neighbors) {
            if (is_forced_pick) break;
            if (!isNodeOccupied(current->env_mask, next)) {
                auto ns = std::make_shared<SearchState>(*current);
                const int nh = moveHeading(current->current_node_id, next);
                const int q = turnQuarters(current->heading, nh);
                ns->current_node_id = next;
                ns->heading = nh;
                ns->g_cost += calculateMoveCost(current->current_node_id, next)
                              + q * config_.cost.turn_cost;
                ns->f_cost = ns->g_cost + calculateHeuristic(*ns); ns->parent = current;
                ns->action_taken = "MOVE to " + std::to_string(next);
                if (!closed_set.count(*ns)) open_list.push(ns);
            }
        }

        // 2. 对相邻的绿色秘籍 (R2_KFS) 的操作
        for (int adj : neighbors) {
            if (isNodeOccupied(current->env_mask, adj) && config_.initial_items.at(adj) == BlockState::R2_KFS) {

                // 规则 4.4.15：强制态下只能抓前排 R2（既不能上台阶也不能推）
                bool is_adj_in_front = std::find(front_row.begin(), front_row.end(), adj) != front_row.end();

                // 抓/推转向代价：块在前/左免转向，右转 90°、后掉头。免转向时朝向不变，
                // 否则转到正对块——之后的朝向再交给后续 MOVE 计算「操作完→上下台阶」的转角。
                // PICK 与 PUSH 同样收费（都要先正对块）。
                const int bh = moveHeading(current->current_node_id, adj);
                const int pq = pickTurnQuarters(current->heading, bh);
                const double turn_extra = pq * config_.cost.turn_cost;
                const int turned_heading = (pq == 0) ? current->heading : bh;

                // PICK 准入：
                //   held==0：强制态只能抓前排 R2；非强制态可自由抓。
                //   held==1：挡道的相邻 R2 直接抓——抓走它既凑满 2 个、又顺手让开了路，
                //            优于推（pick 比 push 便宜，A* 会自动取代推）；
                //            万一抓满后反被堵死到不了出口，A* 会回退到"先推、后面再抓"。
                bool pick_allowed = false;
                if (current->kfs_held_count == 0) {
                    pick_allowed = (!is_forced_pick || is_adj_in_front);
                } else if (current->kfs_held_count == 1) {
                    pick_allowed = true;
                }
                if (pick_allowed && current->kfs_held_count < KFS_TARGET_COUNT) {
                    auto ns = std::make_shared<SearchState>(*current);
                    ns->kfs_held_count += 1; ns->env_mask = clearNodeOccupied(ns->env_mask, adj);
                    double pick_cost = calculatePickCost();
                    // 偏好列 PICK 减免：鼓励抓机器人物理左手列（红 {3,6,9,12} / 蓝 {1,4,7,10}）。
                    if (config_.preferred_pick_nodes.count(adj)) {
                        pick_cost -= config_.cost.preferred_column_pick_bonus;
                    }
                    pick_cost += turn_extra;
                    ns->heading = turned_heading;
                    ns->g_cost += pick_cost; ns->f_cost = ns->g_cost + calculateHeuristic(*ns);
                    ns->parent = current; ns->action_taken = "PICK at " + std::to_string(adj);
                    if (!closed_set.count(*ns)) open_list.push(ns);
                }

                // PUSH（强制态下禁止推走，必须先抓前排 R2；抓满 2 个后失去推走能力）
                if (!is_forced_pick && current->kfs_held_count < KFS_TARGET_COUNT) {
                    auto ns_push = std::make_shared<SearchState>(*current);
                    ns_push->env_mask = clearNodeOccupied(ns_push->env_mask, adj);
                    ns_push->heading = turned_heading;
                    ns_push->g_cost += calculatePushCost() + turn_extra;
                    ns_push->f_cost = ns_push->g_cost + calculateHeuristic(*ns_push);
                    ns_push->parent = current; ns_push->action_taken = "PUSH " + std::to_string(adj);
                    if (!closed_set.count(*ns_push)) open_list.push(ns_push);
                }
            }
        }
    }
    return {"FAIL: No path found."};
}

namespace {
double normalize_angle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// 字符串解析助手；返回 false 表示该步格式异常应跳过。
struct ParsedAction {
    enum Op { MOVE, PICK, PUSH, UNKNOWN } op = UNKNOWN;
    int target = 0;
};

ParsedAction parseAction(const std::string & s) {
    ParsedAction p;
    if (s.rfind("MOVE to ", 0) == 0) {
        p.op = ParsedAction::MOVE;
        try { p.target = std::stoi(s.substr(8)); } catch (...) { p.op = ParsedAction::UNKNOWN; }
    } else if (s.rfind("PICK at ", 0) == 0) {
        p.op = ParsedAction::PICK;
        try { p.target = std::stoi(s.substr(8)); } catch (...) { p.op = ParsedAction::UNKNOWN; }
    } else if (s.rfind("PUSH ", 0) == 0) {
        p.op = ParsedAction::PUSH;
        try { p.target = std::stoi(s.substr(5)); } catch (...) { p.op = ParsedAction::UNKNOWN; }
    }
    return p;
}
}  // namespace

std::vector<robot_interfaces::msg::PlanStep> R2MeilinPlanner::planPathStruct() {
    std::vector<robot_interfaces::msg::PlanStep> out;
    if (!has_blocks_) {
        throw std::runtime_error("R2MeilinPlanner::planPathStruct() called without a BlockTable");
    }

    const std::vector<std::string> raw = planPath();
    if (raw.empty() || raw[0].rfind("FAIL", 0) == 0) {
        return out;  // 上游可据此判断
    }

    int robot_node = ENTRY_NODE_ID;  // 机器人初始位置 = 入口
    double prev_heading = 0.0;       // 车头初始朝 +x（对着 1/2/3 侧），与 A* 起点一致
    bool has_prev_heading = true;
    int robot_heading = 0;           // 整数车头 0/1/2/3=+x/+y/-x/-y，与 A* 同步；
                                     // 供 PICK/PUSH 按块相对当前车头判方位（不是世界系）

    // prep = 机器人站立的 from 格中心，沿主方向(±x 或 ±y)朝目标轻推 offset 的位置。
    // 机器人在 from 格上对齐到这一点。MOVE/PICK/PUSH 共用，仅偏移量(offset)不同。
    //   - 进入/离开赛场（涉及 0 或 13 strip）强制 +x 轴；
    //   - 否则取 |dx|, |dy| 中较大者作主方向；浮点等量时退化为 +x 兜底。
    // theta 在外部按动作类型另行设置（MOVE 朝向目标；PICK/PUSH 用离散方位规则）。
    constexpr double AXIS_EPS = 1e-3;   // m，浮点比较容差
    auto fill_prep_pose = [&](int from_id, int target_id, double offset,
                              robot_interfaces::msg::PlanStep & step) {
        if (!blocks_.has(from_id) || !blocks_.has(target_id)) {
            throw std::runtime_error(
                "BlockTable missing id " +
                std::to_string(blocks_.has(from_id) ? target_id : from_id));
        }
        const auto & f = blocks_.at(from_id);
        const auto & t = blocks_.at(target_id);
        const double dx = t.x - f.x;
        const double dy = t.y - f.y;
        double ux = 0.0;
        double uy = 0.0;

        if (from_id == ENTRY_NODE_ID || target_id == EXIT_NODE_ID) {
            ux = 1.0;                       // 进场 / 出场都朝 +x 走
        } else if (target_id == ENTRY_NODE_ID || from_id == EXIT_NODE_ID) {
            ux = -1.0;                      // 极少见；防御性处理
        } else if (std::abs(dx) > std::abs(dy) + AXIS_EPS) {
            ux = (dx >= 0.0) ? 1.0 : -1.0;
        } else if (std::abs(dy) > std::abs(dx) + AXIS_EPS) {
            uy = (dy >= 0.0) ? 1.0 : -1.0;
        } else {
            // 浮点等量：1..12 内部理论上不会出现；此处仅做兜底
            ux = (dx >= 0.0) ? 1.0 : -1.0;
        }

        step.prep_pose.x = f.x + offset * ux;   // 锚在 from 格，朝目标推 offset
        step.prep_pose.y = f.y + offset * uy;
        // 入场（from=0）特例：x 仍以 0 的 x + offset 为锚，但 y 必须对齐到目标块，
        // 使 1/2/3 落在车的正前方。0 与 2 本身同 y，抓 2 时该改写无差别。
        if (from_id == ENTRY_NODE_ID) {
            step.prep_pose.y = t.y;
        }
        step.prep_pose.theta = std::atan2(uy, ux);  // 默认朝向目标；PICK/PUSH 在外部覆盖
    };

    for (const std::string & line : raw) {
        const ParsedAction a = parseAction(line);
        if (a.op == ParsedAction::UNKNOWN) {
            continue;
        }

        robot_interfaces::msg::PlanStep step;

        if (a.op == ParsedAction::MOVE) {
            step.type = robot_interfaces::msg::PlanStep::TYPE_MOVE;
            step.from_id = robot_node;
            step.target_id = a.target;

            fill_prep_pose(step.from_id, step.target_id, config_.move_prep_offset, step);

            // 转角 = 本段行进朝向 - 上一段行进朝向（仅 MOVE 之间累计；PICK/PUSH 不转底盘）。
            const double cur_heading = step.prep_pose.theta;
            step.turn_deg = has_prev_heading
                ? normalize_angle(cur_heading - prev_heading) * 180.0 / M_PI
                : 0.0;
            prev_heading = cur_heading;
            has_prev_heading = true;

            // 应用 prep_pose.theta 偏移（仅影响导航 goal 朝向；turn_deg 已用未偏移值算完）
            step.prep_pose.theta += config_.move_prep_theta_offset;

            const auto & f = blocks_.at(step.from_id);
            const auto & t = blocks_.at(step.target_id);
            const double dh = t.height - f.height;
            step.abs_dh = std::abs(dh);
            if (dh > 1e-3) {
                step.stair_dir = robot_interfaces::msg::PlanStep::STAIR_UP;
            } else if (dh < -1e-3) {
                step.stair_dir = robot_interfaces::msg::PlanStep::STAIR_DOWN;
            } else {
                step.stair_dir = robot_interfaces::msg::PlanStep::STAIR_NONE;
            }

            robot_node = step.target_id;  // 完成 MOVE 后机器人在新节点
            robot_heading = moveHeading(step.from_id, step.target_id);  // 同步整数车头
        } else if (a.op == ParsedAction::PICK || a.op == ParsedAction::PUSH) {
            step.type = (a.op == ParsedAction::PICK)
                ? robot_interfaces::msg::PlanStep::TYPE_PICK
                : robot_interfaces::msg::PlanStep::TYPE_PUSH;
            step.from_id = robot_node;     // 机器人原地不动
            step.target_id = a.target;

            // PICK/PUSH 复用 prep 公式：robot 在 from 块上对齐到 target 那一侧
            fill_prep_pose(step.from_id, step.target_id, config_.grasp_prep_offset, step);
            const auto & f = blocks_.at(step.from_id);
            const auto & t = blocks_.at(step.target_id);
            // 准备朝向按目标相对【机器人当前车头】的方位决定（与 A* pickTurnQuarters 一致，
            // 不再用世界系）：块在 前/左 → 车头角度不变；右 → 向右转 90°；后 → 掉头。
            const int bh = moveHeading(step.from_id, step.target_id);
            const int pq = pickTurnQuarters(robot_heading, bh);
            const int turned_heading = (pq == 0) ? robot_heading : bh;
            double prep_theta = normalize_angle(turned_heading * M_PI_2)
                                + config_.grasp_prep_theta_offset;
            step.prep_pose.theta = prep_theta;
            step.grasp_yaw = prep_theta;       // 已弃用，留同值兼容下游 BT
            // 旋转后更新车头，供后续 MOVE 计算「操作完→上下台阶」的转角（用未叠偏移值）。
            robot_heading = turned_heading;
            prev_heading = normalize_angle(turned_heading * M_PI_2);

            step.cube_x = t.cube_x;
            step.cube_y = t.cube_y;
            // 发布的是 target 相对 from 的高度差，再叠加可调偏移；与 A* 代价无关。
            step.block_height = (t.height - f.height) + config_.block_height_offset;
        } else {
            continue;
        }

        out.push_back(step);
    }

    return out;
}
} // namespace r2_planner
