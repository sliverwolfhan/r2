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
    double translation_cost = 1.0;
    double height_diff = 0.0;
    if (config_.node_heights.count(to_node) && config_.node_heights.count(from_node)) {
        height_diff = config_.node_heights.at(to_node) - config_.node_heights.at(from_node);
    }
    double climb_cost = 0.0;
    if (height_diff > 0) {
        if (height_diff <= 200.0) climb_cost = 1.5;
        else if (height_diff <= 400.0) climb_cost = 3.5;
        else climb_cost = 10000.0;
    } else if (height_diff < 0) climb_cost = 0.2;
    return translation_cost + climb_cost;
}

double R2MeilinPlanner::calculatePickCost() const { return 2.0; }
double R2MeilinPlanner::calculatePushCost() const { return 4.0; } // 推走 KFS 代价比拾取高

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
    return remaining * 2.0 + h_exit;
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
    start->current_node_id = ENTRY_NODE_ID; start->kfs_held_count = 0; start->r1_cleared_count = 0;
    start->env_mask = generateInitialMask(); start->g_cost = 0.0;
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

        // 1. MOVE 动作
        for (int next : neighbors) {
            if (!isNodeOccupied(current->env_mask, next)) {
                auto ns = std::make_shared<SearchState>(*current);
                ns->current_node_id = next; ns->g_cost += calculateMoveCost(current->current_node_id, next);
                ns->f_cost = ns->g_cost + calculateHeuristic(*ns); ns->parent = current;
                ns->action_taken = "MOVE to " + std::to_string(next);
                if (!closed_set.count(*ns)) open_list.push(ns);
            }
            else if (config_.initial_items.count(next) && config_.initial_items.at(next) == BlockState::R1_KFS 
                     && current->r1_cleared_count < 2) {
                auto ns = std::make_shared<SearchState>(*current);
                ns->current_node_id = next; ns->r1_cleared_count += 1;
                ns->env_mask = clearNodeOccupied(ns->env_mask, next);
                ns->g_cost += (calculateMoveCost(current->current_node_id, next) + R1_ASSIST_COST);
                ns->f_cost = ns->g_cost + calculateHeuristic(*ns); ns->parent = current;
                ns->action_taken = "R1_CLEAR " + std::to_string(next) + " then MOVE";
                if (!closed_set.count(*ns)) open_list.push(ns);
            }
        }

        // 2. 对相邻的绿色秘籍 (R2_KFS) 的操作
        for (int adj : neighbors) {
            if (isNodeOccupied(current->env_mask, adj) && config_.initial_items.at(adj) == BlockState::R2_KFS) {
                
                // 规则 4.4.15 修正：如果前排有目标，且还没抓过，则必须先抓前排的
                bool is_forced_pick = (front_row_has_target && current->kfs_held_count == 0);
                bool is_adj_in_front = std::find(front_row.begin(), front_row.end(), adj) != front_row.end();
                
                // PICK
                if (current->kfs_held_count < KFS_TARGET_COUNT) {
                    if (!is_forced_pick || is_adj_in_front) {
                        auto ns = std::make_shared<SearchState>(*current);
                        ns->kfs_held_count += 1; ns->env_mask = clearNodeOccupied(ns->env_mask, adj);
                        ns->g_cost += calculatePickCost(); ns->f_cost = ns->g_cost + calculateHeuristic(*ns);
                        ns->parent = current; ns->action_taken = "PICK at " + std::to_string(adj);
                        if (!closed_set.count(*ns)) open_list.push(ns);
                    }
                }

                // PUSH
                if (!is_forced_pick || is_adj_in_front) {
                    auto ns_push = std::make_shared<SearchState>(*current);
                    ns_push->env_mask = clearNodeOccupied(ns_push->env_mask, adj);
                    ns_push->g_cost += calculatePushCost(); ns_push->f_cost = ns_push->g_cost + calculateHeuristic(*ns_push);
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
    enum Op { MOVE, PICK, PUSH, R1_CLEAR_MOVE, UNKNOWN } op = UNKNOWN;
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
    } else if (s.rfind("R1_CLEAR ", 0) == 0) {
        p.op = ParsedAction::R1_CLEAR_MOVE;
        try {
            const auto pos = s.find(" then");
            p.target = std::stoi(s.substr(9, (pos == std::string::npos ? std::string::npos : pos - 9)));
        } catch (...) { p.op = ParsedAction::UNKNOWN; }
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

    // prep = 从 target_center 沿主方向(±x 或 ±y)退 1 格距离的位置，机器人在 from
    // 上对齐到这一点；yaw 朝向 target。MOVE/PICK/PUSH 共用同一套规则。
    //   - 进入/离开赛场（涉及 0 或 13 strip）强制 +x 轴；
    //   - 否则取 |dx|, |dy| 中较大者作主方向；浮点等量时退化为 +x 兜底。
    constexpr double CELL_PITCH = 1.2;  // m
    constexpr double AXIS_EPS = 1e-3;   // m，浮点比较容差
    auto fill_prep_pose = [&](int from_id, int target_id, robot_interfaces::msg::PlanStep & step) {
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

        step.prep_pose.x = t.x - CELL_PITCH * ux;
        step.prep_pose.y = t.y - CELL_PITCH * uy;
        step.prep_pose.theta = std::atan2(uy, ux);
    };

    for (const std::string & line : raw) {
        const ParsedAction a = parseAction(line);
        if (a.op == ParsedAction::UNKNOWN) {
            continue;
        }

        robot_interfaces::msg::PlanStep step;

        if (a.op == ParsedAction::MOVE || a.op == ParsedAction::R1_CLEAR_MOVE) {
            step.type = robot_interfaces::msg::PlanStep::TYPE_MOVE;
            step.from_id = robot_node;
            step.target_id = a.target;

            fill_prep_pose(step.from_id, step.target_id, step);

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
        } else if (a.op == ParsedAction::PICK || a.op == ParsedAction::PUSH) {
            step.type = (a.op == ParsedAction::PICK)
                ? robot_interfaces::msg::PlanStep::TYPE_PICK
                : robot_interfaces::msg::PlanStep::TYPE_PUSH;
            step.from_id = robot_node;     // 机器人原地不动
            step.target_id = a.target;

            // PICK/PUSH 复用 prep 公式：robot 在 from 块上对齐到 target 那一侧
            fill_prep_pose(step.from_id, step.target_id, step);
            const auto & f = blocks_.at(step.from_id);
            const auto & t = blocks_.at(step.target_id);
            const double dir_to_target = std::atan2(t.y - f.y, t.x - f.x);
            // 机械臂在车体右侧：让 base 的右侧方向(yaw - pi/2)对准目标方向。
            step.grasp_yaw = normalize_angle(dir_to_target + M_PI_2);

            step.cube_x = t.cube_x;
            step.cube_y = t.cube_y;
            step.block_height = t.height;
        } else {
            continue;
        }

        out.push_back(step);
    }

    return out;
}
} // namespace r2_planner
