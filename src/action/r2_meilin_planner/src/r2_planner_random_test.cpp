// R2 梅林规划——随机布局批量测试。
//
// 与实际算法同步：本测试用与生产节点 kfs_subscriber_node 相同的规划核心——
//   * 拓扑与高度：fill_default_forest_topology（高度与 block_*.yaml 同源，写死一致）；
//   * 代价：默认 CostConfig，其默认值与 planner_params.yaml 默认值一致；
//   * 搜索：planPath()，即生产侧 planPathStruct() 内部实际调用的 A* 核心。
// 因此这里统计的成功率/步数即真实管线的结果。
//
// 每次随机生成一个"1 假 + 1 R1 + 4 R2"的布局（已模拟 R1 赛前取走 2 个的场景）。

#include "r2_meilin_planner/planner.hpp"
#include "r2_meilin_planner/forest_defaults.hpp"
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <random>
#include <algorithm>
#include <numeric>
#include <iomanip>

using namespace r2_planner;

ForestConfig buildRandomConfig(int seed) {
    ForestConfig config;
    fill_default_forest_topology(config);
    for (int i = 1; i <= 12; ++i) config.initial_items[i] = BlockState::EMPTY;

    std::mt19937 g(seed);

    // 1. 1 个假 KFS（4-12 号）
    std::vector<int> fake_c; for (int i = 4; i <= 12; ++i) fake_c.push_back(i);
    std::shuffle(fake_c.begin(), fake_c.end(), g);
    config.initial_items[fake_c[0]] = BlockState::FAKE_KFS;

    // 2. 1 个 R1 干扰项（模拟 R1 已取走 2 个、场上只剩 1 个未取的红块当障碍）
    std::vector<int> side = {1, 2, 3, 4, 6, 7, 9, 10, 11, 12};
    std::shuffle(side.begin(), side.end(), g);
    for (int s : side) {
        if (config.initial_items[s] == BlockState::EMPTY) {
            config.initial_items[s] = BlockState::R1_KFS;
            break;
        }
    }

    // 3. 4 个 R2 KFS（绿色目标）
    std::vector<int> rem;
    for (int i = 1; i <= 12; ++i) if (config.initial_items[i] == BlockState::EMPTY) rem.push_back(i);
    std::shuffle(rem.begin(), rem.end(), g);
    for (int i = 0; i < 4; ++i) config.initial_items[rem[i]] = BlockState::R2_KFS;

    return config;
}

namespace {

std::string node_name(int id) {
    if (id == ENTRY_NODE_ID) return "入口";
    if (id == EXIT_NODE_ID) return "出口";
    return std::to_string(id);
}

double height_of(const ForestConfig & cfg, int id) {
    auto it = cfg.node_heights.find(id);
    return it != cfg.node_heights.end() ? it->second : 0.0;
}

// 由 planPath() 的字符串路径重建经过的节点序列：入口 + 每个 "MOVE to N" 的目标。
std::string route_summary(const ForestConfig & cfg, const std::vector<std::string> & path) {
    std::vector<int> route{ENTRY_NODE_ID};
    for (const auto & a : path) {
        if (a.rfind("MOVE to ", 0) == 0) {
            try { route.push_back(std::stoi(a.substr(8))); } catch (...) {}
        }
    }
    std::ostringstream so;
    so << std::fixed << std::setprecision(2);
    for (size_t i = 0; i < route.size(); ++i) {
        so << node_name(route[i]) << "(h=" << height_of(cfg, route[i]) << ")";
        if (i + 1 < route.size()) so << " -> ";
    }
    return so.str();
}

// 动作明细：把 planPath() 的原始动作翻成中文序列，便于核对
// “前排抓1个 -> 中间挡道推走 -> 出口前抓最后1个 -> 出场”。
std::string action_detail(const std::vector<std::string> & path) {
    std::ostringstream so;
    for (size_t i = 0; i < path.size(); ++i) {
        const std::string & a = path[i];
        std::string txt = a;
        if (a.rfind("MOVE to ", 0) == 0) {
            txt = "走到" + node_name(std::stoi(a.substr(8)));
        } else if (a.rfind("PICK at ", 0) == 0) {
            txt = "抓取" + node_name(std::stoi(a.substr(8)));
        } else if (a.rfind("PUSH ", 0) == 0) {
            txt = "推走" + node_name(std::stoi(a.substr(5)));
        }
        so << txt;
        if (i + 1 < path.size()) so << " | ";
    }
    return so.str();
}

// 失败时打印布局，便于排查哪种摆法无解。
std::string layout_str(const ForestConfig & cfg) {
    std::ostringstream so;
    for (int i = 1; i <= 12; ++i) {
        const char * t = "空";
        switch (cfg.initial_items.at(i)) {
            case BlockState::R1_KFS:   t = "R1"; break;
            case BlockState::R2_KFS:   t = "R2"; break;
            case BlockState::FAKE_KFS: t = "假"; break;
            default: break;
        }
        so << "[" << i << "]" << t << " ";
    }
    return so.str();
}

}  // namespace

int main() {
    const int TEST_COUNT = 100;
    std::vector<int> steps_results;
    int success_count = 0;

    std::cout << "随机布局批量测试（" << TEST_COUNT << " 次，1 假 + 1 R1 + 4 R2）\n";
    std::cout << "------------------------------------------------\n";

    std::random_device rd;  // 硬件真随机种子
    for (int i = 0; i < TEST_COUNT; ++i) {
        ForestConfig config = buildRandomConfig(rd());
        R2MeilinPlanner planner(config);
        std::vector<std::string> path = planner.planPath();

        const bool ok = !path.empty() && path[0].find("FAIL") == std::string::npos;
        if (ok) {
            success_count++;
            steps_results.push_back(static_cast<int>(path.size()));
            std::cout << "第 " << (i + 1) << " 次: 成功  步数=" << path.size()
                      << "  路线: " << route_summary(config, path) << "\n";
            std::cout << "        动作: " << action_detail(path) << "\n";
        } else {
            std::cout << "第 " << (i + 1) << " 次: 失败（无可行路径）  布局: "
                      << layout_str(config) << "\n";
        }
    }

    std::cout << "------------------------------------------------\n";
    const double rate = 100.0 * success_count / TEST_COUNT;
    std::cout << std::fixed << std::setprecision(1);
    std::cout << "成功率: " << rate << "%  (" << success_count << "/" << TEST_COUNT << ")\n";
    if (!steps_results.empty()) {
        const double avg =
            std::accumulate(steps_results.begin(), steps_results.end(), 0.0) / steps_results.size();
        auto mm = std::minmax_element(steps_results.begin(), steps_results.end());
        std::cout << std::setprecision(2);
        std::cout << "平均步数: " << avg << "  最少: " << *mm.first << "  最多: " << *mm.second << "\n";
    } else {
        std::cout << "全部失败，无步数统计。\n";
    }

    return 0;
}

