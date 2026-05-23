#include "r2_meilin_planner/planner.hpp"
#include "r2_meilin_planner/forest_defaults.hpp"
#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#include <numeric>

using namespace r2_planner;

ForestConfig buildRandomConfig(int seed) {
    ForestConfig config;
    fill_default_forest_topology(config);
    for (int i = 1; i <= 12; ++i) config.initial_items[i] = BlockState::EMPTY;
    
    std::mt19937 g(seed);
    
    // 1. 生成 1 个假 KFS (4-12号)
    std::vector<int> fake_c; for(int i=4; i<=12; ++i) fake_c.push_back(i);
    std::shuffle(fake_c.begin(), fake_c.end(), g);
    config.initial_items[fake_c[0]] = BlockState::FAKE_KFS;
    
    // 2. 生成 3 个 R1 KFS (1-12号，边缘优先，但在仿真中我们直接模拟 R1 已经拿走了2个的情况)
    // 为了公平测试 R2 规划，我们假设 R1 已经清除了 2 个红色障碍，只剩下 1 个
    std::vector<int> side = {1, 2, 3, 4, 6, 7, 9, 10, 11, 12};
    std::shuffle(side.begin(), side.end(), g);
    int p_r1 = 0;
    for(int s : side) {
        if(config.initial_items[s] == BlockState::EMPTY) {
            config.initial_items[s] = BlockState::R1_KFS;
            p_r1++;
            if(p_r1 >= 1) break; // 模拟 R1 已经帮我们清掉了两个，剩下一个干扰项
        }
    }
    
    // 3. 生成 4 个 R2 KFS (绿色目标)
    std::vector<int> rem;
    for(int i=1; i<=12; ++i) if(config.initial_items[i] == BlockState::EMPTY) rem.push_back(i);
    std::shuffle(rem.begin(), rem.end(), g);
    for(int i=0; i<4; ++i) config.initial_items[rem[i]] = BlockState::R2_KFS;
    
    return config;
}

int main() {
    const int TEST_COUNT = 100;
    std::vector<int> steps_results;
    int success_count = 0;

    std::cout << "Starting Batch Evaluation (100 Random Layouts)..." << std::endl;
    std::cout << "------------------------------------------------" << std::endl;

    std::random_device rd; // 硬件真随机数生成器
    for (int i = 0; i < TEST_COUNT; ++i) {
        ForestConfig config = buildRandomConfig(rd()); // 使用真随机数作为种子
        R2MeilinPlanner planner(config);
        std::vector<std::string> path = planner.planPath();

        if (!path.empty() && path[0].find("FAIL") == std::string::npos) {
            success_count++;
            steps_results.push_back(path.size());
            std::cout << "Run " << i + 1 << ": SUCCESS, Steps = " << path.size() << std::endl;
        } else {
            std::cout << "Run " << i + 1 << ": FAILED to find path." << std::endl;
        }
    }

    if (!steps_results.empty()) {
        double avg = std::accumulate(steps_results.begin(), steps_results.end(), 0.0) / steps_results.size();
        auto min_max = std::minmax_element(steps_results.begin(), steps_results.end());
        
        std::cout << "------------------------------------------------" << std::endl;
        std::cout << "BATCH RESULTS (SUCCESS RATE: " << success_count << "/" << TEST_COUNT << ")" << std::endl;
        std::cout << "Average Steps: " << avg << std::endl;
        std::cout << "Minimum Steps: " << *min_max.first << std::endl;
        std::cout << "Maximum Steps: " << *min_max.second << std::endl;
    } else {
        std::cout << "All tests failed." << std::endl;
    }

    return 0;
}
