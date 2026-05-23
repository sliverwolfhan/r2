#ifndef R2_MEILIN_PLANNER_HPP_
#define R2_MEILIN_PLANNER_HPP_

#include <vector>
#include <unordered_map>
#include <queue>
#include <string>
#include <memory>
#include <cmath>

#include "robot_interfaces/msg/plan_step.hpp"

#include "r2_meilin_planner/block_table.hpp"

namespace r2_planner {

// --- 宏定义与常量 ---
constexpr int NUM_FOREST_BLOCKS = 12;
constexpr int ENTRY_NODE_ID = 0;   // 入口虚拟节点 (0mm 高度)
constexpr int EXIT_NODE_ID = 13;   // 出口虚拟节点 (0mm 高度)
// 方块 1–12：1=右下，沿底行向左再逐层向上，12=左上（与 Qt 显示编号一致）
constexpr int KFS_TARGET_COUNT = 2; // R2 只需要收集 2 个 R2 KFS 即可 (用户最新要求)
// R1_ASSIST_COST 调到极大，使规划器层不再产出 R1_CLEAR；BT 端不感知 R1。
constexpr double R1_ASSIST_COST = 1e9;

// 方块类型枚举
enum class BlockState {
    EMPTY,       // 空的，可通行
    R1_KFS,      // R1 秘籍 (R2的红色障碍)
    R2_KFS,      // R2 秘籍 (R2的绿色目标)
    FAKE_KFS     // 假秘籍 (黑色障碍)
};

// --- 环境模型 ---
struct ForestConfig {
    // 节点高度字典, key 为节点 ID (0-13), value 为高度 (mm)
    std::unordered_map<int, double> node_heights;
    
    // 初始状态下各个节点上的物品
    std::unordered_map<int, BlockState> initial_items;

    // 相邻节点的映射表, key 为节点 ID, value 为相邻节点列表
    std::unordered_map<int, std::vector<int>> adjacency_list;
    
    // 定义曼哈顿距离或者欧式距离的基础网格距离
    double block_size = 350.0; // KFS 是 350mm 正方体，假设方块尺寸相关
};

// --- 状态表示 ---
// 复合状态向量，用于 A* 搜索中的节点
struct SearchState {
    int current_node_id;       // R2 当前所在的节点 ID
    int kfs_held_count;        // 当前抓取的 KFS 数量 (0, 1, 2)
    int r1_cleared_count;      // R1 已经被“征用”去开了几个洞 (0, 1, 2)
    uint16_t env_mask;         // 环境掩码，12 个位代表 1-12 号方块是否仍被占据 (True=不可通行)

    // 用于优先级队列和路径回溯的属性
    double g_cost;             // 从起点到当前状态的实际累计代价
    double f_cost;             // f(n) = g(n) + h(n)，总代价预估

    std::shared_ptr<SearchState> parent; // 父状态指针，用于回溯路径
    std::string action_taken;            // 从父状态到当前状态采取的动作

    // 重载判等运算符
    bool operator==(const SearchState& other) const {
        return current_node_id == other.current_node_id &&
               kfs_held_count == other.kfs_held_count &&
               r1_cleared_count == other.r1_cleared_count &&
               env_mask == other.env_mask;
    }
};

// 自定义哈希函数
struct StateHasher {
    std::size_t operator()(const SearchState& state) const {
        std::size_t h1 = std::hash<int>()(state.current_node_id);
        std::size_t h2 = std::hash<int>()(state.kfs_held_count);
        std::size_t h3 = std::hash<int>()(state.r1_cleared_count);
        std::size_t h4 = std::hash<uint16_t>()(state.env_mask);
        return h1 ^ (h2 << 1) ^ (h3 << 2) ^ (h4 << 3);
    }
};


// 优先级队列比较器
struct StateComparator {
    bool operator()(const std::shared_ptr<SearchState>& a, const std::shared_ptr<SearchState>& b) const {
        return a->f_cost > b->f_cost; // 最小顶堆
    }
};

// --- 规划器核心类 ---
class R2MeilinPlanner {
public:
    explicit R2MeilinPlanner(const ForestConfig& config);
    R2MeilinPlanner(const ForestConfig& config, const BlockTable & blocks);

    // 字符串路径（GUI 文本框、日志使用）
    std::vector<std::string> planPath();

    /**
     * 结构化路径，每步已带好 map 系坐标和高度差信息，BT 端可直接消费。
     * 必须事先通过构造函数或 setBlockTable() 注入 BlockTable。
     */
    std::vector<robot_interfaces::msg::PlanStep> planPathStruct();

    void setBlockTable(const BlockTable & blocks) { blocks_ = blocks; has_blocks_ = true; }
    bool hasBlockTable() const { return has_blocks_; }

private:
    ForestConfig config_;
    BlockTable blocks_;
    bool has_blocks_ = false;

    // 代价计算函数
    double calculateMoveCost(int from_node, int to_node) const;
    double calculatePickCost() const;
    double calculatePushCost() const; // 新增推走 KFS 的代价

    // 启发式函数 h(n)
    double calculateHeuristic(const SearchState& state) const;

    // 根据初始配置生成初始掩码
    uint16_t generateInitialMask() const;
    
    // 检查掩码对应位是否为 1 (被占据)
    bool isNodeOccupied(uint16_t mask, int node_id) const;
    
    // 清除掩码对应位
    uint16_t clearNodeOccupied(uint16_t mask, int node_id) const;
};

} // namespace r2_planner

#endif // R2_MEILIN_PLANNER_HPP_
