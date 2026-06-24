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

// 方块类型枚举
enum class BlockState {
    EMPTY,       // 空的，可通行
    R1_KFS,      // R1 秘籍 (R2的红色障碍)
    R2_KFS,      // R2 秘籍 (R2的绿色目标)
    FAKE_KFS     // 假秘籍 (黑色障碍)
};

// --- 环境模型 ---
// 代价配置：默认值等于历史写死的常量，保证不传配置的调用者行为不变。
// 高度阈值单位为米（与 block_*.yaml 的 height 一致）。
struct CostConfig {
    double move_cost = 1.0;          // 平移基础代价
    double descend_200_cost = 0.2;   // 下台阶 高度差落在 [-0.2, 0) 的额外代价
    double descend_400_cost = 0.4;   // 下台阶 高度差落在 [-0.4, -0.2) 的额外代价
    double pick_cost = 2.0;          // 抓取 R2 KFS
    double push_cost = 4.0;          // 推走挡路 KFS
    double climb_200_cost = 1.5;     // 高度差 (0, 0.2]
    double climb_400_cost = 3.5;     // 高度差 (0.2, 0.4]，can_climb_400=true 时生效
    double turn_cost = 0.5;          // 每转 90° 的额外代价（掉头 180° = 2×）
    bool   can_climb_400 = true;     // false → 400 档不可上（代价极大，绕开）
    double prohibitive_cost = 10000.0; // 不可通行档（>0.4 或 400 被禁）
};

struct ForestConfig {
    // 节点高度字典, key 为节点 ID (0-13), value 为高度 (m)
    std::unordered_map<int, double> node_heights;

    // 初始状态下各个节点上的物品
    std::unordered_map<int, BlockState> initial_items;

    // 相邻节点的映射表, key 为节点 ID, value 为相邻节点列表
    std::unordered_map<int, std::vector<int>> adjacency_list;

    // 定义曼哈顿距离或者欧式距离的基础网格距离
    double block_size = 350.0; // KFS 是 350mm 正方体，假设方块尺寸相关

    // 准备位姿相对目标中心的偏移量（米，沿单轴向目标方向退此距离；越小越靠近目标）
    double move_prep_offset  = 0.1;  // MOVE 准备点
    double grasp_prep_offset = 0.1;  // PICK / PUSH 准备点

    // PICK / PUSH 步骤发布的 block_height 偏移量（米）。
    // block_height = (target.height - from.height) + block_height_offset。
    // 仅影响发布给下游的字段，不参与 A* 代价计算。
    double block_height_offset = 0.0;

    // PICK / PUSH 步骤 prep_pose.theta 的偏移量（弧度），左右两档共用此偏移。
    // step.prep_pose.theta = base_theta + grasp_prep_theta_offset；grasp_yaw 同步加。
    double grasp_prep_theta_offset = 0.0;

    // MOVE 步骤 prep_pose.theta 的偏移量（弧度）。
    // 仅作用在导航 goal 朝向；turn_deg 用未偏移值计算，保证 MOVE 之间转角不被污染。
    double move_prep_theta_offset = 0.0;

    // 代价配置（可由 yaml 覆盖）
    CostConfig cost;
};

// --- 状态表示 ---
// 复合状态向量，用于 A* 搜索中的节点
struct SearchState {
    int current_node_id;       // R2 当前所在的节点 ID
    int kfs_held_count;        // 当前抓取的 KFS 数量 (0, 1, 2)
    uint16_t env_mask;         // 环境掩码，12 个位代表 1-12 号方块是否仍被占据 (True=不可通行)
    int heading;               // 车头朝向 0/1/2/3 = +x/+y/-x/-y；起点朝 +x（对着 1/2/3）

    // 用于优先级队列和路径回溯的属性
    double g_cost;             // 从起点到当前状态的实际累计代价
    double f_cost;             // f(n) = g(n) + h(n)，总代价预估

    std::shared_ptr<SearchState> parent; // 父状态指针，用于回溯路径
    std::string action_taken;            // 从父状态到当前状态采取的动作

    // 重载判等运算符
    bool operator==(const SearchState& other) const {
        return current_node_id == other.current_node_id &&
               kfs_held_count == other.kfs_held_count &&
               env_mask == other.env_mask &&
               heading == other.heading;
    }
};

// 自定义哈希函数
struct StateHasher {
    std::size_t operator()(const SearchState& state) const {
        std::size_t h1 = std::hash<int>()(state.current_node_id);
        std::size_t h2 = std::hash<int>()(state.kfs_held_count);
        std::size_t h4 = std::hash<uint16_t>()(state.env_mask);
        std::size_t h5 = std::hash<int>()(state.heading);
        return h1 ^ (h2 << 1) ^ (h4 << 3) ^ (h5 << 5);
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

    // 由 from→to 的网格关系算出行进方向 0/1/2/3 = +x/+y/-x/-y
    // （+x=前进/对着 1·2·3 侧，+y=左）。进出场（含 0、13）一律按 +x。
    int moveHeading(int from_node, int to_node) const;
    // 从 cur 朝向转到 next 朝向需要的 90° 档数（0/1/2），用于转弯代价。
    int turnQuarters(int from_heading, int to_heading) const;

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
