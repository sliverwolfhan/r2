#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "r2_meilin_planner/planner.hpp"
#include "r2_meilin_planner/forest_defaults.hpp"
#include <iostream>
#include <map>
#include <random>
#include <numeric>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <algorithm>
#include <set>
#include <vector>

using namespace r2_planner;

enum class GamePhase { BLANK, KFS_LOADED, R1_ASSIST, PATH_PLANNED, EXECUTING, FINISHED };

// --- 键盘交互 ---
int kbhit(void) {
    struct termios oldt, newt; int ch, oldf;
    tcgetattr(STDIN_FILENO, &oldt); newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if(ch != EOF) { ungetc(ch, stdin); return 1; }
    return 0;
}
char getkey() { if (kbhit()) return getchar(); return 0; }

// 与规划器一致：方块 id 1=右下 … 12=左上；坐标沿用旧 id 的物理位置做 13-id 映射
std::map<int, std::pair<double, double>> node_coords = {
    {0, {0.4, -1.6}},
    {1, {0.8, -1.2}},
    {2, {0.4, -1.2}},
    {3, {0.0, -1.2}},
    {4, {0.8, -0.8}},
    {5, {0.4, -0.8}},
    {6, {0.0, -0.8}},
    {7, {0.8, -0.4}},
    {8, {0.4, -0.4}},
    {9, {0.0, -0.4}},
    {10, {0.8, 0.0}},
    {11, {0.4, 0.0}},
    {12, {0.0, 0.0}},
    {13, {0.4, 0.4}}
};

// 随机配置
ForestConfig buildRandomConfig() {
    ForestConfig config;
    fill_default_forest_topology(config);
    for (int i = 1; i <= 12; ++i) config.initial_items[i] = BlockState::EMPTY;
    std::random_device rd; std::mt19937 g(rd());
    std::vector<int> fake_c; for(int i=4; i<=9; ++i) fake_c.push_back(i); // 假块放中间
    std::shuffle(fake_c.begin(), fake_c.end(), g);
    config.initial_items[fake_c[0]] = BlockState::FAKE_KFS;
    std::vector<int> side = {1, 2, 3, 4, 6, 7, 9, 10, 11, 12};
    std::shuffle(side.begin(), side.end(), g);
    int p_r1 = 0;
    for(int s : side) if(config.initial_items[s]==BlockState::EMPTY){ config.initial_items[s]=BlockState::R1_KFS; p_r1++; if(p_r1>=3) break; }
    std::vector<int> rem; for(int i=1; i<=12; ++i) if(config.initial_items[i]==BlockState::EMPTY) rem.push_back(i);
    std::shuffle(rem.begin(), rem.end(), g);
    for(int i=0; i<4; ++i) config.initial_items[rem[i]] = BlockState::R2_KFS;
    return config;
}

// 参数配置
ForestConfig buildFromParams(rclcpp::Node::SharedPtr node) {
    ForestConfig config;
    fill_default_forest_topology(config);
    for (int i = 1; i <= 12; ++i) config.initial_items[i] = BlockState::EMPTY;
    std::vector<int64_t> r2, r1;
    int64_t fk = -1;
    node->get_parameter("r2_kfs", r2); node->get_parameter("r1_kfs", r1); node->get_parameter("fake_kfs", fk);
    for(auto id : r2) if(id>=1 && id<=12) config.initial_items[id] = BlockState::R2_KFS;
    for(auto id : r1) if(id>=1 && id<=12) config.initial_items[id] = BlockState::R1_KFS;
    if(fk>=1 && fk<=12) config.initial_items[fk] = BlockState::FAKE_KFS;
    return config;
}

void publishMarkers(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub, 
                    GamePhase phase, const ForestConfig& config, const std::vector<std::string>& path,
                    int current_node, const std::set<int>& picked_kfs, const std::set<int>& pushed_kfs, const std::set<int>& removed_r1) {
    visualization_msgs::msg::MarkerArray markers; int id = 0; auto now = rclcpp::Clock().now();
    visualization_msgs::msg::Marker d; d.header.frame_id = "map"; d.header.stamp = now;
    d.action = visualization_msgs::msg::Marker::DELETEALL; markers.markers.push_back(d);

    for (int i = 1; i <= 12; ++i) {
        visualization_msgs::msg::Marker m; m.header.frame_id = "map"; m.header.stamp = now; m.ns = "blocks"; m.id = id++;
        m.type = visualization_msgs::msg::Marker::CUBE; m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = node_coords[i].first; m.pose.position.y = node_coords[i].second;
        m.pose.position.z = (config.node_heights.count(i)) ? config.node_heights.at(i)/2000.0 : 0.2;
        m.scale.x = 0.38; m.scale.y = 0.38; m.scale.z = (config.node_heights.count(i)) ? config.node_heights.at(i)/1000.0 : 0.4;
        m.color.r = 0.5f; m.color.g = 0.5f; m.color.b = 0.5f; m.color.a = 0.6f; markers.markers.push_back(m);
        
        // Skip rendering if R1 removed it, or R2 pushed it off board
        if (removed_r1.count(i) || pushed_kfs.count(i)) continue;

        if (phase != GamePhase::BLANK && config.initial_items.count(i) && config.initial_items.at(i) != BlockState::EMPTY) {
            visualization_msgs::msg::Marker k; k.header.frame_id = "map"; k.header.stamp = now; k.ns = "kfs"; k.id = id++;
            k.type = visualization_msgs::msg::Marker::CUBE; k.action = visualization_msgs::msg::Marker::ADD;
            k.pose.position.x = node_coords[i].first; k.pose.position.y = node_coords[i].second;
            k.pose.position.z = config.node_heights.at(i)/1000.0 + 0.175;
            k.scale.x = 0.35; k.scale.y = 0.35; k.scale.z = 0.35;
            if (picked_kfs.count(i)) { k.color.r = 1.0f; k.color.g = 1.0f; k.color.b = 0.0f; k.color.a = 1.0f; }
            else { auto t = config.initial_items.at(i);
                if(t==BlockState::R1_KFS){ k.color.r=1.0; k.color.g=0.0; k.color.b=0.0; k.color.a=1.0; }
                else if(t==BlockState::R2_KFS){ k.color.r=0.0; k.color.g=1.0; k.color.b=0.0; k.color.a=1.0; }
                else { k.color.r=0.0; k.color.g=0.4; k.color.b=1.0; k.color.a=1.0; } 
            }
            markers.markers.push_back(k);
        }
    }
    if (phase >= GamePhase::PATH_PLANNED) {
        visualization_msgs::msg::Marker p; p.header.frame_id = "map"; p.header.stamp = now; p.ns = "path"; p.id = id++;
        p.type = visualization_msgs::msg::Marker::LINE_STRIP; p.action = visualization_msgs::msg::Marker::ADD;
        p.scale.x = 0.02; p.color.r = 1.0f; p.color.g = 1.0f; p.color.b = 0.0f; p.color.a = 0.5f;
        geometry_msgs::msg::Point pt; pt.x = node_coords[0].first; pt.y = node_coords[0].second; pt.z = 0.05; p.points.push_back(pt);
        for(const auto& s : path) if(s.find("MOVE to ") != std::string::npos) {
            int nid = std::stoi(s.substr(8)); pt.x = node_coords[nid].first; pt.y = node_coords[nid].second;
            pt.z = config.node_heights.at(nid)/1000.0 + 0.05; p.points.push_back(pt);
        }
        markers.markers.push_back(p);
        visualization_msgs::msg::Marker r; r.header.frame_id = "map"; r.header.stamp = now; r.ns = "robot"; r.id = id++;
        r.type = visualization_msgs::msg::Marker::SPHERE; r.action = visualization_msgs::msg::Marker::ADD;
        r.pose.position.x = node_coords[current_node].first; r.pose.position.y = node_coords[current_node].second;
        r.pose.position.z = config.node_heights.at(current_node)/1000.0 + 0.25;
        r.scale.x = 0.25; r.scale.y = 0.25; r.scale.z = 0.25; r.color.r = 1.0f; r.color.g = 1.0f; r.color.b = 0.0f; r.color.a = 1.0f;
        markers.markers.push_back(r);
    }
    pub->publish(markers);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv); auto node = rclcpp::Node::make_shared("r2_planner_node");
    node->declare_parameter<std::vector<int64_t>>("r2_kfs", std::vector<int64_t>{});
    node->declare_parameter<std::vector<int64_t>>("r1_kfs", std::vector<int64_t>{});
    node->declare_parameter<int64_t>("fake_kfs", -1);

    auto marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);
    RCLCPP_INFO(node->get_logger(), "=== SMART SIMULATOR (WITH PUSH) READY ===");
    RCLCPP_INFO(node->get_logger(), "Usage: No params = Random Mode, With params = Fixed Mode. [p] Next, [o] Reset.");

    GamePhase phase = GamePhase::BLANK; ForestConfig config; std::vector<std::string> path;
    int current_node = 0; size_t step_idx = 0; 
    std::set<int> picked_kfs, pushed_kfs, removed_r1; 
    rclcpp::WallRate rate(30);

    while (rclcpp::ok()) {
        char key = getkey();
        if (key == 'o' || key == 'O') {
            phase = GamePhase::BLANK; picked_kfs.clear(); pushed_kfs.clear(); removed_r1.clear();
            step_idx = 0; current_node = 0; path.clear(); config.initial_items.clear();
            RCLCPP_INFO(node->get_logger(), ">>> RESET: Screen Cleared.");
        } else if (key == 'p' || key == 'P') {
            if (phase == GamePhase::BLANK) {
                std::vector<int64_t> r2_p; node->get_parameter("r2_kfs", r2_p);
                if (r2_p.empty()) { config = buildRandomConfig(); RCLCPP_INFO(node->get_logger(), ">>> Phase 1: Generated ALL 8 KFS."); }
                else { config = buildFromParams(node); RCLCPP_INFO(node->get_logger(), ">>> Phase 1: Loaded CUSTOM Layout from CLI."); }
                phase = GamePhase::KFS_LOADED;
            } else if (phase == GamePhase::KFS_LOADED) {
                // R1 Assist Phase: 找出 R1 KFS 并拿走两个
                std::vector<int> r1_list;
                for (int i = 1; i <= 12; ++i) {
                    if (config.initial_items[i] == BlockState::R1_KFS) r1_list.push_back(i);
                }
                std::random_device rd; std::mt19937 g(rd());
                std::shuffle(r1_list.begin(), r1_list.end(), g);
                
                int removed_count = 0;
                for (int r1_idx : r1_list) {
                    if (removed_count >= 2) break; // R1 最多拿走2个
                    removed_r1.insert(r1_idx);
                    config.initial_items[r1_idx] = BlockState::EMPTY; // 在物理世界上清空
                    removed_count++;
                }
                
                RCLCPP_INFO(node->get_logger(), ">>> Phase 2: R1 Assist. Removed %d Red KFS.", removed_count);
                for(int r : removed_r1) RCLCPP_INFO(node->get_logger(), "    - R1 removed obstacle at Block %d", r);
                phase = GamePhase::R1_ASSIST;
            } else if (phase == GamePhase::R1_ASSIST) {
                R2MeilinPlanner planner(config); path = planner.planPath(); phase = GamePhase::PATH_PLANNED;
                current_node = 0; step_idx = 0;
                RCLCPP_INFO(node->get_logger(), ">>> Phase 3: R2 Path Planned (Target: 2 R2 KFS, Push allowed).");
            } else if (phase >= GamePhase::PATH_PLANNED && step_idx < path.size()) {
                phase = GamePhase::EXECUTING; const std::string& step = path[step_idx++];
                if (step.find("MOVE to ") != std::string::npos) {
                    current_node = std::stoi(step.substr(8));
                    RCLCPP_INFO(node->get_logger(), "[STEP %zu] MOVE: Block %d (%.0fmm)", step_idx, current_node, config.node_heights.at(current_node));
                } else if (step.find("PICK at ") != std::string::npos) {
                    int target = std::stoi(step.substr(8)); picked_kfs.insert(target);
                    RCLCPP_INFO(node->get_logger(), "[STEP %zu] PICK: Green KFS at %d -> Turned YELLOW", step_idx, target);
                } else if (step.find("PUSH ") != std::string::npos) {
                    int target = std::stoi(step.substr(5)); pushed_kfs.insert(target);
                    RCLCPP_INFO(node->get_logger(), "[STEP %zu] PUSH: Pushed Green KFS at %d OUT of bounds! (Destroyed)", step_idx, target);
                } else if (step.find("R1_CLEAR ") != std::string::npos) {
                    int target = std::stoi(step.substr(9, step.find(" then") - 9));
                    current_node = target;
                    removed_r1.insert(target);
                    config.initial_items[target] = BlockState::EMPTY;
                    RCLCPP_INFO(node->get_logger(), "[STEP %zu] R1_CLEAR: R1 cleared Block %d for R2!", step_idx, target);
                }
                if (step_idx >= path.size()) {
                    phase = GamePhase::FINISHED;
                    RCLCPP_INFO(node->get_logger(), "================ MISSION COMPLETE ================");
                }
            }
        }
        publishMarkers(marker_pub, phase, config, path, current_node, picked_kfs, pushed_kfs, removed_r1);
        rclcpp::spin_some(node); rate.sleep();
    }
    rclcpp::shutdown(); return 0;
}

