#include "r2_meilin_planner/forest_defaults.hpp"

namespace r2_planner {

void fill_default_forest_topology(ForestConfig & config)
{
  config.adjacency_list.clear();
  config.node_heights.clear();

  auto add_edge = [&](int a, int b) {
    config.adjacency_list[a].push_back(b);
    config.adjacency_list[b].push_back(a);
  };

  // 内部编号约定：1=右下，沿底行向左再逐层向上，12=左上
  add_edge(1, 2);
  add_edge(2, 3);
  add_edge(4, 5);
  add_edge(5, 6);
  add_edge(7, 8);
  add_edge(8, 9);
  add_edge(10, 11);
  add_edge(11, 12);

  add_edge(1, 4);
  add_edge(4, 7);
  add_edge(7, 10);
  add_edge(2, 5);
  add_edge(5, 8);
  add_edge(8, 11);
  add_edge(3, 6);
  add_edge(6, 9);
  add_edge(9, 12);

  // 入口保持在 1/2/3 侧
  add_edge(0, 1);
  add_edge(0, 2);
  add_edge(0, 3);

  // 出口在对侧（10/11/12）
  config.adjacency_list[10].push_back(13);
  config.adjacency_list[11].push_back(13);
  config.adjacency_list[12].push_back(13);
  config.adjacency_list[13] = {};

  // 高度保持原物理布局（新编号 = 13 - 旧编号）
  config.node_heights[1] = 200;
  config.node_heights[2] = 400;
  config.node_heights[3] = 200;
  config.node_heights[4] = 400;
  config.node_heights[5] = 600;
  config.node_heights[6] = 400;
  config.node_heights[7] = 200;
  config.node_heights[8] = 400;
  config.node_heights[9] = 600;
  config.node_heights[10] = 400;
  config.node_heights[11] = 200;
  config.node_heights[12] = 400;
  config.node_heights[0] = 0;
  config.node_heights[13] = 0;
}

}  // namespace r2_planner
