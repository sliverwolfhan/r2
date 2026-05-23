#ifndef R2_MEILIN_PLANNER_FOREST_DEFAULTS_HPP_
#define R2_MEILIN_PLANNER_FOREST_DEFAULTS_HPP_

#include "r2_meilin_planner/planner.hpp"

namespace r2_planner {

/**
 * Fills adjacency_list and node_heights for the forest grid.
 * Block ids 1–12 use the same convention as the Qt UI: 1 = bottom-right, then left along the bottom row,
 * then the row above from right to left, … 12 = top-left. Entry = 0, exit = 13.
 */
void fill_default_forest_topology(ForestConfig & config);

}  // namespace r2_planner

#endif
