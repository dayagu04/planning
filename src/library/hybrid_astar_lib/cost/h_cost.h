#pragma once

#include <cmath>

namespace planning {

struct NodeHeuristicCost {
  float euler_dist;
  // dynamic_programing_cost: get distance for node without any collision.
  float astar_dist;
  float ref_line_heading_cost;
  float rs_path_dist;
  float rs_path_gear;
  float rs_path_steer;
  float expected_gear;
  float expected_dist;

  float total_cost;
};

}  // namespace planning