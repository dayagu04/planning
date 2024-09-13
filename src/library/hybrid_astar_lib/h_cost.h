#pragma once

#include <cmath>

namespace planning {

struct NodeHeuristicCost {
  double euler_dist;
  // dynamic_programing_cost: get distance for node without any collision.
  double astar_dist;
  double ref_line_heading_cost;
  double rs_path_dist;
  double rs_path_gear;
  double rs_path_steer;
  double expected_gear;
  double expected_dist;

  double total_cost;
};

}  // namespace planning