#pragma once

namespace planning {

// todo, move g cost related codes to this file
// todo, searching path consider path shape
enum class PathShapeType {
  none,
  straight,
  left,
  right,
  s_turn,
  u_turn,
  acute_angle,
  zigzag_path,
  max_num,
};

struct NodeGCost {
  double dist_cost;
  double gear_cost;
  double steer_change_cost;
  double steer_cost;
  double expected_gear_cost;

  // 小于期望值，发生换档，那么产生一个cost
  double expected_dist_cost;
  double obs_dist_cost;
  double ref_line_heading_cost;

  double total_cost;
};

}  // namespace planning