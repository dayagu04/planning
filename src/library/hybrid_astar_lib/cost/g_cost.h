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
  float dist_cost;
  float gear_cost;
  float steer_change_cost;
  float steer_cost;
  float expected_gear_cost;

  // 小于期望值，发生换档，那么产生一个cost
  float expected_dist_cost;
  float obs_dist_cost;

  float total_cost;
};

}  // namespace planning