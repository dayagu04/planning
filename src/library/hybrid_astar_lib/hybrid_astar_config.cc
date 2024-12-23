#include "hybrid_astar_config.h"

namespace planning {

void PlannerOpenSpaceConfig::InitConfig() {
  heuristic_grid_resolution = 0.3;

  heuristic_safe_dist = 0.05;

  // a star search resolution
  // if change reso, some bug will appear
  xy_grid_resolution = 0.2;
  xy_grid_resolution_inv = 1.0 / xy_grid_resolution;

  // 3 degree, 2 degree, 1 degree. default is 3.
  // todo: maybe 1 is better.
  phi_grid_resolution = 0.0524;
  phi_grid_resolution_inv = 1.0 / phi_grid_resolution;
  next_node_num = 10;
  // todo:step 0.4,0.2,0.1. maybe 0.1 is better.
  node_step = 0.4;
  // node_step = 0.3;
  // node_step = 0.2;

  // node path sampling dist
  node_path_dist_resolution = 0.1;

  rs_path_dist_resolution = 0.1;

  traj_forward_penalty = 1.0;
  traj_reverse_penalty = 1.0;
  gear_switch_penalty = 10.0;
  // gear_switch_penalty = 4.0;
  traj_steer_penalty = 0.0;
  // traj_steer_change_penalty = 4.0;
  traj_steer_change_penalty = 0.0;
  lat_hierarchy_safe_buffer.emplace_back(0.2);
  lat_hierarchy_safe_buffer.emplace_back(0.1);
  lon_hierarchy_safe_buffer.push_back(0.4);
  lon_hierarchy_safe_buffer.push_back(0.2);
  lon_front_safe_buffer = 0.4;
  lon_back_safe_buffer = 0.4;
  lon_min_safe_buffer = 0.01;

  expect_gear_penalty = 50.0;
  expect_dist_penalty = 7.0;
  gear_switch_penalty_heu = 10.0;

  enable_euler_cost_for_vertical_park = true;
  enable_dp_cost_for_vertical_park = true;
  enable_ref_line_h_cost_for_vertical_park = true;
  enable_rs_path_h_cost_for_vertical_park = true;
  max_gear_change_num = 10;
  ref_line_heading_penalty = 0.0;

  map_bound_min_x = -3.0;
  map_bound_max_x = 20.0;
  map_bound_min_y = -20.0;
  map_bound_max_x = 20.0;

  tie_breaker_ = 1e-5;

  // reverse gear path searching, no gear switch
  single_shot_path_end_straight_dist = 0.7;
  perpendicular_slot_node_step = 0.4;
  parallel_slot_node_step = 0.3;

  return;
}

}  // namespace planning