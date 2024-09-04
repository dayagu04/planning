#pragma once

#include <vector>

namespace planning {

struct PlannerOpenSpaceConfig {
  // heuristic cost
  // xy resolution for heuristic
  double heuristic_grid_resolution = 0.3;

  // safe dist for heuristic cost
  double heuristic_safe_dist = 0.05;

  // vertical parking
  bool enable_euler_cost_for_vertical_park = false;
  bool enable_dp_cost_for_vertical_park = false;
  bool enable_ref_line_h_cost_for_vertical_park = false;
  bool enable_rs_path_h_cost_for_vertical_park = false;
  bool enable_obs_dist_g_cost = false;

  // parallel parking

  // a star search resolution
  double xy_grid_resolution = 0.3;
  double phi_grid_resolution = 0.1;

  // for now, front and back sampling node number both are 5.
  int next_node_num = 10;
  double node_step = 0.2;

  double xy_grid_resolution_inv;
  double phi_grid_resolution_inv;

  // node path sampling dist
  double node_path_dist_resolution = 0.1;
  double rs_path_dist_resolution = 0.5;

  double traj_forward_penalty = 1.0;
  double traj_reverse_penalty = 1.0;
  double traj_gear_switch_penalty = 20.0;
  double traj_steer_penalty = 0.0;
  double traj_steer_change_penalty = 0.0;
  double ref_line_heading_penalty = 0.0;
  double mirror_safe_buffer;
  std::vector<double> lat_hierarchy_safe_buffer;

  double lon_front_safe_buffer = 0.5;
  double lon_back_safe_buffer = 0.1;
  double lon_min_safe_buffer = 0.1;

  double rs_path_seg_advised_dist = 0.35;

  // base coordinate can be vehicle coordinates or slot back edge coordinates
  double map_bound_min_x;
  double map_bound_max_x;
  double map_bound_min_y;
  double map_bound_max_y;

  int max_gear_change_num = 8;

  double front_overhanging;
  double rear_overhanging;
  double width_mirror;

  void InitConfig();
};

}  // namespace planning