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

  // node path sampling dist
  node_path_dist_resolution = 0.1;

  rs_path_dist_resolution = 0.1;

  traj_forward_penalty = 1.0;
  traj_reverse_penalty = 1.0;
  traj_forward_penalty_park_out = 3.0;
  gear_switch_penalty = 10.0;
  traj_steer_penalty = 0.0;
  traj_steer_change_penalty = 4.0;
  s_curve_penalty = 0.1;
  traj_kappa_penalty = 0.0;
  traj_kappa_change_penalty = 0.0;

  expect_gear_penalty = 7.0;
  expect_steer_penalty = 0.4;
  expect_dist_penalty = 7.0;
  recommend_box_penalty = 7.0;

  exceed_pre_search_box_penalty = 0.0;
  exceed_intersting_box_penalty = 0.0;
  borrow_slot_penalty = 0.0;

  enable_euler_cost_for_vertical_park = true;
  enable_dp_cost_for_vertical_park = true;
  enable_ref_line_h_cost_for_vertical_park = true;
  enable_rs_path_h_cost_for_vertical_park = true;
  max_gear_change_num = 14;
  ref_line_heading_penalty = 0.0;

  rs_path_seg_advised_dist = 0.35;
  tie_breaker_ = 1e-5;

  single_shot_path_width_thresh = 0.10;
  perpendicular_slot_node_step = 0.4;
  perpendicular_slot_head_out_node_step = 0.3;
  parallel_slot_node_step = 0.3;

  max_search_time_ms = 4500;
  max_search_time_ms_for_no_gear_switch = 120;
  search_time_ms_scenario_try = 50;
  search_time_by_buffer[0] = 800;
  search_time_by_buffer[1] = 1200;
  search_time_by_buffer[2] = 3600;
  adjust_dist_inside_slot = 2.5;
  s_curve_max_num_for_node = 3;

  // update safe buffer
  // todo: use more safe buffer in release version.
  safe_buffer.lat_safe_buffer_outside[0] = 0.4;
  safe_buffer.lat_safe_buffer_outside[1] = 0.2;
  safe_buffer.lat_safe_buffer_outside[2] = 0.1;
  safe_buffer.circle_path_extra_buffer_outside = 0.10;

  safe_buffer.head_out_lat_safe_buffer_outside[0] = 0.5;
  safe_buffer.head_out_lat_safe_buffer_outside[1] = 0.2;
  safe_buffer.head_out_lat_safe_buffer_outside[2] = 0.1;

  safe_buffer.lat_safe_buffer_inside[0] = 0.2;
  safe_buffer.lat_safe_buffer_inside[1] = 0.15;
  safe_buffer.lat_safe_buffer_inside[2] = 0.08;
  safe_buffer.circle_path_extra_buffer_inside = 0.03;

  safe_buffer.lon_safe_buffer[0] = 0.4;
  safe_buffer.lon_safe_buffer[1] = 0.35;
  safe_buffer.lon_safe_buffer[2] = 0.30;
  safe_buffer.lon_min_safe_buffer = 0.01;

  // slot release related
  safe_buffer.scenario_try_lat_buffer_outside = 0.10;
  safe_buffer.scenario_try_lat_buffer_inside = 0.08;
  safe_buffer.scenario_try_lon_buffer = 0.3;

  safe_buffer.lat_dist_to_obs_for_target_pose = 0.17;

  node_turn_radius_buffer = 0.01;
  rs_radius_buffer = 0.4f;
  enable_clear_zone = false;

  vertical_map_bound_x_lower = -3.0f;
  vertical_map_bound_min_x_lower = -10.0f;

  return;
}

}  // namespace planning