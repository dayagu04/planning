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

  // headin vertical parking
  double headin_limit_y_shrink = 1.2;
  // parallel parking

  // a star search resolution
  double xy_grid_resolution = 0.3;
  double phi_grid_resolution = 0.1;

  // for now, front and back sampling node number both are 5.
  int next_node_num = 10;
  double node_step = 0.2;
  double perpendicular_slot_node_step;
  double parallel_slot_node_step;

  double xy_grid_resolution_inv;
  double phi_grid_resolution_inv;

  // node path sampling dist
  double node_path_dist_resolution = 0.1;
  double rs_path_dist_resolution = 0.5;

  double traj_forward_penalty = 1.0;
  double traj_reverse_penalty = 1.0;
  double gear_switch_penalty = 20.0;
  double traj_steer_penalty = 0.0;
  double traj_steer_change_penalty = 0.0;
  double ref_line_heading_penalty = 0.0;

  // 车在车位外部的横向安全buffer，需要设定大一些
  std::vector<double> lat_hierarchy_safe_buffer;
  std::vector<double> lon_hierarchy_safe_buffer;

  // 车在车位内部的横向安全buffer，需要设定小一些
  std::vector<double> lat_safe_buffer_for_inside;

  double expect_gear_penalty;
  double expect_dist_penalty;
  double gear_switch_penalty_heu;

  double lon_front_safe_buffer;
  double lon_back_safe_buffer;
  double lon_min_safe_buffer;

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

  double tie_breaker_;

  double single_shot_path_end_straight_dist;

  // scenario try: add more safe buffer than parking in state.
  double scenario_try_lat_buffer;
  double scenario_try_lon_buffer;

  // A星最大搜索时间: 暂时设定为5秒，超过这个时间不要继续搜索
  double max_search_time_ms;

  // 无换档路径最大搜索时间: 默认100ms
  double max_search_time_ms_for_no_gear_switch;

  // todo: 为了增加成功率，4米内的不换档路径可以使用精细碰撞检测.

  void InitConfig();
};

}  // namespace planning