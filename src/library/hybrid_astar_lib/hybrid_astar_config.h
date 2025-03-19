#pragma once

#include <vector>

namespace planning {

// 1. 库内库外可以使用不同的安全buffer;
// 2. 在大buffer计算失败时，再使用小buffer;
struct OpenSpaceSafeBuffer {
  // 车在车位外部的横向安全buffer，需要设定大一些
  std::vector<double> lat_safe_buffer_outside;
  // 车位内部的横向安全buffer，需要设定小一些
  std::vector<double> lat_safe_buffer_inside;

  // 纵向buffer不区分库内库外
  std::vector<double> lon_safe_buffer;
  // 非移动方向的buffer. 即D档时，车尾的lon buffer; R档时，车头的lon buffer;
  double lon_min_safe_buffer;

  // scenario try for prepare planning: add more safe buffer than parking in
  // state.
  double scenario_try_lat_buffer_outside;
  double scenario_try_lat_buffer_inside;
  double scenario_try_lon_buffer;
};

struct PlannerOpenSpaceConfig {
  // heuristic cost
  // xy resolution for heuristic
  double heuristic_grid_resolution;

  // safe dist for heuristic cost
  double heuristic_safe_dist;

  // vertical parking
  bool enable_euler_cost_for_vertical_park = false;
  bool enable_dp_cost_for_vertical_park = false;
  bool enable_ref_line_h_cost_for_vertical_park = false;
  bool enable_rs_path_h_cost_for_vertical_park = false;

  // headin vertical parking
  double headin_limit_y_shrink = 1.2;

  // a star search resolution
  double xy_grid_resolution;
  double phi_grid_resolution;

  // for now, front and back sampling node number both are 5.
  int next_node_num = 10;
  double node_step;
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

  double expect_gear_penalty;
  double expect_dist_penalty;
  double gear_switch_penalty_heu;

  // check rs path segment min distance
  double rs_path_seg_advised_dist = 0.35;

  int max_gear_change_num;

  double tie_breaker_;

  // 安全宽度超过阈值，可以尝试单个档位搜索
  double single_shot_path_width_thresh;

  // A星最大搜索时间: 暂时设定为5秒，超过这个时间不要继续搜索
  double max_search_time_ms;
  // 无换档路径最大搜索时间: 默认100ms
  double max_search_time_ms_for_no_gear_switch;

  // for sampling method, set a distance for adjust car inside slot.
  double adjust_dist_inside_slot;

  OpenSpaceSafeBuffer safe_buffer;

  void InitConfig();
};

}  // namespace planning