#pragma once

#include <vector>
#include <array>

namespace planning {

// 1. 库内库外可以使用不同的安全buffer;
// 2. 在大buffer计算失败时，再使用小buffer;
struct OpenSpaceSafeBuffer {
  // If vehicle is outside slot, need more safe buffer than inside slot;
  // If vehicle's path is circle, need more safe buffer than straight path;
  std::array<float, 3> lat_safe_buffer_outside;
  float circle_path_extra_buffer_outside;

  // 车位内部的横向安全buffer，需要设定小一些
  std::array<float, 3> lat_safe_buffer_inside;
  float circle_path_extra_buffer_inside;

  // 纵向buffer不区分库内库外
  std::array<float, 3> lon_safe_buffer;
  // 非移动方向的buffer. 即D档时，车尾的lon buffer; R档时，车头的lon buffer;
  float lon_min_safe_buffer;

  // scenario try for prepare planning: add more safe buffer than parking in
  // state.
  float scenario_try_lat_buffer_outside;
  float scenario_try_lat_buffer_inside;
  float scenario_try_lon_buffer;
};

struct PlannerOpenSpaceConfig {
  // heuristic cost
  // xy resolution for heuristic
  float heuristic_grid_resolution;

  // safe dist for heuristic cost
  float heuristic_safe_dist;

  // vertical parking
  bool enable_euler_cost_for_vertical_park = false;
  bool enable_dp_cost_for_vertical_park = false;
  bool enable_ref_line_h_cost_for_vertical_park = false;
  bool enable_rs_path_h_cost_for_vertical_park = false;

  // headin vertical parking
  float headin_limit_y_shrink = 1.2;

  // a star search resolution
  float xy_grid_resolution;
  float phi_grid_resolution;

  // for now, front and back sampling node number both are 5.
  int next_node_num = 10;
  float node_step;
  float perpendicular_slot_node_step;
  float perpendicular_head_out_slot_node_step;
  float parallel_slot_node_step;

  float xy_grid_resolution_inv;
  float phi_grid_resolution_inv;

  // node path sampling dist
  float node_path_dist_resolution = 0.1;
  float rs_path_dist_resolution = 0.5;

  float traj_forward_penalty = 1.0;
  float traj_reverse_penalty = 1.0;
  float gear_switch_penalty = 20.0;
  float traj_steer_penalty = 0.0;
  float traj_steer_change_penalty = 0.0;
  float ref_line_heading_penalty = 0.0;

  float expect_gear_penalty;
  float expect_dist_penalty;

  // check rs path segment min distance
  float rs_path_seg_advised_dist = 0.35;

  int max_gear_change_num;

  float tie_breaker_;

  // 安全宽度超过阈值，可以尝试单个档位搜索
  float single_shot_path_width_thresh;

  // A星最大搜索时间: 暂时设定为5秒，超过这个时间不要继续搜索
  double max_search_time_ms;
  // 无换档路径最大搜索时间: 默认100ms
  double max_search_time_ms_for_no_gear_switch;

  // for sampling method, set a distance for adjust car inside slot.
  float adjust_dist_inside_slot;

  // Control module has some tracking errors, so add a buffer for minimum turn
  // radius.
  float turn_radius_buffer;

  OpenSpaceSafeBuffer safe_buffer;

  void InitConfig();
};

}  // namespace planning