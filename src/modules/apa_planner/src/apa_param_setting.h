#ifndef __APA_PARAM_SETTING_H__
#define __APA_PARAM_SETTING_H__

#include <cstdint>
#include <memory>
#include <vector>

#ifndef apa_param
#define apa_param planning::apa_planner::ApaParametersSetting::GetInstance()
#endif

namespace planning {
namespace apa_planner {

struct ApaParameters {
  // length unit: m   deg unit: deg  time unit: s

  // schedule params
  double plan_time = 0.1;

  // car params
  double front_overhanging = 0.924;
  double rear_overhanging = 0.94;
  double wheel_base = 2.7;
  double car_width = 1.89;
  double mirror_width = 0.2;
  std::vector<double> x_vec = {
      3.187342,  3.424531,  3.593071,  3.593071,  3.424531,  3.187342,
      2.177994,  1.916421,  1.96496,   -0.476357, -0.798324, -0.879389,
      -0.879389, -0.798324, -0.476357, 1.96496,   1.916421,  2.177994};
  std::vector<double> y_vec = {
      0.887956,  0.681712, 0.334651,  -0.334651, -0.681712, -0.887956,
      -0.887956, -1.06715, -0.887956, -0.887956, -0.706505, -0.334845,
      0.334845,  0.706505, 0.887956,  0.887956,  1.06715,   0.887956};

  // slot params
  double normal_slot_length = 5.2;

  // check finish params
  double finish_lat_err = 0.08;
  double finish_lon_err = 0.2;
  double finish_heading_err = 0.88;

  // check fail params
  double stuck_failed_time = 9.0;

  // check static params
  double car_static_pos_err = 0.005;
  double car_static_velocity = 0.01;
  double car_static_keep_time_by_pos = 1.5;
  double car_static_keep_time_by_vel = 1.5;

  // uss params
  double safe_uss_remain_dist_in_slot = 0.35;
  double safe_uss_remain_dist_out_slot = 0.55;
  double uss_stuck_replan_wait_time = 2.0;

  // check replan params
  double stuck_replan_time = 4.0;
  double max_replan_remain_dist = 0.2;

  // construct t_lane params
  double nearby_slot_corner_dist = 0.6;
  double vacant_pt_outside_dx = 0.5;
  double vacant_pt_outside_dy = 0.3;
  double vacant_pt_inside_dx = 0.8;
  double vacant_pt_inside_dy = 0.6;
  double occupied_pt_outside_dx = 0.2;
  double occupied_pt_outside_dy = 0.0;
  double occupied_pt_inside_dx = 0.3;
  double occupied_pt_inside_dy = 0.0;
  bool force_both_side_occupied = true;
  double width_threshold = 0.1;

  // construce obstacles params
  double channel_width = 12.28;
  double channel_length = 12.28;
  double max_obs2car_dist = 1.2;

  // terminal pose params
  double terminal_target_x = 1.35;
  double terminal_target_y = 0.0;
  double terminal_target_x_to_limiter = 0.15;

  // dynamic update path params
  double car_to_limiter_dis = 1.0;
  double pose_y_err = 0.15;
  double pose_heading_err = 6.6;
  double pose_slot_occupied_ratio = 0.48;
  double pose_min_remain_dis = 2.18;

  // slot update params when parking
  double fix_slot_occupied_ratio = 0.938;

  // slot update params when replan
  double uss_slot_occupied_ratio = 0.25;
  double path_heading_slot = 55.5;
  double path_pos_err = 6.6;
  double last_update_slot_occupied_ratio = 0.836;

  // path planner params
  double prepare_line_x_offset_slot = 7.2;
  double prepare_line_heading_offset_slot_deg = 8.8;
  double min_turn_radius = 5.5;
  double max_one_step_arc_radius = 8.5;
  double max_radius_in_slot = 12.66;
  double min_radius_out_slot = 7.8;
  double radius_eps = 0.01;
  double min_line_length = 0.3;
  double min_one_step_path_length = 0.6;
  double static_pos_eps = 0.01;
  double static_heading_eps = 0.8;

  // slot managent params
  double res_limiter = 0.45;

  // gen output params
  double max_velocity = 0.6;
};

class ApaParametersSetting {
 public:
  static ApaParametersSetting &GetInstance() {
    static ApaParametersSetting instance;
    return instance;
  }

  // planning parameters
  const ApaParameters &GetParam() const { return param_; }
  ApaParameters &SetPram() { return param_; }

 private:
  ApaParametersSetting(){};

  // apa parameters
  ApaParameters param_;
};

}  // namespace apa_planner
}  // namespace planning

#endif
