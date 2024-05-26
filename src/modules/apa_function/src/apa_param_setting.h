#ifndef __APA_PARAM_SETTING_H__
#define __APA_PARAM_SETTING_H__

#include <cstddef>
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
  double car_length = 4.88;
  double mirror_width = 0.2;
  double lon_dist_mirror_to_rear_axle = 1.844;
  double lat_dist_mirror_to_center = 1.135;
  double steer_ratio = 16.5;
  double arc_line_shift_steer_angle_deg = 2.5;
  double c1 = 0.3790;
  std::vector<double> car_vertex_x_vec = {
      3.518,  3.718,  3.718,  3.518,  2.092, 2.092, 1.906, 1.906,
      -0.885, -1.085, -1.085, -0.885, 1.906, 1.906, 2.092, 2.092};
  std::vector<double> car_vertex_y_vec = {
      0.9595,  0.7595,  -0.7595, -0.9595, -0.9595, -1.1095, -1.1095, -0.9595,
      -0.9595, -0.7595, 0.7595,  0.9595,  0.9595,  1.1095,  1.1095,  0.9595};

  // slot params
  double normal_slot_length = 5.2;
  double normal_slot_width = 2.2;
  double slot_compare_to_car_length = 0.3;
  double slot_compare_to_car_width = 0.2;
  double car2line_dist_threshold = 0.02;

  // terminal pose params
  double terminal_target_x = 1.35;
  double terminal_target_y = 0.0;
  double terminal_target_heading = 0.0;
  double terminal_target_x_to_limiter = 0.15;
  double terminal_parallel_y_offset = 0.0;
  double terminal_parallel_y_offset_with_curb = 0.45;

  // check finish params
  double finish_lat_err = 0.08;
  double finish_lat_err_strict = 0.036;
  double finish_lon_err = 0.2;
  double finish_heading_err = 0.88;
  double finish_uss_slot_occupied_ratio = 0.668;
  double finish_heading_err_loose = 2.868;
  double finish_parallel_lat_err = 0.1;
  double finish_parallel_lon_err = 0.3;
  double finish_parallel_heading_err = 2.3;
  double finish_parallel_rear_stop_buffer = 0.55;
  double finish_parallel_lat_rac_err = 0.35;

  // check fail params
  double stuck_failed_time = 9.0;
  double pause_failed_time = 12.68;

  // check static params
  double car_static_pos_err = 0.005;
  double car_static_velocity = 0.01;
  double car_static_keep_time_by_pos = 1.5;
  double car_static_keep_time_by_vel = 1.5;

  // uss params
  double detection_distance = 2.5;
  double lat_inflation = 0.1;
  double safe_uss_remain_dist_in_slot = 0.35;
  double safe_uss_remain_dist_out_slot = 0.55;
  double uss_stuck_replan_wait_time = 2.0;
  double uss_scan_angle_deg = 60;
  double uss_apa_scan_angle_deg = 80;
  double uss_upa_scan_angle_deg = 120;
  bool enable_corner_uss_process = false;
  double uss_corner_scan_angle_gain = 1.0;
  double uss_face_scan_angle_gain = 1.0;
  std::vector<double> uss_vertex_x_vec = {
      3.187342,  3.424531,  3.593071,  3.593071,  3.424531,  3.187342,
      -0.476357, -0.798324, -0.879389, -0.879389, -0.798324, -0.476357};
  std::vector<double> uss_vertex_y_vec = {
      0.887956,  0.681712,  0.334651,  -0.334651, -0.681712, -0.887956,
      -0.887956, -0.706505, -0.334845, 0.334845,  0.706505,  0.887956};
  std::vector<double> uss_normal_angle_deg_vec = {170.0, 130.0, 92.0,  88.0,
                                                  50.0,  8.0,   352.0, 298.0,
                                                  275.0, 264.0, 242.0, 187.0};
  std::vector<int> uss_wdis_index_front = {0, 9, 6, 3, 1, 11};
  std::vector<int> uss_wdis_index_back = {0, 1, 3, 6, 9, 11};
  std::vector<int> uss_directly_behind_index = {8, 9};

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
  double safe_threshold = 0.2;
  double virtual_obs_y_pos = 2.5;
  double virtual_obs_x_pos = 2.68;
  double obs_consider_long_threshold = 6.68;
  double obs_consider_lat_threshold = 2.5;
  double max_pt_inside_drop_dx_mono = 1.68;
  double max_pt_inside_drop_dx_multi = 0.368;
  // parallel t lane params
  double parallel_vacant_pt_outside_dx = 2.0;
  double parallel_vacant_pt_outside_dy = 1.7;
  double parallel_vacant_pt_inside_dx = 3.0;
  double parallel_vacant_pt_inside_dy = 1.7;
  double parallel_occupied_pt_outside_dx = 0.0;
  double parallel_occupied_pt_outside_dy = 0.1;
  double parallel_occupied_pt_inside_dx = 0.0;
  double parallel_occupied_pt_inside_dy = 0.3;
  double curb_offset = 3.0;

  // construce obstacles params
  double channel_width = 12.28;
  double min_channel_width = 5.2;
  double channel_length = 12.28;
  double max_obs2car_dist_in_slot = 1.2;
  double max_obs2car_dist_out_slot = 1.2;
  double max_obs2car_dist_slot_occupied_ratio = 0.086;
  double obstacle_ds = 0.5;
  double col_obs_safe_dist = 0.36;
  double car_lat_inflation_for_obs = 0.0;
  bool tmp_no_consider_obs_dy = true;
  double tmp_virtual_obs_dy = 0.05;
  double tlane_safe_dx = 0.1;
  double obs_safe_dx = 0.1;
  double obs2slot_max_dist = 4.68;
  double slot_max_jump_dist = 0.068;
  double line_arc_obs_slot_occupied_ratio = 0.168;
  double line_arc_obs_channel_width = 8.886;
  double line_arc_obs_channel_length = 5.086;
  bool dynamic_col_det_enable = false;
  double car_lat_inflation_for_obs_radical = 0.1;
  double col_obs_safe_dist_radical = 0.2;
  double parallel_obs2slot_max_dist = 16.66;
  double parallel_channel_y_mag = 8.5;
  double parallel_channel_x_mag = 16.6;
  double parallel_ego_side_to_obs_in_buffer = 0.3;
  double parallel_ego_front_corner_to_obs_in_buffer = 0.2;

  // dynamic update path params
  double car_to_limiter_dis = 1.0;
  double pose_y_err = 0.15;
  double pose_heading_err = 6.6;
  double max_y_err_2 = 0.088;
  double max_heading_err_2 = 1.08;
  double max_y_err_3 = 0.06;
  double max_heading_err_3 = 0.88;
  double pose_slot_occupied_ratio = 0.368;
  double pose_slot_occupied_ratio_2 = 0.588;
  double pose_slot_occupied_ratio_3 = 0.788;
  double pose_min_remain_dis = 0.4;
  double max_slot_jump_dist = 0.088;

  // slot update params when parking
  double fix_slot_occupied_ratio = 0.938;

  // slot update params when replan
  double uss_slot_occupied_ratio = 0.25;
  double path_heading_slot = 55.5;
  double path_pos_err = 6.6;
  double last_update_slot_occupied_ratio = 0.836;

  // path planner params
  double prepare_line_min_x_offset_slot = 7.2;
  double prepare_line_dx_offset_slot = 0.1;
  double prepare_line_max_x_offset_slot = 9.7;
  double prepare_line_max_heading_offset_slot_deg = 8.8;
  double prepare_line_dheading_offset_slot_deg = 1.0;
  double prepare_line_min_heading_offset_slot_deg = 0.0;
  double prepare_directly_use_tangent_pos_err = 0.106;
  double prepare_directly_use_tangent_heading_err = 2.6;
  double prepare_adjust_drive_max_length = 2.28;
  double prepare_adjust_reverse_max_length = 3.6;
  double third_prepare_heading_threshold = 26.68;
  double min_turn_radius = 5.5;
  double max_one_step_arc_radius = 8.5;
  double max_radius_in_slot = 12.66;
  double min_radius_out_slot = 7.8;
  double radius_eps = 0.01;
  double min_line_length = 0.3;
  double max_line_length_for_third_prepare = 1.2;
  double min_one_step_path_length = 0.6;
  double min_one_step_path_length_in_slot = 1.2;
  double static_pos_eps = 0.01;
  double static_heading_eps = 0.08;
  double target_pos_err = 0.068;
  double target_heading_err = 0.88;
  double target_radius_err = 0.036;
  double path_extend_distance = 0.3;
  bool mono_plan_enable = false;
  bool conservative_mono_enable = false;
  double multi_plan_min_lat_err = 0.5;
  double multi_plan_min_heading_err = 12.0;
  double multi_plan_max_occupied_ratio = 0.8;
  double adjust_plan_max_heading1_err = 8.8;
  double adjust_plan_max_lat_err = 0.8;
  double adjust_plan_max_heading2_err = 15.0;
  double min_gear_path_length = 0.25;
  double parallel_multi_plan_radius_eps = 0.03;
  double parallel_search_out_heading = 0.0;
  // path optimizer params
  bool cilqr_path_optimization_enable = true;
  bool perpendicular_lat_opt_enable = true;
  bool parallel_lat_opt_enable = false;
  double min_opt_path_length = 0.7;
  // slot managent params
  bool release_slot_by_prepare = false;
  size_t max_slot_window_size = 3;
  size_t max_limiter_window_size = 6;
  // slot update
  double slot_update_in_or_out_occupied_ratio = 0.001;
  double slot_update_out_heading_max = 66.8;
  double slot_update_out_heading_min = 46.8;
  double slot_update_out_lat_max = 2.68;
  double slot_update_out_lat_min = 0.68;
  double slot_update_in_heading = 20;
  double slot_update_in_lat = 0.22;
  int slot_reset_threshold = 10;
  // limiter update
  double limiter_update_min_occupied_ratio = 0.4;
  double limiter_update_max_occupied_ratio = 0.6;
  double limiter_update_distance_to_car = 1.7;
  double limiter_update_occupied_ratio = 0.6;
  double limiter_move_dist = 0.45;

  double max_slots_update_angle_dis_limit_deg = 36.6;
  double max_slot_boundary_line_angle_dif_deg = 10.0;
  double outside_lon_dist_max_slot2mirror = 2.63;
  double outside_lon_dist_min_slot2mirror = 0.88;
  double inside_lon_dist_max_slot2mirror = 4.54;
  double inside_lon_dist_min_slot2mirror = 3.5;
  double min_slot_release_long_dist_slot2mirror = 3.86;

  double terminal_length = 1.2;
  double limiter_length = 0.0;

  double slot_occupied_ratio_max_lat_err = 0.9;
  double slot_occupied_ratio_max_heading_err = 75.0;

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
