#include "apa_param_config.h"

#include "common/config_context.h"
#include "log_glog.h"
#include "src/common/debug_info_log.h"

namespace planning {
namespace apa_planner {

void SyncParkingParameters(const bool is_simulation) {
  std::string path = "/asw/planning/res/conf/apa_params.json";
  if (!is_simulation) {
    auto engine_config =
        common::ConfigurationContext::Instance()->engine_config();

    if (engine_config.vehicle_cfg_dir.empty()) {
      std::string engine_config_path = PLANNING_ENGINE_CONFIG_PATH;
      common::ConfigurationContext::Instance()->load_engine_config_from_json(
          engine_config_path);

      ILOG_INFO << "load vehicle config: " << engine_config_path;
      engine_config = common::ConfigurationContext::Instance()->engine_config();
      ILOG_INFO << "vehicle config path: " << engine_config.vehicle_cfg_dir;
    }

    path = engine_config.vehicle_cfg_dir + "/apa_params.json";
  }

  std::string config_file = planning::common::util::ReadFile(path);
  auto config = mjson::Reader(config_file);
  // schedule params
  JSON_READ_VALUE(apa_param.SetPram().plan_time, double, "plan_time");

  // head in param

  JSON_READ_VALUE(apa_param.SetPram().headin_fix_slot_occupied_ratio, double,
                  "headin_fix_slot_occupied_ratio");

  JSON_READ_VALUE(apa_param.SetPram().headin_multi_plan_min_heading_err, double,
                  "headin_multi_plan_min_heading_err");

  JSON_READ_VALUE(apa_param.SetPram().headin_multi_plan_min_lat_err, double,
                  "headin_multi_plan_min_lat_err");

  JSON_READ_VALUE(apa_param.SetPram().headin_multi_plan_max_occupied_ratio,
                  double, "headin_multi_plan_max_occupied_ratio");

  JSON_READ_VALUE(apa_param.SetPram().headin_linearc_plan_min_lat_err, double,
                  "headin_linearc_plan_min_lat_err");

  JSON_READ_VALUE(apa_param.SetPram().headin_linearc_plan_min_heading_err,
                  double, "headin_linearc_plan_min_heading_err");

  JSON_READ_VALUE(apa_param.SetPram().headin_linearc_plan_max_occupied_ratio,
                  double, "headin_linearc_plan_max_occupied_ratio");

  JSON_READ_VALUE(apa_param.SetPram().headin_adjust_plan_max_heading1_err,
                  double, "headin_adjust_plan_max_heading1_err");

  JSON_READ_VALUE(apa_param.SetPram().headin_adjust_plan_max_heading2_err,
                  double, "headin_adjust_plan_max_heading2_err");

  JSON_READ_VALUE(apa_param.SetPram().headin_adjust_plan_max_lon_err, double,
                  "headin_adjust_plan_max_lon_err");

  JSON_READ_VALUE(apa_param.SetPram().headin_extend_line_min_heading_err,
                  double, "headin_extend_line_min_heading_err");

  JSON_READ_VALUE(apa_param.SetPram().headin_extend_length, double,
                  "headin_extend_length");

  JSON_READ_VALUE(apa_param.SetPram().headin_reverse_deg, double,
                  "headin_reverse_deg");

  JSON_READ_VALUE(apa_param.SetPram().headin_prepare_line_min_x_offset_slot,
                  double, "headin_prepare_line_min_x_offset_slot");

  JSON_READ_VALUE(apa_param.SetPram().headin_prepare_line_max_x_offset_slot,
                  double, "headin_prepare_line_max_x_offset_slot");

  JSON_READ_VALUE(
      apa_param.SetPram().headin_prepare_line_max_heading_offset_slot_deg,
      double, "headin_prepare_line_max_heading_offset_slot_deg");

  JSON_READ_VALUE(
      apa_param.SetPram().headin_prepare_line_min_heading_offset_slot_deg,
      double, "headin_prepare_line_min_heading_offset_slot_deg");

  JSON_READ_VALUE(apa_param.SetPram().headin_max_pt_inside_drop_dy, double,
                  "headin_max_pt_inside_drop_dy");

  JSON_READ_VALUE(apa_param.SetPram().max_obs_invasion_slot_dist, double,
                  "max_obs_invasion_slot_dist");

  JSON_READ_VALUE(apa_param.SetPram().headin_virtual_obs_y_pos, double,
                  "headin_virtual_obs_y_pos");

  JSON_READ_VALUE(apa_param.SetPram().headin_virtual_obs_x_pos, double,
                  "headin_virtual_obs_x_pos");

  JSON_READ_VALUE(apa_param.SetPram().headin_obs_consider_lat_threshold, double,
                  "headin_obs_consider_lat_threshold");

  JSON_READ_VALUE(apa_param.SetPram().headin_tlane_obs_omit_x, double,
                  "headin_tlane_obs_omit_x");

  JSON_READ_VALUE(apa_param.SetPram().headin_target_pos_err, double,
                  "headin_target_pos_err");

  JSON_READ_VALUE(apa_param.SetPram().headin_target_heading_err, double,
                  "headin_target_heading_err");

  JSON_READ_VALUE(apa_param.SetPram().headin_max_radius_in_slot, double,
                  "headin_max_radius_in_slot");

  JSON_READ_VALUE(apa_param.SetPram().headin_min_radius_out_slot, double,
                  "headin_min_radius_out_slot");

  JSON_READ_VALUE(apa_param.SetPram().headin_sturn_steer_ratio_dist, double,
                  "headin_sturn_steer_ratio_dist");

  JSON_READ_VALUE(apa_param.SetPram().headin_max_replan_count, int,
                  "headin_max_replan_count");

  JSON_READ_VALUE(
      apa_param.SetPram().astar_config.max_replan_number_inside_slot, int,
      "max_replan_number_inside_slot");

  // car params
  JSON_READ_VALUE(apa_param.SetPram().has_intelligent_fold_mirror, bool,
                  "has_intelligent_fold_mirror");

  JSON_READ_VALUE(apa_param.SetPram().force_fold_mirror, bool,
                  "force_fold_mirror");

  if (apa_param.GetParam().force_fold_mirror) {
    apa_param.SetPram().has_intelligent_fold_mirror = false;
  }

  JSON_READ_VALUE(apa_param.SetPram().front_overhanging, double,
                  "front_overhanging");

  JSON_READ_VALUE(apa_param.SetPram().rear_overhanging, double,
                  "rear_overhanging");

  JSON_READ_VALUE(apa_param.SetPram().wheel_base, double, "wheel_base");
  JSON_READ_VALUE(apa_param.SetPram().car_width, double, "car_width");
  JSON_READ_VALUE(apa_param.SetPram().car_length, double, "car_length");
  JSON_READ_VALUE(apa_param.SetPram().max_car_width, double, "max_car_width");
  JSON_READ_VALUE(apa_param.SetPram().fold_mirror_max_car_width, double,
                  "fold_mirror_max_car_width");
  JSON_READ_VALUE(apa_param.SetPram().lon_dist_mirror_to_rear_axle, double,
                  "lon_dist_mirror_to_rear_axle");

  JSON_READ_VALUE(apa_param.SetPram().lat_dist_mirror_to_center, double,
                  "lat_dist_mirror_to_center");
  JSON_READ_VALUE(apa_param.SetPram().steer_ratio, double, "steer_ratio");
  JSON_READ_VALUE(apa_param.SetPram().arc_line_shift_steer_angle_deg, double,
                  "arc_line_shift_steer_angle_deg");
  JSON_READ_VALUE(apa_param.SetPram().c1, double, "c1");

  JSON_READ_VALUE(apa_param.SetPram().car_vertex_x_vec, std::vector<double>,
                  "car_vertex_x_vec");
  JSON_READ_VALUE(apa_param.SetPram().car_vertex_y_vec, std::vector<double>,
                  "car_vertex_y_vec");

  JSON_READ_VALUE(apa_param.SetPram().fold_mirror_car_vertex_x_vec,
                  std::vector<double>, "fold_mirror_car_vertex_x_vec");
  JSON_READ_VALUE(apa_param.SetPram().fold_mirror_car_vertex_y_vec,
                  std::vector<double>, "fold_mirror_car_vertex_y_vec");

  // slot params
  JSON_READ_VALUE(apa_param.SetPram().normal_slot_length, double,
                  "normal_slot_length");

  JSON_READ_VALUE(apa_param.SetPram().normal_slot_width, double,
                  "normal_slot_width");

  JSON_READ_VALUE(apa_param.SetPram().slot_compare_to_car_length, double,
                  "slot_compare_to_car_length");

  JSON_READ_VALUE(apa_param.SetPram().slot_compare_to_car_width, double,
                  "slot_compare_to_car_width");

  JSON_READ_VALUE(apa_param.SetPram().car2line_dist_threshold, double,
                  "car2line_dist_threshold");

  // terminal pose params
  JSON_READ_VALUE(apa_param.SetPram().terminal_target_x, double,
                  "terminal_target_x");

  JSON_READ_VALUE(apa_param.SetPram().terminal_target_y, double,
                  "terminal_target_y");

  JSON_READ_VALUE(apa_param.SetPram().terminal_target_heading, double,
                  "terminal_target_heading");

  JSON_READ_VALUE(apa_param.SetPram().terminal_target_x_to_limiter, double,
                  "terminal_target_x_to_limiter");

  JSON_READ_VALUE(apa_param.SetPram().terminal_parallel_y_offset, double,
                  "terminal_parallel_y_offset");

  JSON_READ_VALUE(apa_param.SetPram().terminal_parallel_y_offset_with_curb,
                  double, "terminal_parallel_y_offset_with_curb");

  JSON_READ_VALUE(apa_param.SetPram().terminal_parallel_y_offset_with_wall,
                  double, "terminal_parallel_y_offset_with_wall");

  JSON_READ_VALUE(apa_param.SetPram().parallel_max_ego_x_offset_with_invasion,
                  double, "parallel_max_ego_x_offset_with_invasion");

  JSON_READ_VALUE(apa_param.SetPram().parallel_ego_ac_x_offset_with_limiter,
                  double, "parallel_ego_ac_x_offset_with_limiter");

  JSON_READ_VALUE(apa_param.SetPram().parallel_terminal_x_offset_with_obs,
                  double, "parallel_terminal_x_offset_with_obs");

  // check finish params
  JSON_READ_VALUE(apa_param.SetPram().finish_lat_err, double, "finish_lat_err");

  JSON_READ_VALUE(apa_param.SetPram().finish_lat_err_strict, double,
                  "finish_lat_err_strict");

  JSON_READ_VALUE(apa_param.SetPram().finish_lon_err, double, "finish_lon_err");

  JSON_READ_VALUE(apa_param.SetPram().finish_heading_err, double,
                  "finish_heading_err");

  JSON_READ_VALUE(apa_param.SetPram().should_stop_lat_err, double,
                  "should_stop_lat_err");

  JSON_READ_VALUE(apa_param.SetPram().should_stop_heading_err, double,
                  "should_stop_heading_err");

  JSON_READ_VALUE(apa_param.SetPram().finish_uss_slot_occupied_ratio, double,
                  "finish_uss_slot_occupied_ratio");
  JSON_READ_VALUE(apa_param.SetPram().finish_heading_err_loose, double,
                  "finish_heading_err_loose");

  JSON_READ_VALUE(apa_param.SetPram().finish_parallel_lat_err, double,
                  "finish_parallel_lat_err");

  JSON_READ_VALUE(apa_param.SetPram().finish_parallel_lat_rac_err, double,
                  "finish_parallel_lat_rac_err");

  JSON_READ_VALUE(apa_param.SetPram().finish_parallel_lon_err, double,
                  "finish_parallel_lon_err");

  JSON_READ_VALUE(apa_param.SetPram().finish_parallel_lon_overhaing_error,
                  double, "finish_parallel_lon_overhaing_error");

  JSON_READ_VALUE(apa_param.SetPram().finish_parallel_heading_err, double,
                  "finish_parallel_heading_err");

  JSON_READ_VALUE(apa_param.SetPram().finish_parallel_out_heading_mag, double,
                  "finish_parallel_out_heading_mag");

  // check fail params
  JSON_READ_VALUE(apa_param.SetPram().stuck_failed_time, double,
                  "stuck_failed_time");

  JSON_READ_VALUE(apa_param.SetPram().pause_failed_time, double,
                  "pause_failed_time");
  JSON_READ_VALUE(apa_param.SetPram().max_replan_failed_time, double,
                  "max_replan_failed_time");

  // check static params
  JSON_READ_VALUE(apa_param.SetPram().car_static_pos_err_strict, double,
                  "car_static_pos_err_strict");

  JSON_READ_VALUE(apa_param.SetPram().car_static_keep_time_by_pos_strict,
                  double, "car_static_keep_time_by_pos_strict");

  JSON_READ_VALUE(apa_param.SetPram().car_static_pos_err_normal, double,
                  "car_static_pos_err_normal");

  JSON_READ_VALUE(apa_param.SetPram().car_static_keep_time_by_pos_normal,
                  double, "car_static_keep_time_by_pos_normal");

  JSON_READ_VALUE(apa_param.SetPram().car_static_velocity_strict, double,
                  "car_static_velocity_strict");

  JSON_READ_VALUE(apa_param.SetPram().car_static_keep_time_by_vel_strict,
                  double, "car_static_keep_time_by_vel_strict");

  JSON_READ_VALUE(apa_param.SetPram().car_static_velocity_normal, double,
                  "car_static_velocity_normal");

  JSON_READ_VALUE(apa_param.SetPram().car_static_keep_time_by_vel_normal,
                  double, "car_static_keep_time_by_vel_normal");

  // uss params
  JSON_READ_VALUE(apa_param.SetPram().is_uss_dist_from_perception, bool,
                  "is_uss_dist_from_perception");

  JSON_READ_VALUE(apa_param.SetPram().min_uss_origin_dist, double,
                  "min_uss_origin_dist");

  JSON_READ_VALUE(apa_param.SetPram().detection_distance, double,
                  "detection_distance");

  JSON_READ_VALUE(apa_param.SetPram().stop_lat_inflation, double,
                  "stop_lat_inflation");

  JSON_READ_VALUE(apa_param.SetPram().heavy_brake_lat_inflation, double,
                  "heavy_brake_lat_inflation");

  JSON_READ_VALUE(apa_param.SetPram().moderate_brake_lat_inflation, double,
                  "moderate_brake_lat_inflation");

  JSON_READ_VALUE(apa_param.SetPram().slight_brake_lat_inflation, double,
                  "slight_brake_lat_inflation");

  JSON_READ_VALUE(apa_param.SetPram().stop_lon_dist, double, "stop_lon_dist");

  JSON_READ_VALUE(apa_param.SetPram().heavy_brake_lon_dist, double,
                  "heavy_brake_lon_dist");

  JSON_READ_VALUE(apa_param.SetPram().moderate_brake_lon_dist, double,
                  "moderate_brake_lon_dist");

  JSON_READ_VALUE(apa_param.SetPram().slight_brake_lon_dist, double,
                  "slight_brake_lon_dist");

  JSON_READ_VALUE(apa_param.SetPram().safe_uss_remain_dist_in_slot, double,
                  "safe_uss_remain_dist_in_slot");

  JSON_READ_VALUE(apa_param.SetPram().limited_safe_uss_remain_dist, double,
                  "limited_safe_uss_remain_dist");

  JSON_READ_VALUE(apa_param.SetPram().safe_uss_remain_dist_in_parallel_slot,
                  double, "safe_uss_remain_dist_in_parallel_slot");

  JSON_READ_VALUE(apa_param.SetPram().safe_uss_remain_dist_out_slot, double,
                  "safe_uss_remain_dist_out_slot");

  JSON_READ_VALUE(apa_param.SetPram().uss_stuck_replan_wait_time, double,
                  "uss_stuck_replan_wait_time");

  JSON_READ_VALUE(apa_param.SetPram().uss_scan_angle_deg, double,
                  "uss_scan_angle_deg");

  JSON_READ_VALUE(apa_param.SetPram().uss_apa_scan_angle_deg, double,
                  "uss_apa_scan_angle_deg");

  JSON_READ_VALUE(apa_param.SetPram().uss_upa_scan_angle_deg, double,
                  "uss_upa_scan_angle_deg");

  JSON_READ_VALUE(apa_param.SetPram().corner_uss_scan_angle_deg_straight,
                  double, "corner_uss_scan_angle_deg_straight");

  JSON_READ_VALUE(apa_param.SetPram().corner_uss_scan_angle_deg_turn, double,
                  "corner_uss_scan_angle_deg_turn");

  JSON_READ_VALUE(apa_param.SetPram().corner_uss_dist_diff, double,
                  "corner_uss_dist_diff");

  JSON_READ_VALUE(apa_param.SetPram().corner_uss_steer_angle, double,
                  "corner_uss_steer_angle");

  JSON_READ_VALUE(apa_param.SetPram().enable_corner_uss_process, bool,
                  "enable_corner_uss_process");

  JSON_READ_VALUE(apa_param.SetPram().uss_corner_scan_angle_gain, double,
                  "uss_corner_scan_angle_gain");

  JSON_READ_VALUE(apa_param.SetPram().uss_face_scan_angle_gain, double,
                  "uss_face_scan_angle_gain");

  JSON_READ_VALUE(apa_param.SetPram().uss_vertex_x_vec, std::vector<double>,
                  "uss_vertex_x_vec");

  JSON_READ_VALUE(apa_param.SetPram().uss_vertex_y_vec, std::vector<double>,
                  "uss_vertex_y_vec");

  JSON_READ_VALUE(apa_param.SetPram().uss_normal_angle_deg_vec,
                  std::vector<double>, "uss_normal_angle_deg_vec");

  JSON_READ_VALUE(apa_param.SetPram().uss_wdis_index_front, std::vector<int>,
                  "uss_wdis_index_front");

  JSON_READ_VALUE(apa_param.SetPram().uss_wdis_index_back, std::vector<int>,
                  "uss_wdis_index_back");

  JSON_READ_VALUE(apa_param.SetPram().uss_directly_behind_index,
                  std::vector<int>, "uss_directly_behind_index");

  // check replan params
  JSON_READ_VALUE(apa_param.SetPram().stuck_replan_time, double,
                  "stuck_replan_time");

  JSON_READ_VALUE(apa_param.SetPram().max_replan_remain_dist, double,
                  "max_replan_remain_dist");

  JSON_READ_VALUE(apa_param.SetPram().max_replan_count, int,
                  "max_replan_count");

  // construct t_lane params
  JSON_READ_VALUE(apa_param.SetPram().vacant_pt_outside_dx, double,
                  "vacant_pt_outside_dx");

  JSON_READ_VALUE(apa_param.SetPram().vacant_pt_outside_dy, double,
                  "vacant_pt_outside_dy");

  JSON_READ_VALUE(apa_param.SetPram().vacant_pt_inside_dx, double,
                  "vacant_pt_inside_dx");

  JSON_READ_VALUE(apa_param.SetPram().vacant_pt_inside_dy, double,
                  "vacant_pt_inside_dy");

  JSON_READ_VALUE(apa_param.SetPram().occupied_pt_outside_dx, double,
                  "occupied_pt_outside_dx");

  JSON_READ_VALUE(apa_param.SetPram().occupied_pt_outside_dy, double,
                  "occupied_pt_outside_dy");

  JSON_READ_VALUE(apa_param.SetPram().occupied_pt_inside_dx, double,
                  "occupied_pt_inside_dx");

  JSON_READ_VALUE(apa_param.SetPram().occupied_pt_inside_dy, double,
                  "occupied_pt_inside_dy");

  JSON_READ_VALUE(apa_param.SetPram().nearby_slot_corner_dist, double,
                  "nearby_slot_corner_dist");

  JSON_READ_VALUE(apa_param.SetPram().enable_use_dynamic_obs, bool,
                  "enable_use_dynamic_obs");

  JSON_READ_VALUE(apa_param.SetPram().safe_threshold, double, "safe_threshold");

  JSON_READ_VALUE(apa_param.SetPram().virtual_head_out_obs_y_pos, double,
                  "virtual_head_out_obs_y_pos");

  JSON_READ_VALUE(apa_param.SetPram().virtual_head_out_obs_x_pos, double,
                  "virtual_head_out_obs_x_pos");

  JSON_READ_VALUE(apa_param.SetPram().virtual_obs_left_y_pos, double,
                  "virtual_obs_left_y_pos");

  JSON_READ_VALUE(apa_param.SetPram().virtual_obs_left_x_pos, double,
                  "virtual_obs_left_x_pos");

  JSON_READ_VALUE(apa_param.SetPram().virtual_obs_right_y_pos, double,
                  "virtual_obs_right_y_pos");

  JSON_READ_VALUE(apa_param.SetPram().virtual_obs_right_x_pos, double,
                  "virtual_obs_right_x_pos");

  JSON_READ_VALUE(apa_param.SetPram().max_pt_inside_drop_dx_mono, double,
                  "max_pt_inside_drop_dx_mono");

  JSON_READ_VALUE(apa_param.SetPram().max_pt_inside_drop_dx_multi, double,
                  "max_pt_inside_drop_dx_multi");

  // construct parallel t-lane params
  JSON_READ_VALUE(apa_param.SetPram().parallel_vacant_pt_outside_dx, double,
                  "parallel_vacant_pt_outside_dx");

  JSON_READ_VALUE(apa_param.SetPram().parallel_vacant_pt_outside_dy, double,
                  "parallel_vacant_pt_outside_dy");

  JSON_READ_VALUE(apa_param.SetPram().parallel_vacant_pt_inside_dx, double,
                  "parallel_vacant_pt_inside_dx");

  JSON_READ_VALUE(apa_param.SetPram().parallel_vacant_pt_inside_dy, double,
                  "parallel_vacant_pt_inside_dy");

  JSON_READ_VALUE(apa_param.SetPram().parallel_occupied_pt_outside_dx, double,
                  "parallel_occupied_pt_outside_dx");

  JSON_READ_VALUE(apa_param.SetPram().parallel_occupied_pt_outside_dy, double,
                  "parallel_occupied_pt_outside_dy");

  JSON_READ_VALUE(apa_param.SetPram().parallel_occupied_pt_inside_dx, double,
                  "parallel_occupied_pt_inside_dx");

  JSON_READ_VALUE(apa_param.SetPram().parallel_occupied_pt_inside_dy, double,
                  "parallel_occupied_pt_inside_dy");

  JSON_READ_VALUE(apa_param.SetPram().curb_offset, double, "curb_offset");

  JSON_READ_VALUE(apa_param.SetPram().curb_offset_when_ego_outside_slot, double,
                  "curb_offset_when_ego_outside_slot");

  // construce obstacles params
  JSON_READ_VALUE(apa_param.SetPram().channel_width, double, "channel_width");

  JSON_READ_VALUE(apa_param.SetPram().min_channel_width, double,
                  "min_channel_width");

  JSON_READ_VALUE(apa_param.SetPram().channel_length, double, "channel_length");

  JSON_READ_VALUE(apa_param.SetPram().max_obs2car_dist_in_slot, double,
                  "max_obs2car_dist_in_slot");

  JSON_READ_VALUE(apa_param.SetPram().max_obs2car_dist_out_slot, double,
                  "max_obs2car_dist_out_slot");

  JSON_READ_VALUE(apa_param.SetPram().max_obs2car_dist_slot_occupied_ratio,
                  double, "max_obs2car_dist_slot_occupied_ratio");

  JSON_READ_VALUE(apa_param.SetPram().col_obs_safe_dist_normal, double,
                  "col_obs_safe_dist_normal");

  JSON_READ_VALUE(apa_param.SetPram().obstacle_ds, double, "obstacle_ds");

  JSON_READ_VALUE(apa_param.SetPram().car_lat_inflation_dynamic_plan, double,
                  "car_lat_inflation_dynamic_plan");

  JSON_READ_VALUE(apa_param.SetPram().car_lat_inflation_normal, double,
                  "car_lat_inflation_normal");

  JSON_READ_VALUE(apa_param.SetPram().believe_in_fus_obs, bool,
                  "believe_in_fus_obs");

  JSON_READ_VALUE(apa_param.SetPram().use_fus_occ_obj, bool, "use_fus_occ_obj");

  JSON_READ_VALUE(apa_param.SetPram().use_fus_occ_column, bool,
                  "use_fus_occ_column");

  JSON_READ_VALUE(apa_param.SetPram().uss_config.use_uss_pt_clound, bool,
                  "use_uss_pt_clound");
  JSON_READ_VALUE(apa_param.SetPram().uss_config.use_uss_pt_for_path, bool,
                  "use_uss_pt_for_path");
  JSON_READ_VALUE(apa_param.SetPram().uss_config.use_uss_pt_for_slot_release,
                  bool, "use_uss_pt_for_slot_release");
  JSON_READ_VALUE(apa_param.SetPram().uss_config.use_uss_pt_for_speed, bool,
                  "use_uss_pt_for_speed");
  JSON_READ_VALUE(apa_param.SetPram().uss_config.point_max_dist, double,
                  "point_max_dist");

  JSON_READ_VALUE(apa_param.SetPram().use_ground_line, bool, "use_ground_line");

  JSON_READ_VALUE(apa_param.SetPram().use_ground_line_wall_column, bool,
                  "use_ground_line_wall_column");

  JSON_READ_VALUE(apa_param.SetPram().use_object_detect, bool,
                  "use_object_detect");

  JSON_READ_VALUE(apa_param.SetPram().tlane_safe_dx, double, "tlane_safe_dx");

  JSON_READ_VALUE(apa_param.SetPram().parallel_obs2slot_max_dist, double,
                  "parallel_obs2slot_max_dist");

  JSON_READ_VALUE(apa_param.SetPram().parallel_channel_y_mag, double,
                  "parallel_channel_y_mag");

  JSON_READ_VALUE(apa_param.SetPram().parallel_channel_x_mag, double,
                  "parallel_channel_x_mag");

  JSON_READ_VALUE(apa_param.SetPram().parallel_ego_side_to_obs_in_buffer,
                  double, "parallel_ego_side_to_obs_in_buffer");

  JSON_READ_VALUE(
      apa_param.SetPram().parallel_ego_front_corner_to_obs_in_buffer, double,
      "parallel_ego_front_corner_to_obs_in_buffer");

  JSON_READ_VALUE(apa_param.SetPram().min_dynamic_plan_proj_dt, double,
                  "min_dynamic_plan_proj_dt");

  JSON_READ_VALUE(apa_param.SetPram().max_dynamic_plan_proj_dt, double,
                  "max_dynamic_plan_proj_dt");

  JSON_READ_VALUE(apa_param.SetPram().max_lat_err, double, "max_lat_err");

  JSON_READ_VALUE(apa_param.SetPram().max_phi_err, double, "max_phi_err");

  JSON_READ_VALUE(apa_param.SetPram().dynamic_col_det_enable, bool,
                  "dynamic_col_det_enable");

  JSON_READ_VALUE(apa_param.SetPram().car_lat_inflation_strict, double,
                  "car_lat_inflation_strict");

  JSON_READ_VALUE(apa_param.SetPram().col_obs_safe_dist_strict, double,
                  "col_obs_safe_dist_strict");

  JSON_READ_VALUE(apa_param.SetPram().max_obs_lat_invasion_slot_dist, double,
                  "max_obs_lat_invasion_slot_dist");

  JSON_READ_VALUE(
      apa_param.SetPram().max_obs_lat_invasion_slot_dist_dynamic_col, double,
      "max_obs_lat_invasion_slot_dist_dynamic_col");

  JSON_READ_VALUE(apa_param.SetPram().max_obs_lon_invasion_slot_dist, double,
                  "max_obs_lon_invasion_slot_dist");

  JSON_READ_VALUE(
      apa_param.SetPram().max_obs_lon_invasion_slot_dist_dynamic_col, double,
      "max_obs_lon_invasion_slot_dist_dynamic_col");

  JSON_READ_VALUE(apa_param.SetPram().slot_entrance_obs_x, double,
                  "slot_entrance_obs_x");

  // dynamic update path params
  JSON_READ_VALUE(apa_param.SetPram().car_to_limiter_dis, double,
                  "car_to_limiter_dis");

  JSON_READ_VALUE(apa_param.SetPram().pose_y_err, double, "pose_y_err");

  JSON_READ_VALUE(apa_param.SetPram().pose_heading_err, double,
                  "pose_heading_err");

  JSON_READ_VALUE(apa_param.SetPram().max_y_err_2, double, "max_y_err_2");

  JSON_READ_VALUE(apa_param.SetPram().max_heading_err_2, double,
                  "max_heading_err_2");

  JSON_READ_VALUE(apa_param.SetPram().max_y_err_3, double, "max_y_err_3");

  JSON_READ_VALUE(apa_param.SetPram().max_heading_err_3, double,
                  "max_heading_err_3");

  JSON_READ_VALUE(apa_param.SetPram().pose_slot_occupied_ratio, double,
                  "pose_slot_occupied_ratio");

  JSON_READ_VALUE(apa_param.SetPram().pose_slot_occupied_ratio_2, double,
                  "pose_slot_occupied_ratio_2");

  JSON_READ_VALUE(apa_param.SetPram().pose_slot_occupied_ratio_3, double,
                  "pose_slot_occupied_ratio_3");

  JSON_READ_VALUE(apa_param.SetPram().pose_min_remain_dis, double,
                  "pose_min_remain_dis");

  JSON_READ_VALUE(apa_param.SetPram().max_slot_jump_dist, double,
                  "max_slot_jump_dist");

  JSON_READ_VALUE(apa_param.SetPram().max_slot_jump_heading, double,
                  "max_slot_jump_heading");

  JSON_READ_VALUE(apa_param.SetPram().dynamic_plan_interval_time, double,
                  "dynamic_plan_interval_time");

  JSON_READ_VALUE(apa_param.SetPram().parallel_dynamic_lat_buffer, double,
                  "parallel_dynamic_lat_buffer");

  JSON_READ_VALUE(apa_param.SetPram().parallel_dynamic_lon_buffer, double,
                  "parallel_dynamic_lon_buffer");

  JSON_READ_VALUE(apa_param.SetPram().parallel_dynamic_lat_buffer_in_slot,
                  double, "parallel_dynamic_lat_buffer_in_slot");

  JSON_READ_VALUE(apa_param.SetPram().parallel_dynamic_lon_buffer_in_slot,
                  double, "parallel_dynamic_lon_buffer_in_slot");

  // slot update params when parking
  JSON_READ_VALUE(apa_param.SetPram().fix_slot_occupied_ratio, double,
                  "fix_slot_occupied_ratio");

  // slot update params when replan
  JSON_READ_VALUE(apa_param.SetPram().uss_slot_occupied_ratio, double,
                  "uss_slot_occupied_ratio");

  JSON_READ_VALUE(apa_param.SetPram().max_front_exceed_line_dx, double,
                  "max_front_exceed_line_dx");

  JSON_READ_VALUE(apa_param.SetPram().path_pos_err, double, "path_pos_err");

  JSON_READ_VALUE(apa_param.SetPram().last_update_slot_occupied_ratio, double,
                  "last_update_slot_occupied_ratio");

  // path planner params
  JSON_READ_VALUE(apa_param.SetPram().use_average_obs_dist, bool,
                  "use_average_obs_dist");

  JSON_READ_VALUE(apa_param.SetPram().min_turn_radius, double,
                  "min_turn_radius");

  JSON_READ_VALUE(apa_param.SetPram().safe_circle_radius, double,
                  "safe_circle_radius");

  JSON_READ_VALUE(apa_param.SetPram().max_radius_in_slot, double,
                  "max_radius_in_slot");

  JSON_READ_VALUE(apa_param.SetPram().min_radius_out_slot, double,
                  "min_radius_out_slot");

  JSON_READ_VALUE(apa_param.SetPram().max_one_step_arc_radius, double,
                  "max_one_step_arc_radius");

  JSON_READ_VALUE(apa_param.SetPram().radius_eps, double, "radius_eps");

  JSON_READ_VALUE(apa_param.SetPram().min_line_length, double,
                  "min_line_length");

  JSON_READ_VALUE(apa_param.SetPram().min_path_length, double,
                  "min_path_length");

  JSON_READ_VALUE(apa_param.SetPram().insert_line_after_arc, double,
                  "insert_line_after_arc");

  JSON_READ_VALUE(apa_param.SetPram().min_one_step_path_length, double,
                  "min_one_step_path_length");

  JSON_READ_VALUE(apa_param.SetPram().min_one_step_path_length_in_slot, double,
                  "min_one_step_path_length_in_slot");

  JSON_READ_VALUE(apa_param.SetPram().prepare_max_reverse_heading_err, double,
                  "prepare_max_reverse_heading_err");

  JSON_READ_VALUE(apa_param.SetPram().prepare_line_min_x_offset_slot, double,
                  "prepare_line_min_x_offset_slot");

  JSON_READ_VALUE(apa_param.SetPram().prepare_line_dx_offset_slot, double,
                  "prepare_line_dx_offset_slot");

  JSON_READ_VALUE(apa_param.SetPram().prepare_line_max_x_offset_slot, double,
                  "prepare_line_max_x_offset_slot");

  JSON_READ_VALUE(apa_param.SetPram().prepare_line_max_heading_offset_slot_deg,
                  double, "prepare_line_max_heading_offset_slot_deg");

  JSON_READ_VALUE(apa_param.SetPram().prepare_line_dheading_offset_slot_deg,
                  double, "prepare_line_dheading_offset_slot_deg");

  JSON_READ_VALUE(apa_param.SetPram().prepare_line_min_heading_offset_slot_deg,
                  double, "prepare_line_min_heading_offset_slot_deg");

  JSON_READ_VALUE(apa_param.SetPram().prepare_directly_use_tangent_pos_err,
                  double, "prepare_directly_use_tangent_pos_err");

  JSON_READ_VALUE(apa_param.SetPram().prepare_directly_use_tangent_heading_err,
                  double, "prepare_directly_use_tangent_heading_err");

  JSON_READ_VALUE(apa_param.SetPram().prepare_adjust_drive_max_length, double,
                  "prepare_adjust_drive_max_length");

  JSON_READ_VALUE(apa_param.SetPram().prepare_adjust_reverse_max_length, double,
                  "prepare_adjust_reverse_max_length");

  JSON_READ_VALUE(apa_param.SetPram().prepare_single_max_allow_time, double,
                  "prepare_single_max_allow_time");

  JSON_READ_VALUE(apa_param.SetPram().prepare_max_try_count, int,
                  "prepare_max_try_count");

  JSON_READ_VALUE(apa_param.SetPram().third_prepare_heading_threshold, double,
                  "third_prepare_heading_threshold");

  JSON_READ_VALUE(apa_param.SetPram().static_pos_eps, double, "static_pos_eps");

  JSON_READ_VALUE(apa_param.SetPram().static_heading_eps, double,
                  "static_heading_eps");

  JSON_READ_VALUE(apa_param.SetPram().target_pos_err, double, "target_pos_err");

  JSON_READ_VALUE(apa_param.SetPram().target_heading_err, double,
                  "target_heading_err");

  JSON_READ_VALUE(apa_param.SetPram().target_radius_err, double,
                  "target_radius_err");

  JSON_READ_VALUE(apa_param.SetPram().perpendicular_park_out_max_target_heading,
                  double, "target_heading_max");

  JSON_READ_VALUE(apa_param.SetPram().perpendicular_park_out_min_target_heading,
                  double, "target_heading_min");

  JSON_READ_VALUE(apa_param.SetPram().path_extend_distance, double,
                  "path_extend_distance");

  JSON_READ_VALUE(apa_param.SetPram().mono_plan_enable, bool,
                  "mono_plan_enable");

  JSON_READ_VALUE(apa_param.SetPram().conservative_mono_enable, bool,
                  "conservative_mono_enable");

  JSON_READ_VALUE(apa_param.SetPram().multi_plan_min_lat_err, double,
                  "multi_plan_min_lat_err");

  JSON_READ_VALUE(apa_param.SetPram().multi_plan_min_heading_err, double,
                  "multi_plan_min_heading_err");

  JSON_READ_VALUE(apa_param.SetPram().multi_plan_max_occupied_ratio, double,
                  "multi_plan_max_occupied_ratio");

  JSON_READ_VALUE(apa_param.SetPram().adjust_plan_max_heading1_err, double,
                  "adjust_plan_max_heading1_err");

  JSON_READ_VALUE(apa_param.SetPram().adjust_plan_max_lat_err, double,
                  "adjust_plan_max_lat_err");

  JSON_READ_VALUE(apa_param.SetPram().adjust_plan_max_heading2_err, double,
                  "adjust_plan_max_heading2_err");

  JSON_READ_VALUE(apa_param.SetPram().parallel_lat_opt_enable, bool,
                  "parallel_lat_opt_enable");

  JSON_READ_VALUE(apa_param.SetPram().perpendicular_lat_opt_enable, bool,
                  "perpendicular_lat_opt_enable");

  JSON_READ_VALUE(apa_param.SetPram().cilqr_path_optimization_enable, bool,
                  "is_cilqr_path_optimization_enable");

  JSON_READ_VALUE(apa_param.SetPram().min_opt_path_length, double,
                  "min_opt_path_length");

  JSON_READ_VALUE(apa_param.SetPram().min_gear_path_length, double,
                  "min_gear_path_length");

  JSON_READ_VALUE(apa_param.SetPram().parallel_multi_plan_radius_eps, double,
                  "parallel_multi_plan_radius_eps");

  JSON_READ_VALUE(apa_param.SetPram().parallel_search_out_heading, double,
                  "parallel_search_out_heading");

  JSON_READ_VALUE(apa_param.SetPram().x_max_internal_obstacles, double,
                  "x_max_internal_obstacles");

  JSON_READ_VALUE(apa_param.SetPram().min_x_value_park_out_position, double,
                  "min_x_value_park_out_position");

  JSON_READ_VALUE(apa_param.SetPram().is_parallel_advanced_method, bool,
                  "is_parallel_advanced_method");

  int path_generator_type = 1;
  JSON_READ_VALUE(path_generator_type, int, "path_generator_type");
  switch (path_generator_type) {
    case 0:
      apa_param.SetPram().path_generator_type =
          ParkPathGenerationType::GEOMETRY_BASED;
      break;
    case 1:
      apa_param.SetPram().path_generator_type =
          ParkPathGenerationType::SEARCH_BASED;
      break;
    default:
      break;
  }
  ILOG_INFO << "path_generator_type "
            << static_cast<int>(apa_param.GetParam().path_generator_type);

  JSON_READ_VALUE(apa_param.SetPram().use_geometry_path_head_out, bool,
                  "use_geometry_path_head_out");

  // slot managent params
  JSON_READ_VALUE(apa_param.SetPram().move_slot_with_little_buffer, bool,
                  "move_slot_with_little_buffer");

  JSON_READ_VALUE(apa_param.SetPram().max_slot_window_size, int,
                  "max_slot_window_size");

  JSON_READ_VALUE(apa_param.SetPram().max_limiter_window_size, int,
                  "max_limiter_window_size");

  JSON_READ_VALUE(apa_param.SetPram().slot_release_channel_width, double,
                  "slot_release_channel_width");

  // slot update
  JSON_READ_VALUE(apa_param.SetPram().slot_update_in_or_out_occupied_ratio,
                  double, "slot_update_in_or_out_occupied_ratio");

  JSON_READ_VALUE(apa_param.SetPram().slot_update_out_heading_max, double,
                  "slot_update_out_heading_max");

  JSON_READ_VALUE(apa_param.SetPram().slot_update_out_heading_min, double,
                  "slot_update_out_heading_min");

  JSON_READ_VALUE(apa_param.SetPram().slot_update_out_lat_max, double,
                  "slot_update_out_lat_max");

  JSON_READ_VALUE(apa_param.SetPram().slot_update_out_lat_min, double,
                  "slot_update_out_lat_min");

  JSON_READ_VALUE(apa_param.SetPram().slot_update_in_heading, double,
                  "slot_update_in_heading");

  JSON_READ_VALUE(apa_param.SetPram().slot_update_in_lat, double,
                  "slot_update_in_lat");

  JSON_READ_VALUE(apa_param.SetPram().slot_reset_threshold, int,
                  "slot_reset_threshold");

  JSON_READ_VALUE(apa_param.SetPram().limiter_move_dist, double,
                  "limiter_move_dist");

  JSON_READ_VALUE(apa_param.SetPram().limiter_update_min_occupied_ratio, double,
                  "limiter_update_min_occupied_ratio");

  JSON_READ_VALUE(apa_param.SetPram().limiter_update_max_occupied_ratio, double,
                  "limiter_update_max_occupied_ratio");

  JSON_READ_VALUE(apa_param.SetPram().limiter_update_distance_to_car, double,
                  "limiter_update_distance_to_car");

  JSON_READ_VALUE(apa_param.SetPram().limiter_update_occupied_ratio, double,
                  "limiter_update_occupied_ratio");

  JSON_READ_VALUE(apa_param.SetPram().max_slots_update_angle_dis_limit_deg,
                  double, "max_slots_update_angle_dis_limit_deg");

  JSON_READ_VALUE(apa_param.SetPram().max_slot_boundary_line_angle_dif_deg,
                  double, "max_slot_boundary_line_angle_dif_deg");

  JSON_READ_VALUE(apa_param.SetPram().outside_lon_dist_max_slot2mirror, double,
                  "outside_lon_dist_max_slot2mirror");

  JSON_READ_VALUE(apa_param.SetPram().outside_lon_dist_min_slot2mirror, double,
                  "outside_lon_dist_min_slot2mirror");

  JSON_READ_VALUE(apa_param.SetPram().inside_lon_dist_max_slot2mirror, double,
                  "inside_lon_dist_max_slot2mirror");

  JSON_READ_VALUE(apa_param.SetPram().inside_lon_dist_min_slot2mirror, double,
                  "inside_lon_dist_min_slot2mirror");

  JSON_READ_VALUE(apa_param.SetPram().min_slot_release_long_dist_slot2mirror,
                  double, "min_slot_release_long_dist_slot2mirror");

  JSON_READ_VALUE(
      apa_param.SetPram().min_parallel_uss_slot_release_long_dist_slot2mirror,
      double, "min_parallel_uss_slot_release_long_dist_slot2mirror");

  JSON_READ_VALUE(
      apa_param.SetPram().min_parallel_vis_slot_release_long_dist_slot2mirror,
      double, "min_parallel_vis_slot_release_long_dist_slot2mirror");

  JSON_READ_VALUE(apa_param.SetPram().one_side_empty_slot_release_channel_width,
                  double, "one_side_empty_slot_release_channel_width");

  JSON_READ_VALUE(apa_param.SetPram().two_side_empty_slot_release_channel_width,
                  double, "two_side_empty_slot_release_channel_width");

  JSON_READ_VALUE(apa_param.SetPram().believe_obs_ego_area, double,
                  "believe_obs_ego_area");

  JSON_READ_VALUE(apa_param.SetPram().limiter_length, double, "limiter_length");

  JSON_READ_VALUE(apa_param.SetPram().slot_occupied_ratio_max_lat_err, double,
                  "slot_occupied_ratio_max_lat_err");

  JSON_READ_VALUE(apa_param.SetPram().slot_occupied_ratio_max_heading_err,
                  double, "slot_occupied_ratio_max_heading_err");

  // gen output params
  JSON_READ_VALUE(apa_param.SetPram().max_velocity, double, "max_velocity");

  JSON_READ_VALUE(apa_param.SetPram().footprint_circle_x, std::vector<float>,
                  "footprint_circle_x");
  JSON_READ_VALUE(apa_param.SetPram().footprint_circle_y, std::vector<float>,
                  "footprint_circle_y");
  JSON_READ_VALUE(apa_param.SetPram().footprint_circle_r, std::vector<float>,
                  "footprint_circle_r");

  JSON_READ_VALUE(apa_param.SetPram().fold_mirror_footprint_circle_x,
                  std::vector<float>, "fold_mirror_footprint_circle_x");
  JSON_READ_VALUE(apa_param.SetPram().fold_mirror_footprint_circle_y,
                  std::vector<float>, "fold_mirror_footprint_circle_y");
  JSON_READ_VALUE(apa_param.SetPram().fold_mirror_footprint_circle_r,
                  std::vector<float>, "fold_mirror_footprint_circle_r");

  JSON_READ_VALUE(
      apa_param.SetPram().astar_config.perpendicular_slot_auto_switch_to_astar,
      bool, "perpendicular_slot_auto_switch_to_astar");
  JSON_READ_VALUE(
      apa_param.SetPram().astar_config.parallel_slot_auto_switch_to_astar, bool,
      "parallel_slot_auto_switch_to_astar");
  JSON_READ_VALUE(
      apa_param.SetPram().astar_config.parallel_slot_end_straight_dist, double,
      "parallel_slot_end_straight_dist");
  JSON_READ_VALUE(
      apa_param.SetPram().astar_config.cubic_polynomial_pose_adjustment, bool,
      "cubic_polynomial_pose_adjustment");
  JSON_READ_VALUE(apa_param.SetPram().astar_config.parallel_finish_lon_err,
                  double, "astar_parallel_finish_lon_err");
  JSON_READ_VALUE(
      apa_param.SetPram().astar_config.parallel_finish_center_lat_err, double,
      "astar_parallel_finish_center_lat_err");
  JSON_READ_VALUE(apa_param.SetPram().astar_config.parallel_finish_head_lat_err,
                  double, "astar_parallel_finish_head_lat_err");
  JSON_READ_VALUE(apa_param.SetPram().astar_config.parallel_finish_heading_err,
                  double, "astar_parallel_finish_heading_err");
  JSON_READ_VALUE(apa_param.SetPram().astar_config.enable_delete_occ_in_slot,
                  bool, "enable_delete_occ_in_slot");
  JSON_READ_VALUE(apa_param.SetPram().astar_config.enable_delete_occ_in_ego,
                  bool, "enable_delete_occ_in_ego");
  JSON_READ_VALUE(
      apa_param.SetPram().astar_config.deadend_uss_stuck_replan_wait_time,
      double, "deadend_uss_stuck_replan_wait_time");
  JSON_READ_VALUE(
      apa_param.SetPram().astar_config.vertical_tail_in_end_straight_dist,
      double, "vertical_tail_in_end_straight_dist");

  JSON_READ_VALUE(
      apa_param.SetPram().astar_config.vertical_head_in_end_straight_dist,
      double, "vertical_head_in_end_straight_dist");

  JSON_READ_VALUE(
      apa_param.SetPram().astar_config.adjust_ego_y_thresh_outside_slot, double,
      "adjust_ego_y_thresh_outside_slot");
  JSON_READ_VALUE(apa_param.SetPram().astar_config.enable_blind_zone, bool,
                  "enable_blind_zone");
  JSON_READ_VALUE(
      apa_param.SetPram().astar_config.vertical_slot_passage_height_bound,
      float, "vertical_slot_passage_height_bound");
  JSON_READ_VALUE(
      apa_param.SetPram().astar_config.vertical_slot_passage_length_bound,
      float, "vertical_slot_passage_length_bound");
  JSON_READ_VALUE(apa_param.SetPram().astar_config.parallel_passage_width,
                  float, "parallel_passage_width");
  JSON_READ_VALUE(apa_param.SetPram().astar_config.parallel_passage_length,
                  float, "parallel_passage_length");

  JSON_READ_VALUE(apa_param.SetPram().speed_config.enable_apa_speed_plan, bool,
                  "enable_apa_speed_plan");
  JSON_READ_VALUE(apa_param.SetPram().speed_config.default_cruise_speed, double,
                  "default_cruise_speed");
  JSON_READ_VALUE(apa_param.SetPram().speed_config.min_cruise_speed, double,
                  "min_cruise_speed");
  JSON_READ_VALUE(apa_param.SetPram().speed_config.min_speed_limit_by_obs_dist,
                  double, "min_speed_limit_by_obs_dist");
  JSON_READ_VALUE(apa_param.SetPram().speed_config.speed_limit_by_kappa, double,
                  "speed_limit_by_kappa");
  JSON_READ_VALUE(apa_param.SetPram().speed_config.speed_limit_by_kappa_switch,
                  double, "speed_limit_by_kappa_switch");
  JSON_READ_VALUE(apa_param.SetPram().speed_config.speed_limit_by_obs_dist,
                  double, "speed_limit_by_obs_dist");
  JSON_READ_VALUE(
      apa_param.SetPram().speed_config.min_path_dist_for_speed_optimizer,
      double, "min_path_dist_for_speed_optimizer");
  JSON_READ_VALUE(
      apa_param.SetPram().speed_config.min_path_dist_for_veh_starting, double,
      "min_path_dist_for_veh_starting");
  JSON_READ_VALUE(apa_param.SetPram().speed_config.optimizer_time_limit, double,
                  "optimizer_time_limit");
  JSON_READ_VALUE(apa_param.SetPram().speed_config.use_remain_dist, bool,
                  "use_remain_dist");

  // hybrid a star params
  JSON_READ_VALUE(
      apa_param.SetPram().astar_config.tail_in_slot_virtual_wall_x_offset,
      float, "tail_in_slot_virtual_wall_x_offset");

  JSON_READ_VALUE(
      apa_param.SetPram().astar_config.tail_in_slot_virtual_wall_y_offset,
      float, "tail_in_slot_virtual_wall_y_offset");

  JSON_READ_VALUE(
      apa_param.SetPram().astar_config.head_in_slot_virtual_wall_x_offset,
      float, "head_in_slot_virtual_wall_x_offset");

  JSON_READ_VALUE(
      apa_param.SetPram().astar_config.head_in_slot_virtual_wall_y_offset,
      float, "head_in_slot_virtual_wall_y_offset");
  JSON_READ_VALUE(apa_param.SetPram().astar_config.max_replan_number, int,
                  "max_replan_number");
  return;
}

}  // namespace apa_planner
}  // namespace planning