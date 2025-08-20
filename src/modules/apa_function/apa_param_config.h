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

enum class ParkPathGenerationType {
  GEOMETRY_BASED = 0,
  SEARCH_BASED = 1,
  OPTIMIZATION_BASED = 2,
  PARK_PATH_GENERATION_MAX_NUM,
};

enum class ParkPathPlanType : uint8_t {
  GEOMETRY = 0,
  HYBRID_ASTAR = 1,
  HYBRID_ASTAR_THREAD = 2,
  PARK_PATH_PLAN_MAX_NUM,
};

enum class AnalyticExpansionType : uint8_t {
  REEDS_SHEEP = 0,
  LINK_POSE_LINE,
  ALL
};

const std::string GetParkPathPlanType(const ParkPathPlanType type);
void PrintParkPathPlanType(const ParkPathPlanType type,
                           const bool enable_log = true);
const std::string GetAnalyticExpansionTypeString(
    const AnalyticExpansionType type);
void PrintAnalyticExpansionType(const AnalyticExpansionType type,
                                const bool enable_log = true);

struct AstarParkingConfig {
  bool perpendicular_slot_auto_switch_to_astar;
  bool parallel_slot_auto_switch_to_astar;
  double parallel_finish_lon_err;
  double parallel_finish_center_lat_err;
  double parallel_finish_head_lat_err;
  double parallel_finish_heading_err;

  double head_in_finish_heading_err;

  bool cubic_polynomial_pose_adjustment = true;
  // move target point in slot to another point for easy tracking
  double vertical_tail_in_end_straight_dist = 1.0;
  double vertical_head_in_end_straight_dist = 0.8;
  double vertical_head_out_end_straight_dist = 1.0;
  double parallel_slot_end_straight_dist = 0.0;
  bool enable_delete_occ_in_slot;
  bool enable_delete_occ_in_ego;
  double deadend_uss_stuck_replan_wait_time;
  // 车辆到中线的距离小于阈值, 可以使用spiral/dubins库外揉库.
  // 注意：要限制库外揉库API的使用，该API只会让车辆来到中心线附近，不能保证车辆能正确进库.
  double adjust_ego_y_thresh_outside_slot;

  // If ego is in blind zone, ignore some obstacles.
  bool enable_blind_zone;

  // hybrid a star params
  float tail_in_slot_virtual_wall_x_offset = 3.0;
  float tail_in_slot_virtual_wall_y_offset = 0.5;
  float head_in_slot_virtual_wall_x_offset = 4.0;
  float head_in_slot_virtual_wall_y_offset = 1.8;

  // perception range is 6 * 6, need set the virtual wall small.
  float vertical_slot_passage_height_bound;
  float vertical_slot_passage_length_bound;

  float parallel_passage_width;
  float parallel_passage_length;

  // max replan number total
  int max_replan_number = 25;
  // max replan number inside slot
  int max_replan_number_inside_slot = 15;

  double vertical_min_path_length;
};

struct ParkingSpeedBound {
  double lower;
  double upper;
};

struct ParkingSpeedConfig {
  bool enable_apa_speed_plan;
  double default_cruise_speed;
  double min_cruise_speed;
  int apa_speed_mode;
  double fast_cruise_speed;
  double middle_cruise_speed;
  double slow_cruise_speed;

  // If obs dist is smaller than this value, add speed limit.
  ParkingSpeedBound speed_limit_by_obs_dist;
  ParkingSpeedBound speed_limit_by_kappa;
  ParkingSpeedBound speed_limit_by_kappa_switch;

  // speed limit
  double path_thresh_for_acc_bound = 0.4;
  double long_path_acc_upper = 0.61;
  double short_path_acc_upper = 0.51;
  double acc_lower = -2.0;
  double jerk_upper = 7.0;
  double jerk_lower = -7.0;

  double min_path_dist_for_speed_optimizer;
  double min_path_dist_for_veh_starting;

  double optimizer_time_limit = 0.4;
  bool use_remain_dist;
};

struct UssConfig {
  bool use_uss_pt_cloud = false;
  bool use_uss_pt_for_path = false;
  bool use_uss_pt_for_speed = false;
  bool use_uss_pt_for_slot_release = false;
  bool use_fusion = true;
  // If dist is bigger, ignore it.
  double uss_lat_attention_dist;
  double uss_lon_attention_dist;
  double uss_dist_coefficient_by_vel;
};

struct CheckFinishParams {
  float min_car2line_dist;
  float max_car2line_dist;

  float lon_err;

  float lat_err_strict;
  float heading_err_strict;

  float lat_err;
  float heading_err;

  float obs_stuck_slot_occupied_ratio;

  float max_remain_path_dist;
  float max_remain_obs_dist;
};

struct SmartFoldMirrorParams {
  bool has_smart_fold_mirror = false;
  // If set to true, the actual rearview mirror must be in folded state
  bool force_fold_mirror = false;
  bool locked_obs_slot_with_fold_mirror = false;

  float consume_time = 0.0;
  float reaction_time = 0.0;
  float lat_buffer = 0.0;
  float min_lat_buffer = 0.0;

  float max_stuck_wait_time = 3.0;

  float min_vel = 0.3f;

  float x_down_offset = 0.0;
  float x_up_offset = 0.0;
  float x_redunant = 0.0;
  float y_offset = 0.0;
  float heading_offset = 0.0;
};

struct DynamicGearSwitchConfig {
  bool enable_dynamic_gear_switch;
  double dist_thresh_for_current_path;
  double dist_thresh_for_next_path;
  double dist_thresh_for_gear_switch_point;
  double vel_thresh_for_gear_switch_point;
  double slot_occupied_ratio_for_gear_switch_point;

  double cur_path_lat_buffer;
  double cur_path_lon_buffer;

  double lat_error_for_dynamic_gear_switch;
  double theta_error_for_dynamic_gear_switch;
};

// Do not publish path to HMI per frame for stable display.
// If path is changed too much, publish it.
struct PreparePlanConfig {
  bool enable_stable_prepare_route;
  double start_point_error = 0.15;
  double terminal_point_error = 0.15;
};

struct ParkingLatLonPathBuffer {
  float lon_buffer;

  float body_lat_buffer;
  float mirror_lat_buffer;
  float extra_lat_buffer;

  float out_slot_body_lat_buffer;
  float out_slot_mirror_lat_buffer;
  float out_slot_extra_turn_lat_buffer;

  float entrance_slot_body_lat_buffer;
  float entrance_slot_mirror_lat_buffer;
  float entrance_slot_extra_turn_lat_buffer;

  float in_slot_body_lat_buffer;
  float in_slot_mirror_lat_buffer;
  float in_slot_extra_turn_lat_buffer;
};

struct ParkingLatLonSpeedBuffer {
  float lon_buffer;

  float stop_body_lat_buffer;
  float stop_mirror_lat_buffer;
  float stop_min_lon_dist;

  float special_stop_body_lat_buffer;
  float special_stop_mirror_lat_buffer;
  float special_stop_min_lon_dist;

  float low_speed_body_lat_buffer;
  float low_speed_mirror_lat_buffer;
  float low_speed_min_lon_dist;

  float mid_speed_body_lat_buffer;
  float mid_speed_mirror_lat_buffer;
  float mid_speed_min_lon_dist;

  float high_speed_body_lat_buffer;
  float high_speed_mirror_lat_buffer;
  float high_speed_min_lon_dist;

  float extreme_case_lon_buffer;

  float extra_reverse_gear_lon_buffer;
};

struct ParkingLatLonSlotReleaseBuffer {
  float lon_buffer;

  float maximum_lat_buffer;
  float accumulate_lat_buffer;
  float hold_lat_buffer;
  float subtract_lat_buffer;
};

struct ParkingLatLonTargetPoseBuffer {
  float lon_buffer;

  float max_lat_buffer;
  float min_lat_buffer;
  float step;

  float special_max_lat_buffer;
};

// todo
// 1. system should use same vehicle configuration file for on lane driving and
// parking.
// 2. for the atom development principle, we should decouple vehicle parameters
// and algorithm parameters into different configurature file or struct.
struct ApaParameters {
  // length unit: m   deg unit: deg  time unit: s

  /***headin***/
  // parking heading in params
  double headin_fix_slot_occupied_ratio = 0.938;
  double headin_multi_plan_min_heading_err = 22.86;    // 28.86
  double headin_multi_plan_min_lat_err = 0.8;          // 1.2
  double headin_multi_plan_max_occupied_ratio = 0.36;  // 0.3
  double headin_linearc_plan_min_lat_err = 0.8;
  double headin_linearc_plan_min_heading_err = 8.68;
  double headin_linearc_plan_max_occupied_ratio = 0.8;
  double headin_adjust_plan_max_heading1_err = 8.68;   // big
  double headin_adjust_plan_max_heading2_err = 20.68;  // big
  double headin_adjust_plan_max_lon_err = 0.68;        // big
  double headin_extend_line_min_heading_err = 6.6;
  double headin_extend_length = 0.0;
  double headin_reverse_deg = 12.68;
  double headin_prepare_line_min_x_offset_slot = 2.0;
  double headin_prepare_line_max_x_offset_slot = 5.2;
  double headin_prepare_line_max_heading_offset_slot_deg = 26.8;
  double headin_prepare_line_min_heading_offset_slot_deg = 8.8;
  double headin_max_pt_inside_drop_dy = 0.3;
  double max_obs_invasion_slot_dist = 0.026;
  double headin_virtual_obs_y_pos = 1.886;
  double headin_virtual_obs_x_pos = 2.868;
  double headin_obs_consider_lat_threshold = 2.0;
  double headin_tlane_obs_omit_x = 0.3;
  double headin_target_pos_err = 0.0568;
  double headin_target_heading_err = 0.88;
  double headin_max_radius_in_slot = 12.66;
  double headin_min_radius_out_slot = 11.8;
  double headin_sturn_steer_ratio_dist = 0.5;
  // double headin_target_pos_err = 0.088;
  // for simulation
  double max_pt_inside_drop_dx_mono_headin = 0.0;
  double max_pt_inside_drop_dx_multi_headin = 0.0;
  double radius_add = 0.0;
  /******/

  // schedule params
  double plan_time = 0.1;

  // car params
  double front_overhanging = 0.924;
  // back edge to rear axis
  double rear_overhanging = 0.94;
  double wheel_base = 2.7;
  double car_width = 1.89;
  double car_length = 4.88;
  double max_car_width = 0.2;
  double fold_mirror_max_car_width = 0.2;
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

  std::vector<double> fold_mirror_car_vertex_x_vec = {
      3.518,  3.718,  3.718,  3.518,  2.092, 2.092, 1.906, 1.906,
      -0.885, -1.085, -1.085, -0.885, 1.906, 1.906, 2.092, 2.092};
  std::vector<double> fold_mirror_car_vertex_y_vec = {
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
  double terminal_parallel_y_offset_with_curb = 0.2;
  double terminal_parallel_y_offset_with_wall = 0.45;
  double parallel_max_ego_x_offset_with_invasion = 0.5;
  double parallel_ego_ac_x_offset_with_limiter = 0.3;
  double parallel_terminal_x_offset_with_obs = 0.35;

  // check finish params
  double finish_lat_err = 0.08;
  double finish_lat_err_strict = 0.036;
  double finish_lon_err = 0.2;
  double finish_heading_err = 0.88;
  double should_stop_lat_err = 0.10;
  double should_stop_heading_err = 1.0;
  double finish_uss_slot_occupied_ratio = 0.668;
  double finish_heading_err_loose = 2.868;
  double finish_parallel_lat_err = 0.1;
  double finish_parallel_lon_err = 0.3;
  double finish_parallel_lon_overhaing_error = 0.2;
  double finish_parallel_heading_err = 2.3;
  double finish_parallel_out_heading_mag = 0.55;
  double finish_parallel_lat_rac_err = 0.35;
  CheckFinishParams check_finish_params;

  // check fail params
  double stuck_failed_time = 9.0;
  double pause_failed_time = 12.68;
  double max_replan_failed_time = 3.0;

  // check static params
  double car_static_pos_err_strict = 0.005;
  double car_static_keep_time_by_pos_strict = 1.5;
  double car_static_pos_err_normal = 0.005;
  double car_static_keep_time_by_pos_normal = 1.5;
  double car_static_velocity_strict = 0.01;
  double car_static_keep_time_by_vel_strict = 1.5;
  double car_static_velocity_normal = 0.01;
  double car_static_keep_time_by_vel_normal = 1.5;

  // uss params
  bool is_uss_dist_from_perception = false;
  double min_uss_origin_dist = 0.3;
  double detection_distance = 2.5;
  double stop_lat_inflation = 0.06;
  double heavy_brake_lat_inflation = 0.1;
  double moderate_brake_lat_inflation = 0.14;
  double slight_brake_lat_inflation = 0.18;
  double stop_lon_dist = -0.1;
  double heavy_brake_lon_dist = 0.41;
  double moderate_brake_lon_dist = 0.66;
  double slight_brake_lon_dist = 0.91;
  double safe_uss_remain_dist_in_slot = 0.35;
  double limited_safe_uss_remain_dist = 0.2;
  double safe_lat_buffer_outside_parallel_slot = 0.2;
  double safe_lat_buffer_in_parallel_slot = 0.1;
  double safe_lat_buffer_with_wall_in_parallel_slot = 0.1;
  double safe_lat_buffer_in_1r_parallel_slot = 0.1;
  double safe_remain_dist_in_1r_parallel_slot = 0.3;
  double safe_uss_remain_dist_in_parallel_slot = 0.25;
  double safe_uss_remain_dist_out_slot = 0.55;
  double uss_stuck_replan_wait_time = 2.0;
  double uss_scan_angle_deg = 60;
  double uss_apa_scan_angle_deg = 80;
  double uss_upa_scan_angle_deg = 120;
  double corner_uss_scan_angle_deg_straight = 6.8;
  double corner_uss_scan_angle_deg_turn = 6.8;
  double corner_uss_dist_diff = 0.1068;
  double corner_uss_steer_angle = 240.0;
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
  std::vector<int> uss_directly_front_index = {2, 3};

  // check replan params
  double stuck_replan_time = 4.0;
  double max_replan_remain_dist = 0.2;
  int max_replan_count = 12;
  int headin_max_replan_count = 20;

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
  bool enable_use_dynamic_obs = true;
  double safe_threshold = 0.2;
  double virtual_head_out_obs_y_pos = 0.68;
  double virtual_head_out_obs_x_pos = 2.68;
  double virtual_obs_left_y_pos = 2.5;
  double virtual_obs_left_x_pos = 2.68;
  double virtual_obs_right_y_pos = 6.68;
  double virtual_obs_right_x_pos = 2.5;
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
  double curb_offset_when_ego_outside_slot = 0.0;

  // construce obstacles params
  double channel_width = 12.28;
  double min_channel_width = 5.2;
  double channel_length = 12.28;
  double max_obs2car_dist_in_slot = 1.2;
  double max_obs2car_dist_out_slot = 1.2;
  double max_obs2car_dist_slot_occupied_ratio = 0.086;
  double obstacle_ds = 0.5;
  double col_obs_safe_dist_normal = 0.36;
  double car_lat_inflation_dynamic_plan = 0.05;
  double car_lat_inflation_normal = 0.0986;
  bool believe_in_fus_obs = false;
  bool use_fus_occ_obj = true;
  bool use_fus_occ_column = true;
  bool use_ground_line = true;
  bool use_ground_line_wall_column = true;
  bool use_object_detect = true;
  bool enable_side_pass_limiter = true;
  double tlane_safe_dx = 0.1;
  double min_dynamic_plan_proj_dt = 0.2;
  double max_dynamic_plan_proj_dt = 0.8;
  double max_lat_err = 0.068;
  double max_phi_err = 2.68;
  bool enable_multi_height_col_det = false;
  double car_lat_inflation_strict = 0.1;
  double max_obs_lat_invasion_slot_dist = -0.026;
  double max_obs_lat_invasion_slot_dist_dynamic_col = -0.026;
  double max_obs_lon_invasion_slot_dist = -0.026;
  double max_obs_lon_invasion_slot_dist_dynamic_col = -0.068;
  double slot_entrance_obs_x = 2.468;
  double col_obs_safe_dist_strict = 0.2;
  double parallel_obs2slot_max_dist = 16.66;
  double parallel_channel_y_mag = 8.5;
  double parallel_channel_x_mag = 16.6;
  double parallel_ego_side_to_obs_in_buffer = 0.3;
  double parallel_ego_front_corner_to_obs_in_buffer = 0.2;

  // dynamic update path params
  bool enable_path_dangerous_replan = false;
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
  double max_slot_jump_heading = 0.68;
  double dynamic_plan_interval_time = 0.4;
  double parallel_dynamic_lat_buffer = 0.4;
  double parallel_dynamic_lon_buffer = 1.68;
  double parallel_dynamic_lat_buffer_in_slot = 0.2;
  double parallel_dynamic_lon_buffer_in_slot = 0.8;

  // slot update params when parking
  double fix_slot_occupied_ratio = 0.938;

  // slot update params when replan
  double uss_slot_occupied_ratio = 0.25;
  double max_front_exceed_line_dx = 1.68;
  double path_pos_err = 6.6;
  double last_update_slot_occupied_ratio = 0.836;

  // path planner params
  bool use_average_obs_dist = false;
  double prepare_max_reverse_heading_err = 1.68;
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
  double prepare_single_max_allow_time = 6.68;
  int prepare_max_try_count = 2;
  double third_prepare_heading_threshold = 26.68;
  double min_turn_radius = 5.5;
  double safe_circle_radius = 5.8;
  double max_one_step_arc_radius = 8.5;
  double max_radius_in_slot = 12.66;
  double min_radius_out_slot = 7.8;
  double radius_eps = 0.01;
  double min_line_length = 0.3;
  double min_path_length = 0.35;
  double insert_line_after_arc = 1.2;
  double min_one_step_path_length = 0.6;
  double min_one_step_path_length_in_slot = 1.2;
  double static_pos_eps = 0.01;
  double static_heading_eps = 0.08;
  double target_pos_err = 0.068;
  double target_heading_err = 0.88;
  double target_radius_err = 0.036;
  double perpendicular_park_out_max_target_heading = 100;
  double perpendicular_park_out_min_target_heading = 70;
  double path_extend_distance = 0.3;
  bool actual_mono_plan_enable = false;
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
  double x_max_internal_obstacles = 6.68;
  double min_x_value_park_out_position = 2.3;
  bool is_parallel_advanced_method = true;
  ParkPathGenerationType path_generator_type =
      ParkPathGenerationType::GEOMETRY_BASED;
  bool use_geometry_path_head_out = false;

  // path optimizer params
  bool cilqr_path_optimization_enable = true;
  bool perpendicular_lat_opt_enable = true;
  bool parallel_lat_opt_enable = false;
  double min_opt_path_length = 0.7;
  // slot managent params
  bool move_slot_with_little_buffer = false;
  size_t max_slot_window_size = 3;
  size_t max_limiter_window_size = 3;
  double slot_release_channel_width = 4.86;
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
  double min_parallel_vis_slot_release_long_dist_slot2mirror = 1.87;
  double min_parallel_uss_slot_release_long_dist_slot2mirror = 3.86;
  double one_side_empty_slot_release_channel_width = 7.68;
  double two_side_empty_slot_release_channel_width = 7.68;

  double believe_obs_ego_area = 2.68;
  double limiter_length = 0.0;

  double slot_occupied_ratio_max_lat_err = 0.9;
  double slot_occupied_ratio_max_heading_err = 75.0;

  // gen output params
  double max_velocity = 0.6;

  std::vector<float> footprint_circle_x = {1.35, 3.3, 3.3, 2.02, -0.55, -0.55,
                                           2.02, 2.7, 1.8, 0.9,  0.0};
  std::vector<float> footprint_circle_y = {0.0,  0.55, -0.55, -0.88, -0.5, 0.5,
                                           0.88, 0.0,  0.0,   0.0,   0.0};
  std::vector<float> footprint_circle_r = {2.4,  0.35, 0.35, 0.18, 0.35, 0.35,
                                           0.18, 0.95, 0.95, 0.95, 0.95};

  std::vector<float> fold_mirror_footprint_circle_x = {
      1.35, 3.3, 3.3, 2.02, -0.55, -0.55, 2.02, 2.7, 1.8, 0.9, 0.0};
  std::vector<float> fold_mirror_footprint_circle_y = {
      0.0, 0.55, -0.55, -0.88, -0.5, 0.5, 0.88, 0.0, 0.0, 0.0, 0.0};
  std::vector<float> fold_mirror_footprint_circle_r = {
      2.4, 0.35, 0.35, 0.18, 0.35, 0.35, 0.18, 0.95, 0.95, 0.95, 0.95};

  AstarParkingConfig astar_config;
  ParkingSpeedConfig speed_config;
  UssConfig uss_config;

  SmartFoldMirrorParams smart_fold_mirror_params;

  DynamicGearSwitchConfig gear_switch_config;
  PreparePlanConfig prepare_plan_config;

  ParkPathPlanType park_path_plan_type = ParkPathPlanType::GEOMETRY;
  AnalyticExpansionType analytic_expansion_type =
      AnalyticExpansionType::LINK_POSE_LINE;
  bool enable_narrow_space_vertical_tail_in = true;
  int max_plan_gear_shift_number_searching = 20;
  int max_plan_gear_shift_number_parking = 25;
  int max_real_gear_shift_number_parking = 30;
  float traj_kappa_change_penalty = 0.83;

  ParkingLatLonPathBuffer lat_lon_path_buffer;
  ParkingLatLonSpeedBuffer lat_lon_speed_buffer;
  ParkingLatLonSlotReleaseBuffer lat_lon_slot_release_buffer;
  ParkingLatLonTargetPoseBuffer lat_lon_target_pose_buffer;
};

class ApaParametersSetting {
 public:
  static ApaParametersSetting &GetInstance() {
    static ApaParametersSetting instance;
    return instance;
  }

  // planning parameters
  const ApaParameters &GetParam() const { return param_; }
  ApaParameters &SetParam() { return param_; }

 private:
  ApaParametersSetting(){};

  // apa parameters
  ApaParameters param_;
};

void SyncParkingParameters(const bool is_simulation = false);

}  // namespace apa_planner
}  // namespace planning

#endif
