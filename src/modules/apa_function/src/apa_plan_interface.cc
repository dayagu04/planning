#include "apa_plan_interface.h"

#include <cstdint>
#include <memory>

#include "apa_param_setting.h"
#include "apa_plan_base.h"
#include "apa_world.h"
#include "debug_info_log.h"
#include "environmental_model.h"
#include "func_state_machine.pb.h"
#include "general_planning_context.h"
#include "local_view.h"
#include "parallel_park_in_planner.h"
#include "perpendicular_park_in_planner.h"
#include "planning_context.h"

// #define PERPENDICULAR_SIMULATION

// #define PERPENDICULAR_SIMULATION

namespace planning {
namespace apa_planner {

void ApaPlanInterface::Init() {
  // sync parameters
  SyncParameters();

  // init apa world
  apa_world_ptr_ = std::make_shared<ApaWorld>();

  // init planners
  apa_planner_stack_.clear();
  apa_planner_stack_.resize(ApaWorld::ApaPlannerType::PLANNER_COUNT);

  // perpendicular park in planner
  apa_planner_stack_[ApaWorld::ApaPlannerType::PERPENDICULAR_PARK_IN_PLANNER] =
      std::make_shared<PerpendicularInPlanner>(apa_world_ptr_);

  // parallel park in planner
  apa_planner_stack_[ApaWorld::ApaPlannerType::PARALLEL_PARK_IN_PLANNER] =
      std::make_shared<ParallelParInPlanner>(apa_world_ptr_);

  if (apa_planner_stack_.size() > 0) {
    planner_ptr_ = apa_planner_stack_.front();
  }
}

void ApaPlanInterface::Reset() {
  // reset planning output
  planning_output_.Clear();
  planning_output_.mutable_planning_status()->set_apa_planning_status(
      PlanningOutput::ApaPlanningStatus::NONE);

  planning_output_.mutable_successful_slot_info_list()->Clear();

  // reset apa world
  apa_world_ptr_->Reset();

  // reset all planner
  for (const auto &planner : apa_planner_stack_) {
    planner->Reset();
  }
}

std::shared_ptr<ApaPlannerBase> ApaPlanInterface::GetPlannerByType(
    const uint8_t apa_planner_id) {
  if (apa_planner_id <= apa_planner_stack_.size()) {
    return apa_planner_stack_[apa_planner_id];
  } else {
    return nullptr;
  }
}

const bool ApaPlanInterface::Update(const LocalView *local_view_ptr) {
  std::cout << "\n------------------------ apa_interface: Update() "
               "------------------------\n";

  const uint8_t last_state =
      apa_world_ptr_->GetMeasurementsPtr()->current_state;

  const uint8_t current_state =
      local_view_ptr->function_state_machine_info.current_state();

  // just used for pybind simulation to clear previous state varible
  if (last_state == FuncStateMachine::STANDBY &&
      (current_state >= FuncStateMachine::PARK_IN_APA_IN &&
       current_state <= FuncStateMachine::PARK_IN_COMPLETED)) {
    Reset();
  }

  // run apa world, always run when enter apa
  DEBUG_PRINT("---- apa_world: Update() ---");
  apa_world_ptr_->Update(local_view_ptr);

  // run planner
  bool success = false;
#ifdef PERPENDICULAR_SIMULATION
  std::cout << "PERPENDICULAR_SIMULATION\n";
  success = ApaPlanOnce(ApaWorld::PERPENDICULAR_PARK_IN_PLANNER);
  // if (current_state == FuncStateMachine::PARK_IN_ACTIVATE_WAIT ||
  //     current_state == FuncStateMachine::PARK_IN_ACTIVATE_CONTROL ||
  //     current_state == FuncStateMachine::PARK_IN_SECURE) {
  //   success = ApaPlanOnce(ApaWorld::PERPENDICULAR_PARK_IN_PLANNER);
  // }
#else
  if (apa_world_ptr_->GetMeasurementsPtr()->planner_type <
      ApaWorld::NONE_PLANNER) {
    success = ApaPlanOnce(apa_world_ptr_->GetMeasurementsPtr()->planner_type);
  }
#endif

  if (success) {
    planning_output_ = planner_ptr_->GetOutput();
  }

  AddReleasedSlotInfo(planning_output_);

  return success;
}

const bool ApaPlanInterface::ApaPlanOnce(const uint8_t planner_type) {
  const auto planner_ptr = GetPlannerByType(planner_type);

  if (planner_ptr != nullptr) {
    planner_ptr_ = planner_ptr;
    planner_ptr_->Update();
    return true;
  } else {
    std::cout << "planner type error!" << std::endl;
    return false;
  }
}

void ApaPlanInterface::AddReleasedSlotInfo(
    PlanningOutput::PlanningOutput &planning_output) {
  planning_output.clear_successful_slot_info_list();

  for (const auto &successful_slot_info :
       apa_world_ptr_->GetSlotManagerPtr()->GetReleasedSlotInfoVec()) {
    // std::cout << "successful_slot_info = " <<
    // successful_slot_info.DebugString()
    //           << std::endl;
    const auto slot_info_list = planning_output.add_successful_slot_info_list();
    slot_info_list->CopyFrom(successful_slot_info);
  }
}

void ApaPlanInterface::UpdateDebugInfo() {
  auto &planning_debug_data = DebugInfoManager::GetInstance().GetDebugInfoPb();
  auto debug_info_json = *DebugInfoManager::GetInstance().GetDebugJson();
  planning_debug_data->set_data_json(mjson::Json(debug_info_json).dump());

  planning_debug_info_ = *planning_debug_data;
}

static std::string ReadFile(const std::string &path) {
  FILE *file = fopen(path.c_str(), "r");
  assert(file != nullptr);
  std::shared_ptr<FILE> fp(file, [](FILE *file) { fclose(file); });
  fseek(fp.get(), 0, SEEK_END);
  std::vector<char> content(ftell(fp.get()));
  fseek(fp.get(), 0, SEEK_SET);
  auto read_bytes = fread(content.data(), 1, content.size(), fp.get());
  assert(read_bytes == content.size());
  (void)read_bytes;
  return std::string(content.begin(), content.end());
}

void ApaPlanInterface::SyncParameters() {
  std::string path = "/asw/planning/res/conf/apa_params.json";

  std::string config_file = ReadFile(path);
  auto config = mjson::Reader(config_file);

  // schedule params
  JSON_READ_VALUE(apa_param.SetPram().plan_time, double, "plan_time");

  // car params
  JSON_READ_VALUE(apa_param.SetPram().front_overhanging, double,
                  "front_overhanging");

  JSON_READ_VALUE(apa_param.SetPram().rear_overhanging, double,
                  "rear_overhanging");

  JSON_READ_VALUE(apa_param.SetPram().wheel_base, double, "wheel_base");
  JSON_READ_VALUE(apa_param.SetPram().car_width, double, "car_width");
  JSON_READ_VALUE(apa_param.SetPram().car_length, double, "car_length");
  JSON_READ_VALUE(apa_param.SetPram().mirror_width, double, "mirror_width");
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

  // check finish params
  JSON_READ_VALUE(apa_param.SetPram().finish_lat_err, double, "finish_lat_err");

  JSON_READ_VALUE(apa_param.SetPram().finish_lat_err_strict, double,
                  "finish_lat_err_strict");

  JSON_READ_VALUE(apa_param.SetPram().finish_lon_err, double, "finish_lon_err");

  JSON_READ_VALUE(apa_param.SetPram().finish_heading_err, double,
                  "finish_heading_err");

  JSON_READ_VALUE(apa_param.SetPram().finish_uss_slot_occupied_ratio, double,
                  "finish_uss_slot_occupied_ratio");
  JSON_READ_VALUE(apa_param.SetPram().finish_heading_err_loose, double,
                  "finish_heading_err_loose");

  JSON_READ_VALUE(apa_param.SetPram().finish_parallel_lat_err, double,
                  "finish_parallel_lat_err");

  JSON_READ_VALUE(apa_param.SetPram().finish_parallel_lon_err, double,
                  "finish_parallel_lon_err");

  JSON_READ_VALUE(apa_param.SetPram().finish_parallel_heading_err, double,
                  "finish_parallel_heading_err");

  JSON_READ_VALUE(apa_param.SetPram().finish_parallel_rear_stop_buffer, double,
                  "finish_parallel_rear_stop_buffer");

  // check fail params
  JSON_READ_VALUE(apa_param.SetPram().stuck_failed_time, double,
                  "stuck_failed_time");

  JSON_READ_VALUE(apa_param.SetPram().pause_failed_time, double,
                  "pause_failed_time");

  // check static params
  JSON_READ_VALUE(apa_param.SetPram().car_static_pos_err, double,
                  "car_static_pos_err");

  JSON_READ_VALUE(apa_param.SetPram().car_static_velocity, double,
                  "car_static_velocity");

  JSON_READ_VALUE(apa_param.SetPram().car_static_keep_time_by_pos, double,
                  "car_static_keep_time_by_pos");

  JSON_READ_VALUE(apa_param.SetPram().car_static_keep_time_by_vel, double,
                  "car_static_keep_time_by_vel");

  // uss params
  JSON_READ_VALUE(apa_param.SetPram().detection_distance, double,
                  "detection_distance");

  JSON_READ_VALUE(apa_param.SetPram().lat_inflation, double, "lat_inflation");

  JSON_READ_VALUE(apa_param.SetPram().safe_uss_remain_dist_in_slot, double,
                  "safe_uss_remain_dist_in_slot");

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

  JSON_READ_VALUE(apa_param.SetPram().force_both_side_occupied, bool,
                  "force_both_side_occupied");

  JSON_READ_VALUE(apa_param.SetPram().safe_threshold, double, "safe_threshold");

  JSON_READ_VALUE(apa_param.SetPram().virtual_obs_y_pos, double,
                  "virtual_obs_y_pos");

  JSON_READ_VALUE(apa_param.SetPram().virtual_obs_x_pos, double,
                  "virtual_obs_x_pos");

  JSON_READ_VALUE(apa_param.SetPram().obs_consider_long_threshold, double,
                  "obs_consider_long_threshold");

  JSON_READ_VALUE(apa_param.SetPram().obs_consider_lat_threshold, double,
                  "obs_consider_lat_threshold");

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

  // construce obstacles params
  JSON_READ_VALUE(apa_param.SetPram().channel_width, double, "channel_width");

  JSON_READ_VALUE(apa_param.SetPram().channel_length, double, "channel_length");

  JSON_READ_VALUE(apa_param.SetPram().max_obs2car_dist_in_slot, double,
                  "max_obs2car_dist_in_slot");

  JSON_READ_VALUE(apa_param.SetPram().max_obs2car_dist_out_slot, double,
                  "max_obs2car_dist_out_slot");

  JSON_READ_VALUE(apa_param.SetPram().max_obs2car_dist_slot_occupied_ratio,
                  double, "max_obs2car_dist_slot_occupied_ratio");

  JSON_READ_VALUE(apa_param.SetPram().col_obs_safe_dist, double,
                  "col_obs_safe_dist");

  JSON_READ_VALUE(apa_param.SetPram().obstacle_ds, double, "obstacle_ds");

  JSON_READ_VALUE(apa_param.SetPram().car_lat_inflation_for_obs, double,
                  "car_lat_inflation_for_obs");

  JSON_READ_VALUE(apa_param.SetPram().tmp_no_consider_obs_dy, bool,
                  "tmp_no_consider_obs_dy");

  JSON_READ_VALUE(apa_param.SetPram().tmp_virtual_obs_dy, double,
                  "tmp_virtual_obs_dy");

  JSON_READ_VALUE(apa_param.SetPram().tlane_safe_dx, double, "tlane_safe_dx");

  JSON_READ_VALUE(apa_param.SetPram().obs_safe_dx, double, "obs_safe_dx");

  JSON_READ_VALUE(apa_param.SetPram().obs2slot_max_dist, double,
                  "obs2slot_max_dist");

  JSON_READ_VALUE(apa_param.SetPram().parallel_obs2slot_max_dist, double,
                  "parallel_obs2slot_max_dist");

  JSON_READ_VALUE(apa_param.SetPram().parallel_channel_y_mag, double,
                  "parallel_channel_y_mag");

  JSON_READ_VALUE(apa_param.SetPram().parallel_channel_x_mag, double,
                  "parallel_channel_x_mag");

  JSON_READ_VALUE(apa_param.SetPram().parallel_ego_side_to_obs_in_buffer, double,
                  "parallel_ego_side_to_obs_in_buffer");

  JSON_READ_VALUE(
      apa_param.SetPram().parallel_ego_front_corner_to_obs_in_buffer, double,
      "parallel_ego_front_corner_to_obs_in_buffer");

  JSON_READ_VALUE(apa_param.SetPram().slot_max_jump_dist, double,
                  "slot_max_jump_dist");

  JSON_READ_VALUE(apa_param.SetPram().line_arc_obs_slot_occupied_ratio, double,
                  "line_arc_obs_slot_occupied_ratio");

  JSON_READ_VALUE(apa_param.SetPram().line_arc_obs_channel_width, double,
                  "line_arc_obs_channel_width");

  JSON_READ_VALUE(apa_param.SetPram().line_arc_obs_channel_length, double,
                  "line_arc_obs_channel_length");

  JSON_READ_VALUE(apa_param.SetPram().dynamic_col_det_enable, bool,
                  "dynamic_col_det_enable");

  JSON_READ_VALUE(apa_param.SetPram().car_lat_inflation_for_trim_path, double,
                  "car_lat_inflation_for_trim_path");

  JSON_READ_VALUE(apa_param.SetPram().safe_dist_for_trim_path, double,
                  "safe_dist_for_trim_path");

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

  // slot update params when parking
  JSON_READ_VALUE(apa_param.SetPram().fix_slot_occupied_ratio, double,
                  "fix_slot_occupied_ratio");

  // slot update params when replan
  JSON_READ_VALUE(apa_param.SetPram().uss_slot_occupied_ratio, double,
                  "uss_slot_occupied_ratio");

  JSON_READ_VALUE(apa_param.SetPram().path_heading_slot, double,
                  "path_heading_slot");

  JSON_READ_VALUE(apa_param.SetPram().path_pos_err, double, "path_pos_err");

  JSON_READ_VALUE(apa_param.SetPram().last_update_slot_occupied_ratio, double,
                  "last_update_slot_occupied_ratio");

  // path planner params
  JSON_READ_VALUE(apa_param.SetPram().min_turn_radius, double,
                  "min_turn_radius");

  JSON_READ_VALUE(apa_param.SetPram().max_radius_in_slot, double,
                  "max_radius_in_slot");

  JSON_READ_VALUE(apa_param.SetPram().min_radius_out_slot, double,
                  "min_radius_out_slot");

  JSON_READ_VALUE(apa_param.SetPram().max_one_step_arc_radius, double,
                  "max_one_step_arc_radius");

  JSON_READ_VALUE(apa_param.SetPram().radius_eps, double, "radius_eps");

  JSON_READ_VALUE(apa_param.SetPram().min_line_length, double,
                  "min_line_length");

  JSON_READ_VALUE(apa_param.SetPram().max_line_length_for_third_prepare, double,
                  "max_line_length_for_third_prepare");

  JSON_READ_VALUE(apa_param.SetPram().min_one_step_path_length, double,
                  "min_one_step_path_length");

  JSON_READ_VALUE(apa_param.SetPram().min_one_step_path_length_in_slot, double,
                  "min_one_step_path_length_in_slot");

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

  // slot managent params
  JSON_READ_VALUE(apa_param.SetPram().release_slot_by_prepare, bool,
                  "release_slot_by_prepare");
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

  JSON_READ_VALUE(apa_param.SetPram().terminal_length, double,
                  "terminal_length");

  JSON_READ_VALUE(apa_param.SetPram().limiter_length, double, "limiter_length");

  JSON_READ_VALUE(apa_param.SetPram().slot_occupied_ratio_max_lat_err, double,
                  "slot_occupied_ratio_max_lat_err");

  JSON_READ_VALUE(apa_param.SetPram().slot_occupied_ratio_max_heading_err,
                  double, "slot_occupied_ratio_max_heading_err");

  // gen output params
  JSON_READ_VALUE(apa_param.SetPram().max_velocity, double, "max_velocity");
}

}  // namespace apa_planner
}  // namespace planning