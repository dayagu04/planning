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
#include "parallel_park_in_planner.h"
#include "perpendicular_park_in_planner.h"
#include "planning_output_context.h"

namespace planning {
namespace apa_planner {

void ApaPlanInterface::Init() {
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

  // sync parameters
  SyncParameters();
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
               "------------------------"
            << std::endl;

  const uint8_t last_state =
      apa_world_ptr_->GetMeasurementsPtr()->current_state;

  const uint8_t current_state =
      local_view_ptr->function_state_machine_info.current_state();

  // just used for pybind simulation to clear previous state varible
  if (last_state == FuncStateMachine::STANDBY &&
      (current_state >= FuncStateMachine::PARK_IN_APA_IN &&
       current_state <= FuncStateMachine::PARK_IN_COMPLETED)) {
    apa_world_ptr_->Reset();
    std::cout << "reset apa world once!" << std::endl;

    if (planner_ptr_ != nullptr) {
      planner_ptr_->Reset();
      std::cout << "reset planner once!" << std::endl;
    }
  }

  // run apa world, always run when enter apa
  std::cout << "---- apa_world: Update() ---" << std::endl;
  apa_world_ptr_->Update(local_view_ptr);

  // run planner
  bool success = false;

  if (apa_world_ptr_->GetMeasurementsPtr()->planner_type <
      ApaWorld::NONE_PLANNER) {
    success = ApaPlanOnce(apa_world_ptr_->GetMeasurementsPtr()->planner_type);
  }

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
    std::cout << "successful_slot_info = " << successful_slot_info.DebugString()
              << std::endl;
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

  JSON_READ_VALUE(apa_param.SetPram().normal_slot_length, double,
                  "normal_slot_length");

  JSON_READ_VALUE(apa_param.SetPram().max_finish_lat_offset, double,
                  "max_finish_lat_offset");

  JSON_READ_VALUE(apa_param.SetPram().max_finish_lon_offset, double,
                  "max_finish_lon_offset");

  JSON_READ_VALUE(apa_param.SetPram().max_finish_heading_offset_deg, double,
                  "max_finish_heading_offset_deg");

  JSON_READ_VALUE(apa_param.SetPram().max_velocity, double, "max_velocity");

  JSON_READ_VALUE(apa_param.SetPram().safe_uss_remain_dist, double,
                  "safe_uss_remain_dist");

  JSON_READ_VALUE(apa_param.SetPram().stuck_failed_time, double,
                  "stuck_failed_time");

  JSON_READ_VALUE(apa_param.SetPram().stuck_replan_time, double,
                  "stuck_replan_time");

  JSON_READ_VALUE(apa_param.SetPram().uss_stuck_replan_wait_time, double,
                  "uss_stuck_replan_wait_time");

  JSON_READ_VALUE(apa_param.SetPram().max_replan_remain_dist, double,
                  "max_replan_remain_dist");

  JSON_READ_VALUE(apa_param.SetPram().vacant_p0_x_diff, double,
                  "vacant_p0_x_diff");

  JSON_READ_VALUE(apa_param.SetPram().vacant_p0_y_diff, double,
                  "vacant_p0_y_diff");

  JSON_READ_VALUE(apa_param.SetPram().vacant_p1_x_diff, double,
                  "vacant_p1_x_diff");

  JSON_READ_VALUE(apa_param.SetPram().vacant_p1_y_diff, double,
                  "vacant_p1_y_diff");

  JSON_READ_VALUE(apa_param.SetPram().occupied_p0_x_diff, double,
                  "occupied_p0_x_diff");

  JSON_READ_VALUE(apa_param.SetPram().occupied_p0_y_diff, double,
                  "occupied_p0_y_diff");

  JSON_READ_VALUE(apa_param.SetPram().occupied_p1_x_diff, double,
                  "occupied_p1_x_diff");

  JSON_READ_VALUE(apa_param.SetPram().occupied_p1_y_diff, double,
                  "occupied_p1_y_diff");

  JSON_READ_VALUE(apa_param.SetPram().nearby_slot_corner_dist, double,
                  "nearby_slot_corner_dist");

  JSON_READ_VALUE(apa_param.SetPram().channel_width, double, "channel_width");

  JSON_READ_VALUE(apa_param.SetPram().terminal_target_x, double,
                  "terminal_target_x");

  JSON_READ_VALUE(apa_param.SetPram().terminal_target_y, double,
                  "terminal_target_y");

  JSON_READ_VALUE(apa_param.SetPram().terminal_target_y_bias, double,
                  "terminal_target_y_bias");

  JSON_READ_VALUE(apa_param.SetPram().terminal_target_x_to_limiter, double,
                  "terminal_target_x_to_limiter");

  JSON_READ_VALUE(apa_param.SetPram().min_turn_radius, double,
                  "min_turn_radius");

  JSON_READ_VALUE(apa_param.SetPram().max_radius_in_slot, double,
                  "max_radius_in_slot");

  JSON_READ_VALUE(apa_param.SetPram().max_one_step_arc_radius, double,
                  "max_one_step_arc_radius");

  JSON_READ_VALUE(apa_param.SetPram().radius_eps, double, "radius_eps");

  JSON_READ_VALUE(apa_param.SetPram().min_line_length, double,
                  "min_line_length");

  JSON_READ_VALUE(apa_param.SetPram().min_one_step_path_length, double,
                  "min_one_step_path_length");

  JSON_READ_VALUE(apa_param.SetPram().prepare_line_x_offset_slot, double,
                  "prepare_line_x_offset_slot");

  JSON_READ_VALUE(apa_param.SetPram().prepare_line_heading_offset_slot_deg,
                  double, "prepare_line_heading_offset_slot_deg");

  JSON_READ_VALUE(apa_param.SetPram().static_pos_eps, double, "static_pos_eps");
  JSON_READ_VALUE(apa_param.SetPram().static_heading_eps, double,
                  "static_heading_eps");

  JSON_READ_VALUE(apa_param.SetPram().plan_time, double, "plan_time");

  JSON_READ_VALUE(apa_param.SetPram().car_static_pos_eps, double,
                  "car_static_pos_eps");

  JSON_READ_VALUE(apa_param.SetPram().car_static_velocity, double,
                  "car_static_velocity");

  JSON_READ_VALUE(apa_param.SetPram().max_standstill_speed, double,
                  "max_standstill_speed");

  JSON_READ_VALUE(apa_param.SetPram().min_standstill_time_by_pos, double,
                  "min_standstill_time_by_pos");

  JSON_READ_VALUE(apa_param.SetPram().min_standstill_time_by_vel, double,
                  "min_standstill_time_by_vel");

  JSON_READ_VALUE(apa_param.SetPram().front_overhanging, double,
                  "front_overhanging");

  JSON_READ_VALUE(apa_param.SetPram().rear_overhanging, double,
                  "rear_overhanging");

  JSON_READ_VALUE(apa_param.SetPram().wheel_base, double, "wheel_base");
  JSON_READ_VALUE(apa_param.SetPram().vehicle_width, double, "vehicle_width");
}

const bool ApaPlanInterface::UpdateFrame(framework::Frame *frame) {
  // main update
  const auto local_view_ptr =
      &(frame->session()->environmental_model().get_local_view());

  if (g_context.GetStatemachine().apa_reset_flag) {
    apa_world_ptr_->Reset();
    std::cout << "reset apa world once!" << std::endl;

    if (planner_ptr_ != nullptr) {
      planner_ptr_->Reset();
      std::cout << "reset planner once!" << std::endl;
    }

    // sync parameters
    SyncParameters();
  }

  const bool success = Update(local_view_ptr);

  auto planning_output_ptr = &(frame->mutable_session()
                                   ->mutable_planning_output_context()
                                   ->mutable_planning_status()
                                   ->planning_result.planning_output);

  if (success) {
    const auto apa_planning_output = planner_ptr_->GetOutput();
    *planning_output_ptr = apa_planning_output;

    const auto &planning_status =
        planner_ptr_->GetPlannerStates().planning_status;

    AddReleasedSlotInfo(*planning_output_ptr);

    return planning_status < ApaPlannerBase::ParkingStatus::PARKING_FAILED;
  } else {
    planning_output_ptr->clear_trajectory();
    planning_output_ptr->clear_gear_command();
    AddReleasedSlotInfo(*planning_output_ptr);
    return false;
  }
}

}  // namespace apa_planner
}  // namespace planning