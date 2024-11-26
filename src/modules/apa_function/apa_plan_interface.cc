#include "apa_plan_interface.h"

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "apa_data.h"
#include "apa_param_config.h"
#include "apa_world.h"
#include "common/config_context.h"
#include "debug_info_log.h"
#include "environmental_model.h"
#include "func_state_machine_c.h"
#include "general_planning_context.h"
#include "ifly_time.h"
#include "local_view.h"
#include "log_glog.h"
#include "parallel_park_in_scenario.h"
#include "perpendicular_head_in_scenario.h"
#include "perpendicular_head_out_scenario.h"
#include "perpendicular_tail_in_path_generator.h"
#include "perpendicular_tail_in_scenario.h"
#include "planning_context.h"
#include "planning_plan_c.h"
#include "src/modules/apa_function/parking_scenario/parking_scenario.h"

namespace planning {
namespace apa_planner {

void ApaPlanInterface::Init(const bool is_simulation) {
  // sync parameters
  SyncParkingParameters(is_simulation);

  // init apa world
  apa_world_ptr_ = std::make_shared<ApaWorld>();

  scenario_manager_.Init(apa_world_ptr_);

  return;
}

void ApaPlanInterface::Reset() {
  // reset planning output
  memset(&planning_output_, 0, sizeof(planning_output_));

  memset(&apa_hmi_, 0, sizeof(apa_hmi_));

  SyncParkingParameters();

  // reset apa world
  apa_world_ptr_->Reset();
  scenario_manager_.Reset();

  return;
}

void ApaPlanInterface::ResetForSearching() {
  // reset planning output
  memset(&planning_output_, 0, sizeof(planning_output_));

  memset(&apa_hmi_, 0, sizeof(apa_hmi_));

  scenario_manager_.Reset();

  return;
}

const bool ApaPlanInterface::Update(const LocalView *local_view_ptr) {
  ILOG_INFO << "\n------------------------ apa_interface: Update() "
               "------------------------";
  if (local_view_ptr == nullptr) {
    ILOG_INFO << "\nlocal_view_ptr is nullptr, quit apa";
    return false;
  }

  RecordNodeReceiveTime(local_view_ptr);

  const double start_timestamp_ms = IflyTime::Now_ms();
  const uint8_t last_state = apa_world_ptr_->GetApaDataPtr()->current_state;

  // todo:move this to scenario manager
  const uint8_t current_state =
      local_view_ptr->function_state_machine_info.current_state;

  // just used for pybind simulation to clear previous state varible
  if ((last_state == iflyauto::FunctionalState_PARK_STANDBY) &&
      (current_state >= iflyauto::FunctionalState_PARK_IN_SEARCHING &&
       current_state <= iflyauto::FunctionalState_PARK_OUT_SEARCHING)) {
    Reset();
  }

  // run apa world, always run when enter apa
  ILOG_INFO << "---- apa_world: Update() ---";
  (void)apa_world_ptr_->Update(local_view_ptr, planning_output_);

  // todo:move this to scenario manager
  if (apa_world_ptr_->GetStateMachineManagerPtr()->GetStateMachine() ==
      ApaStateMachineT::INVALID) {
    Reset();
  } else if (apa_world_ptr_->GetStateMachineManagerPtr()->IsSeachingStatus()) {
    ResetForSearching();
  }

  // run planner
  ParkingScenarioStatus scenario_status;
  scenario_status = scenario_manager_.Excute(apa_world_ptr_->GetApaDataPtr());
  std::shared_ptr<ParkingScenario> scenario_ =
      scenario_manager_.MutableScenarioPtr();
  if (scenario_ != nullptr &&
      scenario_status == ParkingScenarioStatus::STATUS_RUNNING) {
    scenario_->Process();

    planning_output_ = scenario_->GetOutput();
    apa_hmi_ = scenario_->GetAPAHmi();
    // DEBUG_PRINT("interface planning hmi----------------");
    // DEBUG_PRINT("remain dist in hmi = " <<
    // apa_hmi_.distance_to_parking_space); DEBUG_PRINT(
    //     "is_parking_pause = " <<
    //     static_cast<int>(apa_hmi_.is_parking_pause));
    // DEBUG_PRINT("parking_pause_reason = " << apa_hmi_.parking_pause_reason);
  }

  AddReleasedSlotInfo(planning_output_);

  const auto end_timestamp_ms = IflyTime::Now_ms();
  const auto frame_duration = end_timestamp_ms - start_timestamp_ms;

  ILOG_INFO << "total time consumption = " << frame_duration << "ms";
  JSON_DEBUG_VALUE("total_plan_consume_time", frame_duration)

  return (scenario_status != ParkingScenarioStatus::STATUS_UNKNOWN) ? true
                                                                    : false;
}

void ApaPlanInterface::AddReleasedSlotInfo(
    iflyauto::PlanningOutput &planning_output) {
  planning_output.successful_slot_info_list_size = 0;
  const std::vector<int> &release_slot_id_vec =
      apa_world_ptr_->GetSlotManagerPtr()->GetReleaseSlotIdVec();

  std::string release_slot_id;
  for (size_t i = 0; i < release_slot_id_vec.size(); ++i) {
    iflyauto::SuccessfulSlotsInfo slot_id;
    slot_id.id = static_cast<uint32>(release_slot_id_vec[i]);
    planning_output.successful_slot_info_list[i] = slot_id;
    planning_output.successful_slot_info_list_size++;
    release_slot_id.append(std::string("[") + std::to_string(slot_id.id) +
                           std::string("]"));
  }
  ILOG_INFO << "plan release slot id = " << release_slot_id;
}

void ApaPlanInterface::UpdateDebugInfo() {
  auto &planning_debug_data = DebugInfoManager::GetInstance().GetDebugInfoPb();
  auto debug_info_json = *DebugInfoManager::GetInstance().GetDebugJson();
  planning_debug_data->set_data_json(mjson::Json(debug_info_json).dump());

  planning_debug_info_ = *planning_debug_data;
}

void ApaPlanInterface::RecordNodeReceiveTime(const LocalView *local_view_ptr) {
  JSON_DEBUG_VALUE("statemachine_timestamp",
                   local_view_ptr->function_state_machine_info_recv_time);
  JSON_DEBUG_VALUE("fusion_slot_timestamp",
                   local_view_ptr->parking_fusion_info_recv_time);
  JSON_DEBUG_VALUE("localiztion_timestamp",
                   local_view_ptr->localization_recv_time);
  JSON_DEBUG_VALUE("uss_wave_timestamp",
                   local_view_ptr->uss_wave_info_recv_time);
  JSON_DEBUG_VALUE("uss_per_timestamp",
                   local_view_ptr->uss_percept_info_recv_time);
  JSON_DEBUG_VALUE("ground_line_timestamp",
                   local_view_ptr->ground_line_perception_recv_time);
  JSON_DEBUG_VALUE("fusion_objects_timestamp",
                   local_view_ptr->fusion_objects_info_recv_time);
  JSON_DEBUG_VALUE("fusion_occupancy_objects_timestamp",
                   local_view_ptr->fusion_occupancy_objects_info_recv_time);
  JSON_DEBUG_VALUE("control_output_timestamp",
                   local_view_ptr->control_output_recv_time);
}

std::shared_ptr<ParkingScenario> ApaPlanInterface::GetPlannerByType(
    const ParkingScenarioType type) {
  return scenario_manager_.GetScenarioByType(type);
}

}  // namespace apa_planner
}  // namespace planning