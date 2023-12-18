#include "apa_plan_interface.h"

#include <cstdint>
#include <memory>

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