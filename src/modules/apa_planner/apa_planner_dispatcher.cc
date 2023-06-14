#include "apa_planner/apa_planner_dispatcher.h"

#include "../../common/log_glog.h"
#include "apa_planner/common/apa_utils.h"
#include "apa_planner/diagonal/diagonal_in_planner.h"
#include "apa_planner/parallel/parallel_in_planner.h"
#include "func_state_machine.pb.h"

namespace planning {
namespace apa_planner {

using framework::Frame;
using ::FuncStateMachine::FuncStateMachine;
using ::FuncStateMachine::FunctionalState;
using ::ParkingFusion::ParkingFusionInfo;

ApaPlannerDispatcher::ApaPlannerDispatcher() {
  RegisterPlanners();
}

void ApaPlannerDispatcher::RegisterPlanners() {
  planner_list_.clear();
  planner_list_.emplace_back(std::make_unique<DiagonalInPlanner>());
  planner_list_.emplace_back(std::make_unique<ParallelInPlanner>());
}

bool ApaPlannerDispatcher::Update(Frame* const frame) {
  const auto& func_state_machine = frame->session()->environmental_model().\
      get_local_view().function_state_machine_info;

  if (func_state_machine.has_current_state()
      && func_state_machine.current_state()
          == FunctionalState::PARK_IN_COMPLETED) {
    AINFO << "apa parking in is finished";
    return true;
  }

  const auto& planning_output = frame->session()->planning_output_context().
      planning_status().planning_result.planning_output;
  if (planning_output.has_planning_status()
      && planning_output.planning_status().has_apa_planning_status()) {
    const auto& apa_planning_status =
        planning_output.planning_status().apa_planning_status();
    if (apa_planning_status == ::PlanningOutput::ApaPlanningStatus::FINISHED) {
      SetFinishedPlanningOutput(frame);
      AINFO << "apa parking in is finished, stop planning";
      return true;
    }
    if (apa_planning_status == ::PlanningOutput::ApaPlanningStatus::FAILED) {
      SetFailedPlanningOutput(frame);
      AERROR << "apa parking in is failed, stop planning";
      return true;
    }
  }

  if (!IsStateMachineStateValid(func_state_machine)) {
    return false;
  }

  if (!IsReplanNecessary(func_state_machine)) {
    return false;
  }

  if (IsReplanEachFrame(func_state_machine)) {
    frame->mutable_session()->mutable_planning_output_context()->
        mutable_planning_status()->planning_result.planning_output.Clear();
  }

  const auto& parking_fusion = frame->session()->environmental_model().\
      get_local_view().parking_fusion_info;
  if (parking_fusion.parking_fusion_slot_lists_size() == 0) {
    AERROR << "parking_fusion_slot_lists size is 0";
    return false;
  }

  bool is_planning_ok = false;
  for (auto& planner : planner_list_) {
    if (planner->Update(frame)) {
      is_planning_ok = true;
    }
  }
  if (!is_planning_ok) {
    SetFailedPlanningOutput(frame);
  }

  return is_planning_ok;
}

bool ApaPlannerDispatcher::IsStateMachineStateValid(
    const FuncStateMachine& func_state_machine) const {
  if (!func_state_machine.has_current_state()) {
    AERROR << "func_state_machine is invalid";
    return false;
  }

  AINFO << "current_state:" << func_state_machine.current_state();

  if (func_state_machine.current_state()
          == FunctionalState::PARK_IN_SEARCHING
      || func_state_machine.current_state()
          == FunctionalState::PARK_IN_NO_READY
      || func_state_machine.current_state()
          == FunctionalState::PARK_IN_READY
      || func_state_machine.current_state()
          == FunctionalState::PARK_IN_ACTIVATE_WAIT
      || func_state_machine.current_state()
          == FunctionalState::PARK_IN_ACTIVATE_CONTROL
      || func_state_machine.current_state()
          == FunctionalState::PARK_IN_SUSPEND_ACTIVATE
      || func_state_machine.current_state()
          == FunctionalState::PARK_IN_SUSPEND_CLOSE
      || func_state_machine.current_state()
          == FunctionalState::PARK_IN_SECURE
      || func_state_machine.current_state()
          == FunctionalState::PARK_IN_COMPLETED) {
    return true;
  }

  AERROR << "func_state_machine is invalid:"
      << func_state_machine.current_state();

  return false;
}

} // namespace apa_planner
} // namespace planning