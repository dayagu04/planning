#include "apa_planner_dispatcher.h"

#include "common/apa_utils.h"
#include "common/planning_log_helper.h"
#include "diagonal/diagonal_in_planner.h"
#include "environmental_model.h"
#include "log_glog.h"
#include "parallel/parallel_in_planner.h"
#include "planning_output_context.h"

namespace planning {
namespace apa_planner {

using ::FuncStateMachine::FuncStateMachine;
using ::FuncStateMachine::FunctionalState;
using ::ParkingFusion::ParkingFusionInfo;
using framework::Frame;

ApaPlannerDispatcher::ApaPlannerDispatcher() { RegisterPlanners(); }

void ApaPlannerDispatcher::RegisterPlanners() {
  planner_list_.clear();
  planner_list_.emplace_back(std::make_unique<DiagonalInPlanner>());
  planner_list_.emplace_back(std::make_unique<ParallelInPlanner>());
}

bool ApaPlannerDispatcher::Update(Frame* const frame) {
  const auto& func_state_machine = frame->session()
                                       ->environmental_model()
                                       .get_local_view()
                                       .function_state_machine_info;

  if (!IsValidParkingState(func_state_machine)) {
    frame->mutable_session()
        ->mutable_planning_output_context()
        ->mutable_planning_status()
        ->planning_result.planning_output.Clear();
    return false;
  }

  if (func_state_machine.has_current_state() &&
      func_state_machine.current_state() ==
          FunctionalState::PARK_IN_COMPLETED) {
    AINFO << "apa parking in is finished";
    return true;
  }

  const auto& planning_output = frame->session()
                                    ->planning_output_context()
                                    .planning_status()
                                    .planning_result.planning_output;
  if (planning_output.has_planning_status() &&
      planning_output.planning_status().has_apa_planning_status()) {
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

  if (!IsReplanNecessary(func_state_machine)) {
    return false;
  }

  if (IsReplanEachFrame(func_state_machine)) {
    frame->mutable_session()
        ->mutable_planning_output_context()
        ->mutable_planning_status()
        ->planning_result.planning_output.Clear();
  }

  const auto& parking_fusion = frame->session()
                                   ->environmental_model()
                                   .get_local_view()
                                   .parking_fusion_info;
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

}  // namespace apa_planner
}  // namespace planning
