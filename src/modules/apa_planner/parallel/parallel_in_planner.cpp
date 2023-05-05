#include "apa_planner/parallel/parallel_in_planner.h"

#include "apa_planner/common/planning_log_helper.h"

namespace planning {
namespace apa_planner {

using PlanningOutput::PlanningOutput;

bool ParallelInPlanner::Update(framework::Frame* const frame) {

  PLANNING_LOG << "+++++++++++++parallel planning+++++++++++++" << std::endl;
  const auto& parking_fusion = frame->session()->environmental_model().\
      get_local_view().parking_fusion_info;
  const auto& slots = parking_fusion.parking_fusion_slot_lists();
  const size_t selected_slot_id = parking_fusion.select_slot_id();
  bool is_parallel_slot = false;
  for (int i = 0; i < parking_fusion.parking_fusion_slot_lists_size(); ++i) {
    if (selected_slot_id == slots[i].id()) {
      if (slots[i].type() ==
              Common::ParkingSlotType::PARKING_SLOT_TYPE_HORIZONTAL) {
        is_parallel_slot = true;
      }
    }
  }
  if (!is_parallel_slot) {
    PLANNING_LOG << "Error: slot type is not parallel" << std::endl;
    return false;
  }

  if (is_stop_planning_) {
    PLANNING_LOG << "last planning failed, stop planning" << std::endl;
    SetFailedPlanningOutput(frame);
    return false;
  }

  const auto& pre_planning_result = frame->session()->planning_output_context()
      .planning_status().pre_planning_result;
  frame->mutable_session()->mutable_planning_output_context()\
      ->mutable_planning_status()->planning_result = pre_planning_result;
  const bool is_planning_ok = trajectory_generator_.Plan(frame);
  if (!is_planning_ok) {
    SetFailedPlanningOutput(frame);
    PLANNING_LOG << "parallel parking failed" << std::endl;
  }

  if (is_planning_ok_ && !is_planning_ok) {
    is_stop_planning_ = true;
  }

  const auto& planning_result = frame->session()->planning_output_context()
      .planning_status().planning_result;
  frame->mutable_session()->mutable_planning_output_context()\
      ->mutable_planning_status()->pre_planning_result = planning_result;

  is_planning_ok_ = is_planning_ok;

  return is_planning_ok;
}

} // namespace apa_planner
} // namespace planning
