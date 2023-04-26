#include "modules/apa_planner/parallel/parallel_in_planner.h"

#include "modules/apa_planner/common/planning_log_helper.h"

namespace planning {
namespace apa_planner {

using PlanningOutput::PlanningOutput;

bool ParallelInPlanner::Update(framework::Frame* const frame) {

  PLANNING_LOG << "+++++++++++++parallel planning+++++++++++++" << std::endl;
  if (is_stop_planning_) {
    PLANNING_LOG << "last planning failed, stop planning" << std::endl;
    SetFailedPlanningOutput(frame);
    return false;
  }

  const auto& pre_planning_output = frame->session()->planning_output_context()
      .planning_status().pre_planning_result;
  frame->mutable_session()->mutable_planning_output_context()\
      ->mutable_planning_status()->planning_result = pre_planning_output;

  const bool is_planning_ok = trajectory_generator_.Plan(frame);
  if (!is_planning_ok) {
    SetFailedPlanningOutput(frame);
    PLANNING_LOG << "parallel parking failed" << std::endl;
  }

  if (is_planning_ok_ && !is_planning_ok) {
    is_stop_planning_ = true;
  }

  is_planning_ok_ = is_planning_ok;

  return is_planning_ok;
}

} // namespace apa_planner
} // namespace planning
