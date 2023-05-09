#include "apa_planner/parallel/parallel_in_planner.h"

#include "apa_planner/common/apa_utils.h"
#include "apa_planner/common/planning_log_helper.h"

namespace planning {
namespace apa_planner {

using PlanningOutput::PlanningOutput;

bool ParallelInPlanner::Update(framework::Frame* const frame) {
  PLANNING_LOG << "+++++++++++++parallel planning+++++++++++++" << std::endl;
  return trajectory_generator_.Plan(frame);
}

} // namespace apa_planner
} // namespace planning
