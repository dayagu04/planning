#include "apa_planner/diagonal/diagonal_in_planner.h"

#include "apa_planner/common/planning_log_helper.h"

namespace planning {
namespace apa_planner {

using PlanningOutput::PlanningOutput;

bool DiagonalInPlanner::Update(framework::Frame* const frame) {
  PLANNING_LOG << "+++++++++++++diagonal planning+++++++++++++" << std::endl;
  return trajectory_generator_.Plan(frame);
}

} // namespace apa_planner
} // namespace planning
