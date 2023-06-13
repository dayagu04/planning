#include "apa_planner/diagonal/diagonal_in_planner.h"

#include "../../../common/log_glog.h"

namespace planning {
namespace apa_planner {

using PlanningOutput::PlanningOutput;

bool DiagonalInPlanner::Update(framework::Frame* const frame) {
  AINFO << "+++++++++++++diagonal planning+++++++++++++";
  return trajectory_generator_.Plan(frame);
}

} // namespace apa_planner
} // namespace planning
