#include "apa_planner/parallel/parallel_in_planner.h"

#include "../../../common/log_glog.h"
#include "apa_planner/common/apa_utils.h"

namespace planning {
namespace apa_planner {

using PlanningOutput::PlanningOutput;

bool ParallelInPlanner::Update(framework::Frame* const frame) {
  AINFO << "+++++++++++++parallel planning+++++++++++++";
  return trajectory_generator_.Plan(frame);
}

} // namespace apa_planner
} // namespace planning
