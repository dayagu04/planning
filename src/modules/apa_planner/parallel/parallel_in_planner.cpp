#include "parallel/parallel_in_planner.h"

#include "common/apa_utils.h"
#include "common/planning_log_helper.h"
#include "environmental_model.h"
#include "log_glog.h"
#include "planning_output_context.h"

namespace planning {
namespace apa_planner {

bool ParallelInPlanner::Update(framework::Frame* const frame) {
  AINFO << "+++++++++++++parallel planning+++++++++++++";
  return trajectory_generator_.Plan(frame);
}

}  // namespace apa_planner
}  // namespace planning
