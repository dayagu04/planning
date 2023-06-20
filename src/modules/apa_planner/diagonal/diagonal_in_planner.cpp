#include "diagonal/diagonal_in_planner.h"

#include "common/planning_log_helper.h"
#include "environmental_model.h"
#include "log_glog.h"
#include "planning_output_context.h"

namespace planning {
namespace apa_planner {

using PlanningOutput::PlanningOutput;

bool DiagonalInPlanner::Update(framework::Frame* const frame) {
  AINFO << "+++++++++++++diagonal planning+++++++++++++";
  return trajectory_generator_.Plan(frame);
}

}  // namespace apa_planner
}  // namespace planning
