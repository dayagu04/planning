#pragma once

#include "planning_plan.pb.h"

#include "frame.h"
#include "apa_planner/apa_planner_base.h"
#include "apa_planner/parallel/parallel_in_trajectory_generator.h"

namespace planning {
namespace apa_planner {

class ParallelInPlanner : public ApaPlannerBase {
 public:
  ParallelInPlanner() = default;
  ~ParallelInPlanner() = default;

  bool Update(framework::Frame* const frame) override;

 private:
  ParallelInTrajectoryGenerator trajectory_generator_;
  bool is_planning_ok_ = false;
  bool is_stop_planning_ = false;
};

} // namespace apa_planner
} // namespace planning
