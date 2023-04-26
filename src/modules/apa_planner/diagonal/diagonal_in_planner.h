#pragma once

#include "../res/include/proto/planning_plan.pb.h"

#include "framework/frame.h"
#include "modules/apa_planner/apa_planner_base.h"
#include "modules/apa_planner/diagonal/diagonal_in_trajectory_generator.h"

namespace planning {
namespace apa_planner {

class DiagonalInPlanner : public ApaPlannerBase {
 public:
  DiagonalInPlanner() = default;
  ~DiagonalInPlanner() = default;

  bool Update(framework::Frame* const frame) override;

 private:
  DiagonalInTrajectoryGenerator trajectory_generator_;
  bool is_planning_ok_ = false;
  bool is_stop_planning_ = false;
};

} // namespace apa_planner
} // namespace planning
