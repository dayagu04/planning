#ifndef __REALTIME_LONGITUDINAL_MOTION_PLANNING_PROBLEM_H__
#define __REALTIME_LONGITUDINAL_MOTION_PLANNING_PROBLEM_H__

#include <cstddef>
#include <vector>

#include "config/basic_type.h"
#include "ilqr_core.h"
#include "longitudinal_motion_planner.pb.h"
#include "math_lib.h"
#include "realtime_longitudinal_motion_planning_cost.h"
#include "realtime_longitudinal_motion_planning_model.h"

namespace pnc {
namespace realtime_longitudinal_planning {

class RealtimeLongitudinalMotionPlanningProblem {
 public:
  void Init();
  uint8_t Update(planning::common::LongitudinalPlanningInput &planning_input);
  const planning::common::LongitudinalPlanningOutput &GetOutput() const {
    return planning_output_;
  }
  void Reset();

  void SetWarmStart(bool flag) { ilqr_core_ptr_->SetWarmStart(flag); }
  void SetMaxIter(size_t max_iter) { ilqr_core_ptr_->SetMaxIter(max_iter); }

  const std::shared_ptr<ilqr_solver::iLqr> GetiLqrCorePtr() const {
    return ilqr_core_ptr_;
  }

 private:
  std::shared_ptr<ilqr_solver::iLqr> ilqr_core_ptr_;
  planning::common::LongitudinalPlanningOutput planning_output_;
  State init_state_;
};

}  // namespace realtime_longitudinal_planning
}  // namespace pnc
#endif
