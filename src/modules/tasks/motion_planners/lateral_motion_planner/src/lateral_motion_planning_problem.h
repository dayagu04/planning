#ifndef __LATERAL_MOTION_PLANNING_PROBLEM_H__
#define __LATERAL_MOTION_PLANNING_PROBLEM_H__

#include <cstddef>
#include <vector>

#include "common/config/basic_type.h"
#include "ilqr_core.h"
#include "lateral_motion_planner.pb.h"
#include "lateral_motion_planning_cost.h"
#include "lateral_motion_planning_model.h"
#include "math_lib.h"
namespace pnc {
namespace lateral_planning {
class LateralMotionPlanningProblem {
public:
  void Init();
  uint8_t Update(planning::common::LateralPlanningInput &planning_input);
  void SetWarmStart(bool flag) { ilqr_core_ptr_->SetWarmStart(flag); };
  void SetMaxIter(size_t max_iter) { ilqr_core_ptr_->SetMaxIter(max_iter); };

  void Reset();

  const std::shared_ptr<ilqr_solver::iLqr> GetiLqrCorePtr() const {
    return ilqr_core_ptr_;
  }

private:
  std::shared_ptr<ilqr_solver::iLqr> ilqr_core_ptr_;
  State init_state_;
};

} // namespace lateral_planning
} // namespace pnc
#endif
