#ifndef __REALTIME_LATERAL_MOTION_PLANNING_PROBLEM_H__
#define __REALTIME_LATERAL_MOTION_PLANNING_PROBLEM_H__

#include <cstddef>
#include <vector>

#include "ilqr_core.h"
#include "ilqr_define.h"
#include "lateral_motion_planner.pb.h"
#include "realtime_lateral_motion_planning_cost.h"
#include "realtime_lateral_motion_planning_model.h"
namespace pnc {
namespace realtime_lateral_planning {
class RealtimeLateralMotionPlanningProblem {
 public:
  void Init();
  uint8_t Update(planning::common::LateralPlanningInput &planning_input);
  const planning::common::LateralPlanningOutput &GetOutput() {
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
  planning::common::LateralPlanningOutput planning_output_;
  ilqr_solver::State init_state_;
};

}  // namespace realtime_lateral_planning
}  // namespace pnc
#endif
