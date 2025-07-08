#ifndef __LATERAL_MOTION_PLANNING_PROBLEM_H__
#define __LATERAL_MOTION_PLANNING_PROBLEM_H__

#include <cstddef>
#include <vector>

#include "ilqr_core.h"
#include "ilqr_define.h"
#include "lateral_motion_planner.pb.h"
#include "lateral_motion_planning_cost.h"
#include "lateral_motion_planning_model.h"
#include "motion_planners/lateral_motion_planner/src/lateral_motion_planning_weight.h"
#include "spline.h"
namespace pnc {
namespace lateral_planning {
class LateralMotionPlanningProblem {
 public:
  void Init();

  uint8_t Update(const double end_ratio_for_qxy,
                 const double end_ratio_for_qtheta,
                 const double end_ratio_for_qjerk,
                 const double concerned_start_q_jerk,
                 const std::shared_ptr<pnc::lateral_planning::LateralMotionPlanningWeight> &planning_weight,
                 planning::common::LateralPlanningInput &planning_input);

  uint8_t Update(double expected_acc, double start_acc, double end_acc,
                 double end_ratio_for_qxy, double end_ratio_for_qtheta,
                 double end_ratio_for_qjerk, double max_iter,
                 const size_t motion_plan_concerned_start_index,
                 const double concerned_start_q_jerk, const double ego_vel,
                 planning::common::LateralPlanningInput &planning_input);

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

}  // namespace lateral_planning
}  // namespace pnc
#endif
