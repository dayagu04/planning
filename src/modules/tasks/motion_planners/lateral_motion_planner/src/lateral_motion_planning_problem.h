#ifndef __LATERAL_MOTION_PLANNING_PROBLEM_H__
#define __LATERAL_MOTION_PLANNING_PROBLEM_H__

#include <cstddef>
#include <vector>

#include "ilqr_core.h"
#include "ilqr_define.h"
#include "lateral_motion_planner.pb.h"
#include "lateral_motion_planning_cost.h"
#include "lateral_motion_planning_model.h"
#include "spline.h"
namespace pnc {
namespace lateral_planning {
class LateralMotionPlanningProblem {
 public:
  void Init();

  uint8_t Update(planning::common::LateralPlanningInput &planning_input);

  uint8_t Update(planning::common::LateralPlanningInput &origin_planning_input,
                 const size_t motion_plan_concerned_start_index,
                 const double motion_plan_concerned_start_ratio,
                 const double motion_plan_concerned_end_ratio,
                 const double jerk_bound, const double decay_factor,
                 const double w_xy, const double w_theta,
                 const double new_dt);

  uint8_t Update(planning::common::LateralPlanningInput &origin_planning_input,
                 const size_t motion_plan_concerned_start_index,
                 const double motion_plan_concerned_start_ratio,
                 const double motion_plan_concerned_end_ratio,
                 const double jerk_bound, const double decay_factor,
                 const double w_xy, const double w_theta,
                 std::vector<double>& new_dt, std::vector<double>& new_ref_vel);

  const planning::common::LateralPlanningOutput &GetOutput() {
    return planning_output_;
  }
  void Reset();

  void SetWarmStart(bool flag) { ilqr_core_ptr_->SetWarmStart(flag); }
  
  void SetMaxIter(size_t max_iter) { ilqr_core_ptr_->SetMaxIter(max_iter); }

  const std::shared_ptr<ilqr_solver::iLqr> GetiLqrCorePtr() const {
    return ilqr_core_ptr_;
  }

  planning::common::LateralPlanningInput SplineInput(
    planning::common::LateralPlanningInput &planning_input, const double dt);

  planning::common::LateralPlanningInput SplineInput(
    planning::common::LateralPlanningInput &planning_input, std::vector<double>& dt);

 private:
  std::shared_ptr<ilqr_solver::iLqr> ilqr_core_ptr_;
  planning::common::LateralPlanningOutput planning_output_;
  ilqr_solver::State init_state_;

  pnc::mathlib::spline x_t_spline_;
  pnc::mathlib::spline y_t_spline_;
  pnc::mathlib::spline theta_t_spline_;
  pnc::mathlib::spline soft_upper_bound_t_spline_;
  pnc::mathlib::spline soft_lower_bound_t_spline_;
  pnc::mathlib::spline hard_lower_bound_t_spline_;
  pnc::mathlib::spline hard_upper_bound_t_spline_;

  std::vector<double> t_vec_;
  std::vector<double> x_vec_;
  std::vector<double> y_vec_;
  std::vector<double> theta_vec_;
  std::vector<double> soft_upper_bound_vec_;
  std::vector<double> soft_lower_bound_vec_;
  std::vector<double> hard_upper_bound_vec_;
  std::vector<double> hard_lower_bound_vec_;
};

}  // namespace lateral_planning
}  // namespace pnc
#endif
