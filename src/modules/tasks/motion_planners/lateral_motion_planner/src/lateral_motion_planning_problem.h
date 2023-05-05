#ifndef __LATERAL_MOTION_PLANNING_PROBLEM_H__
#define __LATERAL_MOTION_PLANNING_PROBLEM_H__

#include <cstddef>
#include <vector>

#include "ilqr_core.h"
#include "lateral_motion_planning_model.h"

namespace pnc {
namespace lateral_planning {

struct LateralMotionPlanningInput {
  // init state
  State init_state;

  // reference
  std::vector<double> ref_x_vec;
  std::vector<double> ref_y_vec;
  std::vector<double> ref_theta_vec;

  // last trajectory
  std::vector<double> last_x_vec;
  std::vector<double> last_y_vec;
  std::vector<double> last_theta_vec;

  // fixed reference
  double ref_vel = 0.0;
  double curv_factor = 0.0;

  // reference soft bound
  std::vector<double> soft_upper_bound_x0_vec;
  std::vector<double> soft_upper_bound_y0_vec;
  std::vector<double> soft_upper_bound_x1_vec;
  std::vector<double> soft_upper_bound_y1_vec;

  std::vector<double> soft_lower_bound_x0_vec;
  std::vector<double> soft_lower_bound_y0_vec;
  std::vector<double> soft_lower_bound_x1_vec;
  std::vector<double> soft_lower_bound_y1_vec;

  // lateral acc and omega bound
  double acc_bound = 0.0;
  double jerk_bound = 0.0;

  // weights
  double q_ref_x = 1.0;
  double q_ref_y = 1.0;
  double q_ref_theta = 1.0;

  double q_continuity = 0.0;
  double q_acc = 1.0;
  double q_jerk = 1.0;
  double q_snap = 1.0;

  double q_acc_bound = 1000.0;
  double q_jerk_bound = 1000.0;

  double q_soft_corridor = 0.0;
  double q_hard_corridor = 0.0;
};

struct LateralMotionPlanningOutput {
  std::vector<double> time_vec;
  std::vector<double> x_vec;
  std::vector<double> y_vec;
  std::vector<double> theta_vec;
  std::vector<double> delta_vec;
  std::vector<double> omega_vec;
  std::vector<double> omega_dot_vec;
  std::vector<double> acc_vec;
  std::vector<double> jerk_vec;
  const ilqr_solver::iLqr::iLqrSolverInfo *solver_info_ptr;
  ilqr_solver::iLqr::iLqrSolverInfo solver_info; // to be removed
};

class LateralMotionPlanningProblem {
public:
  void Init();
  uint8_t Update(LateralMotionPlanningInput &planning_input);
  void SetWarmStart(bool flag) { ilqr_core_ptr_->SetWarmStart(flag); };
  void SetMaxIter(size_t max_iter) { ilqr_core_ptr_->SetMaxIter(max_iter); };

  void Reset();

  const LateralMotionPlanningOutput GetOutput() const {
    return planning_output_;
  }

  const LateralMotionPlanningOutput *GetOutputPtr() const {
    return &planning_output_;
  }

private:
  std::shared_ptr<ilqr_solver::iLqr> ilqr_core_ptr_;
  LateralMotionPlanningOutput planning_output_;
};

} // namespace lateral_planning
} // namespace pnc
#endif
