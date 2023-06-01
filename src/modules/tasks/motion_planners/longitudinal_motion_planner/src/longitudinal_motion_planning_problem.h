#ifndef __LONGITUDINAL_MOTION_PLANNING_PROBLEM_H__
#define __LONGITUDINAL_MOTION_PLANNING_PROBLEM_H__

#include <cstddef>
#include <vector>

#include "common/config/basic_type.h"
#include "ilqr_core.h"
#include "longitudinal_motion_planner.pb.h"
#include "longitudinal_motion_planning_cost.h"
#include "longitudinal_motion_planning_model.h"
#include "math_lib.h"

namespace pnc {
namespace longitudinal_planning {

struct LongitudinalMotionPlanningInput {
  // init state
  State init_state;  // p v a

  // reference
  std::vector<double> ref_pos_vec;
  std::vector<double> ref_vel_vec;

  // longitudinal bound
  std::vector<double> pos_max_vec;
  std::vector<double> pos_min_vec;

  std::vector<double> vel_max_vec;
  std::vector<double> vel_min_vec;

  std::vector<double> acc_max_vec;
  std::vector<double> acc_min_vec;

  std::vector<double> jerk_max_vec;
  std::vector<double> jerk_min_vec;

  // s stop
  double s_stop = 15.0;

  // weights
  double q_ref_pos = 2.0;
  double q_ref_vel = 2.0;

  double q_acc = 2.0;
  double q_jerk = 2.0;
  double q_snap = 0.7;

  double q_pos_bound = 500.0;
  double q_vel_bound = 500.0;
  double q_acc_bound = 500.0;
  double q_jerk_bound = 500.0;

  double q_stop_s = 500.0;
};

struct LongitudinalMotionPlanningOutput {
  std::vector<double> time_vec;
  std::vector<double> pos_vec;
  std::vector<double> vel_vec;
  std::vector<double> acc_vec;
  std::vector<double> jerk_vec;
  const ilqr_solver::iLqr::iLqrSolverInfo *solver_info_ptr;
  ilqr_solver::iLqr::iLqrSolverInfo solver_info;  // to be removed
};

class LongitudinalMotionPlanningProblem {
 public:
  void Init();
  uint8_t Update(planning::common::LongitudinalPlanningInput &planning_input);
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

}  // namespace longitudinal_planning
}  // namespace pnc
#endif
