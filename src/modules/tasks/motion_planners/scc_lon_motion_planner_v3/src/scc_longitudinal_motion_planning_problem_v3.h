#ifndef __SCC_LONGITUDINAL_MOTION_PLANNING_PROBLEM_V3_H__
#define __SCC_LONGITUDINAL_MOTION_PLANNING_PROBLEM_V3_H__

#include <algorithm>
#include <cstddef>
#include <vector>

#include "al_ilqr_core.h"
#include "config/basic_type.h"
#include "longitudinal_motion_planner.pb.h"
#include "math_lib.h"
#include "scc_longitudinal_motion_planning_cost_v3.h"
#include "scc_longitudinal_motion_planning_model_v3.h"

namespace pnc {
namespace scc_longitudinal_planning_v3 {

class SccLonAliLqr : public al_ilqr_solver::AliLqr {
 public:
  double MaxConstraintViolation() override {
    double max_violation = 0.0;
    const size_t N = solver_config_ptr_->horizon + 1;
    for (size_t k = 0; k < N; ++k) {
      const double pos = xk_vec_[k][POS];
      const double vel = xk_vec_[k][VEL];
      const double hard_pos_max = cost_config_vec_ptr_->at(k)[HARD_POS_MAX];
      const double hard_pos_min = cost_config_vec_ptr_->at(k)[HARD_POS_MIN];
      const double safe_distance = cost_config_vec_ptr_->at(k)[SAFE_DISTANCE];
      const double hard_pos_max_safe =
          std::max(hard_pos_max - safe_distance, 0.0);
      const double c_pos_upper = pos - hard_pos_max_safe;
      max_violation = std::max(max_violation, std::max(0.0, c_pos_upper));
      const double c_pos_lower = hard_pos_min - pos;
      max_violation = std::max(max_violation, std::max(0.0, c_pos_lower));
      const double c_vel_lower = -vel;
      max_violation = std::max(max_violation, std::max(0.0, c_vel_lower));
    }
    return max_violation;
  }

  void UpdateAugmentedLagragian() override {
    const size_t N = solver_config_ptr_->horizon + 1;
    const double phi = solver_config_ptr_->penalty_scaling;
    const double rho_max = solver_config_ptr_->penalty_max;

    for (size_t k = 0; k < N; ++k) {
      auto &cfg = constraint_config_vec_ptr_->at(k);
      const double pos = xk_vec_[k][POS];
      const double vel = xk_vec_[k][VEL];
      const double hard_pos_max = cost_config_vec_ptr_->at(k)[HARD_POS_MAX];
      const double hard_pos_min = cost_config_vec_ptr_->at(k)[HARD_POS_MIN];
      const double safe_distance = cost_config_vec_ptr_->at(k)[SAFE_DISTANCE];

      const double hard_pos_max_safe =
          std::max(hard_pos_max - safe_distance, 0.0);
      const double c_pos_upper = pos - hard_pos_max_safe;
      cfg[MU_HARD_POS_UPPER] = std::max(
          0.0, cfg[MU_HARD_POS_UPPER] + cfg[RHO_HARD_POS_UPPER] * c_pos_upper);
      if (c_pos_upper > 0.0) {
        cfg[RHO_HARD_POS_UPPER] =
            std::min(cfg[RHO_HARD_POS_UPPER] * phi, rho_max);
      }

      const double c_pos_lower = hard_pos_min - pos;
      cfg[MU_HARD_POS_LOWER] = std::max(
          0.0, cfg[MU_HARD_POS_LOWER] + cfg[RHO_HARD_POS_LOWER] * c_pos_lower);
      if (c_pos_lower > 0.0) {
        cfg[RHO_HARD_POS_LOWER] =
            std::min(cfg[RHO_HARD_POS_LOWER] * phi, rho_max);
      }

      const double c_vel_lower = -vel;
      cfg[MU_HARD_VEL_LOWER] = std::max(
          0.0, cfg[MU_HARD_VEL_LOWER] + cfg[RHO_HARD_VEL_LOWER] * c_vel_lower);
      if (c_vel_lower > 0.0) {
        cfg[RHO_HARD_VEL_LOWER] =
            std::min(cfg[RHO_HARD_VEL_LOWER] * phi, rho_max);
      }
    }
  }
};

class SccLongitudinalMotionPlanningProblemV3 {
 public:
  void Init();
  uint8_t Update(planning::common::LongitudinalPlanningInput &planning_input);
  const planning::common::LongitudinalPlanningOutput &GetOutput() const {
    return planning_output_;
  }
  void Reset();

  void SetWarmStart(bool flag) { alilqr_core_ptr_->SetWarmStart(flag); }
  void SetMaxIter(size_t max_iter) { alilqr_core_ptr_->SetMaxIter(max_iter); }

  const std::shared_ptr<SccLonAliLqr> GetAliLqrCorePtr() const {
    return alilqr_core_ptr_;
  }

 private:
  std::shared_ptr<SccLonAliLqr> alilqr_core_ptr_;
  planning::common::LongitudinalPlanningOutput planning_output_;
  al_ilqr_solver::State init_state_;
};

}  // namespace scc_longitudinal_planning_v3
}  // namespace pnc
#endif
