#pragma once

#include <cstddef>
#include <vector>

#include "ilqr_core.h"
#include "ilqr_define.h"
#include "lateral_motion_planner.pb.h"
#include "../dynamic_model/dynamic_model.h"
#include "../weight/base_weight.h"

namespace pnc {
namespace lateral_planning {

class iLQRSolver {
 public:
  iLQRSolver();

  ~iLQRSolver() = default;

  void Init(
      const ilqr_solver::iLqrSolverConfig& solver_config,
      const std::shared_ptr<DynamicModel> dynamic_model);

  uint8_t Update(
      const double end_ratio_for_qxy, const double end_ratio_for_qtheta,
      const double end_ratio_for_qjerk, const double concerned_start_q_jerk,
      const double wheel_base, const std::vector<double>& virtual_ref_x,
      const std::vector<double>& virtual_ref_y,
      const std::vector<double>& virtual_ref_theta,
      const std::shared_ptr<pnc::lateral_planning::BaseWeight>& planning_weight,
      planning::common::LateralPlanningInput& planning_input);

  // simulation
  void SimInit();

  uint8_t SimUpdate(double expected_acc, double start_acc, double end_acc,
                    double end_ratio_for_qxy, double end_ratio_for_qtheta,
                    double end_ratio_for_qjerk, double max_iter,
                    const size_t motion_plan_concerned_start_index,
                    const double concerned_start_q_jerk, const double ego_vel,
                    double max_delta, double max_omega,
                    const double wheel_base, const double q_front_xy,
                    double q_virtual_ref_xy, double q_virtual_ref_theta,
                    std::vector<double>& virtual_ref_x,
                    std::vector<double>& virtual_ref_y,
                    std::vector<double>& virtual_ref_theta,
                    planning::common::LateralPlanningInput& planning_input);

  const std::shared_ptr<ilqr_solver::iLqr> GetiLqrCorePtr() const {
    return ilqr_core_ptr_;
  }

 private:
  std::shared_ptr<ilqr_solver::iLqr> ilqr_core_ptr_;
  ilqr_solver::State init_state_;
};

}  // namespace lateral_planning
}  // namespace pnc
