#include "lateral_motion_planning_problem.h"

#include <stdio.h>

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>

using namespace ilqr_solver;
using namespace pnc::mathlib;

static const double kMaxWheelAngle = 360.0 / 14.5 / 57.3;      // 360 deg steering angle for scc
static const double kMaxWheelAngleRate = 240.0 / 14.5 / 57.3;  // 240 deg/s steering angle rate for scc
namespace pnc {
namespace lateral_planning {
void LateralMotionPlanningProblem::Init() {
  // STEP 0: set solver config parmeters
  iLqrSolverConfig solver_config;
  solver_config.horizon = 25;
  solver_config.state_size = STATE_SIZE;
  solver_config.input_size = INPUT_SIZE;
  solver_config.model_dt = 0.2;
  solver_config.warm_start_enable = true;
  solver_config.du_tol = 0.01 / 57.3 / 14.5;
  init_state_.resize(STATE_SIZE);
  // STEP 1: init core with solver config
  ilqr_core_ptr_ = std::make_shared<iLqr>();
  ilqr_core_ptr_->Init(std::make_shared<LateralMotionPlanningModel>(), solver_config);

  // STEP 2: add cost
  ilqr_core_ptr_->AddCost(std::make_shared<ReferenceCostTerm>());  // reference cost
  // ilqr_core_ptr_->AddCost(
  //     std::make_shared<ContinuityCostTerm>()); // continuity cost
  ilqr_core_ptr_->AddCost(std::make_shared<LatAccCostTerm>());        // lateral acc cost
  ilqr_core_ptr_->AddCost(std::make_shared<LatJerkCostTerm>());       // lateral jerk cost
  ilqr_core_ptr_->AddCost(std::make_shared<LatAccBoundCostTerm>());   // lateral acc bound cost
  ilqr_core_ptr_->AddCost(std::make_shared<LatJerkBoundCostTerm>());  // lateral jerk bound cost
  // ilqr_core_ptr_->AddCost(
  //     std::make_shared<PathSoftCorridorCostTerm>()); // path soft corridor
  //                                                    // cost
  ilqr_core_ptr_->AddCost(std::make_shared<LatSnapCostTerm>());  // omega dot cost term

  // STEP 3: init debug info, must run after add cost
  ilqr_core_ptr_->InitAdvancedInfo();
}

uint8_t LateralMotionPlanningProblem::Update(planning::common::LateralPlanningInput &planning_input) {
  // set cost config
  const size_t N = ilqr_core_ptr_->GetSolverConfigPtr()->horizon + 1;

  std::vector<IlqrCostConfig> cost_config_vec;
  cost_config_vec.resize(N);

  // calculate delta_bound and omega_bound
  double delta_bound = std::min(
      kMaxWheelAngle, planning_input.acc_bound() / (planning_input.curv_factor() * Square(planning_input.ref_vel())));
  double omega_bound = std::min(kMaxWheelAngleRate, planning_input.jerk_bound() / (planning_input.curv_factor() *
                                                                                   Square(planning_input.ref_vel())));

  for (size_t i = 0; i < N; ++i) {
    // // reference
    cost_config_vec.at(i)[REF_X] = planning_input.ref_x_vec(i);
    cost_config_vec.at(i)[REF_Y] = planning_input.ref_y_vec(i);
    cost_config_vec.at(i)[REF_THETA] = planning_input.ref_theta_vec(i);
    cost_config_vec.at(i)[REF_VEL] = planning_input.ref_vel();
    cost_config_vec.at(i)[CURV_FACTOR] = planning_input.curv_factor();

    cost_config_vec.at(i)[CONTINUITY_X] = planning_input.last_x_vec(i);
    cost_config_vec.at(i)[CONTINUITY_Y] = planning_input.last_y_vec(i);
    cost_config_vec.at(i)[CONTINUITY_THETA] = planning_input.last_theta_vec(i);

    // bounds
    cost_config_vec.at(i)[DELTA_BOUND] = delta_bound;
    cost_config_vec.at(i)[OMEGA_BOUND] = omega_bound;

    cost_config_vec.at(i)[SOFT_UPPER_BOUND_X0] = planning_input.soft_upper_bound_x0_vec(i);
    cost_config_vec.at(i)[SOFT_UPPER_BOUND_Y0] = planning_input.soft_upper_bound_y0_vec(i);
    cost_config_vec.at(i)[SOFT_UPPER_BOUND_X1] = planning_input.soft_upper_bound_x1_vec(i);
    cost_config_vec.at(i)[SOFT_UPPER_BOUND_Y1] = planning_input.soft_upper_bound_y1_vec(i);

    cost_config_vec.at(i)[SOFT_LOWER_BOUND_X0] = planning_input.soft_lower_bound_x0_vec(i);
    cost_config_vec.at(i)[SOFT_LOWER_BOUND_Y0] = planning_input.soft_lower_bound_y0_vec(i);
    cost_config_vec.at(i)[SOFT_LOWER_BOUND_X1] = planning_input.soft_lower_bound_x1_vec(i);
    cost_config_vec.at(i)[SOFT_LOWER_BOUND_Y1] = planning_input.soft_lower_bound_y1_vec(i);

    // weights
    cost_config_vec.at(i)[W_REF_X] = planning_input.q_ref_x();
    cost_config_vec.at(i)[W_REF_Y] = planning_input.q_ref_y();
    cost_config_vec.at(i)[W_REF_THETA] = planning_input.q_ref_theta();
    cost_config_vec.at(i)[W_CONTINUITY_X] = planning_input.q_ref_x() * planning_input.q_continuity();
    cost_config_vec.at(i)[W_CONTINUITY_Y] = planning_input.q_ref_y() * planning_input.q_continuity();
    cost_config_vec.at(i)[W_CONTINUITY_THETA] = planning_input.q_ref_theta() * planning_input.q_continuity();

    cost_config_vec.at(i)[W_ACC] = planning_input.q_acc();
    cost_config_vec.at(i)[W_JERK] = planning_input.q_jerk();
    cost_config_vec.at(i)[W_ACC_BOUND] = planning_input.q_acc_bound();
    cost_config_vec.at(i)[W_JERK_BOUND] = planning_input.q_jerk_bound();
    cost_config_vec.at(i)[W_SNAP] = planning_input.q_snap();

    cost_config_vec.at(i)[W_SOFT_CORRIDOR] = planning_input.q_soft_corridor();
    cost_config_vec.at(i)[W_HARD_CORRIDOR] = planning_input.q_hard_corridor();

    if (i == N - 1) {
      cost_config_vec.at(i)[TERMINAL_FLAG] = 1;
      cost_config_vec.at(i)[W_REF_X] *= 10.0;
      cost_config_vec.at(i)[W_REF_Y] *= 10.0;
      cost_config_vec.at(i)[W_REF_THETA] *= 10.0;
      cost_config_vec.at(i)[W_SOFT_CORRIDOR] *= 10.0;
      cost_config_vec.at(i)[W_HARD_CORRIDOR] *= 10.0;
    } else {
      cost_config_vec.at(i)[TERMINAL_FLAG] = 0;
    }
  }

  // set const config
  ilqr_core_ptr_->SetCostConfig(cost_config_vec);

  // solve the ilqr problem
  init_state_ << planning_input.init_state().x(), planning_input.init_state().y(), planning_input.init_state().theta(),
      planning_input.init_state().delta(), planning_input.init_state().omega();

  ilqr_core_ptr_->Solve(init_state_);

  return true;
}

}  // namespace lateral_planning
}  // namespace pnc