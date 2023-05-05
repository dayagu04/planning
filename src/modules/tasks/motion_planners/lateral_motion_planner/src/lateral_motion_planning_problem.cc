#include "lateral_motion_planning_problem.h"
#include "lateral_motion_planning_cost.h"
#include "math_lib.h"
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <stdio.h>

using namespace ilqr_solver;
using namespace pnc::mathlib;

static const double kMaxWheelAngle =
    360.0 / 14.5 / 57.3; // 360 deg steering angle for scc
static const double kMaxWheelAngleRate =
    240.0 / 14.5 / 57.3; // 240 deg/s steering angle rate for scc
namespace pnc {
namespace lateral_planning {
void LateralMotionPlanningProblem::Init() {
  // STEP 0: set solver config parmeters
  iLqrSolverConfig solver_config;
  solver_config.horizion = 25;
  solver_config.state_size = STATE_SIZE;
  solver_config.input_size = INPUT_SIZE;
  solver_config.model_dt = 0.1;
  solver_config.warm_start_enable = true;
  solver_config.du_tol = 0.01 / 57.3 / 14.5;

  // STEP 1: init core with solver config
  ilqr_core_ptr_ = std::make_shared<iLqr>();
  ilqr_core_ptr_->Init(std::make_shared<LateralMotionPlanningModel>(),
                       solver_config);

  // STEP 2: add cost
  ilqr_core_ptr_->AddCost(
      std::make_shared<ReferenceCostTerm>()); // reference cost
  ilqr_core_ptr_->AddCost(
      std::make_shared<ContinuityCostTerm>()); // continuity cost
  ilqr_core_ptr_->AddCost(
      std::make_shared<LatAccCostTerm>()); // lateral acc cost
  ilqr_core_ptr_->AddCost(
      std::make_shared<LatJerkCostTerm>()); // lateral jerk cost
  ilqr_core_ptr_->AddCost(
      std::make_shared<LatAccBoundCostTerm>()); // lateral acc bound cost
  ilqr_core_ptr_->AddCost(
      std::make_shared<LatJerkBoundCostTerm>()); // lateral jerk bound cost
  ilqr_core_ptr_->AddCost(
      std::make_shared<PathSoftCorridorCostTerm>()); // path soft corridor
                                                     // cost
  ilqr_core_ptr_->AddCost(
      std::make_shared<LatSnapCostTerm>()); // omega dot cost term

  // STEP 3: init debug info, must run after add cost
  ilqr_core_ptr_->InitAdvancedInfo();

  // STEP 4: init some vars
  // set time vec once
  const size_t N = ilqr_core_ptr_->GetSolverConfigPtr()->horizion + 1;
  const double dt = ilqr_core_ptr_->GetSolverConfigPtr()->model_dt;

  planning_output_.time_vec.resize(N, 0.0);
  double time = 0.0;
  for (size_t i = 0; i < N; ++i) {
    planning_output_.time_vec[i] = time;
    time += dt;
  }

  // init other output
  planning_output_.x_vec.resize(N, 0.0);
  planning_output_.y_vec.resize(N, 0.0);
  planning_output_.theta_vec.resize(N, 0.0);
  planning_output_.delta_vec.resize(N, 0.0);
  planning_output_.acc_vec.resize(N, 0.0);
  planning_output_.jerk_vec.resize(N, 0.0);

  // note that omega(control input) is actually size = N - 1
  planning_output_.omega_vec.resize(N, 0.0);
  planning_output_.omega_dot_vec.resize(N, 0.);
}

uint8_t LateralMotionPlanningProblem::Update(
    LateralMotionPlanningInput &planning_input) {
  // set cost config
  const size_t N = ilqr_core_ptr_->GetSolverConfigPtr()->horizion + 1;

  std::vector<IlqrCostConfig> cost_config_vec;
  cost_config_vec.resize(N);

  // calculate delta_bound and omega_bound
  double delta_bound =
      std::min(kMaxWheelAngle, planning_input.acc_bound /
                                   (planning_input.curv_factor *
                                    mathlib::Square(planning_input.ref_vel)));
  double omega_bound = std::min(kMaxWheelAngleRate,
                                planning_input.jerk_bound /
                                    (planning_input.curv_factor *
                                     mathlib::Square(planning_input.ref_vel)));

  for (size_t i = 0; i < N; ++i) {
    // // reference
    cost_config_vec.at(i)[REF_X] = planning_input.ref_x_vec[i];
    cost_config_vec.at(i)[REF_Y] = planning_input.ref_y_vec[i];
    cost_config_vec.at(i)[REF_THETA] = planning_input.ref_theta_vec[i];
    cost_config_vec.at(i)[REF_VEL] = planning_input.ref_vel;
    cost_config_vec.at(i)[CURV_FACTOR] = planning_input.curv_factor;

    cost_config_vec.at(i)[CONTINUITY_X] = planning_input.last_x_vec[i];
    cost_config_vec.at(i)[CONTINUITY_Y] = planning_input.last_y_vec[i];
    cost_config_vec.at(i)[CONTINUITY_THETA] = planning_input.last_theta_vec[i];

    // bounds
    cost_config_vec.at(i)[DELTA_BOUND] = delta_bound;
    cost_config_vec.at(i)[OMEGA_BOUND] = omega_bound;

    cost_config_vec.at(i)[SOFT_UPPER_BOUND_X0] =
        planning_input.soft_upper_bound_x0_vec[i];
    cost_config_vec.at(i)[SOFT_UPPER_BOUND_Y0] =
        planning_input.soft_upper_bound_y0_vec[i];
    cost_config_vec.at(i)[SOFT_UPPER_BOUND_X1] =
        planning_input.soft_upper_bound_x1_vec[i];
    cost_config_vec.at(i)[SOFT_UPPER_BOUND_Y1] =
        planning_input.soft_upper_bound_y1_vec[i];

    cost_config_vec.at(i)[SOFT_LOWER_BOUND_X0] =
        planning_input.soft_lower_bound_x0_vec[i];
    cost_config_vec.at(i)[SOFT_LOWER_BOUND_Y0] =
        planning_input.soft_lower_bound_y0_vec[i];
    cost_config_vec.at(i)[SOFT_LOWER_BOUND_X1] =
        planning_input.soft_lower_bound_x1_vec[i];
    cost_config_vec.at(i)[SOFT_LOWER_BOUND_Y1] =
        planning_input.soft_lower_bound_y1_vec[i];

    // weights
    cost_config_vec.at(i)[W_REF_X] = planning_input.q_ref_x;
    cost_config_vec.at(i)[W_REF_Y] = planning_input.q_ref_y;
    cost_config_vec.at(i)[W_REF_THETA] = planning_input.q_ref_theta;
    cost_config_vec.at(i)[W_CONTINUITY_X] =
        planning_input.q_ref_x * planning_input.q_continuity;
    cost_config_vec.at(i)[W_CONTINUITY_Y] =
        planning_input.q_ref_y * planning_input.q_continuity;
    cost_config_vec.at(i)[W_CONTINUITY_THETA] =
        planning_input.q_ref_theta * planning_input.q_continuity;

    cost_config_vec.at(i)[W_ACC] = planning_input.q_acc;
    cost_config_vec.at(i)[W_JERK] = planning_input.q_jerk;
    cost_config_vec.at(i)[W_ACC_BOUND] = planning_input.q_acc_bound;
    cost_config_vec.at(i)[W_JERK_BOUND] = planning_input.q_jerk_bound;
    cost_config_vec.at(i)[W_SNAP] = planning_input.q_snap;

    cost_config_vec.at(i)[W_SOFT_CORRIDOR] = planning_input.q_soft_corridor;
    cost_config_vec.at(i)[W_HARD_CORRIDOR] = planning_input.q_hard_corridor;

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

  // set cost config
  ilqr_core_ptr_->SetCostConfig(cost_config_vec);

  // solve the ilqr problem
  ilqr_core_ptr_->Solve(planning_input.init_state);

  // get solver info
  planning_output_.solver_info_ptr = ilqr_core_ptr_->GetSolverInfoPtr();
  planning_output_.solver_info = *ilqr_core_ptr_->GetSolverInfoPtr();

  // assembling planning output
  auto const &state_result = ilqr_core_ptr_->GetStateResultPtr();
  auto const &control_result = ilqr_core_ptr_->GetControlResultPtr();

  for (size_t i = 0; i < N; ++i) {
    planning_output_.x_vec[i] = state_result->at(i)[StateId::X];
    planning_output_.y_vec[i] = state_result->at(i)[StateId::Y];
    planning_output_.theta_vec[i] = state_result->at(i)[StateId::THETA];
    planning_output_.delta_vec[i] = state_result->at(i)[StateId::DELTA];
    planning_output_.omega_vec[i] = state_result->at(i)[StateId::OMEGA];
    planning_output_.acc_vec[i] = planning_input.curv_factor *
                                  Square(planning_input.ref_vel) *
                                  (planning_output_.delta_vec[i]);
    planning_output_.jerk_vec[i] = planning_input.curv_factor *
                                   Square(planning_input.ref_vel) *
                                   (planning_output_.omega_vec[i]);

    if (i < N - 1) {
      planning_output_.omega_dot_vec[i] =
          control_result->at(i)[ControlId::OMEGA_DOT];
    } else {
      planning_output_.omega_dot_vec[i] = planning_output_.omega_dot_vec[i - 1];
    };
  }

  return ilqr_core_ptr_->GetSolverInfoPtr()->solver_condition;
}

} // namespace lateral_planning
} // namespace pnc