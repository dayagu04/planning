#include "longitudinal_motion_planning_problem.h"

#include <stdio.h>

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>

#include "longitudinal_motion_planning_cost.h"
#include "math_lib.h"

using namespace ilqr_solver;
using namespace pnc::mathlib;

namespace pnc {
namespace longitudinal_planning {
void LongitudinalMotionPlanningProblem::Init() {
  // STEP 0: set solver config parmeters
  iLqrSolverConfig solver_config;
  solver_config.horizion = 40;
  solver_config.state_size = STATE_SIZE;
  solver_config.input_size = INPUT_SIZE;
  solver_config.model_dt = 0.1;
  solver_config.warm_start_enable = true;
  solver_config.du_tol = 0.001;
  solver_config.max_iter = 50;
  solver_config.lambda_min = 1e-5;

  // STEP 1: init core with solver config
  ilqr_core_ptr_ = std::make_shared<iLqr>();
  ilqr_core_ptr_->Init(std::make_shared<LongitudinalMotionPlanningModel>(),
                       solver_config);

  // STEP 2: add cost
  ilqr_core_ptr_->AddCost(
      std::make_shared<ReferenceCostTerm>());  // reference cost for s and v
  ilqr_core_ptr_->AddCost(
      std::make_shared<LonAccCostTerm>());  // longitudinal acc cost
  ilqr_core_ptr_->AddCost(
      std::make_shared<LonJerkCostTerm>());  // longitudinal jerk cost
  ilqr_core_ptr_->AddCost(
      std::make_shared<LonPosBoundCostTerm>());  // longitudinal pos bound cost
  ilqr_core_ptr_->AddCost(
      std::make_shared<LonVelBoundCostTerm>());  // longitudinal vel bound cost
  ilqr_core_ptr_->AddCost(
      std::make_shared<LonAccBoundCostTerm>());  // longitudinal acc bound cost
  ilqr_core_ptr_->AddCost(
      std::make_shared<LonJerkBoundCostTerm>());  // longitudinal jerk bound
                                                  // cost
  ilqr_core_ptr_->AddCost(
      std::make_shared<LonStopPointCost>());  // longitudinal stop point cost
  ilqr_core_ptr_->AddCost(
      std::make_shared<LonSnapCostTerm>());  // longitudinal snap cost cost

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
  planning_output_.pos_vec.resize(N, 0.0);
  planning_output_.vel_vec.resize(N, 0.0);
  planning_output_.acc_vec.resize(N, 0.0);
  planning_output_.jerk_vec.resize(N, 0.0);
}

uint8_t LongitudinalMotionPlanningProblem::Update(
    LongitudinalMotionPlanningInput &planning_input) {
  // set cost config
  const size_t N = ilqr_core_ptr_->GetSolverConfigPtr()->horizion + 1;

  std::vector<IlqrCostConfig> cost_config_vec;
  cost_config_vec.resize(N);

  for (size_t i = 0; i < N; ++i) {
    // reference
    cost_config_vec.at(i)[REF_POS] = planning_input.ref_pos_vec[i];
    cost_config_vec.at(i)[REF_VEL] = planning_input.ref_vel_vec[i];
    cost_config_vec.at(i)[S_STOP] = planning_input.s_stop;

    // bounds
    cost_config_vec.at(i)[POS_MAX] = planning_input.pos_max_vec[i];
    cost_config_vec.at(i)[POS_MIN] = planning_input.pos_min_vec[i];
    cost_config_vec.at(i)[VEL_MAX] = planning_input.vel_max_vec[i];
    cost_config_vec.at(i)[VEL_MIN] = planning_input.vel_min_vec[i];
    cost_config_vec.at(i)[ACC_MAX] = planning_input.acc_max_vec[i];
    cost_config_vec.at(i)[ACC_MIN] = planning_input.acc_min_vec[i];
    cost_config_vec.at(i)[JERK_MAX] = planning_input.jerk_max_vec[i];
    cost_config_vec.at(i)[JERK_MIN] = planning_input.jerk_min_vec[i];

    // weights
    cost_config_vec.at(i)[W_REF_POS] = planning_input.q_ref_pos;
    cost_config_vec.at(i)[W_REF_VEL] = planning_input.q_ref_vel;
    cost_config_vec.at(i)[W_ACC] = planning_input.q_acc;
    cost_config_vec.at(i)[W_JERK] = planning_input.q_jerk;
    cost_config_vec.at(i)[W_SNAP] = planning_input.q_snap;
    cost_config_vec.at(i)[W_POS_BOUND] = planning_input.q_pos_bound;
    cost_config_vec.at(i)[W_VEL_BOUND] = planning_input.q_vel_bound;
    cost_config_vec.at(i)[W_ACC_BOUND] = planning_input.q_acc_bound;
    cost_config_vec.at(i)[W_JERK_BOUND] = planning_input.q_jerk_bound;
    cost_config_vec.at(i)[W_S_STOP] = planning_input.q_stop_s;

    if (i == N - 1) {
      cost_config_vec.at(i)[TERMINAL_FLAG] = 1;
      cost_config_vec.at(i)[W_S_STOP] = planning_input.q_stop_s;
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
  // auto const &control_result = ilqr_core_ptr_->GetControlResultPtr();

  for (size_t i = 0; i < N; ++i) {
    planning_output_.pos_vec[i] = state_result->at(i)[POS];
    planning_output_.vel_vec[i] = state_result->at(i)[VEL];
    planning_output_.acc_vec[i] = state_result->at(i)[ACC];
    planning_output_.jerk_vec[i] = state_result->at(i)[JERK];
  }

  return ilqr_core_ptr_->GetSolverInfoPtr()->solver_condition;
}

}  // namespace longitudinal_planning
}  // namespace pnc