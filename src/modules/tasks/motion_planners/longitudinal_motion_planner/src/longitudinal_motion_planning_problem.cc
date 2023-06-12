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
  solver_config.horizon = 25;
  solver_config.state_size = STATE_SIZE;
  solver_config.input_size = INPUT_SIZE;
  solver_config.model_dt = 0.2;
  solver_config.warm_start_enable = true;
  solver_config.du_tol = 0.001;
  solver_config.max_iter = 15;
  solver_config.lambda_min = 1e-5;
  init_state_.resize(STATE_SIZE);
  // STEP 1: init core with solver config
  ilqr_core_ptr_ = std::make_shared<iLqr>();
  ilqr_core_ptr_->Init(std::make_shared<LongitudinalMotionPlanningModel>(), solver_config);

  // STEP 2: add cost
  ilqr_core_ptr_->AddCost(std::make_shared<ReferenceCostTerm>());     // reference cost for s and v
  ilqr_core_ptr_->AddCost(std::make_shared<LonAccCostTerm>());        // longitudinal acc cost
  ilqr_core_ptr_->AddCost(std::make_shared<LonJerkCostTerm>());       // longitudinal jerk cost
  ilqr_core_ptr_->AddCost(std::make_shared<LonPosBoundCostTerm>());   // longitudinal pos bound cost
  ilqr_core_ptr_->AddCost(std::make_shared<LonVelBoundCostTerm>());   // longitudinal vel bound cost
  ilqr_core_ptr_->AddCost(std::make_shared<LonAccBoundCostTerm>());   // longitudinal acc bound cost
  ilqr_core_ptr_->AddCost(std::make_shared<LonJerkBoundCostTerm>());  // longitudinal jerk bound
                                                                      // cost
  ilqr_core_ptr_->AddCost(std::make_shared<LonStopPointCost>());      // longitudinal stop point cost
  ilqr_core_ptr_->AddCost(std::make_shared<LonSnapCostTerm>());       // longitudinal snap cost cost

  // STEP 3: init debug info, must run after add cost
  ilqr_core_ptr_->InitAdvancedInfo();
}

uint8_t LongitudinalMotionPlanningProblem::Update(planning::common::LongitudinalPlanningInput &planning_input) {
  // set cost config
  const size_t N = ilqr_core_ptr_->GetSolverConfigPtr()->horizon + 1;

  std::vector<IlqrCostConfig> cost_config_vec;
  cost_config_vec.resize(N);

  for (size_t i = 0; i < N; ++i) {
    // reference
    cost_config_vec.at(i)[REF_POS] = planning_input.ref_pos_vec(i);
    cost_config_vec.at(i)[REF_VEL] = planning_input.ref_vel_vec(i);
    cost_config_vec.at(i)[S_STOP] = planning_input.s_stop();

    // bounds
    cost_config_vec.at(i)[POS_MAX] = planning_input.pos_max_vec(i);
    cost_config_vec.at(i)[POS_MIN] = planning_input.pos_min_vec(i);
    cost_config_vec.at(i)[VEL_MAX] = planning_input.vel_max_vec(i);
    cost_config_vec.at(i)[VEL_MIN] = planning_input.vel_min_vec(i);
    cost_config_vec.at(i)[ACC_MAX] = planning_input.acc_max_vec(i);
    cost_config_vec.at(i)[ACC_MIN] = planning_input.acc_min_vec(i);
    cost_config_vec.at(i)[JERK_MAX] = planning_input.jerk_max_vec(i);
    cost_config_vec.at(i)[JERK_MIN] = planning_input.jerk_min_vec(i);

    // weights
    cost_config_vec.at(i)[W_REF_POS] = planning_input.q_ref_pos();
    cost_config_vec.at(i)[W_REF_VEL] = planning_input.q_ref_vel();
    cost_config_vec.at(i)[W_ACC] = planning_input.q_acc();
    cost_config_vec.at(i)[W_JERK] = planning_input.q_jerk();
    cost_config_vec.at(i)[W_SNAP] = planning_input.q_snap();
    cost_config_vec.at(i)[W_POS_BOUND] = planning_input.q_pos_bound();
    cost_config_vec.at(i)[W_VEL_BOUND] = planning_input.q_vel_bound();
    cost_config_vec.at(i)[W_ACC_BOUND] = planning_input.q_acc_bound();
    cost_config_vec.at(i)[W_JERK_BOUND] = planning_input.q_jerk_bound();
    cost_config_vec.at(i)[W_S_STOP] = planning_input.q_stop_s();

    if (i == N - 1) {
      cost_config_vec.at(i)[TERMINAL_FLAG] = 1;
      cost_config_vec.at(i)[W_S_STOP] = planning_input.q_stop_s();
    } else {
      cost_config_vec.at(i)[TERMINAL_FLAG] = 0;
    }
  }

  // set cost config
  ilqr_core_ptr_->SetCostConfig(cost_config_vec);

  // solve the ilqr problem
  init_state_ << planning_input.init_state().s(), planning_input.init_state().v(), planning_input.init_state().a(),
      planning_input.init_state().j();
  ilqr_core_ptr_->Solve(init_state_);

  return true;
}

}  // namespace longitudinal_planning
}  // namespace pnc