#include "scc_longitudinal_motion_planning_problem_v3.h"

#include <stdio.h>

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>

#include "math_lib.h"
#include "scc_longitudinal_motion_planning_cost_v3.h"

using namespace ilqr_solver;
using namespace pnc::mathlib;

namespace pnc {
namespace scc_longitudinal_planning_v3 {

void SccLongitudinalMotionPlanningProblemV3::Init() {
  // STEP 0: set solver config parmeters
  iLqrSolverConfig solver_config;
  solver_config.horizon = 25;
  solver_config.state_size = STATE_SIZE;
  solver_config.input_size = INPUT_SIZE;
  solver_config.model_dt = 0.2;
  solver_config.warm_start_enable = false;
  solver_config.du_tol = 0.005;
  solver_config.max_iter = 15;
  solver_config.lambda_min = 1e-5;
  init_state_.resize(STATE_SIZE);
  // STEP 1: init core with solver config
  ilqr_core_ptr_ = std::make_shared<iLqr>();
  ilqr_core_ptr_->Init(std::make_shared<SccLongitudinalMotionPlanningModelV3>(),
                       solver_config);

  // STEP 2: add cost
  ilqr_core_ptr_->AddCost(
      std::make_shared<ReferenceCostTerm>());  // reference cost for s and v
  ilqr_core_ptr_->AddCost(
      std::make_shared<LonAccCostTerm>());  // longitudinal acc cost
  ilqr_core_ptr_->AddCost(
      std::make_shared<LonJerkCostTerm>());  // longitudinal jerk cost
                                             //   ilqr_core_ptr_->AddCost(
  //       std::make_shared<LonSoftPosBoundCostTerm>());  // longitudinal soft
  //       pos
  //                                                      //  bound cost
  ilqr_core_ptr_->AddCost(
      std::make_shared<LonHardPosBoundCostTerm>());  // longitudinal hard pos
                                                     // bound cost
  //   ilqr_core_ptr_->AddCost(std::make_shared<LonSVBoundCost>());  //
  //   longitudinal
  //                                                                 // sv bound
  //                                                                 cost
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
      std::make_shared<NonNegativeVelCost>());  // longitudinal non-negative vel

  // STEP 3: init debug info, must run after add cost
  ilqr_core_ptr_->InitAdvancedInfo();

  // init planning output
  const auto &N = ilqr_core_ptr_->GetSolverConfigPtr()->horizon + 1;
  planning_output_.mutable_time_vec()->Resize(N, 0.0);
  planning_output_.mutable_pos_vec()->Resize(N, 0.0);
  planning_output_.mutable_vel_vec()->Resize(N, 0.0);
  planning_output_.mutable_acc_vec()->Resize(N, 0.0);
  planning_output_.mutable_jerk_vec()->Resize(N, 0.0);
}

uint8_t SccLongitudinalMotionPlanningProblemV3::Update(
    planning::common::LongitudinalPlanningInput &planning_input) {
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
    // cost_config_vec.at(i)[SOFT_POS_MAX] = planning_input.soft_pos_max_vec(i);
    // cost_config_vec.at(i)[SOFT_POS_MIN] = planning_input.soft_pos_min_vec(i);
    cost_config_vec.at(i)[HARD_POS_MAX] = planning_input.hard_pos_max_vec(i);
    cost_config_vec.at(i)[HARD_POS_MIN] = planning_input.hard_pos_min_vec(i);
    cost_config_vec.at(i)[VEL_MAX] = planning_input.vel_max_vec(i);
    cost_config_vec.at(i)[VEL_MIN] = planning_input.vel_min_vec(i);
    cost_config_vec.at(i)[ACC_MAX] = planning_input.acc_max_vec(i);
    cost_config_vec.at(i)[ACC_MIN] = planning_input.acc_min_vec(i);
    cost_config_vec.at(i)[JERK_MAX] = planning_input.jerk_max_vec(i);
    cost_config_vec.at(i)[JERK_MIN] = planning_input.jerk_min_vec(i);

    // bounds: s-v bounds
    // cost_config_vec.at(i)[SV_BOUND_S_0] = planning_input.sv_bound_s_0(i);
    // cost_config_vec.at(i)[SV_BOUND_S_1] = planning_input.sv_bound_s_1(i);
    // cost_config_vec.at(i)[SV_BOUND_S_2] = planning_input.sv_bound_s_2(i);
    // cost_config_vec.at(i)[SV_BOUND_S_3] = planning_input.sv_bound_s_3(i);
    // cost_config_vec.at(i)[SV_BOUND_S_4] = planning_input.sv_bound_s_4(i);
    // cost_config_vec.at(i)[SV_BOUND_S_5] = planning_input.sv_bound_s_5(i);
    // cost_config_vec.at(i)[SV_BOUND_V_MAX_0] = planning_input.sv_bound_v_0(i);
    // cost_config_vec.at(i)[SV_BOUND_V_MAX_1] = planning_input.sv_bound_v_1(i);
    // cost_config_vec.at(i)[SV_BOUND_V_MAX_2] = planning_input.sv_bound_v_2(i);
    // cost_config_vec.at(i)[SV_BOUND_V_MAX_3] = planning_input.sv_bound_v_3(i);
    // cost_config_vec.at(i)[SV_BOUND_V_MAX_4] = planning_input.sv_bound_v_4(i);
    // cost_config_vec.at(i)[SV_BOUND_V_MAX_5] = planning_input.sv_bound_v_5(i);

    // weights
    // cost_config_vec.at(i)[W_REF_POS] = planning_input.s_weights(i); //
    // dynamic weight
    cost_config_vec.at(i)[W_REF_POS] =
        planning_input.q_ref_pos();  // fixed weight
    cost_config_vec.at(i)[W_REF_VEL] = planning_input.v_weights(i);
    cost_config_vec.at(i)[W_ACC] = planning_input.q_acc();
    cost_config_vec.at(i)[W_JERK] = planning_input.q_jerk();
    cost_config_vec.at(i)[W_SNAP] = planning_input.q_snap();
    cost_config_vec.at(i)[W_POS_BOUND] = planning_input.q_soft_pos_bound();
    cost_config_vec.at(i)[W_VEL_BOUND] = planning_input.q_vel_bound();
    cost_config_vec.at(i)[W_ACC_BOUND] = planning_input.q_acc_bound();
    cost_config_vec.at(i)[W_JERK_BOUND] = planning_input.q_jerk_bound();
    cost_config_vec.at(i)[W_S_STOP] = planning_input.q_stop_s();
    cost_config_vec.at(i)[W_HARD_POS_BOUND] = planning_input.q_hard_pos_bound();
    cost_config_vec.at(i)[W_SV_BOUND] = planning_input.q_sv_bound();

    if (i <= 2) {
      cost_config_vec.at(i)[W_HARD_POS_BOUND] = 0.0;
      cost_config_vec.at(i)[W_SV_BOUND] = 0.0;
    }

    cost_config_vec.at(i)[W_NON_NEGATIVE_VEL] = 2000.0;

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
  init_state_ << planning_input.init_state().s(),
      planning_input.init_state().v(), planning_input.init_state().a();
  ilqr_core_ptr_->Solve(init_state_);

  // assemble planning result
  const auto &state_result = ilqr_core_ptr_->GetStateResultPtr();
  const auto &control_result = ilqr_core_ptr_->GetControlResultPtr();
  const auto &dt = ilqr_core_ptr_->GetSolverConfigPtr()->model_dt;

  double t = 0.0;
  for (size_t i = 0; i < N; ++i) {
    planning_output_.mutable_time_vec()->Set(i, t);
    t += dt;

    auto s =
        state_result->at(i)[pnc::scc_longitudinal_planning_v3::StateId::POS];
    auto v =
        state_result->at(i)[pnc::scc_longitudinal_planning_v3::StateId::VEL];
    auto a =
        state_result->at(i)[pnc::scc_longitudinal_planning_v3::StateId::ACC];

    double j = 0.0;

    if (i < N - 1) {
      j = control_result->at(
          i)[pnc::scc_longitudinal_planning_v3::ControlId::JERK];
    } else {
      j = control_result->at(
          i - 1)[pnc::scc_longitudinal_planning_v3::ControlId::JERK];
    }

    // post process for longitudinal motion planning, ensure non-negative vel
    if (v < 0.0 && i > 0) {
      s = state_result->at(i -
                           1)[pnc::scc_longitudinal_planning_v3::StateId::POS];
      v = 0.0;
      a = 0.0;
      j = 0.0;
    }

    planning_output_.mutable_pos_vec()->Set(i, s);
    planning_output_.mutable_vel_vec()->Set(i, v);
    planning_output_.mutable_acc_vec()->Set(i, a);
    planning_output_.mutable_jerk_vec()->Set(i, j);
  }

  // load solver and iteration info
  planning_output_.clear_solver_info();

  const auto &solver_info_ptr = ilqr_core_ptr_->GetSolverInfoPtr();

  planning_output_.mutable_solver_info()->set_solver_condition(
      solver_info_ptr->solver_condition);
  planning_output_.mutable_solver_info()->set_cost_size(
      solver_info_ptr->cost_size);
  planning_output_.mutable_solver_info()->set_iter_count(
      solver_info_ptr->iter_count);
  planning_output_.mutable_solver_info()->set_init_cost(
      solver_info_ptr->init_cost);

  for (size_t i = 0; i < solver_info_ptr->iter_count; ++i) {
    const auto &iter_info =
        planning_output_.mutable_solver_info()->add_iter_info();
    // const auto &iter_cost =
    //     planning_output_.mutable_solver_info()->add_cost_vec();

    iter_info->set_linesearch_success(
        solver_info_ptr->iteration_info_vec[i].linesearch_success);
    iter_info->set_backward_pass_count(
        solver_info_ptr->iteration_info_vec[i].backward_pass_count);
    iter_info->set_lambda(solver_info_ptr->iteration_info_vec[i].lambda);
    iter_info->set_cost(solver_info_ptr->iteration_info_vec[i].cost);
    iter_info->set_dcost(solver_info_ptr->iteration_info_vec[i].dcost);
    iter_info->set_expect(solver_info_ptr->iteration_info_vec[i].expect);
    iter_info->set_du_norm(solver_info_ptr->iteration_info_vec[i].du_norm);
    for (size_t j = 0; j < solver_info_ptr->cost_size; ++j) {
      //   iter_cost->set_cost_vec(solver_info_ptr->cost_iter_vec[i].at(j));
      planning_output_.mutable_solver_info()->add_cost_vec(
          solver_info_ptr->cost_iter_vec[i].at(j));
    }
  }

  return true;
}

}  // namespace scc_longitudinal_planning_v3
}  // namespace pnc