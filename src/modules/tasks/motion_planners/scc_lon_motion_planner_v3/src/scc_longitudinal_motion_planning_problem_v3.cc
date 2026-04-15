#include "scc_longitudinal_motion_planning_problem_v3.h"

#include <stdio.h>

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>

#include "math_lib.h"
#include "scc_longitudinal_motion_planning_cost_v3.h"

using namespace al_ilqr_solver;
using namespace pnc::mathlib;

namespace pnc {
namespace scc_longitudinal_planning_v3 {

void SccLongitudinalMotionPlanningProblemV3::Init() {
  // STEP 0: set solver config parmeters
  AliLqrSolverConfig solver_config;
  solver_config.horizon = 25;
  solver_config.state_size = STATE_SIZE;
  solver_config.input_size = INPUT_SIZE;
  solver_config.model_dt = 0.2;
  solver_config.warm_start_enable = false;
  solver_config.lambda_min = 1e-5;
  solver_config.du_tol = 1e-3;
  solver_config.max_iter = 30;
  solver_config.cost_tol = 1e-6;
  solver_config.al_ilqr_enable = true;
  solver_config.max_al_iter = 20;
  solver_config.constraint_num = 3;
  solver_config.penalty_init = 100.0;
  solver_config.penalty_scaling = 5.0;
  solver_config.penalty_max = 1e6;
  solver_config.constraint_tolerance_tol = 1e-3;
  solver_config.cost_tolerance_tol = 1e-8;
  solver_config.max_al_solve_time = 30.0;
  solver_config.cost_scale = 1.0;

  init_state_.resize(STATE_SIZE);

  // STEP 1: init core with solver config
  alilqr_core_ptr_ = std::make_shared<SccLonAliLqr>();
  alilqr_core_ptr_->Init(
      std::make_shared<SccLongitudinalMotionPlanningModelV3>(), solver_config);

  // STEP 2: add cost terms (including AL constraint cost)
  alilqr_core_ptr_->AddCost(std::make_shared<ReferenceCostTerm>());
  alilqr_core_ptr_->AddCost(std::make_shared<LonAccCostTerm>());
  alilqr_core_ptr_->AddCost(std::make_shared<LonJerkCostTerm>());
  alilqr_core_ptr_->AddCost(std::make_shared<LonSoftPosBoundCostTerm>());
  alilqr_core_ptr_->AddCost(std::make_shared<LonHardPosBoundCostTerm>());
  alilqr_core_ptr_->AddCost(std::make_shared<LonExtendPosBoundCostTerm>());
  alilqr_core_ptr_->AddCost(std::make_shared<LonVelBoundCostTerm>());
  alilqr_core_ptr_->AddCost(std::make_shared<LonAccBoundCostTerm>());
  alilqr_core_ptr_->AddCost(std::make_shared<LonJerkBoundCostTerm>());
  alilqr_core_ptr_->AddCost(std::make_shared<LonHardNonNegativeVelCostTerm>());
  alilqr_core_ptr_->AddCost(std::make_shared<LonEmergencyStopCostTerm>());

  // STEP 3: init debug info, must run after add cost
  alilqr_core_ptr_->InitAdvancedInfo();

  // init planning output
  const auto &N = alilqr_core_ptr_->GetSolverConfigPtr()->horizon + 1;
  planning_output_.mutable_time_vec()->Resize(N, 0.0);
  planning_output_.mutable_pos_vec()->Resize(N, 0.0);
  planning_output_.mutable_vel_vec()->Resize(N, 0.0);
  planning_output_.mutable_acc_vec()->Resize(N, 0.0);
  planning_output_.mutable_jerk_vec()->Resize(N, 0.0);
}

void SccLongitudinalMotionPlanningProblemV3::Reset() { init_state_.setZero(); }

uint8_t SccLongitudinalMotionPlanningProblemV3::Update(
    planning::common::LongitudinalPlanningInput &planning_input) {
  // set cost config
  const size_t N = alilqr_core_ptr_->GetSolverConfigPtr()->horizon + 1;

  std::vector<AliLqrCostConfig> cost_config_vec;
  cost_config_vec.resize(N);

  for (size_t i = 0; i < N; ++i) {
    // reference
    cost_config_vec.at(i)[REF_POS] = planning_input.ref_pos_vec(i);
    cost_config_vec.at(i)[REF_VEL] = planning_input.ref_vel_vec(i);
    cost_config_vec.at(i)[REF_ACC] = planning_input.ref_acc_vec(i);
    cost_config_vec.at(i)[S_STOP] = planning_input.s_stop();

    // bounds
    cost_config_vec.at(i)[SOFT_POS_MAX] = planning_input.soft_pos_max_vec(i);
    cost_config_vec.at(i)[SOFT_POS_MIN] = planning_input.soft_pos_min_vec(i);
    cost_config_vec.at(i)[EXTEND_POS_MAX] =
        planning_input.extend_pos_max_vec(i);
    cost_config_vec.at(i)[EXTEND_POS_MIN] =
        planning_input.extend_pos_min_vec(i);
    cost_config_vec.at(i)[HARD_POS_MAX] = planning_input.hard_pos_max_vec(i);
    cost_config_vec.at(i)[HARD_POS_MIN] = planning_input.hard_pos_min_vec(i);
    cost_config_vec.at(i)[VEL_MAX] = planning_input.vel_max_vec(i);
    cost_config_vec.at(i)[VEL_MIN] = planning_input.vel_min_vec(i);
    cost_config_vec.at(i)[ACC_MAX] = planning_input.acc_max_vec(i);
    cost_config_vec.at(i)[ACC_MIN] = planning_input.acc_min_vec(i);
    cost_config_vec.at(i)[JERK_MAX] = planning_input.jerk_max_vec(i);
    cost_config_vec.at(i)[JERK_MIN] = planning_input.jerk_min_vec(i);

    const double start_increase_rate = 0.15;
    double start_increase_factor = 1.0 - std::exp(-start_increase_rate * i);

    cost_config_vec.at(i)[W_REF_POS] =
        planning_input.s_weights(i);  // dynamic weight

    cost_config_vec.at(i)[W_REF_VEL] = planning_input.v_weights(i);

    cost_config_vec.at(i)[W_REF_ACC] = planning_input.a_weights(i);

    cost_config_vec.at(i)[W_ACC] =
        planning_input.q_acc_start() +
        start_increase_factor *
            (planning_input.q_acc() - planning_input.q_acc_start());

    cost_config_vec.at(i)[W_JERK] =
        planning_input.q_jerk_start() +
        start_increase_factor *
            (planning_input.q_jerk() - planning_input.q_jerk_start());

    cost_config_vec.at(i)[W_SNAP] = planning_input.q_snap();

    double boundary_decay_rate = 0.15;
    double boundary_decay_factor = std::exp(-boundary_decay_rate * i);
    cost_config_vec.at(i)[W_POS_BOUND] =
        planning_input.q_soft_pos_bound() * boundary_decay_factor;

    cost_config_vec.at(i)[W_EXTEND_POS_BOUND] =
        planning_input.q_extend_pos_bound();

    cost_config_vec.at(i)[W_VEL_BOUND] = planning_input.q_vel_bound();
    cost_config_vec.at(i)[W_ACC_BOUND] = planning_input.q_acc_bound();
    cost_config_vec.at(i)[W_JERK_BOUND] = planning_input.q_jerk_bound();
    cost_config_vec.at(i)[W_S_STOP] = planning_input.q_stop_s();

    cost_config_vec.at(i)[W_HARD_POS_BOUND] = 0.0;

    cost_config_vec.at(i)[SAFE_DISTANCE] = planning_input.safe_distance();

    cost_config_vec.at(i)[W_EMERGENCY_STOP] = planning_input.q_emergency_stop();

    if (i == N - 1) {
      cost_config_vec.at(i)[TERMINAL_FLAG] = 1;
      cost_config_vec.at(i)[W_S_STOP] = planning_input.q_stop_s();
    } else {
      cost_config_vec.at(i)[TERMINAL_FLAG] = 0;
    }
  }

  // set cost config
  alilqr_core_ptr_->SetCostConfig(cost_config_vec);
  const double rho_init = alilqr_core_ptr_->GetSolverConfigPtr()->penalty_init;
  std::vector<AliLqrConstraintConfig> constraint_config_vec(N);
  for (size_t i = 0; i < N; ++i) {
    constraint_config_vec[i].fill(0.0);
    constraint_config_vec[i][RHO_HARD_POS_UPPER] = rho_init;
    constraint_config_vec[i][RHO_HARD_POS_LOWER] = rho_init;
    constraint_config_vec[i][RHO_HARD_VEL_LOWER] = rho_init;
  }
  alilqr_core_ptr_->SetConstraintConfig(constraint_config_vec);

  // solve the AL-iLQR problem
  init_state_ << planning_input.init_state().s(),
      planning_input.init_state().v(), planning_input.init_state().a();
  alilqr_core_ptr_->SolveForAliLqr(init_state_);

  // assemble planning result
  const auto &state_result = alilqr_core_ptr_->GetStateResultPtr();
  const auto &control_result = alilqr_core_ptr_->GetControlResultPtr();
  const auto &dt = alilqr_core_ptr_->GetSolverConfigPtr()->model_dt;

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

  const auto &solver_info_ptr = alilqr_core_ptr_->GetSolverInfoPtr();

  planning_output_.mutable_solver_info()->set_solver_condition(
      solver_info_ptr->solver_condition);
  planning_output_.mutable_solver_info()->set_cost_size(
      solver_info_ptr->cost_size);
  planning_output_.mutable_solver_info()->set_iter_count(
      solver_info_ptr->iter_count);
  planning_output_.mutable_solver_info()->set_init_cost(
      solver_info_ptr->init_cost);

  planning_output_.mutable_solver_info()->set_outer_iter_count(
      solver_info_ptr->outer_iter_count);
  planning_output_.mutable_solver_info()->set_constraint_violation(
      solver_info_ptr->constraint_violation);
  planning_output_.mutable_solver_info()->set_dcost_outer(
      solver_info_ptr->dcost_outer);

  for (size_t i = 0; i < solver_info_ptr->iter_count; ++i) {
    const auto &iter_info =
        planning_output_.mutable_solver_info()->add_iter_info();

    iter_info->set_linesearch_success(
        solver_info_ptr->iteration_info_vec[i].linesearch_success);
    iter_info->set_backward_pass_count(
        solver_info_ptr->iteration_info_vec[i].backward_pass_count);
    iter_info->set_lambda_value(solver_info_ptr->iteration_info_vec[i].lambda);
    iter_info->set_cost(solver_info_ptr->iteration_info_vec[i].cost);
    iter_info->set_dcost(solver_info_ptr->iteration_info_vec[i].dcost);
    iter_info->set_expect(solver_info_ptr->iteration_info_vec[i].expect);
    iter_info->set_du_norm(solver_info_ptr->iteration_info_vec[i].du_norm);
    for (size_t j = 0; j < solver_info_ptr->cost_size; ++j) {
      planning_output_.mutable_solver_info()->add_cost_vec(
          solver_info_ptr->cost_iter_vec[i].at(j));
    }
  }

  for (const auto &outer_rec : solver_info_ptr->outer_iter_records) {
    auto *al_info =
        planning_output_.mutable_solver_info()->add_al_outer_iter_info();
    al_info->set_inner_iter_count(outer_rec.inner_iter_count);
    al_info->set_init_cost(outer_rec.init_cost);
    al_info->set_final_cost(outer_rec.final_cost);
    al_info->set_constraint_violation(outer_rec.constraint_violation);
    al_info->set_dcost_outer(outer_rec.dcost_outer);

    for (size_t i = 0; i < outer_rec.inner_iter_count; ++i) {
      auto *inner_info = al_info->add_inner_iter_info();
      inner_info->set_linesearch_success(
          outer_rec.inner_iter_info_vec[i].linesearch_success);
      inner_info->set_backward_pass_count(
          outer_rec.inner_iter_info_vec[i].backward_pass_count);
      inner_info->set_lambda_value(outer_rec.inner_iter_info_vec[i].lambda);
      inner_info->set_cost(outer_rec.inner_iter_info_vec[i].cost);
      inner_info->set_dcost(outer_rec.inner_iter_info_vec[i].dcost);
      inner_info->set_expect(outer_rec.inner_iter_info_vec[i].expect);
      inner_info->set_du_norm(outer_rec.inner_iter_info_vec[i].du_norm);
    }

    for (size_t i = 0; i <= outer_rec.inner_iter_count; ++i) {
      if (i < outer_rec.inner_cost_iter_vec.size()) {
        for (size_t j = 0; j < solver_info_ptr->cost_size; ++j) {
          al_info->add_cost_vec(outer_rec.inner_cost_iter_vec[i].at(j));
        }
      }
    }
  }

  return true;
}

}  // namespace scc_longitudinal_planning_v3
}  // namespace pnc
