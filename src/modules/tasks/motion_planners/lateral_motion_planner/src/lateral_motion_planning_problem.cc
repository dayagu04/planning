#include "lateral_motion_planning_problem.h"

#include <stdio.h>

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>

#include "ilqr_core.h"

using namespace ilqr_solver;

static const double kMaxWheelAngle =
    360.0 / 14.5 / 57.3;  // 360 deg steering angle for scc
static const double kMaxWheelAngleRate =
    240.0 / 14.5 / 57.3;  // 240 deg/s steering angle rate for scc
namespace pnc {
namespace lateral_planning {
void LateralMotionPlanningProblem::Init() {
  // STEP 0: set solver config parmeters
  ilqr_solver::iLqrSolverConfig solver_config;
  solver_config.horizon = 25;
  solver_config.state_size = STATE_SIZE;
  solver_config.input_size = INPUT_SIZE;
  solver_config.model_dt = 0.2;
  solver_config.warm_start_enable = false;
  solver_config.du_tol = 0.01 / 57.3 / 14.5;
  init_state_.resize(STATE_SIZE);
  // STEP 1: init core with solver config
  ilqr_core_ptr_ = std::make_shared<iLqr>();
  ilqr_core_ptr_->Init(std::make_shared<LateralMotionPlanningModel>(),
                       solver_config);

  // STEP 2: add cost
  ilqr_core_ptr_->AddCost(
      std::make_shared<ReferenceCostTerm>());  // reference cost
  // ilqr_core_ptr_->AddCost(
  //     std::make_shared<ContinuityCostTerm>()); // continuity cost
  ilqr_core_ptr_->AddCost(
      std::make_shared<LatAccCostTerm>());  // lateral acc cost
  ilqr_core_ptr_->AddCost(
      std::make_shared<LatJerkCostTerm>());  // lateral jerk cost
  ilqr_core_ptr_->AddCost(
      std::make_shared<LatAccBoundCostTerm>());  // lateral acc bound cost
  ilqr_core_ptr_->AddCost(
      std::make_shared<LatJerkBoundCostTerm>());  // lateral jerk bound cost
//   ilqr_core_ptr_->AddCost(
//       std::make_shared<PathSoftCorridorCostTerm>());  // path soft corridor
                                                      // cost

  // STEP 3: init debug info, must run after add cost
  ilqr_core_ptr_->InitAdvancedInfo();

  // init planning output
  const auto N = ilqr_core_ptr_->GetSolverConfigPtr()->horizon + 1;
  planning_output_.mutable_time_vec()->Resize(N, 0.0);
  planning_output_.mutable_x_vec()->Resize(N, 0.0);
  planning_output_.mutable_y_vec()->Resize(N, 0.0);
  planning_output_.mutable_theta_vec()->Resize(N, 0.0);
  planning_output_.mutable_delta_vec()->Resize(N, 0.0);
  planning_output_.mutable_omega_vec()->Resize(N, 0.0);
  planning_output_.mutable_acc_vec()->Resize(N, 0.0);
  planning_output_.mutable_jerk_vec()->Resize(N, 0.0);

  t_vec_.resize(solver_config.horizon + 1);
  x_vec_.resize(solver_config.horizon + 1);
  y_vec_.resize(solver_config.horizon + 1);
  theta_vec_.resize(solver_config.horizon + 1);
  soft_lower_bound_vec_.resize(solver_config.horizon + 1);
  soft_upper_bound_vec_.resize(solver_config.horizon + 1);
  hard_lower_bound_vec_.resize(solver_config.horizon + 1);
  hard_lower_bound_vec_.resize(solver_config.horizon + 1);
}

uint8_t LateralMotionPlanningProblem::Update(
    planning::common::LateralPlanningInput &planning_input) {
  // set cost config
  const size_t N = ilqr_core_ptr_->GetSolverConfigPtr()->horizon + 1;
  const auto v2 = planning_input.ref_vel() * planning_input.ref_vel();

  std::vector<ilqr_solver::IlqrCostConfig> cost_config_vec;
  cost_config_vec.resize(N);

  // calculate delta_bound and omega_bound
  double delta_bound =
      std::min(kMaxWheelAngle, planning_input.acc_bound() /
                                   (planning_input.curv_factor() * v2));
  double omega_bound =
      std::min(kMaxWheelAngleRate, planning_input.jerk_bound() /
                                       (planning_input.curv_factor() * v2));

  for (size_t i = 0; i < N; ++i) {
    // reference
    cost_config_vec.at(i)[REF_X] = planning_input.ref_x_vec(i);
    cost_config_vec.at(i)[REF_Y] = planning_input.ref_y_vec(i);
    cost_config_vec.at(i)[REF_THETA] =
        i > 0 ? planning_input.ref_theta_vec(i) : 0.0;
    cost_config_vec.at(i)[REF_VEL] = planning_input.ref_vel();
    cost_config_vec.at(i)[CURV_FACTOR] = planning_input.curv_factor();

    cost_config_vec.at(i)[CONTINUITY_X] = planning_input.last_x_vec(i);
    cost_config_vec.at(i)[CONTINUITY_Y] = planning_input.last_y_vec(i);
    cost_config_vec.at(i)[CONTINUITY_THETA] = planning_input.last_theta_vec(i);

    // bounds
    cost_config_vec.at(i)[DELTA_BOUND] = delta_bound;
    cost_config_vec.at(i)[OMEGA_BOUND] = omega_bound;

    cost_config_vec.at(i)[SOFT_UPPER_BOUND_X0] =
        planning_input.soft_upper_bound_x0_vec(i);
    cost_config_vec.at(i)[SOFT_UPPER_BOUND_Y0] =
        planning_input.soft_upper_bound_y0_vec(i);
    cost_config_vec.at(i)[SOFT_UPPER_BOUND_X1] =
        planning_input.soft_upper_bound_x1_vec(i);
    cost_config_vec.at(i)[SOFT_UPPER_BOUND_Y1] =
        planning_input.soft_upper_bound_y1_vec(i);

    cost_config_vec.at(i)[SOFT_LOWER_BOUND_X0] =
        planning_input.soft_lower_bound_x0_vec(i);
    cost_config_vec.at(i)[SOFT_LOWER_BOUND_Y0] =
        planning_input.soft_lower_bound_y0_vec(i);
    cost_config_vec.at(i)[SOFT_LOWER_BOUND_X1] =
        planning_input.soft_lower_bound_x1_vec(i);
    cost_config_vec.at(i)[SOFT_LOWER_BOUND_Y1] =
        planning_input.soft_lower_bound_y1_vec(i);

    // weights
    cost_config_vec.at(i)[W_REF_X] = planning_input.q_ref_x();
    cost_config_vec.at(i)[W_REF_Y] = planning_input.q_ref_y();

    const auto &ref_vel = planning_input.ref_vel();

    cost_config_vec.at(i)[W_REF_THETA] =
        planning_input.q_ref_theta() * (1.0 + ref_vel * ref_vel);
    cost_config_vec.at(i)[W_CONTINUITY_X] =
        planning_input.q_ref_x() * planning_input.q_continuity();
    cost_config_vec.at(i)[W_CONTINUITY_Y] =
        planning_input.q_ref_y() * planning_input.q_continuity();
    cost_config_vec.at(i)[W_CONTINUITY_THETA] =
        planning_input.q_ref_theta() * planning_input.q_continuity();

    cost_config_vec.at(i)[W_ACC] = planning_input.q_acc();
    cost_config_vec.at(i)[W_JERK] = planning_input.q_jerk();
    cost_config_vec.at(i)[W_ACC_BOUND] = planning_input.q_acc_bound();
    cost_config_vec.at(i)[W_JERK_BOUND] = planning_input.q_jerk_bound();

    cost_config_vec.at(i)[W_SOFT_CORRIDOR] = planning_input.q_soft_corridor();
    cost_config_vec.at(i)[W_HARD_CORRIDOR] = planning_input.q_hard_corridor();

    if (!planning_input.complete_follow()) {
      if (i > planning_input.motion_plan_concerned_index()) {
        cost_config_vec.at(i)[W_REF_X] *= 0.2;
        cost_config_vec.at(i)[W_REF_Y] *= 0.2;
        cost_config_vec.at(i)[W_REF_THETA] *= 0.2;
        // cost_config_vec.at(i)[W_SOFT_CORRIDOR] *= end_ratio;
        // cost_config_vec.at(i)[W_HARD_CORRIDOR] *= end_ratio;
      }
    }

    if (i == N - 1) {
      cost_config_vec.at(i)[TERMINAL_FLAG] = 1;
    } else {
      cost_config_vec.at(i)[TERMINAL_FLAG] = 0;
    }
  }

  // set const config
  ilqr_core_ptr_->SetCostConfig(cost_config_vec);

  // solve the ilqr problem
  init_state_ << planning_input.init_state().x(),
      planning_input.init_state().y(), planning_input.init_state().theta(),
      planning_input.init_state().delta();

  ilqr_core_ptr_->Solve(init_state_);

  // fail protection
  const uint8_t solver_condition =
      ilqr_core_ptr_->GetSolverInfoPtr()->solver_condition;
  const auto solver_config = ilqr_core_ptr_->GetSolverConfigPtr();

  ilqr_solver::ControlVec u_vec;
  u_vec.resize(solver_config->horizon + 1);

  for (size_t i = 0; i < u_vec.size(); ++i) {
    Control u;
    u.resize(1);
    u[0] = planning_input.control_vec(i);
    u_vec[i] = u;
  }

  // fail protection
  if (solver_condition >= iLqr::BACKWARD_PASS_FAIL) {
    ilqr_core_ptr_->Simulation(init_state_, u_vec);
  }

  // assemble planning result
  const auto &state_result = ilqr_core_ptr_->GetStateResultPtr();
  const auto &control_result = ilqr_core_ptr_->GetControlResultPtr();
  const auto &dt = ilqr_core_ptr_->GetSolverConfigPtr()->model_dt;
  const double kv2 = planning_input.curv_factor() * v2;

  double t = 0.0;
  for (size_t i = 0; i < N; ++i) {
    planning_output_.mutable_time_vec()->Set(i, t);
    t += dt;

    planning_output_.mutable_x_vec()->Set(
        i, state_result->at(i)[pnc::lateral_planning::StateId::X]);
    planning_output_.mutable_y_vec()->Set(
        i, state_result->at(i)[pnc::lateral_planning::StateId::Y]);
    planning_output_.mutable_theta_vec()->Set(
        i, state_result->at(i)[pnc::lateral_planning::StateId::THETA]);
    planning_output_.mutable_delta_vec()->Set(
        i, state_result->at(i)[pnc::lateral_planning::StateId::DELTA]);
    planning_output_.mutable_acc_vec()->Set(
        i, kv2 * state_result->at(i)[pnc::lateral_planning::StateId::DELTA]);

    if (i < N - 1) {
      planning_output_.mutable_omega_vec()->Set(
          i, control_result->at(i)[pnc::lateral_planning::ControlId::OMEGA]);
      planning_output_.mutable_jerk_vec()->Set(
          i,
          kv2 * control_result->at(i)[pnc::lateral_planning::ControlId::OMEGA]);
    } else {
      planning_output_.mutable_omega_vec()->Set(
          i, planning_output_.omega_vec(i - 1));
      planning_output_.mutable_jerk_vec()->Set(
          i, planning_output_.jerk_vec(i - 1));
    }
  }

  // load solver and iteration info
  planning_output_.clear_solver_info();

  const auto &soler_info_ptr = ilqr_core_ptr_->GetSolverInfoPtr();

  planning_output_.mutable_solver_info()->set_solver_condition(
      soler_info_ptr->solver_condition);
  planning_output_.mutable_solver_info()->set_cost_size(
      soler_info_ptr->cost_size);
  planning_output_.mutable_solver_info()->set_iter_count(
      soler_info_ptr->iter_count);
  planning_output_.mutable_solver_info()->set_init_cost(
      soler_info_ptr->init_cost);

  for (size_t i = 0; i < soler_info_ptr->iter_count; ++i) {
    const auto &iter_info =
        planning_output_.mutable_solver_info()->add_iter_info();

    iter_info->set_linesearch_success(
        soler_info_ptr->iteration_info_vec[i].linesearch_success);
    iter_info->set_backward_pass_count(
        soler_info_ptr->iteration_info_vec[i].backward_pass_count);
    iter_info->set_lambda(soler_info_ptr->iteration_info_vec[i].lambda);
    iter_info->set_cost(soler_info_ptr->iteration_info_vec[i].cost);
    iter_info->set_dcost(soler_info_ptr->iteration_info_vec[i].dcost);
    iter_info->set_expect(soler_info_ptr->iteration_info_vec[i].expect);
    iter_info->set_du_norm(soler_info_ptr->iteration_info_vec[i].du_norm);
  }

  return solver_condition;
}

uint8_t LateralMotionPlanningProblem::Update(
    planning::common::LateralPlanningInput &planning_input,
    const size_t motion_plan_concerned_start_index,
    const double motion_plan_concerned_start_ratio,
    const double motion_plan_concerned_end_ratio,
    const double jerk_bound, const double decay_factor,
    const double w_xy, const double w_theta,
    const double new_dt) {
  // const double model_dt = ilqr_core_ptr_->GetSolverConfigPtr()->model_dt;
  // auto planning_input = SplineInput(origin_planning_input, model_dt * model_dt / new_dt);
  // set cost config
  // const size_t N = 5 / dt + 1;
  const size_t N = ilqr_core_ptr_->GetSolverConfigPtr()->horizon + 1;
  planning_output_.mutable_time_vec()->Resize(N, 0.0);
  planning_output_.mutable_x_vec()->Resize(N, 0.0);
  planning_output_.mutable_y_vec()->Resize(N, 0.0);
  planning_output_.mutable_theta_vec()->Resize(N, 0.0);
  planning_output_.mutable_delta_vec()->Resize(N, 0.0);
  planning_output_.mutable_omega_vec()->Resize(N, 0.0);
  planning_output_.mutable_acc_vec()->Resize(N, 0.0);
  planning_output_.mutable_jerk_vec()->Resize(N, 0.0);
  const auto &ref_vel = planning_input.ref_vel();
  const auto v2 = ref_vel * ref_vel;

  std::vector<ilqr_solver::IlqrCostConfig> cost_config_vec;
  cost_config_vec.resize(N);

  // calculate delta_bound and omega_bound
  double delta_bound =
      std::min(kMaxWheelAngle, planning_input.acc_bound() /
                                   (planning_input.curv_factor() * v2));
  double omega_bound =
      std::min(kMaxWheelAngleRate, planning_input.jerk_bound() /
                                       (planning_input.curv_factor() * v2));

  double start_ratio = motion_plan_concerned_start_ratio;
  double end_ratio = motion_plan_concerned_end_ratio;

  for (size_t i = 0; i < N; ++i) {
    // reference
    cost_config_vec.at(i)[REF_X] = planning_input.ref_x_vec(i);
    cost_config_vec.at(i)[REF_Y] = planning_input.ref_y_vec(i);
    cost_config_vec.at(i)[REF_THETA] =
        i > 0 ? planning_input.ref_theta_vec(i) : 0.0;
    cost_config_vec.at(i)[REF_VEL] = ref_vel;
    cost_config_vec.at(i)[CURV_FACTOR] = planning_input.curv_factor();

    cost_config_vec.at(i)[CONTINUITY_X] = planning_input.last_x_vec(i);
    cost_config_vec.at(i)[CONTINUITY_Y] = planning_input.last_y_vec(i);
    cost_config_vec.at(i)[CONTINUITY_THETA] = planning_input.last_theta_vec(i);

    // bounds
    cost_config_vec.at(i)[DELTA_BOUND] = delta_bound;
    cost_config_vec.at(i)[OMEGA_BOUND] = omega_bound;

    cost_config_vec.at(i)[SOFT_UPPER_BOUND_X0] =
        planning_input.soft_upper_bound_x0_vec(i);
    cost_config_vec.at(i)[SOFT_UPPER_BOUND_Y0] =
        planning_input.soft_upper_bound_y0_vec(i);
    cost_config_vec.at(i)[SOFT_UPPER_BOUND_X1] =
        planning_input.soft_upper_bound_x1_vec(i);
    cost_config_vec.at(i)[SOFT_UPPER_BOUND_Y1] =
        planning_input.soft_upper_bound_y1_vec(i);

    cost_config_vec.at(i)[SOFT_LOWER_BOUND_X0] =
        planning_input.soft_lower_bound_x0_vec(i);
    cost_config_vec.at(i)[SOFT_LOWER_BOUND_Y0] =
        planning_input.soft_lower_bound_y0_vec(i);
    cost_config_vec.at(i)[SOFT_LOWER_BOUND_X1] =
        planning_input.soft_lower_bound_x1_vec(i);
    cost_config_vec.at(i)[SOFT_LOWER_BOUND_Y1] =
        planning_input.soft_lower_bound_y1_vec(i);

    // weights
    cost_config_vec.at(i)[W_REF_X] = planning_input.q_ref_x();
    cost_config_vec.at(i)[W_REF_Y] = planning_input.q_ref_y();

    cost_config_vec.at(i)[W_REF_THETA] =
        planning_input.q_ref_theta() * (1.0 + ref_vel * ref_vel);
    cost_config_vec.at(i)[W_CONTINUITY_X] =
        planning_input.q_ref_x() * planning_input.q_continuity();
    cost_config_vec.at(i)[W_CONTINUITY_Y] =
        planning_input.q_ref_y() * planning_input.q_continuity();
    cost_config_vec.at(i)[W_CONTINUITY_THETA] =
        planning_input.q_ref_theta() * planning_input.q_continuity();

    cost_config_vec.at(i)[W_ACC] = planning_input.q_acc();
    cost_config_vec.at(i)[W_JERK] = planning_input.q_jerk();
    cost_config_vec.at(i)[W_ACC_BOUND] = planning_input.q_acc_bound();
    cost_config_vec.at(i)[W_JERK_BOUND] = planning_input.q_jerk_bound();

    cost_config_vec.at(i)[W_SOFT_CORRIDOR] = planning_input.q_soft_corridor();
    cost_config_vec.at(i)[W_HARD_CORRIDOR] = planning_input.q_hard_corridor();

    if (!planning_input.complete_follow()) {
      if (i < motion_plan_concerned_start_index) {
        // cost_config_vec.at(i)[W_REF_X] = std::min(start_ratio * std::max(planning_input.q_ref_x(), planning_input.q_jerk()), planning_input.q_ref_x());
        // cost_config_vec.at(i)[W_REF_Y] = std::min(start_ratio * std::max(planning_input.q_ref_y(), planning_input.q_jerk()), planning_input.q_ref_y());
        // cost_config_vec.at(i)[W_REF_THETA] *= start_ratio;
        // cost_config_vec.at(i)[W_JERK] = std::max(planning_input.q_ref_x(), planning_input.q_jerk());
        // cost_config_vec.at(i)[W_REF_X] = w_xy;
        // cost_config_vec.at(i)[W_REF_Y] = w_xy;
        // cost_config_vec.at(i)[W_REF_THETA] = w_theta;
        cost_config_vec.at(i)[W_JERK] = std::max(start_ratio * planning_input.q_ref_x(), planning_input.q_jerk());
        // cost_config_vec.at(i)[W_SOFT_CORRIDOR] *= start_ratio;
        // cost_config_vec.at(i)[W_HARD_CORRIDOR] *= start_ratio;
        start_ratio *= decay_factor;
        // cost_config_vec.at(i)[OMEGA_BOUND] =
        //     std::min(kMaxWheelAngleRate,
        //              jerk_bound / (planning_input.curv_factor() * v2));
      } else if (i > planning_input.motion_plan_concerned_index()) {
        // cost_config_vec.at(i)[W_REF_X] = end_ratio * std::max(planning_input.q_ref_x(), planning_input.q_jerk());
        // cost_config_vec.at(i)[W_REF_Y] = end_ratio * std::max(planning_input.q_ref_x(), planning_input.q_jerk());
        // cost_config_vec.at(i)[W_REF_THETA] *= end_ratio;
        // cost_config_vec.at(i)[W_JERK] = std::max(planning_input.q_ref_x(), planning_input.q_jerk());
        cost_config_vec.at(i)[W_JERK] = std::max(end_ratio * planning_input.q_ref_x(), planning_input.q_jerk());
        // cost_config_vec.at(i)[W_SOFT_CORRIDOR] *= end_ratio;
        // cost_config_vec.at(i)[W_HARD_CORRIDOR] *= end_ratio;
        end_ratio *= 1.5;
      }
    }

    if (i == N - 1) {
      cost_config_vec.at(i)[TERMINAL_FLAG] = 1;
    } else {
      cost_config_vec.at(i)[TERMINAL_FLAG] = 0;
    }
  }

  // set const config
  ilqr_core_ptr_->SetCostConfig(cost_config_vec);

  // solve the ilqr problem
  init_state_ << planning_input.init_state().x(),
      planning_input.init_state().y(), planning_input.init_state().theta(),
      planning_input.init_state().delta();

  ilqr_core_ptr_->Solve(init_state_);

  // fail protection
  const uint8_t solver_condition =
      ilqr_core_ptr_->GetSolverInfoPtr()->solver_condition;
  const auto solver_config = ilqr_core_ptr_->GetSolverConfigPtr();

  ilqr_solver::ControlVec u_vec;
  u_vec.resize(solver_config->horizon + 1);

  for (size_t i = 0; i < u_vec.size(); ++i) {
    Control u;
    u.resize(1);
    u[0] = planning_input.control_vec(i);
    u_vec[i] = u;
  }

  // fail protection
  if (solver_condition >= iLqr::BACKWARD_PASS_FAIL) {
    ilqr_core_ptr_->Simulation(init_state_, u_vec);
  }

  // assemble planning result
  const auto &state_result = ilqr_core_ptr_->GetStateResultPtr();
  const auto &control_result = ilqr_core_ptr_->GetControlResultPtr();
  const auto &dt = ilqr_core_ptr_->GetSolverConfigPtr()->model_dt;
  const double kv2 = planning_input.curv_factor() * v2;

  double t = 0.0;
  for (size_t i = 0; i < N; ++i) {
    planning_output_.mutable_time_vec()->Set(i, t);
    t += dt;

    planning_output_.mutable_x_vec()->Set(
        i, state_result->at(i)[pnc::lateral_planning::StateId::X]);
    planning_output_.mutable_y_vec()->Set(
        i, state_result->at(i)[pnc::lateral_planning::StateId::Y]);
    planning_output_.mutable_theta_vec()->Set(
        i, state_result->at(i)[pnc::lateral_planning::StateId::THETA]);
    planning_output_.mutable_delta_vec()->Set(
        i, state_result->at(i)[pnc::lateral_planning::StateId::DELTA]);
    planning_output_.mutable_acc_vec()->Set(
        i, kv2 * state_result->at(i)[pnc::lateral_planning::StateId::DELTA]);

    if (i < N - 1) {
      planning_output_.mutable_omega_vec()->Set(
          i, control_result->at(i)[pnc::lateral_planning::ControlId::OMEGA]);
      planning_output_.mutable_jerk_vec()->Set(
          i,
          kv2 * control_result->at(i)[pnc::lateral_planning::ControlId::OMEGA]);
    } else {
      planning_output_.mutable_omega_vec()->Set(
          i, planning_output_.omega_vec(i - 1));
      planning_output_.mutable_jerk_vec()->Set(
          i, planning_output_.jerk_vec(i - 1));
    }
  }

  // load solver and iteration info
  planning_output_.clear_solver_info();

  const auto &soler_info_ptr = ilqr_core_ptr_->GetSolverInfoPtr();

  planning_output_.mutable_solver_info()->set_solver_condition(
      soler_info_ptr->solver_condition);
  planning_output_.mutable_solver_info()->set_cost_size(
      soler_info_ptr->cost_size);
  planning_output_.mutable_solver_info()->set_iter_count(
      soler_info_ptr->iter_count);
  planning_output_.mutable_solver_info()->set_init_cost(
      soler_info_ptr->init_cost);

  for (size_t i = 0; i < soler_info_ptr->iter_count; ++i) {
    const auto &iter_info =
        planning_output_.mutable_solver_info()->add_iter_info();

    iter_info->set_linesearch_success(
        soler_info_ptr->iteration_info_vec[i].linesearch_success);
    iter_info->set_backward_pass_count(
        soler_info_ptr->iteration_info_vec[i].backward_pass_count);
    iter_info->set_lambda(soler_info_ptr->iteration_info_vec[i].lambda);
    iter_info->set_cost(soler_info_ptr->iteration_info_vec[i].cost);
    iter_info->set_dcost(soler_info_ptr->iteration_info_vec[i].dcost);
    iter_info->set_expect(soler_info_ptr->iteration_info_vec[i].expect);
    iter_info->set_du_norm(soler_info_ptr->iteration_info_vec[i].du_norm);
  }

  return solver_condition;
}

planning::common::LateralPlanningInput
LateralMotionPlanningProblem::SplineInput(
    planning::common::LateralPlanningInput &planning_input, const double dt) {
  const size_t N = 26;
  std::vector<double> hard_upper_bound_x_vec;
  std::vector<double> hard_upper_bound_y_vec;
  std::vector<double> hard_lower_bound_x_vec;
  std::vector<double> hard_lower_bound_y_vec;
  std::vector<double> soft_upper_bound_x_vec;
  std::vector<double> soft_upper_bound_y_vec;
  std::vector<double> soft_lower_bound_x_vec;
  std::vector<double> soft_lower_bound_y_vec;

  for (auto i = 0; i < N; i++) {
    t_vec_[i] = i * 0.2;
    x_vec_[i] = planning_input.ref_x_vec(i);
    y_vec_[i] = planning_input.ref_y_vec(i);
    theta_vec_[i] = planning_input.ref_theta_vec(i);

    hard_lower_bound_x_vec.emplace_back(
        planning_input.hard_lower_bound_x0_vec(i));
    hard_lower_bound_y_vec.emplace_back(
        planning_input.hard_lower_bound_y0_vec(i));
    hard_upper_bound_x_vec.emplace_back(
        planning_input.hard_upper_bound_x0_vec(i));
    hard_upper_bound_y_vec.emplace_back(
        planning_input.hard_upper_bound_y0_vec(i));
    soft_upper_bound_x_vec.emplace_back(
        planning_input.soft_upper_bound_x0_vec(i));
    soft_upper_bound_y_vec.emplace_back(
        planning_input.soft_upper_bound_y0_vec(i));
    soft_lower_bound_x_vec.emplace_back(
        planning_input.soft_lower_bound_x0_vec(i));
    soft_lower_bound_y_vec.emplace_back(
        planning_input.soft_lower_bound_y0_vec(i));
  }
  x_t_spline_.set_points(t_vec_, x_vec_);
  y_t_spline_.set_points(t_vec_, y_vec_);
  theta_t_spline_.set_points(t_vec_, theta_vec_);
  pnc::mathlib::spline h_l_x_spline;
  pnc::mathlib::spline h_l_y_spline;
  pnc::mathlib::spline h_u_x_spline;
  pnc::mathlib::spline h_u_y_spline;
  pnc::mathlib::spline s_l_x_spline;
  pnc::mathlib::spline s_l_y_spline;
  pnc::mathlib::spline s_u_x_spline;
  pnc::mathlib::spline s_u_y_spline;

  h_l_x_spline.set_points(t_vec_, hard_lower_bound_x_vec);
  h_l_y_spline.set_points(t_vec_, hard_lower_bound_y_vec);
  h_u_x_spline.set_points(t_vec_, hard_upper_bound_x_vec);
  h_u_y_spline.set_points(t_vec_, hard_upper_bound_y_vec);
  s_l_x_spline.set_points(t_vec_, soft_lower_bound_x_vec);
  s_l_y_spline.set_points(t_vec_, soft_lower_bound_y_vec);
  s_u_x_spline.set_points(t_vec_, soft_upper_bound_x_vec);
  s_u_y_spline.set_points(t_vec_, soft_upper_bound_y_vec);
  planning::common::LateralPlanningInput new_planning_input;
  new_planning_input.mutable_ref_x_vec()->Resize(N, 0.0);
  new_planning_input.mutable_ref_y_vec()->Resize(N, 0.0);
  new_planning_input.mutable_ref_theta_vec()->Resize(N, 0.0);

  new_planning_input.mutable_last_x_vec()->Resize(N, 0.0);
  new_planning_input.mutable_last_y_vec()->Resize(N, 0.0);
  new_planning_input.mutable_last_theta_vec()->Resize(N, 0.0);

  new_planning_input.mutable_soft_upper_bound_x0_vec()->Resize(N, 0.0);
  new_planning_input.mutable_soft_upper_bound_y0_vec()->Resize(N, 0.0);
  new_planning_input.mutable_soft_upper_bound_x1_vec()->Resize(N, 0.0);
  new_planning_input.mutable_soft_upper_bound_y1_vec()->Resize(N, 0.0);

  new_planning_input.mutable_soft_lower_bound_x0_vec()->Resize(N, 0.0);
  new_planning_input.mutable_soft_lower_bound_y0_vec()->Resize(N, 0.0);
  new_planning_input.mutable_soft_lower_bound_x1_vec()->Resize(N, 0.0);
  new_planning_input.mutable_soft_lower_bound_y1_vec()->Resize(N, 0.0);

  new_planning_input.mutable_hard_upper_bound_x0_vec()->Resize(N, 0.0);
  new_planning_input.mutable_hard_upper_bound_y0_vec()->Resize(N, 0.0);
  new_planning_input.mutable_hard_upper_bound_x1_vec()->Resize(N, 0.0);
  new_planning_input.mutable_hard_upper_bound_y1_vec()->Resize(N, 0.0);

  new_planning_input.mutable_hard_lower_bound_x0_vec()->Resize(N, 0.0);
  new_planning_input.mutable_hard_lower_bound_y0_vec()->Resize(N, 0.0);
  new_planning_input.mutable_hard_lower_bound_x1_vec()->Resize(N, 0.0);
  new_planning_input.mutable_hard_lower_bound_y1_vec()->Resize(N, 0.0);

  new_planning_input.mutable_control_vec()->Resize(N, 0.0);
  // int new_horizon_num = 5 / dt;
  int new_horizon_num = 26;
  for (auto i = 0; i < new_horizon_num; i++) {
    new_planning_input.mutable_ref_x_vec()->Set(i, x_t_spline_(i * dt));
    new_planning_input.mutable_ref_y_vec()->Set(i, y_t_spline_(i * dt));
    new_planning_input.mutable_ref_theta_vec()->Set(i, theta_t_spline_(i * dt));
    // new_planning_input.add_ref_x_vec(x_t_spline_(i * dt));
    // new_planning_input.add_ref_y_vec(y_t_spline_(i * dt));
    // new_planning_input.add_ref_theta_vec(theta_t_spline_(i * dt));
    // new_planning_input.add_hard_lower_bound_x0_vec(h_l_x_spline(i * dt));
    // new_planning_input.add_hard_lower_bound_y0_vec(h_l_y_spline(i * dt));
    // new_planning_input.add_soft_lower_bound_x0_vec(s_l_x_spline(i * dt));
    // new_planning_input.add_soft_lower_bound_y0_vec(s_l_y_spline(i * dt));
    // new_planning_input.add_hard_upper_bound_x0_vec(h_u_x_spline(i * dt));
    // new_planning_input.add_hard_upper_bound_y0_vec(h_u_y_spline(i * dt));
    // new_planning_input.add_soft_upper_bound_x0_vec(s_u_x_spline(i * dt));
    // new_planning_input.add_soft_upper_bound_y0_vec(s_u_y_spline(i * dt));
    // new_planning_input.mutable_control_vec()->Set(i, planning_input.control_vec(i));
  }

  new_planning_input.mutable_init_state()->set_x(planning_input.init_state().x());
  new_planning_input.mutable_init_state()->set_y(planning_input.init_state().y());
  new_planning_input.mutable_init_state()->set_theta(planning_input.init_state().theta());
  new_planning_input.mutable_init_state()->set_delta(planning_input.init_state().delta());
  new_planning_input.set_ref_vel(planning_input.ref_vel());
  new_planning_input.set_curv_factor(planning_input.curv_factor());
  new_planning_input.set_q_ref_x(planning_input.q_ref_x());
  new_planning_input.set_q_ref_y(planning_input.q_ref_y());
  new_planning_input.set_q_ref_theta(planning_input.q_ref_theta());
  new_planning_input.set_q_acc(planning_input.q_acc());
  new_planning_input.set_q_jerk(planning_input.q_jerk());
  new_planning_input.set_q_acc_bound(planning_input.q_acc_bound());
  new_planning_input.set_q_jerk_bound(planning_input.q_jerk_bound());
  new_planning_input.set_q_soft_corridor(planning_input.q_soft_corridor());
  new_planning_input.set_q_hard_corridor(planning_input.q_hard_corridor());
  new_planning_input.set_acc_bound(planning_input.acc_bound());
  new_planning_input.set_jerk_bound(planning_input.jerk_bound());
  new_planning_input.set_complete_follow(planning_input.complete_follow());
  new_planning_input.set_motion_plan_concerned_index(planning_input.motion_plan_concerned_index());

  return new_planning_input;
}

uint8_t LateralMotionPlanningProblem::Update(
    planning::common::LateralPlanningInput &origin_planning_input,
    const size_t motion_plan_concerned_start_index,
    const double motion_plan_concerned_start_ratio,
    const double motion_plan_concerned_end_ratio,
    const double jerk_bound, const double decay_factor,
    const double w_xy, const double w_theta,
    std::vector<double>& new_dt, std::vector<double>& new_ref_vel) {
  auto planning_input = SplineInput(origin_planning_input, new_dt);
  // set cost config
  // const size_t N = 5 / dt + 1;
  const size_t N = ilqr_core_ptr_->GetSolverConfigPtr()->horizon + 1;
  planning_output_.mutable_time_vec()->Resize(N, 0.0);
  planning_output_.mutable_x_vec()->Resize(N, 0.0);
  planning_output_.mutable_y_vec()->Resize(N, 0.0);
  planning_output_.mutable_theta_vec()->Resize(N, 0.0);
  planning_output_.mutable_delta_vec()->Resize(N, 0.0);
  planning_output_.mutable_omega_vec()->Resize(N, 0.0);
  planning_output_.mutable_acc_vec()->Resize(N, 0.0);
  planning_output_.mutable_jerk_vec()->Resize(N, 0.0);

  std::vector<ilqr_solver::IlqrCostConfig> cost_config_vec;
  cost_config_vec.resize(N);

  double start_ratio = motion_plan_concerned_start_ratio;
  double end_ratio = motion_plan_concerned_end_ratio;

  for (size_t i = 0; i < N; ++i) {
    const double ref_vel = new_ref_vel[i];
    const double v2 = ref_vel * ref_vel;

    // calculate delta_bound and omega_bound
    double delta_bound =
        std::min(kMaxWheelAngle, planning_input.acc_bound() /
                                    (planning_input.curv_factor() * v2));
    double omega_bound =
        std::min(kMaxWheelAngleRate, planning_input.jerk_bound() /
                                        (planning_input.curv_factor() * v2));
    // reference
    cost_config_vec.at(i)[REF_X] = planning_input.ref_x_vec(i);
    cost_config_vec.at(i)[REF_Y] = planning_input.ref_y_vec(i);
    cost_config_vec.at(i)[REF_THETA] =
        i > 0 ? planning_input.ref_theta_vec(i) : 0.0;
    // cost_config_vec.at(i)[REF_VEL] = planning_input.ref_vel();
    cost_config_vec.at(i)[REF_VEL] = ref_vel;
    cost_config_vec.at(i)[CURV_FACTOR] = planning_input.curv_factor();

    cost_config_vec.at(i)[CONTINUITY_X] = planning_input.last_x_vec(i);
    cost_config_vec.at(i)[CONTINUITY_Y] = planning_input.last_y_vec(i);
    cost_config_vec.at(i)[CONTINUITY_THETA] = planning_input.last_theta_vec(i);

    // bounds
    cost_config_vec.at(i)[DELTA_BOUND] = delta_bound;
    cost_config_vec.at(i)[OMEGA_BOUND] = omega_bound;

    cost_config_vec.at(i)[SOFT_UPPER_BOUND_X0] =
        planning_input.soft_upper_bound_x0_vec(i);
    cost_config_vec.at(i)[SOFT_UPPER_BOUND_Y0] =
        planning_input.soft_upper_bound_y0_vec(i);
    cost_config_vec.at(i)[SOFT_UPPER_BOUND_X1] =
        planning_input.soft_upper_bound_x1_vec(i);
    cost_config_vec.at(i)[SOFT_UPPER_BOUND_Y1] =
        planning_input.soft_upper_bound_y1_vec(i);

    cost_config_vec.at(i)[SOFT_LOWER_BOUND_X0] =
        planning_input.soft_lower_bound_x0_vec(i);
    cost_config_vec.at(i)[SOFT_LOWER_BOUND_Y0] =
        planning_input.soft_lower_bound_y0_vec(i);
    cost_config_vec.at(i)[SOFT_LOWER_BOUND_X1] =
        planning_input.soft_lower_bound_x1_vec(i);
    cost_config_vec.at(i)[SOFT_LOWER_BOUND_Y1] =
        planning_input.soft_lower_bound_y1_vec(i);

    // weights
    cost_config_vec.at(i)[W_REF_X] = planning_input.q_ref_x();
    cost_config_vec.at(i)[W_REF_Y] = planning_input.q_ref_y();

    // const auto &ref_vel = planning_input.ref_vel();

    cost_config_vec.at(i)[W_REF_THETA] =
        planning_input.q_ref_theta() * (1.0 + ref_vel * ref_vel);
    cost_config_vec.at(i)[W_CONTINUITY_X] =
        planning_input.q_ref_x() * planning_input.q_continuity();
    cost_config_vec.at(i)[W_CONTINUITY_Y] =
        planning_input.q_ref_y() * planning_input.q_continuity();
    cost_config_vec.at(i)[W_CONTINUITY_THETA] =
        planning_input.q_ref_theta() * planning_input.q_continuity();

    cost_config_vec.at(i)[W_ACC] = planning_input.q_acc();
    cost_config_vec.at(i)[W_JERK] = planning_input.q_jerk();
    cost_config_vec.at(i)[W_ACC_BOUND] = planning_input.q_acc_bound();
    cost_config_vec.at(i)[W_JERK_BOUND] = planning_input.q_jerk_bound();

    cost_config_vec.at(i)[W_SOFT_CORRIDOR] = planning_input.q_soft_corridor();
    cost_config_vec.at(i)[W_HARD_CORRIDOR] = planning_input.q_hard_corridor();

    if (!planning_input.complete_follow()) {
      if (i < motion_plan_concerned_start_index) {
        // cost_config_vec.at(i)[W_REF_X] = std::min(start_ratio * std::max(planning_input.q_ref_x(), planning_input.q_jerk()), planning_input.q_ref_x());
        // cost_config_vec.at(i)[W_REF_Y] = std::min(start_ratio * std::max(planning_input.q_ref_y(), planning_input.q_jerk()), planning_input.q_ref_y());
        // cost_config_vec.at(i)[W_REF_THETA] *= start_ratio;
        // cost_config_vec.at(i)[W_JERK] = std::max(planning_input.q_ref_x(), planning_input.q_jerk());
        // cost_config_vec.at(i)[W_REF_X] = w_xy;
        // cost_config_vec.at(i)[W_REF_Y] = w_xy;
        // cost_config_vec.at(i)[W_REF_THETA] = w_theta;
        cost_config_vec.at(i)[W_JERK] = std::max(start_ratio * planning_input.q_ref_x(), planning_input.q_jerk());
        // cost_config_vec.at(i)[W_SOFT_CORRIDOR] *= start_ratio;
        // cost_config_vec.at(i)[W_HARD_CORRIDOR] *= start_ratio;
        start_ratio *= decay_factor;
        // cost_config_vec.at(i)[OMEGA_BOUND] =
        //     std::min(kMaxWheelAngleRate,
        //              jerk_bound / (planning_input.curv_factor() * v2));
      } else if (i > planning_input.motion_plan_concerned_index()) {
        // cost_config_vec.at(i)[W_REF_X] = end_ratio * std::max(planning_input.q_ref_x(), planning_input.q_jerk());
        // cost_config_vec.at(i)[W_REF_Y] = end_ratio * std::max(planning_input.q_ref_x(), planning_input.q_jerk());
        // cost_config_vec.at(i)[W_REF_THETA] *= end_ratio;
        // cost_config_vec.at(i)[W_JERK] = std::max(planning_input.q_ref_x(), planning_input.q_jerk());
        cost_config_vec.at(i)[W_JERK] = std::max(end_ratio * planning_input.q_ref_x(), planning_input.q_jerk());
        // cost_config_vec.at(i)[W_SOFT_CORRIDOR] *= end_ratio;
        // cost_config_vec.at(i)[W_HARD_CORRIDOR] *= end_ratio;
        end_ratio *= 1.5;
      }
    }

    if (i == N - 1) {
      cost_config_vec.at(i)[TERMINAL_FLAG] = 1;
    } else {
      cost_config_vec.at(i)[TERMINAL_FLAG] = 0;
    }
  }

  // set const config
  ilqr_core_ptr_->SetCostConfig(cost_config_vec);

  // solve the ilqr problem
  init_state_ << planning_input.init_state().x(),
      planning_input.init_state().y(), planning_input.init_state().theta(),
      planning_input.init_state().delta();

  ilqr_core_ptr_->Solve(init_state_);

  // fail protection
  const uint8_t solver_condition =
      ilqr_core_ptr_->GetSolverInfoPtr()->solver_condition;
  const auto solver_config = ilqr_core_ptr_->GetSolverConfigPtr();

  ilqr_solver::ControlVec u_vec;
  u_vec.resize(solver_config->horizon + 1);

  for (size_t i = 0; i < u_vec.size(); ++i) {
    Control u;
    u.resize(1);
    u[0] = planning_input.control_vec(i);
    u_vec[i] = u;
  }

  // fail protection
  if (solver_condition >= iLqr::BACKWARD_PASS_FAIL) {
    ilqr_core_ptr_->Simulation(init_state_, u_vec);
  }

  // assemble planning result
  const auto &state_result = ilqr_core_ptr_->GetStateResultPtr();
  const auto &control_result = ilqr_core_ptr_->GetControlResultPtr();
  const auto &dt = ilqr_core_ptr_->GetSolverConfigPtr()->model_dt;

  double t = 0.0;
  for (size_t i = 0; i < N; ++i) {
    const auto &ref_vel = new_ref_vel[i];
    const auto v2 = ref_vel * ref_vel;
    const double kv2 = planning_input.curv_factor() * v2;
    planning_output_.mutable_time_vec()->Set(i, t);
    t += dt;

    planning_output_.mutable_x_vec()->Set(
        i, state_result->at(i)[pnc::lateral_planning::StateId::X]);
    planning_output_.mutable_y_vec()->Set(
        i, state_result->at(i)[pnc::lateral_planning::StateId::Y]);
    planning_output_.mutable_theta_vec()->Set(
        i, state_result->at(i)[pnc::lateral_planning::StateId::THETA]);
    planning_output_.mutable_delta_vec()->Set(
        i, state_result->at(i)[pnc::lateral_planning::StateId::DELTA]);
    planning_output_.mutable_acc_vec()->Set(
        i, kv2 * state_result->at(i)[pnc::lateral_planning::StateId::DELTA]);

    if (i < N - 1) {
      planning_output_.mutable_omega_vec()->Set(
          i, control_result->at(i)[pnc::lateral_planning::ControlId::OMEGA]);
      planning_output_.mutable_jerk_vec()->Set(
          i,
          kv2 * control_result->at(i)[pnc::lateral_planning::ControlId::OMEGA]);
    } else {
      planning_output_.mutable_omega_vec()->Set(
          i, planning_output_.omega_vec(i - 1));
      planning_output_.mutable_jerk_vec()->Set(
          i, planning_output_.jerk_vec(i - 1));
    }
  }

  // load solver and iteration info
  planning_output_.clear_solver_info();

  const auto &soler_info_ptr = ilqr_core_ptr_->GetSolverInfoPtr();

  planning_output_.mutable_solver_info()->set_solver_condition(
      soler_info_ptr->solver_condition);
  planning_output_.mutable_solver_info()->set_cost_size(
      soler_info_ptr->cost_size);
  planning_output_.mutable_solver_info()->set_iter_count(
      soler_info_ptr->iter_count);
  planning_output_.mutable_solver_info()->set_init_cost(
      soler_info_ptr->init_cost);

  for (size_t i = 0; i < soler_info_ptr->iter_count; ++i) {
    const auto &iter_info =
        planning_output_.mutable_solver_info()->add_iter_info();

    iter_info->set_linesearch_success(
        soler_info_ptr->iteration_info_vec[i].linesearch_success);
    iter_info->set_backward_pass_count(
        soler_info_ptr->iteration_info_vec[i].backward_pass_count);
    iter_info->set_lambda(soler_info_ptr->iteration_info_vec[i].lambda);
    iter_info->set_cost(soler_info_ptr->iteration_info_vec[i].cost);
    iter_info->set_dcost(soler_info_ptr->iteration_info_vec[i].dcost);
    iter_info->set_expect(soler_info_ptr->iteration_info_vec[i].expect);
    iter_info->set_du_norm(soler_info_ptr->iteration_info_vec[i].du_norm);
  }

  return solver_condition;
}

planning::common::LateralPlanningInput
LateralMotionPlanningProblem::SplineInput(
    planning::common::LateralPlanningInput &planning_input, std::vector<double>& dt) {
  const size_t N = 26;
  std::vector<double> hard_upper_bound_x_vec;
  std::vector<double> hard_upper_bound_y_vec;
  std::vector<double> hard_lower_bound_x_vec;
  std::vector<double> hard_lower_bound_y_vec;
  std::vector<double> soft_upper_bound_x_vec;
  std::vector<double> soft_upper_bound_y_vec;
  std::vector<double> soft_lower_bound_x_vec;
  std::vector<double> soft_lower_bound_y_vec;

  for (auto i = 0; i < N; i++) {
    t_vec_[i] = i * 0.2;
    x_vec_[i] = planning_input.ref_x_vec(i);
    y_vec_[i] = planning_input.ref_y_vec(i);
    theta_vec_[i] = planning_input.ref_theta_vec(i);

    hard_lower_bound_x_vec.emplace_back(
        planning_input.hard_lower_bound_x0_vec(i));
    hard_lower_bound_y_vec.emplace_back(
        planning_input.hard_lower_bound_y0_vec(i));
    hard_upper_bound_x_vec.emplace_back(
        planning_input.hard_upper_bound_x0_vec(i));
    hard_upper_bound_y_vec.emplace_back(
        planning_input.hard_upper_bound_y0_vec(i));
    soft_upper_bound_x_vec.emplace_back(
        planning_input.soft_upper_bound_x0_vec(i));
    soft_upper_bound_y_vec.emplace_back(
        planning_input.soft_upper_bound_y0_vec(i));
    soft_lower_bound_x_vec.emplace_back(
        planning_input.soft_lower_bound_x0_vec(i));
    soft_lower_bound_y_vec.emplace_back(
        planning_input.soft_lower_bound_y0_vec(i));
  }
  x_t_spline_.set_points(t_vec_, x_vec_);
  y_t_spline_.set_points(t_vec_, y_vec_);
  theta_t_spline_.set_points(t_vec_, theta_vec_);
  pnc::mathlib::spline h_l_x_spline;
  pnc::mathlib::spline h_l_y_spline;
  pnc::mathlib::spline h_u_x_spline;
  pnc::mathlib::spline h_u_y_spline;
  pnc::mathlib::spline s_l_x_spline;
  pnc::mathlib::spline s_l_y_spline;
  pnc::mathlib::spline s_u_x_spline;
  pnc::mathlib::spline s_u_y_spline;

  h_l_x_spline.set_points(t_vec_, hard_lower_bound_x_vec);
  h_l_y_spline.set_points(t_vec_, hard_lower_bound_y_vec);
  h_u_x_spline.set_points(t_vec_, hard_upper_bound_x_vec);
  h_u_y_spline.set_points(t_vec_, hard_upper_bound_y_vec);
  s_l_x_spline.set_points(t_vec_, soft_lower_bound_x_vec);
  s_l_y_spline.set_points(t_vec_, soft_lower_bound_y_vec);
  s_u_x_spline.set_points(t_vec_, soft_upper_bound_x_vec);
  s_u_y_spline.set_points(t_vec_, soft_upper_bound_y_vec);
  planning::common::LateralPlanningInput new_planning_input;
  new_planning_input.mutable_ref_x_vec()->Resize(N, 0.0);
  new_planning_input.mutable_ref_y_vec()->Resize(N, 0.0);
  new_planning_input.mutable_ref_theta_vec()->Resize(N, 0.0);

  new_planning_input.mutable_last_x_vec()->Resize(N, 0.0);
  new_planning_input.mutable_last_y_vec()->Resize(N, 0.0);
  new_planning_input.mutable_last_theta_vec()->Resize(N, 0.0);

  new_planning_input.mutable_soft_upper_bound_x0_vec()->Resize(N, 0.0);
  new_planning_input.mutable_soft_upper_bound_y0_vec()->Resize(N, 0.0);
  new_planning_input.mutable_soft_upper_bound_x1_vec()->Resize(N, 0.0);
  new_planning_input.mutable_soft_upper_bound_y1_vec()->Resize(N, 0.0);

  new_planning_input.mutable_soft_lower_bound_x0_vec()->Resize(N, 0.0);
  new_planning_input.mutable_soft_lower_bound_y0_vec()->Resize(N, 0.0);
  new_planning_input.mutable_soft_lower_bound_x1_vec()->Resize(N, 0.0);
  new_planning_input.mutable_soft_lower_bound_y1_vec()->Resize(N, 0.0);

  new_planning_input.mutable_hard_upper_bound_x0_vec()->Resize(N, 0.0);
  new_planning_input.mutable_hard_upper_bound_y0_vec()->Resize(N, 0.0);
  new_planning_input.mutable_hard_upper_bound_x1_vec()->Resize(N, 0.0);
  new_planning_input.mutable_hard_upper_bound_y1_vec()->Resize(N, 0.0);

  new_planning_input.mutable_hard_lower_bound_x0_vec()->Resize(N, 0.0);
  new_planning_input.mutable_hard_lower_bound_y0_vec()->Resize(N, 0.0);
  new_planning_input.mutable_hard_lower_bound_x1_vec()->Resize(N, 0.0);
  new_planning_input.mutable_hard_lower_bound_y1_vec()->Resize(N, 0.0);

  new_planning_input.mutable_control_vec()->Resize(N, 0.0);
  // int new_horizon_num = 5 / dt;
  int new_horizon_num = 26;
  for (auto i = 0; i < new_horizon_num; i++) {
    new_planning_input.mutable_ref_x_vec()->Set(i, x_t_spline_(dt[i]));
    new_planning_input.mutable_ref_y_vec()->Set(i, y_t_spline_(dt[i]));
    new_planning_input.mutable_ref_theta_vec()->Set(i, theta_t_spline_(dt[i]));
    // new_planning_input.add_ref_x_vec(x_t_spline_(i * dt));
    // new_planning_input.add_ref_y_vec(y_t_spline_(i * dt));
    // new_planning_input.add_ref_theta_vec(theta_t_spline_(i * dt));
    // new_planning_input.add_hard_lower_bound_x0_vec(h_l_x_spline(i * dt));
    // new_planning_input.add_hard_lower_bound_y0_vec(h_l_y_spline(i * dt));
    // new_planning_input.add_soft_lower_bound_x0_vec(s_l_x_spline(i * dt));
    // new_planning_input.add_soft_lower_bound_y0_vec(s_l_y_spline(i * dt));
    // new_planning_input.add_hard_upper_bound_x0_vec(h_u_x_spline(i * dt));
    // new_planning_input.add_hard_upper_bound_y0_vec(h_u_y_spline(i * dt));
    // new_planning_input.add_soft_upper_bound_x0_vec(s_u_x_spline(i * dt));
    // new_planning_input.add_soft_upper_bound_y0_vec(s_u_y_spline(i * dt));
    // new_planning_input.mutable_control_vec()->Set(i, planning_input.control_vec(i));
  }

  new_planning_input.mutable_init_state()->set_x(planning_input.init_state().x());
  new_planning_input.mutable_init_state()->set_y(planning_input.init_state().y());
  new_planning_input.mutable_init_state()->set_theta(planning_input.init_state().theta());
  new_planning_input.mutable_init_state()->set_delta(planning_input.init_state().delta());
  new_planning_input.set_ref_vel(planning_input.ref_vel());
  new_planning_input.set_curv_factor(planning_input.curv_factor());
  new_planning_input.set_q_ref_x(planning_input.q_ref_x());
  new_planning_input.set_q_ref_y(planning_input.q_ref_y());
  new_planning_input.set_q_ref_theta(planning_input.q_ref_theta());
  new_planning_input.set_q_acc(planning_input.q_acc());
  new_planning_input.set_q_jerk(planning_input.q_jerk());
  new_planning_input.set_q_acc_bound(planning_input.q_acc_bound());
  new_planning_input.set_q_jerk_bound(planning_input.q_jerk_bound());
  new_planning_input.set_q_soft_corridor(planning_input.q_soft_corridor());
  new_planning_input.set_q_hard_corridor(planning_input.q_hard_corridor());
  new_planning_input.set_acc_bound(planning_input.acc_bound());
  new_planning_input.set_jerk_bound(planning_input.jerk_bound());
  new_planning_input.set_complete_follow(planning_input.complete_follow());
  new_planning_input.set_motion_plan_concerned_index(planning_input.motion_plan_concerned_index());

  return new_planning_input;
}

}  // namespace lateral_planning
}  // namespace pnc