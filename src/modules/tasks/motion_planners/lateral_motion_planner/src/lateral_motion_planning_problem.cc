#include "lateral_motion_planning_problem.h"

#include <stdio.h>

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>

#include "ilqr_core.h"
#include "log.h"
#include "tasks/motion_planners/lateral_motion_planner/src/lateral_motion_planning_cost.h"

using namespace ilqr_solver;

static const double kMaxWheelAngle =
    360.0 / 13.0 / 57.3;  // 360 deg steering angle for scc
static const double kMaxWheelAngleRate =
    240.0 / 13.0 / 57.3;  // 240 deg/s steering angle rate for scc
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
  solver_config.du_tol = 0.01 / 57.3 / 13.0;
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
  ilqr_core_ptr_->AddCost(
      std::make_shared<PathSoftCorridorCostTerm>());  // path soft corridor cost
  ilqr_core_ptr_->AddCost(
      std::make_shared<PathHardCorridorCostTerm>());  // path hard corridor cost

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
}

uint8_t LateralMotionPlanningProblem::Update(
    const double end_ratio_for_qref, const double end_ratio_for_qjerk,
    const size_t motion_plan_concerned_start_index,
    const double concerned_start_q_jerk, const double ego_vel,
    planning::common::LateralPlanningInput &planning_input) {
  // set cost config
  const size_t N = ilqr_core_ptr_->GetSolverConfigPtr()->horizon + 1;
  std::vector<ilqr_solver::IlqrCostConfig> cost_config_vec;
  cost_config_vec.resize(N);

  const double ref_vel = planning_input.ref_vel();
  const double kv2 = planning_input.curv_factor() * ego_vel * ego_vel;

  for (size_t i = 0; i < N; ++i) {
    // calculate delta_bound and omega_bound
    double delta_bound =
        std::min(kMaxWheelAngle, planning_input.acc_bound() / kv2);
    double omega_bound =
        std::min(kMaxWheelAngleRate, planning_input.jerk_bound() / kv2);
    // reference
    cost_config_vec.at(i)[REF_X] = planning_input.ref_x_vec(i);
    cost_config_vec.at(i)[REF_Y] = planning_input.ref_y_vec(i);
    cost_config_vec.at(i)[REF_THETA] = planning_input.ref_theta_vec(i);
    cost_config_vec.at(i)[REF_VEL] = ref_vel;
    cost_config_vec.at(i)[EGO_VEL] = ego_vel;
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

    cost_config_vec.at(i)[HARD_UPPER_BOUND_X0] =
        planning_input.hard_upper_bound_x0_vec(i);
    cost_config_vec.at(i)[HARD_UPPER_BOUND_Y0] =
        planning_input.hard_upper_bound_y0_vec(i);
    cost_config_vec.at(i)[HARD_UPPER_BOUND_X1] =
        planning_input.hard_upper_bound_x1_vec(i);
    cost_config_vec.at(i)[HARD_UPPER_BOUND_Y1] =
        planning_input.hard_upper_bound_y1_vec(i);

    cost_config_vec.at(i)[HARD_LOWER_BOUND_X0] =
        planning_input.hard_lower_bound_x0_vec(i);
    cost_config_vec.at(i)[HARD_LOWER_BOUND_Y0] =
        planning_input.hard_lower_bound_y0_vec(i);
    cost_config_vec.at(i)[HARD_LOWER_BOUND_X1] =
        planning_input.hard_lower_bound_x1_vec(i);
    cost_config_vec.at(i)[HARD_LOWER_BOUND_Y1] =
        planning_input.hard_lower_bound_y1_vec(i);

    // weights
    cost_config_vec.at(i)[W_REF_X] = planning_input.q_ref_x();
    cost_config_vec.at(i)[W_REF_Y] = planning_input.q_ref_y();

    cost_config_vec.at(i)[W_REF_THETA] = planning_input.q_ref_theta();
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
        double start_step =
            std::max((concerned_start_q_jerk - planning_input.q_jerk()) /
                         motion_plan_concerned_start_index,
                     0.0);
        cost_config_vec.at(i)[W_JERK] = concerned_start_q_jerk - start_step * i;
      } else if (i > planning_input.motion_plan_concerned_index()) {
        cost_config_vec.at(i)[W_REF_X] =
            end_ratio_for_qref * cost_config_vec.at(i - 1)[W_REF_X];
        cost_config_vec.at(i)[W_REF_Y] =
            end_ratio_for_qref * cost_config_vec.at(i - 1)[W_REF_Y];
        cost_config_vec.at(i)[W_REF_THETA] =
            end_ratio_for_qref * cost_config_vec.at(i - 1)[W_REF_THETA];
        cost_config_vec.at(i)[W_JERK] =
            end_ratio_for_qjerk * cost_config_vec.at(i - 1)[W_JERK];
        // LOG_DEBUG(
        //   "motion_plan_concerned_start_index: %zu, remote_qxy: %f,
        //   remote_qjerk: %f", motion_plan_concerned_start_index, remote_xy,
        //   cost_config_vec.at(i)[W_JERK]);
      }
    }

    if (i == N - 1) {
      cost_config_vec.at(i)[TERMINAL_FLAG] = 1;
    } else {
      cost_config_vec.at(i)[TERMINAL_FLAG] = 0;
    }
  }

  // set cost config
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
    double end_ratio_for_qref, double end_ratio_for_qjerk, double max_iter,
    const size_t motion_plan_concerned_start_index,
    const double concerned_start_q_jerk, const double ego_vel,
    planning::common::LateralPlanningInput &planning_input) {
  // set cost config
  ilqr_core_ptr_->GetSolverConfigPtr()->max_iter = max_iter;
  const size_t N = ilqr_core_ptr_->GetSolverConfigPtr()->horizon + 1;
  std::vector<ilqr_solver::IlqrCostConfig> cost_config_vec;
  cost_config_vec.resize(N);

  const double ref_vel = planning_input.ref_vel();
  const double kv2 = planning_input.curv_factor() * ego_vel * ego_vel;

  for (size_t i = 0; i < N; ++i) {
    // calculate delta_bound and omega_bound
    double delta_bound =
        std::min(kMaxWheelAngle, planning_input.acc_bound() / kv2);
    double omega_bound =
        std::min(kMaxWheelAngleRate, planning_input.jerk_bound() / kv2);
    // reference
    cost_config_vec.at(i)[REF_X] = planning_input.ref_x_vec(i);
    cost_config_vec.at(i)[REF_Y] = planning_input.ref_y_vec(i);
    cost_config_vec.at(i)[REF_THETA] = planning_input.ref_theta_vec(i);
    cost_config_vec.at(i)[REF_VEL] = ref_vel;
    cost_config_vec.at(i)[EGO_VEL] = ego_vel;
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

    cost_config_vec.at(i)[W_REF_THETA] = planning_input.q_ref_theta();
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
        double start_step =
            std::max((concerned_start_q_jerk - planning_input.q_jerk()) /
                         motion_plan_concerned_start_index,
                     0.0);
        cost_config_vec.at(i)[W_JERK] = concerned_start_q_jerk - start_step * i;
      } else if (i > planning_input.motion_plan_concerned_index()) {
        cost_config_vec.at(i)[W_REF_X] =
            end_ratio_for_qref * cost_config_vec.at(i - 1)[W_REF_X];
        cost_config_vec.at(i)[W_REF_Y] =
            end_ratio_for_qref * cost_config_vec.at(i - 1)[W_REF_Y];
        cost_config_vec.at(i)[W_REF_THETA] =
            end_ratio_for_qref * cost_config_vec.at(i - 1)[W_REF_THETA];
        cost_config_vec.at(i)[W_JERK] =
            end_ratio_for_qjerk * cost_config_vec.at(i - 1)[W_JERK];
      }
    }

    if (i == N - 1) {
      cost_config_vec.at(i)[TERMINAL_FLAG] = 1;
    } else {
      cost_config_vec.at(i)[TERMINAL_FLAG] = 0;
    }
  }

  // set cost config
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

}  // namespace lateral_planning
}  // namespace pnc