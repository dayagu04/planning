#include "joint_motion_planning_problem.h"

#include <stdio.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>

#include "ilqr_define.h"
#include "joint_motion_planning_cost.h"
#include "joint_motion_planning_model.h"
#include "planning_context.h"
using namespace ilqr_solver;
using namespace pnc::mathlib;
namespace pnc {
namespace joint_motion_planning {
void JointMotionPlanningProblem::Init() {
  iLqrSolverConfig solver_config;
  solver_config.horizon = 25;
  solver_config.state_size = 6;
  solver_config.input_size = 2;
  solver_config.model_dt = 0.2;
  solver_config.du_tol = 0.004;
  solver_config.max_iter = 15;
  solver_config.lambda_min = 1e-5;
  ilqr_core_ptr_ = std::make_shared<iLqr>();
  ilqr_core_ptr_->Init(std::make_shared<JointMotionPlanningModel>(),
                       solver_config);
  ilqr_core_ptr_->AddCost(std::make_shared<EgoReferenceCostTerm>());
  ilqr_core_ptr_->AddCost(std::make_shared<EgoThreeDiscSafeCostTerm>());
  ilqr_core_ptr_->AddCost(std::make_shared<HardHalfplaneCostTerm>());
  ilqr_core_ptr_->AddCost(std::make_shared<SoftHalfplaneCostTerm>());
  ego_road_boundary_cost_term_ = std::make_shared<EgoRoadBoundaryCostTerm>();
  ilqr_core_ptr_->AddCost(ego_road_boundary_cost_term_);
  ilqr_core_ptr_->AddCost(std::make_shared<EgoAccCostTerm>());
  ilqr_core_ptr_->AddCost(std::make_shared<EgoJerkCostTerm>());
  ilqr_core_ptr_->AddCost(std::make_shared<EgoDeltaCostTerm>());
  ilqr_core_ptr_->AddCost(std::make_shared<EgoOmegaCostTerm>());
  ilqr_core_ptr_->AddCost(std::make_shared<EgoAccBoundCostTerm>());
  ilqr_core_ptr_->AddCost(std::make_shared<EgoJerkBoundCostTerm>());
  // Obstacles use reference trajectories, no optimization cost terms.
  ilqr_core_ptr_->InitAdvancedInfo();
  const auto &N = ilqr_core_ptr_->GetSolverConfigPtr()->horizon + 1;
  planning_output_.mutable_time_vec()->Resize(N, 0.0);
  planning_output_.mutable_x_vec()->Resize(N, 0.0);
  planning_output_.mutable_y_vec()->Resize(N, 0.0);
  planning_output_.mutable_theta_vec()->Resize(N, 0.0);
  planning_output_.mutable_vel_vec()->Resize(N, 0.0);
  planning_output_.mutable_acc_vec()->Resize(N, 0.0);
  planning_output_.mutable_delta_vec()->Resize(N, 0.0);
  planning_output_.mutable_omega_vec()->Resize(N, 0.0);
  planning_output_.mutable_jerk_vec()->Resize(N, 0.0);
  planning_output_.mutable_s_vec()->Resize(N, 0.0);
}
void JointMotionPlanningProblem::SetBoundaryPaths(
    std::shared_ptr<planning::planning_math::KDPath> road_left,
    std::shared_ptr<planning::planning_math::KDPath> road_right) {
  if (ego_road_boundary_cost_term_) {
    ego_road_boundary_cost_term_->UpdateBoundaryPaths(road_left, road_right);
  }
}

uint8_t JointMotionPlanningProblem::Update(
    planning::common::JointMotionPlanningInput &planning_input) {
  obs_num_ = planning_input.obs_num();

  const size_t N = ilqr_core_ptr_->GetSolverConfigPtr()->horizon + 1;

  std::vector<IlqrCostConfig> cost_config_vec;
  cost_config_vec.resize(N);
  for (size_t i = 0; i < N; ++i) {
    // Ego vehicle reference trajectory configuration.
    cost_config_vec.at(i)[EGO_REF_X] = planning_input.ref_x_vec(i);
    cost_config_vec.at(i)[EGO_REF_Y] = planning_input.ref_y_vec(i);
    cost_config_vec.at(i)[EGO_REF_THETA] = planning_input.ref_theta_vec(i);
    cost_config_vec.at(i)[EGO_REF_DELTA] = planning_input.ref_delta_vec(i);
    cost_config_vec.at(i)[EGO_REF_VEL] = planning_input.ref_vel_vec(i);
    cost_config_vec.at(i)[EGO_REF_ACC] = planning_input.ref_acc_vec(i);

    // Store obstacle reference trajectories for cost evaluation.
    for (size_t j = 0; j < obs_num_; ++j) {
      cost_config_vec.at(i)[GetObsRefStateIdx(j, obs_num_, OBS_X)] =
          planning_input.obs_ref_trajectory(j).ref_x_vec(i);
      cost_config_vec.at(i)[GetObsRefStateIdx(j, obs_num_, OBS_Y)] =
          planning_input.obs_ref_trajectory(j).ref_y_vec(i);
      cost_config_vec.at(i)[GetObsRefStateIdx(j, obs_num_, OBS_THETA)] =
          planning_input.obs_ref_trajectory(j).ref_theta_vec(i);
      cost_config_vec.at(i)[GetObsRefStateIdx(j, obs_num_, OBS_DELTA)] =
          planning_input.obs_ref_trajectory(j).ref_delta_vec(i);
      cost_config_vec.at(i)[GetObsRefStateIdx(j, obs_num_, OBS_VEL)] =
          planning_input.obs_ref_trajectory(j).ref_vel_vec(i);
      cost_config_vec.at(i)[GetObsRefStateIdx(j, obs_num_, OBS_ACC)] =
          planning_input.obs_ref_trajectory(j).ref_acc_vec(i);
    }

    // Ego vehicle reference trajectory weights.
    cost_config_vec.at(i)[W_EGO_REF_X] = planning_input.q_ego_ref_x();
    cost_config_vec.at(i)[W_EGO_REF_Y] = planning_input.q_ego_ref_y();
    cost_config_vec.at(i)[W_EGO_REF_THETA] = planning_input.q_ego_ref_theta();
    cost_config_vec.at(i)[W_EGO_REF_DELTA] = planning_input.q_ego_ref_delta();
    cost_config_vec.at(i)[W_EGO_REF_VEL] = planning_input.q_ego_ref_vel();
    cost_config_vec.at(i)[W_EGO_REF_ACC] = planning_input.q_ego_ref_acc();

    double three_disc_safe_dist_weight_decay_rate = 0.1;
    double three_disc_safe_dist_weight_decay_factor =
        std::exp(-three_disc_safe_dist_weight_decay_rate * i);
    cost_config_vec.at(i)[W_THREE_DISC_SAFE_DIST_WEIGHT] =
        planning_input.q_three_disc_safe_dist_weight() *
        three_disc_safe_dist_weight_decay_factor;

    double boundary_decay_rate = 0.1;
    double boundary_decay_factor = std::exp(-boundary_decay_rate * i);
    cost_config_vec.at(i)[W_ROAD_BOUNDARY] =
        planning_input.q_road_boundary_weight() * boundary_decay_factor;

    cost_config_vec.at(i)[W_EGO_ACC] = planning_input.q_ego_acc_weight();

    cost_config_vec.at(i)[W_EGO_JERK] = planning_input.q_ego_jerk_weight();

    cost_config_vec.at(i)[W_EGO_OMEGA] = planning_input.q_ego_omega_weight();

    cost_config_vec.at(i)[W_EGO_DELTA] = planning_input.q_ego_delta_weight();

    cost_config_vec.at(i)[W_EGO_ACC_BOUND] =
        planning_input.q_ego_acc_bound_weight();
    if (i < planning_input.ego_acc_max_size()) {
      cost_config_vec.at(i)[EGO_ACC_MAX] = planning_input.ego_acc_max(i);
    }
    if (i < planning_input.ego_acc_min_size()) {
      cost_config_vec.at(i)[EGO_ACC_MIN] = planning_input.ego_acc_min(i);
    }

    cost_config_vec.at(i)[W_EGO_JERK_BOUND] =
        planning_input.q_ego_jerk_bound_weight();
    if (i < planning_input.ego_jerk_max_size()) {
      cost_config_vec.at(i)[EGO_JERK_MAX] = planning_input.ego_jerk_max(i);
    }
    if (i < planning_input.ego_jerk_min_size()) {
      cost_config_vec.at(i)[EGO_JERK_MIN] = planning_input.ego_jerk_min(i);
    }

    double hard_halfplane_bound_decay_rate = 0.15;
    double hard_halfplane_bound_decay =
        std::exp(-hard_halfplane_bound_decay_rate * i);
    cost_config_vec.at(i)[W_HARD_HALFPLANE] =
        planning_input.q_hard_halfplane_weight() * hard_halfplane_bound_decay;
    cost_config_vec.at(i)[HARD_HALFPLANE_DIST] =
        planning_input.hard_halfplane_dist();
    cost_config_vec.at(i)[HALFPLANE_COST_ALLOCATION_RATIO] =
        planning_input.halfplane_cost_allocation_ratio();

    double soft_halfplane_bound_decay_rate = 0.15;
    double soft_halfplane_bound_decay =
        std::exp(-soft_halfplane_bound_decay_rate * i);
    cost_config_vec.at(i)[W_SOFT_HALFPLANE] =
        planning_input.q_soft_halfplane_weight() * soft_halfplane_bound_decay;
    cost_config_vec.at(i)[SOFT_HALFPLANE_S0] =
        planning_input.soft_halfplane_s0();
    cost_config_vec.at(i)[SOFT_HALFPLANE_TAU] =
        planning_input.soft_halfplane_tau();
    cost_config_vec.at(i)[SOFT_HALFPLANE_COST_ALLOCATION_RATIO] =
        planning_input.soft_halfplane_cost_allocation_ratio();

    const auto &vehicle_param =
        planning::VehicleConfigurationContext::Instance()->get_vehicle_param();
    cost_config_vec.at(i)[CURV_FACTOR] = planning_input.curv_factor_vec(i);
    cost_config_vec.at(i)[EGO_LENGTH] = vehicle_param.length;
    cost_config_vec.at(i)[EGO_WIDTH] = vehicle_param.width;
    cost_config_vec.at(i)[EGO_FRONT_EDGE_TO_REAR_AXLE] =
        vehicle_param.front_edge_to_rear_axle;
    cost_config_vec.at(i)[EGO_WHEEL_BASE] = vehicle_param.wheel_base;

    cost_config_vec.at(i)[THREE_DISC_SAFE_DIST] =
        planning_input.three_disc_safe_dist();
    cost_config_vec.at(i)[ROAD_BOUNDARY_SAFE_DIST] =
        planning_input.road_boundary_safe_dist();

    cost_config_vec.at(i)[OBS_NUM] = planning_input.obs_num();
    for (size_t j = 0; j < obs_num_; ++j) {
      cost_config_vec.at(i)[GetObsCurvFactorIdx(j)] =
          planning_input.obs_ref_trajectory(j).curv_factor_vec(i);
      cost_config_vec.at(i)[GetObsLengthIdx(j, obs_num_)] =
          planning_input.obs_ref_trajectory(j).length();
      cost_config_vec.at(i)[GetObsWidthIdx(j, obs_num_)] =
          planning_input.obs_ref_trajectory(j).width();
      cost_config_vec.at(i)[GetObsLongitudinalLabelIdx(j, obs_num_)] =
          planning_input.obs_ref_trajectory(j).longitudinal_label();
    }
  }
  ilqr_core_ptr_->SetCostConfig(cost_config_vec);

  init_state_.resize(EGO_STATE_SIZE);
  init_state_ << planning_input.ego_init_state().x(),
      planning_input.ego_init_state().y(),
      planning_input.ego_init_state().theta(),
      planning_input.ego_init_state().delta(),
      planning_input.ego_init_state().vel(),
      planning_input.ego_init_state().acc();

  ilqr_core_ptr_->Solve(init_state_);

  const uint8_t solver_condition =
      ilqr_core_ptr_->GetSolverInfoPtr()->solver_condition;
  if (solver_condition >= iLqr::BACKWARD_PASS_FAIL) {
    u_vec_.clear();
    u_vec_.resize(N);
    for (size_t i = 0; i < N; ++i) {
      ilqr_solver::Control u(EGO_CONTROL_SIZE);
      u.setZero();
      u_vec_[i] = u;
    }
    ilqr_core_ptr_->Simulation(init_state_, u_vec_);
  }
  const auto &state_result = ilqr_core_ptr_->GetStateResultPtr();
  const auto &control_result = ilqr_core_ptr_->GetControlResultPtr();
  const auto &dt = ilqr_core_ptr_->GetSolverConfigPtr()->model_dt;
  double t = 0.0;
  double s = 0.0;
  for (size_t i = 0; i < N; ++i) {
    planning_output_.mutable_time_vec()->Set(i, t);
    t += dt;
    planning_output_.mutable_x_vec()->Set(i, state_result->at(i)[EGO_X]);
    planning_output_.mutable_y_vec()->Set(i, state_result->at(i)[EGO_Y]);
    planning_output_.mutable_theta_vec()->Set(i,
                                              state_result->at(i)[EGO_THETA]);
    planning_output_.mutable_vel_vec()->Set(i, state_result->at(i)[EGO_VEL]);
    planning_output_.mutable_acc_vec()->Set(i, state_result->at(i)[EGO_ACC]);
    planning_output_.mutable_delta_vec()->Set(i,
                                              state_result->at(i)[EGO_DELTA]);
    planning_output_.mutable_omega_vec()->Set(i,
                                              control_result->at(i)[EGO_OMEGA]);
    planning_output_.mutable_jerk_vec()->Set(i,
                                             control_result->at(i)[EGO_JERK]);

    if (i > 0) {
      const double dx =
          state_result->at(i)[EGO_X] - state_result->at(i - 1)[EGO_X];
      const double dy =
          state_result->at(i)[EGO_Y] - state_result->at(i - 1)[EGO_Y];
      s += std::max(1e-3, std::sqrt(dx * dx + dy * dy));
    }
    planning_output_.mutable_s_vec()->Set(i, s);
  }

  // Output obstacle reference trajectories.
  planning_output_.clear_obs_opt_trajectory();
  for (size_t j = 0; j < obs_num_; ++j) {
    auto *obs_traj = planning_output_.add_obs_opt_trajectory();
    const size_t ref_N = planning_input.obs_ref_trajectory(j).ref_x_vec_size();
    for (size_t i = 0; i < std::min(ref_N, N); ++i) {
      obs_traj->add_x_vec(planning_input.obs_ref_trajectory(j).ref_x_vec(i));
      obs_traj->add_y_vec(planning_input.obs_ref_trajectory(j).ref_y_vec(i));
      obs_traj->add_theta_vec(
          planning_input.obs_ref_trajectory(j).ref_theta_vec(i));
      obs_traj->add_vel_vec(
          planning_input.obs_ref_trajectory(j).ref_vel_vec(i));
      obs_traj->add_acc_vec(
          planning_input.obs_ref_trajectory(j).ref_acc_vec(i));
      obs_traj->add_delta_vec(
          planning_input.obs_ref_trajectory(j).ref_delta_vec(i));
      obs_traj->add_omega_vec(0.0);
      obs_traj->add_jerk_vec(0.0);
      obs_traj->add_s_vec(planning_input.obs_ref_trajectory(j).ref_s_vec(i));
    }
  }
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
      planning_output_.mutable_solver_info()->add_cost_vec(
          solver_info_ptr->cost_iter_vec[i].at(j));
    }
  }
  return true;
}
}  // namespace joint_motion_planning
}  // namespace pnc
