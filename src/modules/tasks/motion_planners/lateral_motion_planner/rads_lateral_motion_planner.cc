
#include "rads_lateral_motion_planner.h"

#include <algorithm>
#include <cstddef>
#include <iostream>
#include <vector>

#include "config/basic_type.h"
#include "constraint_terms/lateral_acc_constraint.h"
#include "constraint_terms/lateral_jerk_constraint.h"
#include "constraint_terms/path_corridor_constraint.h"
#include "cost_terms/continuity_cost.h"
#include "cost_terms/edt_distance_cost.h"
#include "cost_terms/lateral_acc_cost.h"
#include "cost_terms/lateral_jerk_cost.h"
#include "cost_terms/reference_path_cost.h"
#include "debug_info_log.h"
#include "dynamic_model/dynamic_model.h"
#include "edt_manager.h"
#include "ilqr_define.h"
#include "lateral_motion_planner.pb.h"
#include "lateral_obstacle.h"
#include "math_lib.h"
#include "planning_context.h"
#include "spline.h"
#include "virtual_lane_manager.h"

static const double avoid_dist_thr = 0.1;
static const int low_speed_lane_change_cd_timer_thr = 10;

namespace planning {
RADSLateralMotionPlanner::RADSLateralMotionPlanner(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : BaseLateralMotionPlanner(config_builder, session) {
  name_ = "RADSLateralMotionPlanner";
  Init();
};

void RADSLateralMotionPlanner::Init() {
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  // 1.init solver config
  ilqr_solver::iLqrSolverConfig solver_config;
  solver_config.horizon = config_.horizon;
  solver_config.state_size = pnc::lateral_planning::StateID::STATE_SIZE;
  solver_config.input_size = pnc::lateral_planning::ControlID::CONTROL_SIZE;
  solver_config.model_dt = config_.delta_t;
  solver_config.warm_start_enable = config_.warm_start_enable;
  solver_config.du_tol = config_.du_tol / vehicle_param.steer_ratio / 57.3;
  solver_config.max_iter = config_.max_iter;
  // 2.init model
  dynamic_model_ = std::make_shared<pnc::lateral_planning::DynamicModel>();
  // 2.1: rear axle reference cost
  dynamic_model_->AddCost(std::make_shared<pnc::lateral_planning::ReferencePathCostTerm>());
  // 2.2: rear axle reference continuity cost
  dynamic_model_->AddCost(std::make_shared<pnc::lateral_planning::ContinuityCostTerm>());
  // 2.3: lateral acc cost
  dynamic_model_->AddCost(std::make_shared<pnc::lateral_planning::LateralAccCostTerm>());
  // 2.4: lateral jerk cost
  dynamic_model_->AddCost(std::make_shared<pnc::lateral_planning::LateralJerkCostTerm>());
  // 2.5: lateral acc bound cost
  dynamic_model_->AddCost(std::make_shared<pnc::lateral_planning::LateralAccBoundCostTerm>());
  // 2.6: lateral jerk soft bound cost
  dynamic_model_->AddCost(std::make_shared<pnc::lateral_planning::LateralJerkBoundCostTerm>());
  // // 2.7: path first soft corridor cost
  // dynamic_model_->AddCost(std::make_shared<pnc::lateral_planning::PathFirstSoftCorridorCostTerm>());
  // 2.8: path second soft corridor cost
  dynamic_model_->AddCost(std::make_shared<pnc::lateral_planning::PathSecondSoftCorridorCostTerm>());
  // 2.9: path hard corridor cost
  dynamic_model_->AddCost(std::make_shared<pnc::lateral_planning::PathHardCorridorCostTerm>());
  // 2.10: edt distance cost
  auto edt_ptr =
      session_->environmental_model().get_edt_manager()->GetEulerDistanceTransform();
  dynamic_model_->AddCost(std::make_shared<pnc::lateral_planning::EdtDistanceCostTerm>(&ego_base_, edt_ptr));
  // 3.init solver
  ilqr_solver_ptr_ =
      std::make_shared<pnc::lateral_planning::iLQRSolver>();
  ilqr_solver_ptr_->Init(solver_config, dynamic_model_);
  // 4.init input and output
  // InitInputAndOutput();
  is_uniform_motion_ = true;
  is_need_reverse_ = true;
}

bool RADSLateralMotionPlanner::Execute() {
  ILOG_DEBUG << "=======RADSLateralMotionPlanner=======";

  if (!PreCheck()) {
    ILOG_DEBUG << "PreCheck failed";
    return false;
  }

  auto start_time = IflyTime::Now_ms();

  // handle input
  if (!HandleInputData()) {
    ILOG_DEBUG << "RADSLateralMotionPlanner AssembleInput failed";
    SaveDebugInfo();
    return false;
  }
  // update
  if (!Update()) {
    ILOG_DEBUG << "RADSLateralMotionPlanner Solve failed";
    SaveDebugInfo();
    return false;
  }
  // handle output
  if (!HandleOutputData()) {
    ILOG_DEBUG << "RADSLateralMotionPlanner Solve failed";
    SaveDebugInfo();
    return false;
  }

  auto end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("LateralMotionCostTime", end_time - start_time);
  SaveDebugInfo();
  return true;
}

bool RADSLateralMotionPlanner::HandleInputData() {
  // reset
  ResetInputAndOutput();
  // assemble input
  bool is_input_valid = AssembleInput();
  return is_input_valid;
}

bool RADSLateralMotionPlanner::AssembleInput() {
  is_uniform_motion_ = true;
  is_need_reverse_ = true;
  if (!HandleReferencePathData()) {
    return false;
  }
  if (!HandleLateralBoundData()) {
    return false;
  }
  if (!HandleFeedbackInfoData()) {
    return false;
  }
  const auto &general_lateral_decider_output =
      session_->planning_context().general_lateral_decider_output();
  const auto &soft_bounds_frenet_point =
      general_lateral_decider_output.second_soft_bounds_frenet_point;
  const auto &hard_bounds_frenet_point =
      general_lateral_decider_output.hard_bounds_frenet_point;
  planning_weight_ptr_->CalculateLatAvoidDistance(soft_bounds_frenet_point);
  //
  planning_weight_ptr_->SetLateralMotionWeightForRADS(planning_input_);
  planning_weight_ptr_->LimitAccBoundAndJerkBound(
      max_wheel_angle_, max_wheel_angle_rate_, planning_input_);
  //
  ego_base_.SetBasePose(Pose2f(planning_input_.init_state().x(),
                               planning_input_.init_state().y(),
                               planning_input_.init_state().theta()));
  auto& edt_manager =
      session_->environmental_model().get_edt_manager();
  edt_manager->UpdateByLatDecision();
  return true;
}

bool RADSLateralMotionPlanner::Update() {
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double concerned_start_q_jerk =
      planning_weight_ptr_->GetConcernedStartQJerk();
  JSON_DEBUG_VALUE("concerned_start_q_jerk", concerned_start_q_jerk);
  const double end_ratio_for_qrefxy =
      planning_weight_ptr_->GetConcernedEndRatioForXY();
  const double end_ratio_for_qreftheta =
      planning_weight_ptr_->GetConcernedEndRatioForTheta();
  auto start_time = IflyTime::Now_ms();
  auto solver_condition = ilqr_solver_ptr_->Update(
      end_ratio_for_qrefxy, end_ratio_for_qreftheta,
      config_.end_ratio_for_qjerk, concerned_start_q_jerk,
      vehicle_param.wheel_base, virtual_ref_x_, virtual_ref_y_,
      virtual_ref_theta_, planning_weight_ptr_, planning_input_);
  JSON_DEBUG_VALUE("solver_condition", solver_condition);
  auto end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("iLqr_lat_update_time", end_time - start_time);

  // get output
  const auto &state_result = ilqr_solver_ptr_->GetiLqrCorePtr()->GetStateResultPtr();
  const auto &control_result = ilqr_solver_ptr_->GetiLqrCorePtr()->GetControlResultPtr();

  const size_t N = config_.horizon + 1;
  const double dt = config_.delta_t;
  const double kv2 = curv_factor_ * planning_input_.ref_vel() * planning_input_.ref_vel();
  double s = 0.0;
  double t = 0.0;
  for (size_t i = 0; i < N; ++i) {
    planning_output_.mutable_time_vec()->Set(i, t);
    t += dt;

    planning_output_.mutable_x_vec()->Set(
        i, state_result->at(i)[pnc::lateral_planning::StateID::X]);
    planning_output_.mutable_y_vec()->Set(
        i, state_result->at(i)[pnc::lateral_planning::StateID::Y]);
    planning_output_.mutable_theta_vec()->Set(
        i, state_result->at(i)[pnc::lateral_planning::StateID::THETA]);
    planning_output_.mutable_delta_vec()->Set(
        i, state_result->at(i)[pnc::lateral_planning::StateID::DELTA]);
    planning_output_.mutable_acc_vec()->Set(
        i, kv2 * state_result->at(i)[pnc::lateral_planning::StateID::DELTA]);

    if (i < N - 1) {
      planning_output_.mutable_omega_vec()->Set(
          i, control_result->at(i)[pnc::lateral_planning::ControlID::OMEGA]);
      planning_output_.mutable_jerk_vec()->Set(
          i,
          kv2 * control_result->at(i)[pnc::lateral_planning::ControlID::OMEGA]);
    } else {
      planning_output_.mutable_omega_vec()->Set(
          i, planning_output_.omega_vec(i - 1));
      planning_output_.mutable_jerk_vec()->Set(
          i, planning_output_.jerk_vec(i - 1));
    }
  }
  // load solver and iteration info
  planning_output_.clear_solver_info();
  const auto &soler_info_ptr = ilqr_solver_ptr_->GetiLqrCorePtr()->GetSolverInfoPtr();
  // const uint8_t  solver_condition = soler_info_ptr->solver_condition;
  planning_output_.mutable_solver_info()->set_solver_condition(
      soler_info_ptr->solver_condition);
  planning_output_.mutable_solver_info()->set_cost_size(
      soler_info_ptr->cost_size);
  planning_output_.mutable_solver_info()->set_iter_count(
      soler_info_ptr->iter_count);
  planning_output_.mutable_solver_info()->set_init_cost(
      soler_info_ptr->init_cost);
  for (size_t i = 0; i < soler_info_ptr->iter_count; ++i) {
    const auto& iter_info =
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
    for (size_t j = 0; j < soler_info_ptr->iteration_info_vec[i].cost_vec.size(); ++j) {
      const auto& cost_info = iter_info->add_cost_info();
      cost_info->set_id(soler_info_ptr->iteration_info_vec[i].cost_vec[j].id);
      cost_info->set_cost(soler_info_ptr->iteration_info_vec[i].cost_vec[j].cost);
    }
  }
  return true;
}
}  // namespace planning
