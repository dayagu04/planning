
#include "scc_lateral_motion_planner.h"

#include <algorithm>
#include <cstddef>
#include <iostream>
#include <vector>

#include "config/basic_type.h"
#include "constraint_terms/lateral_acc_constraint.h"
#include "constraint_terms/lateral_jerk_constraint.h"
#include "constraint_terms/path_corridor_constraint.h"
#include "cost_terms/continuity_cost.h"
#include "cost_terms/lateral_acc_cost.h"
#include "cost_terms/lateral_jerk_cost.h"
#include "cost_terms/reference_path_cost.h"
#include "debug_info_log.h"
#include "dynamic_model/dynamic_model.h"
#include "ilqr_define.h"
#include "lateral_motion_planner.pb.h"
#include "lateral_obstacle.h"
#include "math_lib.h"
#include "planning_context.h"
#include "problem_solver/solver_define.h"
#include "spline.h"
#include "virtual_lane_manager.h"

static const double avoid_dist_thr = 0.1;
static const int low_speed_lane_change_cd_timer_thr = 10;

namespace planning {
SCCLateralMotionPlanner::SCCLateralMotionPlanner(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : BaseLateralMotionPlanner(config_builder, session) {
  name_ = "SCCLateralMotionPlanner";
  Init();
};

void SCCLateralMotionPlanner::Init() {
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
  // 2.3: front axle reference cost
  dynamic_model_->AddCost(std::make_shared<pnc::lateral_planning::FrontReferencePathCostTerm>());
  // 2.4: virtual rear axle reference cost
  dynamic_model_->AddCost(std::make_shared<pnc::lateral_planning::VirtualReferencePathCostTerm>());
  // 2.5: lateral acc cost
  dynamic_model_->AddCost(std::make_shared<pnc::lateral_planning::LateralAccCostTerm>());
  // 2.6: lateral jerk cost
  dynamic_model_->AddCost(std::make_shared<pnc::lateral_planning::LateralJerkCostTerm>());
  // 2.7: lateral acc bound cost
  dynamic_model_->AddCost(std::make_shared<pnc::lateral_planning::LateralAccBoundCostTerm>());
  // 2.8: lateral jerk soft bound cost
  dynamic_model_->AddCost(std::make_shared<pnc::lateral_planning::LateralJerkBoundCostTerm>());
  // 2.9: path first soft corridor cost
  dynamic_model_->AddCost(std::make_shared<pnc::lateral_planning::PathFirstSoftCorridorCostTerm>());
  // 2.10: path second soft corridor cost
  dynamic_model_->AddCost(std::make_shared<pnc::lateral_planning::PathSecondSoftCorridorCostTerm>());
  // 2.11: path hard corridor cost
  dynamic_model_->AddCost(std::make_shared<pnc::lateral_planning::PathHardCorridorCostTerm>());
  // 3.init solver
  ilqr_solver_ptr_ =
      std::make_shared<pnc::lateral_planning::iLQRSolver>();
  ilqr_solver_ptr_->Init(solver_config, dynamic_model_);
  // 4.init input and output
  // InitInputAndOutput();
  // 5.init variable
  is_uniform_motion_ = false;
  is_need_reverse_ = false;
  is_divide_lane_into_two_ = false;
  is_last_low_speed_lane_change_ = false;
  low_speed_lane_change_cd_timer_ = 0;
  avoid_back_time_ = 0.0;
  enter_split_time_ = 0.0;
  enter_lccnoa_time_ = 0.0;
  driving_away_lane_time_ = 100.0;
  const size_t N = config_.horizon + 1;
  expected_steer_vec_.resize(N, 0.0);
  history_steer_vec_.reserve(N);
}

bool SCCLateralMotionPlanner::Execute() {
  ILOG_DEBUG << "=======SCCLateralMotionPlanner=======";

  if (!PreCheck()) {
    ILOG_DEBUG << "PreCheck failed";
    return false;
  }

  auto start_time = IflyTime::Now_ms();

  // handle input
  if (!HandleInputData()) {
    ILOG_DEBUG << "SCCLateralMotionPlanner AssembleInput failed";
    SaveDebugInfo();
    return false;
  }
  // update
  if (!Update()) {
    ILOG_DEBUG << "SCCLateralMotionPlanner Solve failed";
    SaveDebugInfo();
    return false;
  }
  // handle output
  if (!HandleOutputData()) {
    ILOG_DEBUG << "SCCLateralMotionPlanner Solve failed";
    SaveDebugInfo();
    return false;
  }

  auto end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("LateralMotionCostTime", end_time - start_time);
  SaveDebugInfo();
  return true;
}

bool SCCLateralMotionPlanner::HandleInputData() {
  // reset
  ResetInput();
  // assemble input
  bool is_input_valid = AssembleInput();
  return is_input_valid;
}

void SCCLateralMotionPlanner::ResetInput() {
  ResetInputAndOutput();
  std::fill(expected_steer_vec_.begin(), expected_steer_vec_.end(), 0.0);
  history_steer_vec_.clear();
  driving_away_lane_time_ = 100.0;
}

bool SCCLateralMotionPlanner::AssembleInput() {
  if (!HandleReferencePathData()) {
    return false;
  }
  if (!HandleLateralBoundData()) {
    return false;
  }
  if (!HandleFeedbackInfoData()) {
    return false;
  }
  //
  bool is_in_function = session_->environmental_model().GetVehicleDbwStatus();
  if (config_.pass_acc_mode) {
    const auto &function_mode =
        session_->environmental_model().function_info().function_mode();
    is_in_function = function_mode == common::DrivingFunctionInfo::SCC ||
                     function_mode == common::DrivingFunctionInfo::NOA;
  }
  if (is_in_function) {
    if (enter_lccnoa_time_ < 3.0) {
      enter_lccnoa_time_ += 0.1;
    }
  } else {
    enter_lccnoa_time_ = 0;
  }
  //
  if (config_.enable_straight_path && is_in_function) {
    StraightPath();
    return true;;
  }
  //
  const auto& motion_planner_output =
      session_->planning_context().motion_planner_output();
  auto& mutable_motion_planner_output =
      session_->mutable_planning_context()->mutable_motion_planner_output();
  const auto& general_lateral_decider_output =  // result from lat decision
      session_->planning_context().general_lateral_decider_output();
  bool complete_follow = general_lateral_decider_output.complete_follow;
  const bool lane_change_scene =
      general_lateral_decider_output.lane_change_scene;
  const bool ramp_scene = general_lateral_decider_output.ramp_scene;
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto& lc_request_direction = lane_change_decider_output.lc_request;
  const auto& coarse_planning_info = lane_change_decider_output.coarse_planning_info;
  const auto& reference_path_ptr = coarse_planning_info.reference_path;
  const auto& planning_init_point =
      reference_path_ptr->get_frenet_ego_state().planning_init_point();
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  planning_weight_ptr_->CalculateExpectedLatAccAndSteerAngle(
      planning_init_point.frenet_state.s, planning_input_.ref_vel(),
      vehicle_param.wheel_base, vehicle_param.steer_ratio, curv_factor_,
      reference_path_ptr, expected_steer_vec_);
  JSON_DEBUG_VECTOR("expected_steer_vec", expected_steer_vec_, 2)
  const auto& second_soft_bounds_frenet_point =
      general_lateral_decider_output.second_soft_bounds_frenet_point;
  const auto& hard_bounds_frenet_point =
      general_lateral_decider_output.hard_bounds_frenet_point;
  planning_weight_ptr_->CalculateLatAvoidDistance(second_soft_bounds_frenet_point);
  const auto& second_soft_bounds = general_lateral_decider_output.second_soft_bounds;
  const auto& hard_bounds = general_lateral_decider_output.hard_bounds;
  const auto& second_soft_bounds_info =
      general_lateral_decider_output.second_soft_bounds_info;
  const auto& hard_bounds_info =
      general_lateral_decider_output.hard_bounds_info;
  planning_weight_ptr_->CalculateLatAvoidBoundPriority(
      second_soft_bounds_frenet_point, hard_bounds_frenet_point,
      second_soft_bounds, hard_bounds, second_soft_bounds_info,
      hard_bounds_info);
  //
  const auto target_state =
      lane_change_decider_output.coarse_planning_info.target_state;
  bool is_merge_lc =
      lane_change_decider_output.lc_request_source == MERGE_REQUEST ||
      lane_change_decider_output.lc_request_source == MAP_REQUEST;
  NudgeDirection drive_away_direction = CalculateDrivingDirectionForLeavingLane();
  bool is_check_left_line = drive_away_direction == NudgeDirection::LEFT;
  bool is_check_right_line = drive_away_direction == NudgeDirection::RIGHT;
  // use spatio result
  bool is_use_spatio_planner_result =
      general_lateral_decider_output.is_use_spatio_planner_result;
  if (is_use_spatio_planner_result) {
    is_check_left_line = true;
    is_check_right_line = true;
  }
  if (target_state == kLaneChangeExecution) {
    if (lc_request_direction == LEFT_CHANGE) {
      is_check_left_line = true;
      if (!is_merge_lc) {
        is_check_right_line = false;
      }
    } else if (lc_request_direction == RIGHT_CHANGE) {
      is_check_right_line = true;
      if (!is_merge_lc) {
        is_check_left_line = false;
      }
    }
  }
  double remain_nonsolid_line_time = CalculateRemainingDrivingTimeToSolidLine(is_check_left_line, is_check_right_line);
  if (remain_nonsolid_line_time <= 4.0) {
    is_use_spatio_planner_result = false;
  }
  planning_weight_ptr_->SetIsUseSpatioPlannerResult(
      is_use_spatio_planner_result);
  // intersection
  auto intersection_state = session_->environmental_model()
                                .get_virtual_lane_manager()
                                ->GetIntersectionState();
  bool is_approach_intersection =
      intersection_state ==
      planning::common::IntersectionState::APPROACH_INTERSECTION;
  bool is_in_intersection =
      intersection_state ==
      planning::common::IntersectionState::IN_INTERSECTION;
  bool is_off_intersection =
      intersection_state ==
      planning::common::IntersectionState::OFF_INTERSECTION;
  // planning_weight_ptr_->SetIsInIntersection(is_in_intersection);
  if (remain_nonsolid_line_time > 4.0 && (is_use_spatio_planner_result ||
      (is_approach_intersection || is_in_intersection))) {
    planning_weight_ptr_->SetIsInIntersection(true);
  } else {
    planning_weight_ptr_->SetIsInIntersection(false);
  }
  // split
  bool split_scene = IsLocatedInSplitArea();
  if (remain_nonsolid_line_time > 4.0 && (is_use_spatio_planner_result ||
      (is_approach_intersection || is_in_intersection))) {
    split_scene = false;
    enter_split_time_ = 0.0;
    is_divide_lane_into_two_ = false;
  }
  // avoid
  bool avoid_back_status = false;
  const LateralOffsetDeciderOutput &lateral_offset_decider_output =
      session_->mutable_planning_context()->lateral_offset_decider_output();
  if (lateral_offset_decider_output.is_valid) {
    avoid_back_time_ = 1.0;
  } else if (avoid_back_time_ > 1e-6) {
    avoid_back_time_ += 0.1;
    avoid_back_status = true;
  }
  if (avoid_back_time_ > config_.avoid_back_time + 1.0) {
    avoid_back_time_ = 0.0;
    avoid_back_status = false;
  }
  double lateral_offset = 0.0;
  if (lateral_offset_decider_output.is_valid) {
    lateral_offset = lateral_offset_decider_output.lateral_offset;
  }
  planning_weight_ptr_->SetLateralOffset(lateral_offset);
  planning_weight_ptr_->SetIsBoundAvoid(
      general_lateral_decider_output.bound_avoid);
  planning_weight_ptr_->SetExpectedAvoidJerk(
      general_lateral_decider_output.recommended_bound_avoid_jerk);
  planning_weight_ptr_->SetRiskLevel(general_lateral_decider_output.risk_level);
  planning_weight_ptr_->SetIsEmergencyAvoid(general_lateral_decider_output.is_emergency_avoid);

  // lane change state
  bool lane_change_back = target_state == kLaneChangeCancel;
  planning_weight_ptr_->SetLCBackFlag(lane_change_back);
  bool lane_change_hold = target_state == kLaneChangeHold;
  planning_weight_ptr_->SetLCHoldFlag(lane_change_hold);
  bool is_cone_lc =
      lane_change_decider_output.lc_request_source == CONE_REQUEST;
  bool merge_point_valid = session_->planning_context()
                               .ego_lane_road_right_decider_output()
                               .boundary_merge_point_valid;
  const auto &merge_point = session_->planning_context()
                                .ego_lane_road_right_decider_output()
                                .boundary_merge_point;
  double dist_to_merge_point = 10000.0;
  if (merge_point_valid) {
    dist_to_merge_point = std::min(
        std::hypot(planning_init_point.lat_init_state.x() - merge_point.x,
                   planning_init_point.lat_init_state.y() - merge_point.y),
        dist_to_merge_point);
  }
  double distance_to_first_road_merge = session_->environmental_model()
                                            .get_route_info()
                                            ->get_route_info_output()
                                            .distance_to_first_road_merge;
  dist_to_merge_point =
      std::min(dist_to_merge_point, distance_to_first_road_merge);
  double lc_time_for_merge_point = std::fabs(dist_to_merge_point) / std::max(planning_input_.ref_vel(), 1e-6);
  bool is_prevent_solid_line_lc =
      lane_change_decider_output.is_dash_not_enough_for_lc;
  double lc_remain_time = 10.0;
  // if (lane_change_scene && !lane_change_back && !lane_change_hold) {
  if (target_state == kLaneChangeExecution) {
    lc_remain_time = remain_nonsolid_line_time;
  }
  // lc_remain_time = std::min(lc_remain_time, lc_time_for_merge_point);
  planning_weight_ptr_->SetLCRemainTime(std::max(lc_remain_time, 0.0));
  bool is_risk_lc = general_lateral_decider_output.risk_level > RiskLevel::NO_RISK;
  bool is_emergency_lc = false;
  // lane_change_decider_output.is_emergency_avoidance_situation;
  // 低速变道优先
  bool is_low_speed_lane_change = false;
  bool is_low_speed_lane_change_without_obstacle = false;
  const size_t avoid_end_idx = planning_weight_ptr_->GetAvoidEndIndex();
  double avoid_end_time = avoid_end_idx * config_.delta_t;
  const double avoid_dist = planning_weight_ptr_->GetAvoidDist();
  std::vector<double> xp_avoid_dist{0.1, 0.2, 0.3};
  std::vector<double> fp_avoid_end_t{1.5, 0.9, 0.5};
  double avoid_end_t_thr = planning::interp(avoid_dist, xp_avoid_dist, fp_avoid_end_t);
  if (general_lateral_decider_output.is_low_speed_lane_change_scene &&
      lane_change_scene) {
    if (((planning_weight_ptr_->GetLaneChangeStyle() ==
        pnc::lateral_planning::LaneChangeStyle::LOW_SPEED_LANE_CHANGE) ||
        ((avoid_dist > avoid_dist_thr && avoid_end_time > avoid_end_t_thr) || general_lateral_decider_output.bound_avoid ||
         general_lateral_decider_output.is_emergency_avoid))) {
      is_low_speed_lane_change = true;
    } else {
      is_low_speed_lane_change_without_obstacle = true;
    }
  }
  if (is_emergency_lc) {
    planning_weight_ptr_->SetLaneChangeStyle(
        pnc::lateral_planning::LaneChangeStyle::EMERGENCY_LANE_CHANGE);
  } else if (is_prevent_solid_line_lc || is_cone_lc || lc_remain_time < 4.5 || is_risk_lc ||
             (is_merge_lc && std::fabs(dist_to_merge_point) < planning_input_.ref_vel() * 5.0)) {
    planning_weight_ptr_->SetLaneChangeStyle(
        pnc::lateral_planning::LaneChangeStyle::QUICKLY_LANE_CHANGE);
  }
  double max_steer_angle_rate_low_speed_lc =
      std::min(vehicle_param.max_steer_angle_rate,
              config_.max_steer_angle_dot_low_speed_lc / 57.3);
  double max_wheel_angle_rate_low_speed_lc = max_steer_angle_rate_low_speed_lc / vehicle_param.steer_ratio;
  double limit_jerk_low_speed_lc =
      max_wheel_angle_rate_low_speed_lc * curv_factor_ * planning_input_.ref_vel() * planning_input_.ref_vel();
  double max_steer_angle_rate_low_speed_lc_without_obstacle =
      std::min(vehicle_param.max_steer_angle_rate,
               config_.max_steer_angle_dot_low_speed_lc_without_obstacle / 57.3);
  double max_wheel_angle_rate_low_speed_lc_without_obstacle =
      max_steer_angle_rate_low_speed_lc_without_obstacle / vehicle_param.steer_ratio;
  double limit_jerk_low_speed_lc_without_obstacle =
      max_wheel_angle_rate_low_speed_lc_without_obstacle *
      curv_factor_ * planning_input_.ref_vel() * planning_input_.ref_vel();
  if (is_low_speed_lane_change) {
    planning_weight_ptr_->SetMaxJerkLC(limit_jerk_low_speed_lc);
    planning_weight_ptr_->SetLaneChangeStyle(
        pnc::lateral_planning::LaneChangeStyle::LOW_SPEED_LANE_CHANGE);
    mutable_motion_planner_output.is_limit_lon_acc_bound = true;
    mutable_motion_planner_output.recommended_acc_bound =
        config_.recommend_low_speed_lc_lon_acc;
  } else if (is_low_speed_lane_change_without_obstacle && !lane_change_back &&
             !lane_change_hold) {
    planning_weight_ptr_->SetMaxJerkLC(
        limit_jerk_low_speed_lc_without_obstacle);
    mutable_motion_planner_output.is_limit_lon_acc_bound = false;
  } else {
    mutable_motion_planner_output.is_limit_lon_acc_bound = false;
  }

  // 需要针对is_low_speed_lane_change给1s的冷却时间，
  // 避免变道结束后，进入车道保持状态，jerk突然降低，纵向加速
  // 目前在冷却时间内仅保持jerk，不限制纵向加速能力
  bool is_enter_low_speed_lane_change_cooldown = false;
  if (is_last_low_speed_lane_change_ && !is_low_speed_lane_change) {
    low_speed_lane_change_cd_timer_ = std::min(low_speed_lane_change_cd_timer_ + 1,
                                               low_speed_lane_change_cd_timer_thr);
  } else {
    low_speed_lane_change_cd_timer_ = 0;
  }
  is_enter_low_speed_lane_change_cooldown = low_speed_lane_change_cd_timer_ <
                                            low_speed_lane_change_cd_timer_thr &&
                                            low_speed_lane_change_cd_timer_ > 0;
  if (is_enter_low_speed_lane_change_cooldown) {
    planning_weight_ptr_->SetMaxJerkLC(limit_jerk_low_speed_lc);
    planning_weight_ptr_->SetLaneChangeStyle(
        pnc::lateral_planning::LaneChangeStyle::LOW_SPEED_LANE_CHANGE);
  }
  // planning_weight_ptr_->SetLowChangeCoolDown(
  //     is_enter_low_speed_lane_change_cooldown);
  if (is_last_low_speed_lane_change_) {
    is_last_low_speed_lane_change_ = is_enter_low_speed_lane_change_cooldown;
  } else {
    is_last_low_speed_lane_change_ = is_low_speed_lane_change;
  }
  // reset LaneChangeStyle
  if (target_state == kLaneKeeping && !is_enter_low_speed_lane_change_cooldown) {
    mutable_motion_planner_output.is_limit_lon_acc_bound = false;
    planning_weight_ptr_->SetLaneChangeStyle(
        pnc::lateral_planning::LaneChangeStyle::STANDARD_LANE_CHANGE);
  }

  if (planning_weight_ptr_->GetLaneChangeStyle() ==
          pnc::lateral_planning::LaneChangeStyle::LOW_SPEED_LANE_CHANGE &&
      (lane_change_back || lane_change_hold)) {
    // 如果处于低速变道，并在kLaneChangeHold或者kLaneChangeCancel状态下，
    // 重置变道类型，避免下一次继续执行低速变道幅度不合理变大
    mutable_motion_planner_output.is_limit_lon_acc_bound = false;
    planning_weight_ptr_->SetLaneChangeStyle(
        pnc::lateral_planning::LaneChangeStyle::STANDARD_LANE_CHANGE);
  }

  // lane borrow
  const auto &lane_borrow_decider_output =
      session_->planning_context().lane_borrow_decider_output();
  bool lane_borrow_scene = lane_borrow_decider_output.is_in_lane_borrow_status;
  // search
  planning_weight_ptr_->SetIsSearchSuccess(false);
  // set weight
  if (lane_change_scene || is_enter_low_speed_lane_change_cooldown) {
    planning_weight_ptr_->SetLateralMotionWeight(
        pnc::lateral_planning::LateralMotionScene::LANE_CHANGE,
        planning_input_);
  } else if (lane_borrow_scene) {
    complete_follow = true;
    planning_weight_ptr_->SetLateralMotionWeight(
        pnc::lateral_planning::LateralMotionScene::LANE_BORROW, planning_input_);
  } else if (split_scene) {
    planning_weight_ptr_->SetLateralMotionWeight(
        pnc::lateral_planning::LateralMotionScene::SPLIT, planning_input_);
  } else if ((ramp_scene) && (config_.ramp_valid)) {
    planning_weight_ptr_->SetLateralMotionWeight(
        pnc::lateral_planning::LateralMotionScene::RAMP, planning_input_);
  // } else if (general_lateral_decider_output.bound_avoid ||
  //            (!is_use_spatio_planner_result &&
  //            (lateral_offset_decider_output.is_valid ||
  //             (avoid_back_status &&
  //              ((ref_vel > config_.avoid_high_vel) ||
  //                is_in_intersection))))) {
  //   planning_weight_ptr_->SetLateralMotionWeight(pnc::lateral_planning::LateralMotionScene::AVOID,
  //                                                planning_input_);
  } else {
    planning_weight_ptr_->SetLateralMotionWeight(
        pnc::lateral_planning::LateralMotionScene::LANE_KEEP, planning_input_);
  }
  // save history path
  planning_weight_ptr_->CalculateLatAccAndSteerAngleByHistoryPath(
      is_in_function, motion_planner_output.lat_init_flag,
      vehicle_param.wheel_base, vehicle_param.steer_ratio, curv_factor_,
      planning_init_point.lat_init_state.x(),
      planning_init_point.lat_init_state.y(), history_steer_vec_);
  JSON_DEBUG_VECTOR("history_steer_vec", history_steer_vec_, 2)
  // handle big shaking for steer
  const bool is_high_priority_back =
      lane_change_decider_output.is_high_priority_back;
  planning_weight_ptr_->CalculateJerkBoundByLastJerk(
      is_high_priority_back, is_in_function, enter_lccnoa_time_,
      reference_path_ptr, planning_output_, planning_input_);
  // set motion_plan_concerned_end_index
  planning_weight_ptr_->SetMotionPlanConcernedEndIndex(
      complete_follow, is_divide_lane_into_two_, reference_path_ptr,
      planning_input_);
  // construct virtual ref
  planning_weight_ptr_->ConstructVirtualRef(
      vehicle_param.wheel_base, curv_factor_, reference_path_ptr,
      planning_input_, virtual_ref_x_, virtual_ref_y_, virtual_ref_theta_);
  JSON_DEBUG_VECTOR("virtual_ref_x", virtual_ref_x_, 3)
  JSON_DEBUG_VECTOR("virtual_ref_y", virtual_ref_y_, 3)
  JSON_DEBUG_VECTOR("virtual_ref_theta", virtual_ref_theta_, 6)
  // set continuity protection
  if (!motion_planner_output.lat_init_flag || !is_ref_consistent_) {
    planning_input_.set_q_continuity(0.0);
  }
  planning_weight_ptr_->SetContinuityWeightByLastPath(
      valid_continuity_idx_, planning_input_);
  // spatio
  if (is_use_spatio_planner_result) {
    planning_input_.set_complete_follow(complete_follow);
  }
  // get emergency level
  const auto lateral_emergency_level =
      planning_weight_ptr_->GetEmergencyLevel();
  JSON_DEBUG_VALUE("lateral_emergency_level",
                   static_cast<int>(lateral_emergency_level));
  return true;
}

bool SCCLateralMotionPlanner::Update() {
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
  double s = 0.0;
  double t = 0.0;
  for (size_t i = 0; i < N; ++i) {
    double ref_vel = planning_input_.ref_vel_vec(i);
    if (i < N - 1) {
      ref_vel = (ref_vel + planning_input_.ref_vel_vec(i + 1)) * 0.5;
    }
    double kv2 = planning_input_.curv_factor() * ref_vel * ref_vel;

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
  // temp debug
  //   if
  //   (planning_output_.solver_info().iter_info(planning_output_.solver_info().iter_info_size()-1).cost()
  //   > 10000) {
  //     solver_condition = ilqr_solver::iLqr::NON_POSITIVE_EXPECT;
  //   }
  return true;
}

bool SCCLateralMotionPlanner::IsLocatedInSplitArea() {
  const auto &virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  // NOA split
  const bool is_exist_ramp_on_road =
      virtual_lane_manager->get_is_exist_ramp_on_road();
  const bool is_exist_split_on_ramp =
      virtual_lane_manager->get_is_exist_split_on_ramp();
  // LCC split
  const bool is_exist_intersection_split =
      virtual_lane_manager->get_is_exist_intersection_split();
  // split
  bool is_arrived_split_point = false;
  bool is_left_in_current = false;
  bool is_right_in_current = false;
  if (is_exist_ramp_on_road || is_exist_split_on_ramp ||
      is_exist_intersection_split) {
    const auto &current_lane = virtual_lane_manager->get_current_lane();
    if (current_lane != nullptr) {
      const auto &planning_init_point = current_lane->get_reference_path()
                                            ->get_frenet_ego_state()
                                            .planning_init_point();
      double end_s =
          planning_init_point.frenet_state.s + planning_input_.ref_vel() * 5.0;
      ReferencePathPoint end_point;
      if (current_lane->get_reference_path()->get_reference_point_by_lon(
              end_s, end_point)) {
        Point2D end_cart(end_point.path_point.x(), end_point.path_point.y());
        Point2D end_frenet;
        const auto &left_lane = virtual_lane_manager->get_left_lane();
        if (left_lane != nullptr) {
          const auto &left_frenet_coord =
              left_lane->get_reference_path()->get_frenet_coord();
          if (left_frenet_coord->XYToSL(end_cart, end_frenet)) {
            if (std::fabs(end_frenet.y) <= 0.1) {
              is_left_in_current = true;
            }
          }
        }
        const auto &right_lane = virtual_lane_manager->get_right_lane();
        if (right_lane != nullptr) {
          const auto &right_frenet_coord =
              right_lane->get_reference_path()->get_frenet_coord();
          if (right_frenet_coord->XYToSL(end_cart, end_frenet)) {
            if (std::fabs(end_frenet.y) <= 0.1) {
              is_right_in_current = true;
            }
          }
        }
      }
    }
    if (is_left_in_current || is_right_in_current) {
      return is_arrived_split_point;
    }
    is_arrived_split_point = true;
    enter_split_time_ = 1.0;
  } else if (enter_split_time_ > 0.9) {
    enter_split_time_ += 0.1;
    is_arrived_split_point = true;
  }
  if (enter_split_time_ > config_.enter_ramp_on_road_time + 1.0) {
    is_arrived_split_point = false;
    enter_split_time_ = 0.0;
  }

  if (is_exist_split_on_ramp) {
    is_divide_lane_into_two_ = true;
  }
  if (is_divide_lane_into_two_ && is_arrived_split_point == false) {
    is_divide_lane_into_two_ = false;
  }
  return is_arrived_split_point;
}

double SCCLateralMotionPlanner::CalculateRemainingDrivingTimeToSolidLine (
    bool is_check_left, bool is_check_right) {
  double remain_time = 10.0;
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto& reference_path =
      session_->environmental_model()
              .get_reference_path_manager()
              ->get_reference_path_by_lane(lane_change_decider_output.origin_lane_virtual_id, false);
  if (reference_path == nullptr) {
    return remain_time;
  }
  const size_t N = config_.horizon + 1;
  double sample_s = reference_path->get_frenet_ego_state().s();
  bool is_exist_dash_line = false;
  bool is_exist_solid_line = false;
  bool is_all_solid_line = false;
  for (size_t i = 0; i < N; ++i) {
    ReferencePathPoint refpath_pt{};
    if (reference_path->get_reference_point_by_lon(sample_s, refpath_pt)) {
      bool is_left_solid = refpath_pt.left_lane_border_type == iflyauto::LaneBoundaryType_MARKING_SOLID ||
                           refpath_pt.left_lane_border_type == iflyauto::LaneBoundaryType_MARKING_DOUBLE_SOLID ||
                           refpath_pt.left_lane_border_type == iflyauto::LaneBoundaryType_MARKING_LEFT_DASHED_RIGHT_SOLID;
      bool is_right_solid = refpath_pt.right_lane_border_type == iflyauto::LaneBoundaryType_MARKING_SOLID ||
                           refpath_pt.right_lane_border_type == iflyauto::LaneBoundaryType_MARKING_DOUBLE_SOLID ||
                           refpath_pt.right_lane_border_type == iflyauto::LaneBoundaryType_MARKING_LEFT_SOLID_RIGHT_DASHED;
      if (is_check_left && is_check_right) {
        if (is_left_solid || is_right_solid) {
          is_all_solid_line = true;
          if (is_exist_dash_line) {
            is_exist_solid_line = true;
          }
        } else {
          is_all_solid_line = false;
          is_exist_dash_line = true;
        }
      } else if (is_check_left) {
        if (is_left_solid) {
          is_all_solid_line = true;
          if (is_exist_dash_line) {
            is_exist_solid_line = true;
          }
        } else {
          is_all_solid_line = false;
          is_exist_dash_line = true;
        }
      } else if (is_check_right) {
        if (is_right_solid) {
          is_all_solid_line = true;
          if (is_exist_dash_line) {
            is_exist_solid_line = true;
          }
        } else {
          is_all_solid_line = false;
          is_exist_dash_line = true;
        }
      }
    }
    if (is_exist_solid_line) {
      break;
    }
    remain_time = config_.delta_t * i;
    if (i >= N - 1) {
      break;
    }
    double d_s = 0.5 * (planning_input_.ref_vel_vec(i) + planning_input_.ref_vel_vec(i + 1)) * config_.delta_t;
    sample_s += d_s;
  }
  if (is_all_solid_line && !is_exist_dash_line) {
    remain_time = 0.0;
  }
  return remain_time;
}

NudgeDirection SCCLateralMotionPlanner::CalculateDrivingDirectionForLeavingLane() {
  NudgeDirection direction = NudgeDirection::NONE;
  const auto& last_path_x_vec = planning_output_.x_vec();
  const auto& last_path_y_vec = planning_output_.y_vec();
  const auto& last_path_theta_vec = planning_output_.theta_vec();
  planning_output_.time_vec();
  if (last_path_x_vec.empty() || last_path_y_vec.empty() || last_path_theta_vec.empty()) {
    return direction;
  }
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto& reference_path =
      session_->environmental_model()
              .get_reference_path_manager()
              ->get_reference_path_by_lane(lane_change_decider_output.origin_lane_virtual_id, false);
  if (reference_path == nullptr) {
    return direction;
  }
  const auto& frenet_coord = reference_path->get_frenet_coord();
  if (frenet_coord == nullptr) {
    return direction;
  }
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const auto& concerned_index =
      session_->planning_context().motion_planner_output().lat_valid_end_idx;
  const auto& raw_path_points = reference_path->get_points();
  double init_lane_width = raw_path_points.front().lane_width;
  const size_t N = config_.horizon + 1;
  for (size_t i = 0; i < N; i++) {
    if (i > concerned_index) {
      break;
    }
    Point2D last_path_cart(last_path_x_vec[i], last_path_y_vec[i]);
    Point2D last_path_frenet;
    if (frenet_coord->XYToSL(last_path_cart, last_path_frenet)) {
      double left_width = 0.5 * init_lane_width;
      double right_width = 0.5 * init_lane_width;
      ReferencePathPoint cur_path_point;
      if (reference_path->get_reference_point_by_lon(last_path_frenet.x, cur_path_point)) {
        left_width = cur_path_point.distance_to_left_lane_border;
        right_width = cur_path_point.distance_to_right_lane_border;
      }
      double center_x = last_path_x_vec[i] +
          std::cos(last_path_theta_vec[i]) * vehicle_param.rear_axle_to_center;
      double center_y = last_path_y_vec[i] +
          std::sin(last_path_theta_vec[i]) * vehicle_param.rear_axle_to_center;
      planning_math::Box2d ego_box({center_x, center_y}, last_path_theta_vec[i],
                                   vehicle_param.length, vehicle_param.max_width);
      std::vector<planning_math::Vec2d> frenet_corners;
      for (auto& pt : ego_box.GetAllCorners()) {
        Point2D frenet_corner, cart_corner;
        cart_corner.x = pt.x();
        cart_corner.y = pt.y();
        if (frenet_coord->XYToSL(cart_corner, frenet_corner)) {
          if (frenet_corner.y > (left_width - avoid_dist_thr)) {
            direction = NudgeDirection::LEFT;
            driving_away_lane_time_ = std::max(planning_output_.time_vec(i) - 0.1, 0.0);
            break;
          } else if (frenet_corner.y < -(right_width - avoid_dist_thr)) {
            direction = NudgeDirection::RIGHT;
            driving_away_lane_time_ = std::max(planning_output_.time_vec(i) - 0.1, 0.0);
            break;
          }
        }
      }
    }
  }
  return direction;
}

}  // namespace planning
