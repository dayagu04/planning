
#include "lateral_motion_planner.h"

#include <algorithm>
#include <cstddef>
#include <iostream>
#include <vector>

#include "config/basic_type.h"
#include "debug_info_log.h"
#include "ilqr_define.h"
#include "lateral_motion_planner.pb.h"
#include "lateral_obstacle.h"
#include "planning_context.h"
#include "spline.h"
#include "src/lateral_motion_planning_cost.h"
#include "src/lateral_motion_planning_weight.h"
#include "virtual_lane_manager.h"

static const double pi_const = 3.141592654;
static const double planning_loop_dt = 0.1;
namespace planning {
LateralMotionPlanner::LateralMotionPlanner(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  config_ = config_builder->cast<LateralMotionPlannerConfig>();
  name_ = "LateralMotionPlanner";

  Init();
};

void LateralMotionPlanner::Init() {
  // init planning weight
  planning_weight_ptr_ =
      std::make_shared<pnc::lateral_planning::LateralMotionPlanningWeight>(
          config_);
  // init planning problem
  planning_problem_ptr_ =
      std::make_shared<pnc::lateral_planning::LateralMotionPlanningProblem>();
  planning_problem_ptr_->Init();

  // init planning input and ouput
  const auto N =
      planning_problem_ptr_->GetiLqrCorePtr()->GetSolverConfigPtr()->horizon +
      1;

  // init planning input
  planning_input_.mutable_ref_x_vec()->Resize(N, 0.0);
  planning_input_.mutable_ref_y_vec()->Resize(N, 0.0);
  planning_input_.mutable_ref_theta_vec()->Resize(N, 0.0);

  planning_input_.mutable_last_x_vec()->Resize(N, 0.0);
  planning_input_.mutable_last_y_vec()->Resize(N, 0.0);
  planning_input_.mutable_last_theta_vec()->Resize(N, 0.0);

  planning_input_.mutable_soft_upper_bound_x0_vec()->Resize(N, 0.0);
  planning_input_.mutable_soft_upper_bound_y0_vec()->Resize(N, 0.0);
  planning_input_.mutable_soft_upper_bound_x1_vec()->Resize(N, 0.0);
  planning_input_.mutable_soft_upper_bound_y1_vec()->Resize(N, 0.0);

  planning_input_.mutable_soft_lower_bound_x0_vec()->Resize(N, 0.0);
  planning_input_.mutable_soft_lower_bound_y0_vec()->Resize(N, 0.0);
  planning_input_.mutable_soft_lower_bound_x1_vec()->Resize(N, 0.0);
  planning_input_.mutable_soft_lower_bound_y1_vec()->Resize(N, 0.0);

  planning_input_.mutable_hard_upper_bound_x0_vec()->Resize(N, 0.0);
  planning_input_.mutable_hard_upper_bound_y0_vec()->Resize(N, 0.0);
  planning_input_.mutable_hard_upper_bound_x1_vec()->Resize(N, 0.0);
  planning_input_.mutable_hard_upper_bound_y1_vec()->Resize(N, 0.0);

  planning_input_.mutable_hard_lower_bound_x0_vec()->Resize(N, 0.0);
  planning_input_.mutable_hard_lower_bound_y0_vec()->Resize(N, 0.0);
  planning_input_.mutable_hard_lower_bound_x1_vec()->Resize(N, 0.0);
  planning_input_.mutable_hard_lower_bound_y1_vec()->Resize(N, 0.0);

  planning_input_.mutable_control_vec()->Resize(N, 0.0);

  avoid_back_time_ = 0.0;
  enter_split_time_ = 0.0;
  is_divide_lane_into_two_ = false;
}

bool LateralMotionPlanner::Execute() {
  LOG_DEBUG("=======LateralMotionPlanner======= \n");

  if (!PreCheck()) {
    LOG_DEBUG("PreCheck failed\n");
    return false;
  }

  auto start_time = IflyTime::Now_ms();

  // assemble input
  if (!AssembleInput()) {
    LOG_DEBUG("LateralMotionPlanner AssembleInput failed\n");
    SaveDebugInfo();
    return false;
  }
  // update
  if (!Update()) {
    LOG_DEBUG("LateralMotionPlanner Solve failed\n");
    SaveDebugInfo();
    return false;
  }

  auto end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("LateralMotionCostTime", end_time - start_time);
  SaveDebugInfo();
  return true;
}

bool LateralMotionPlanner::AssembleInput() {
  // set init state
  const auto &reference_path_ptr = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;
  const auto &planning_init_point =
      reference_path_ptr->get_frenet_ego_state().planning_init_point();
  const auto &motion_planner_output =
      session_->planning_context().motion_planner_output();

  JSON_DEBUG_VALUE("init_pos_x1", planning_init_point.lat_init_state.x())
  JSON_DEBUG_VALUE("init_pos_y1", planning_init_point.lat_init_state.y())
  JSON_DEBUG_VALUE("coarse_planning_info_ref_pnts_size",
                   reference_path_ptr->get_points().size())
  JSON_DEBUG_VALUE("coarse_planning_info_ref_line_s",
                   reference_path_ptr->get_points().back().path_point.s())
  planning_input_.mutable_init_state()->set_x(
      planning_init_point.lat_init_state.x());
  planning_input_.mutable_init_state()->set_y(
      planning_init_point.lat_init_state.y());
  planning_input_.mutable_init_state()->set_theta(
      planning_init_point.lat_init_state.theta());
  planning_input_.mutable_init_state()->set_delta(
      planning_init_point.lat_init_state.delta());

  // set reference
  const auto &general_lateral_decider_output =  // result from lat decision
      session_->planning_context().general_lateral_decider_output();

  const auto &enu_ref_path = general_lateral_decider_output.enu_ref_path;
  const auto &enu_ref_theta = general_lateral_decider_output.enu_ref_theta;
  bool complete_follow = general_lateral_decider_output.complete_follow;
  const bool &lane_change_scene =
      general_lateral_decider_output.lane_change_scene;
  const bool &ramp_scene = general_lateral_decider_output.ramp_scene;
  assert(enu_ref_path.size() == enu_ref_theta.size());

  if (enu_ref_path.size() == 0 || enu_ref_theta.size() == 0 ||
      !session_->environmental_model().location_valid()) {
    return false;
  }

  // static const double min_v_cruise = 0.5;
  const double ref_vel =
      std::max(general_lateral_decider_output.v_cruise, config_.min_v_cruise);

  std::vector<double> ref_theta_vec(enu_ref_theta.size());
  if (session_->environmental_model().function_info().function_mode() ==
      common::DrivingFunctionInfo_DrivingFunctionMode::
          DrivingFunctionInfo_DrivingFunctionMode_RADS) {
    planning_input_.set_ref_vel(-ref_vel);

    // set reference trajectory
    for (size_t i = 0; i < enu_ref_path.size(); ++i) {
      planning_input_.mutable_ref_x_vec()->Set(i, enu_ref_path[i].first);
      planning_input_.mutable_ref_y_vec()->Set(i, enu_ref_path[i].second);
      double enu_ref_theta_i = enu_ref_theta[i] - M_PI;
      if (enu_ref_theta_i > M_PI) {
        enu_ref_theta_i -= 2.0 * M_PI;
      } else if (enu_ref_theta_i < -M_PI) {
        enu_ref_theta_i += 2.0 * M_PI;
      }
      ref_theta_vec[i] = enu_ref_theta_i;
    }
  } else {
    planning_input_.set_ref_vel(ref_vel);

    // set reference trajectory
    for (size_t i = 0; i < enu_ref_path.size(); ++i) {
      planning_input_.mutable_ref_x_vec()->Set(i, enu_ref_path[i].first);
      planning_input_.mutable_ref_y_vec()->Set(i, enu_ref_path[i].second);
      ref_theta_vec[i] = enu_ref_theta[i];
    }
  }

  // angle fix of difference between theta and ref_theta, such as [-179deg and
  // 179deg]
  double angle_compensate = 0.0;
  const auto d_theta =
      ref_theta_vec.front() - planning_init_point.lat_init_state.theta();
  if (d_theta > 1.5 * pi_const) {
    angle_compensate = -2.0 * pi_const;
  } else if (d_theta < -1.5 * pi_const) {
    angle_compensate = 2.0 * pi_const;
  } else {
    angle_compensate = 0.0;
  }

  // angle fix of ref_theta
  double angle_offset = 0.0;
  for (size_t i = 0; i < ref_theta_vec.size(); ++i) {
    if (i == 0) {
      planning_input_.mutable_ref_theta_vec()->Set(
          i, ref_theta_vec[i] + angle_compensate);
    } else {
      const auto delta_theta = ref_theta_vec[i] - ref_theta_vec[i - 1];
      if (delta_theta > 1.5 * pi_const) {
        angle_offset -= 2.0 * pi_const;
      } else if (delta_theta < -1.5 * pi_const) {
        angle_offset += 2.0 * pi_const;
      }
      planning_input_.mutable_ref_theta_vec()->Set(
          i, ref_theta_vec[i] + angle_offset + angle_compensate);
    }
  }

  // set init theta by ref_theta: not solid
  // planning_input_.mutable_init_state()->set_theta(
  //     planning_input_.ref_theta_vec(0));

  // set last trajectory: temporarily same as reference: TODO
  double final_t = 5.0;  // hack now
  double tmp_t = 0.0;
  auto last_s_vec = motion_planner_output.s_lat_vec;
  double last_path_length = last_s_vec.size() > 0 ? last_s_vec.back() : 0.0;
  bool is_ref_consistent = (ref_vel * final_t - last_path_length) <= 2.0;
  if (motion_planner_output.lat_init_flag) {
    for (size_t i = 0; i < enu_ref_path.size(); ++i) {
      tmp_t = std::fmin(planning_loop_dt + i * 0.2, final_t);
      planning_input_.mutable_last_x_vec()->Set(
          i, motion_planner_output.lateral_x_t_spline(tmp_t));
      planning_input_.mutable_last_y_vec()->Set(
          i, motion_planner_output.lateral_y_t_spline(tmp_t));
      double lateral_ref_theta = planning_input_.ref_theta_vec(i);
      double last_lateral_theta =
          motion_planner_output.lateral_theta_t_spline(tmp_t);
      double theta_err = lateral_ref_theta - last_lateral_theta;
      const double pi2 = 2.0 * M_PI;
      if (theta_err > M_PI) {
        last_lateral_theta += pi2;
      } else if (theta_err < -M_PI) {
        last_lateral_theta -= pi2;
      }
      planning_input_.mutable_last_theta_vec()->Set(i, last_lateral_theta);
    }
    planning_input_.set_q_continuity(config_.q_continuity);
  } else {
    for (size_t i = 0; i < enu_ref_path.size(); ++i) {
      planning_input_.mutable_last_x_vec()->Set(i, enu_ref_path[i].first);
      planning_input_.mutable_last_y_vec()->Set(i, enu_ref_path[i].second);
      // planning_input_.mutable_last_theta_vec()->Set(i, enu_ref_theta[i]);
      planning_input_.mutable_last_theta_vec()->Set(
          i, planning_input_.ref_theta_vec(i));
    }
    planning_input_.set_q_continuity(0.0);
  }

  // set soft and hard bound
  const auto &soft_bounds_cart_point =
      general_lateral_decider_output.soft_bounds_cart_point;
  const auto &hard_bounds_cart_point =
      general_lateral_decider_output.hard_bounds_cart_point;
  assert(soft_bounds_cart_point.size() == hard_bounds_cart_point.size());

  for (size_t i = 0; i < soft_bounds_cart_point.size(); ++i) {
    size_t index = i;
    size_t next_index = i + 1;

    if (i == soft_bounds_cart_point.size() - 1) {
      index = i - 1;
      next_index = i;
    }

    const auto &soft_lower_bound = soft_bounds_cart_point[index].first;
    const auto &soft_upper_bound = soft_bounds_cart_point[index].second;
    const auto &next_soft_lower_bound = soft_bounds_cart_point[next_index].first;
    const auto &next_soft_upper_bound = soft_bounds_cart_point[next_index].second;

    planning_input_.mutable_soft_lower_bound_x0_vec()->Set(i,
                                                           soft_lower_bound.x);
    planning_input_.mutable_soft_lower_bound_y0_vec()->Set(i,
                                                           soft_lower_bound.y);
    planning_input_.mutable_soft_lower_bound_x1_vec()->Set(
        i, next_soft_lower_bound.x);
    planning_input_.mutable_soft_lower_bound_y1_vec()->Set(
        i, next_soft_lower_bound.y);

    planning_input_.mutable_soft_upper_bound_x0_vec()->Set(i,
                                                           soft_upper_bound.x);
    planning_input_.mutable_soft_upper_bound_y0_vec()->Set(i,
                                                           soft_upper_bound.y);
    planning_input_.mutable_soft_upper_bound_x1_vec()->Set(
        i, next_soft_upper_bound.x);
    planning_input_.mutable_soft_upper_bound_y1_vec()->Set(
        i, next_soft_upper_bound.y);

    const auto &hard_lower_bound = hard_bounds_cart_point[index].first;
    const auto &hard_upper_bound = hard_bounds_cart_point[index].second;
    const auto &next_hard_lower_bound = hard_bounds_cart_point[next_index].first;
    const auto &next_hard_upper_bound = hard_bounds_cart_point[next_index].second;

    planning_input_.mutable_hard_lower_bound_x0_vec()->Set(i,
                                                           hard_lower_bound.x);
    planning_input_.mutable_hard_lower_bound_y0_vec()->Set(i,
                                                           hard_lower_bound.y);
    planning_input_.mutable_hard_lower_bound_x1_vec()->Set(
        i, next_hard_lower_bound.x);
    planning_input_.mutable_hard_lower_bound_y1_vec()->Set(
        i, next_hard_lower_bound.y);

    planning_input_.mutable_hard_upper_bound_x0_vec()->Set(i,
                                                           hard_upper_bound.x);
    planning_input_.mutable_hard_upper_bound_y0_vec()->Set(i,
                                                           hard_upper_bound.y);
    planning_input_.mutable_hard_upper_bound_x1_vec()->Set(
        i, next_hard_upper_bound.x);
    planning_input_.mutable_hard_upper_bound_y1_vec()->Set(
        i, next_hard_upper_bound.y);
  }

  planning_input_.set_curv_factor(config_.curv_factor);

  // set init info
  Point2D cart_ref0(planning_input_.ref_x_vec(0), planning_input_.ref_y_vec(0));
  Point2D frenet_ref0;
  Point2D cart_init(planning_input_.init_state().x(),
                    planning_input_.init_state().y());
  Point2D frenet_init;
  if (reference_path_ptr->get_frenet_coord() != nullptr &&
      reference_path_ptr->get_frenet_coord()->XYToSL(cart_ref0, frenet_ref0) &&
      reference_path_ptr->get_frenet_coord()->XYToSL(cart_init, frenet_init)) {
    planning_weight_ptr_->SetInitDisToRef((frenet_init.y - frenet_ref0.y));
    planning_weight_ptr_->SetInitRefThetaError(
        (planning_input_.init_state().theta() -
         planning_input_.ref_theta_vec(0)) *
        57.3);
  } else {
    planning_weight_ptr_->CalculateInitInfo(planning_input_);
  }
  const double ego_v =
      session_->environmental_model().get_ego_state_manager()->ego_v();
  const double ego_l = reference_path_ptr->get_frenet_ego_state().l();
  planning_weight_ptr_->SetEgoVel(ego_v);
  planning_weight_ptr_->SetEgoL(ego_l);
  planning_weight_ptr_->SetRefVel(ref_vel);
  planning_weight_ptr_->SetInitL(planning_init_point.frenet_state.r);
  planning_weight_ptr_->CalculateLastPathDistToRef(reference_path_ptr, planning_input_);
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double kv2 =
      config_.curv_factor * ref_vel * ref_vel;
  double steer_ratio = vehicle_param.steer_ratio;
  double max_steer_angle = vehicle_param.max_steer_angle;  // rad
  double max_steer_angle_rate =
      std::min(vehicle_param.max_steer_angle_rate, config_.max_steer_angle_dot / 57.3);
  double max_wheel_angle =
      max_steer_angle / steer_ratio;
  double max_wheel_angle_rate =
      max_steer_angle_rate / steer_ratio;
  double max_acc = std::min(max_wheel_angle * kv2, 5.0);
  double limit_jerk = max_wheel_angle_rate * kv2;
  std::vector<double> xp_v{4.167, 8.333, 15.0, 25.0};
  std::vector<double> fp_max_jerk{limit_jerk, 1.538, 1.231, 0.8};
  double max_jerk = planning::interp(ref_vel, xp_v, fp_max_jerk);
  max_jerk = std::min(limit_jerk, max_jerk);
  planning_weight_ptr_->SetMaxAcc(max_acc);
  planning_weight_ptr_->SetMaxJerk(max_jerk);
  std::vector<double> expected_steer_vec;
  expected_steer_vec.resize(26, 0.0);
  planning_weight_ptr_->CalculateExpectedLatAccAndSteerAngle(
      planning_init_point.frenet_state.s, ref_vel,
      vehicle_param.wheel_base, steer_ratio, reference_path_ptr,
      expected_steer_vec);
  JSON_DEBUG_VECTOR("expected_steer_vec", expected_steer_vec, 2)
  const auto &soft_bounds_frenet_point = general_lateral_decider_output.soft_bounds_frenet_point;
  const auto &hard_bounds_frenet_point = general_lateral_decider_output.hard_bounds_frenet_point;
  planning_weight_ptr_->CalculateLatAvoidDistance(soft_bounds_frenet_point);
  // const auto &soft_bounds = general_lateral_decider_output.soft_bounds;
  const auto &hard_bounds = general_lateral_decider_output.hard_bounds;
  const auto &soft_bounds_info = general_lateral_decider_output.soft_bounds_info;
  const auto &hard_bounds_info = general_lateral_decider_output.hard_bounds_info;
  planning_weight_ptr_->CalculateLatAvoidBoundPriority(
      soft_bounds_frenet_point, hard_bounds_frenet_point,
      hard_bounds,
      soft_bounds_info, hard_bounds_info);

  if (session_->is_hpp_scene()) {
    // const bool &search_success = session_->mutable_planning_context()
    //                                ->mutable_lateral_obstacle_decider_output()
    //                                .search_success;
    const bool &search_success = general_lateral_decider_output.enable_ara_ref;
    if (is_ref_consistent) {
      planning_weight_ptr_->SetIsSearchSuccess(search_success);
    } else {
      planning_weight_ptr_->SetIsSearchSuccess(false);
    }

    planning_weight_ptr_->SetLateralMotionWeight(
        pnc::lateral_planning::LANE_KEEP, planning_input_);
    planning_input_.set_complete_follow(complete_follow);
    planning_input_.set_motion_plan_concerned_index(
        config_.motion_plan_concerned_end_index);
    return true;
  } else {
    planning_weight_ptr_->SetIsSearchSuccess(false);
  }

  // split
  bool split_scene = IsLocatedInSplitArea();

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
  if ((is_approach_intersection || is_in_intersection) &&
      is_ref_consistent) {
    planning_weight_ptr_->SetIsInIntersection(true);
  } else {
    planning_weight_ptr_->SetIsInIntersection(false);
  }

  // avoid back
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

  // lane change back
  const auto target_state = session_->planning_context()
                                .lane_change_decider_output()
                                .coarse_planning_info.target_state;
  bool lane_change_back = target_state == kLaneChangeCancel;
  planning_weight_ptr_->SetLCBackFlag(lane_change_back);

  planning_weight_ptr_->SetIsSearchSuccess(false);
  // set weight
  if (lane_change_scene) {
    planning_weight_ptr_->SetLateralMotionWeight(
        pnc::lateral_planning::LANE_CHANGE, planning_input_);
  } else if (session_->environmental_model()
                 .get_lateral_obstacle()
                 ->is_static_avoid_scene()) {
    planning_weight_ptr_->SetLateralMotionWeight(
        pnc::lateral_planning::STATIC_AVOID, planning_input_);
  } else if (lateral_offset_decider_output.is_valid ||
             (avoid_back_status &&
              ((ref_vel > config_.avoid_high_vel) || is_in_intersection))) {
    planning_weight_ptr_->SetLateralMotionWeight(pnc::lateral_planning::AVOID,
                                                 planning_input_);
  } else if (split_scene) {
    planning_weight_ptr_->SetLateralMotionWeight(pnc::lateral_planning::SPLIT,
                                                 planning_input_);
  } else if ((ramp_scene) && (config_.ramp_valid)) {
    planning_weight_ptr_->SetLateralMotionWeight(pnc::lateral_planning::RAMP,
                                                 planning_input_);
  } else {
    planning_weight_ptr_->SetLateralMotionWeight(
        pnc::lateral_planning::LANE_KEEP, planning_input_);
  }
  // handle big shaking for steer
  planning_weight_ptr_->CalculateJerkBoundByLastJerk(
      reference_path_ptr,
      planning_problem_ptr_->GetOutput(), planning_input_);
  // set motion_plan_concerned_end_index
  planning_weight_ptr_->SetMotionPlanConcernedEndIndex(
      complete_follow, is_divide_lane_into_two_,
      reference_path_ptr, planning_input_);
  // set continuity protection
  if (!motion_planner_output.lat_init_flag) {
    planning_input_.set_q_continuity(0.0);
  }
  // [hack](bsniu):
  if (session_->environmental_model().function_info().function_mode() ==
      common::DrivingFunctionInfo_DrivingFunctionMode::
          DrivingFunctionInfo_DrivingFunctionMode_RADS) {
    planning_input_.set_q_soft_corridor(0);
    planning_input_.set_q_hard_corridor(0);
    planning_input_.set_complete_follow(true);
  }
  return true;
}

bool LateralMotionPlanner::Update() {
  const double concerned_start_q_jerk =
      planning_weight_ptr_->GetConcernedStartQJerk();
  JSON_DEBUG_VALUE("concerned_start_q_jerk", concerned_start_q_jerk);
  const double end_ratio_for_qrefxy =
      planning_weight_ptr_->GetConcernedEndRatioForXY();
  const double end_ratio_for_qreftheta =
      planning_weight_ptr_->GetConcernedEndRatioForTheta();
  auto start_time = IflyTime::Now_ms();
  auto solver_condition = planning_problem_ptr_->Update(
      end_ratio_for_qrefxy, end_ratio_for_qreftheta,
      config_.end_ratio_for_qjerk,
      concerned_start_q_jerk, planning_weight_ptr_, planning_input_);
  JSON_DEBUG_VALUE("solver_condition", solver_condition);
  auto end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("iLqr_lat_update_time", end_time - start_time);

  // if (solver_condition >= ilqr_solver::iLqr::BACKWARD_PASS_FAIL) {
  //   return false;
  // }
  bool is_solver_success = true;
  // update planning_output
  const auto &planning_output = planning_problem_ptr_->GetOutput();

  // assembling planning output proto
  const auto N =
      planning_problem_ptr_->GetiLqrCorePtr()->GetSolverConfigPtr()->horizon +
      1;

  std::vector<double> x_vec(N + 1);
  std::vector<double> y_vec(N + 1);
  std::vector<double> theta_vec(N + 1);
  std::vector<double> delta_vec(N + 1);
  std::vector<double> omega_vec(N + 1);
  std::vector<double> curv_vec(N + 1);
  std::vector<double> d_curv_vec(N + 1);
  std::vector<double> s_vec(N + 1);
  std::vector<double> t_vec(N + 1);

  double s = 0.0;
  double t = 0.0;
  for (size_t i = 0; i < N; ++i) {
    x_vec[i + 1] = planning_output.x_vec(i);
    y_vec[i + 1] = planning_output.y_vec(i);
    theta_vec[i + 1] =
        planning_output.theta_vec(i);  // note that theta cannot be limited with
                                       // [-pi, pi] to avoid incorrect spline
    delta_vec[i + 1] = planning_output.delta_vec(i);
    curv_vec[i + 1] =
        planning_input_.curv_factor() * planning_output.delta_vec(i);

    omega_vec[i + 1] = planning_output.omega_vec(i);
    d_curv_vec[i + 1] = planning_input_.curv_factor() * omega_vec[i + 1];

    if (i == 0) {
      s = 0.0;
      t = 0.0;
    } else {
      const double ds =
          std::hypot(x_vec[i + 1] - x_vec[i], y_vec[i + 1] - y_vec[i]);
      s += std::max(ds, 1e-3);
      t += 0.2;
    }
    s_vec[i + 1] = s;
    t_vec[i + 1] = t;
    if (std::fabs(planning_output.theta_vec(i) -
                  planning_input_.ref_theta_vec(i)) *
            57.3 >
        90) {
      is_solver_success = false;
    }
  }

  if ((!is_solver_success) ||
      (solver_condition >= ilqr_solver::iLqr::BACKWARD_PASS_FAIL)) {
    return false;
  }
  // generate motion planning output into planning_context
  auto &motion_planner_output =
      session_->mutable_planning_context()->mutable_motion_planner_output();

  // append the planning traj anti-direction for decoupling lat & lon replan
  const static double appended_length = config_.path_backward_appended_length;
  motion_planner_output.path_backward_appended_length = appended_length;
  Eigen::Vector2d unit_vector(x_vec[1] - x_vec[2], y_vec[1] - y_vec[2]);
  unit_vector.normalize();

  s_vec[0] = -appended_length;
  x_vec[0] = x_vec[1] + unit_vector.x() * appended_length;
  y_vec[0] = y_vec[1] + unit_vector.y() * appended_length;
  theta_vec[0] = theta_vec[1];
  delta_vec[0] = delta_vec[1];
  omega_vec[0] = omega_vec[1];
  theta_vec[0] = theta_vec[1];
  curv_vec[0] = curv_vec[1];
  d_curv_vec[0] = d_curv_vec[1];
  t_vec[0] = -0.2;

  const double concerned_index = 20;  // planning_input_.motion_plan_concerned_index();
  double concerned_dis_to_ref = std::hypot(
      x_vec[concerned_index + 2] - planning_input_.ref_x_vec(concerned_index + 1),
      y_vec[concerned_index + 2] - planning_input_.ref_y_vec(concerned_index + 1));
  const double end_points_size = concerned_index + 1;
  if ((!(session_->environmental_model().function_info().function_mode() ==
         common::DrivingFunctionInfo_DrivingFunctionMode::
             DrivingFunctionInfo_DrivingFunctionMode_RADS)) &&
      (!planning_input_.complete_follow()) && (concerned_dis_to_ref <= 0.1) &&
      (!session_->planning_context()
            .general_lateral_decider_output()
            .lane_change_scene)) {
    const double end_points_size = concerned_index + 1;
    std::vector<double> end_x_vec(end_points_size + 1);
    std::vector<double> end_y_vec(end_points_size + 1);
    std::vector<double> end_s_vec(end_points_size + 1);
    for (size_t i = 0; i < end_points_size + 1; ++i) {
      if (i > concerned_index) {
        end_x_vec[i] = planning_input_.ref_x_vec(N - 1);
        end_y_vec[i] = planning_input_.ref_y_vec(N - 1);
        end_s_vec[i] = end_s_vec[i - 1] +
                       std::max(std::hypot(end_x_vec[i] - end_x_vec[i - 1],
                                           end_y_vec[i] - end_y_vec[i - 1]),
                                1e-3);
      } else {
        end_x_vec[i] = planning_output.x_vec(i);
        end_y_vec[i] = planning_output.y_vec(i);
        end_s_vec[i] = s_vec[i + 1];
      }
    }
    pnc::mathlib::spline end_x_s_spline;
    pnc::mathlib::spline end_y_s_spline;
    end_x_s_spline.set_points(end_s_vec, end_x_vec);
    end_y_s_spline.set_points(end_s_vec, end_y_vec);
    double end_ds =
        (end_s_vec[end_points_size] - end_s_vec[end_points_size - 1]) /
        (N - end_points_size);
    end_ds = std::min(end_ds, planning_input_.ref_vel() * 0.2);
    double end_s = end_s_vec[end_points_size - 1];
    for (size_t i = end_points_size; i < N; ++i) {
      end_s += end_ds;
      x_vec[i + 1] = end_x_s_spline(end_s);
      y_vec[i + 1] = end_y_s_spline(end_s);
      double next_theta = std::atan2(end_y_s_spline.deriv(1, end_s),
                                     end_x_s_spline.deriv(1, end_s));
      double theta_err = theta_vec[i] - next_theta;
      const double pi2 = 2.0 * M_PI;
      if (theta_err > M_PI) {
        next_theta += pi2;
      } else if (theta_err < -M_PI) {
        next_theta -= pi2;
      }
      theta_vec[i + 1] = next_theta;
      s_vec[i + 1] = end_s;
    }
  }

  // construct lateral kd path
  motion_planner_output.lateral_path_coord =
      ConstructLateralKDPath(x_vec, y_vec);

  // set state spline
  motion_planner_output.x_s_spline.set_points(s_vec, x_vec);
  motion_planner_output.y_s_spline.set_points(s_vec, y_vec);
  motion_planner_output.theta_s_spline.set_points(s_vec, theta_vec);
  motion_planner_output.delta_s_spline.set_points(s_vec, delta_vec);
  motion_planner_output.omega_s_spline.set_points(s_vec, omega_vec);
  motion_planner_output.curv_s_spline.set_points(s_vec, curv_vec);
  motion_planner_output.d_curv_s_spline.set_points(s_vec, d_curv_vec);
  motion_planner_output.lateral_x_t_spline.set_points(t_vec, x_vec);
  motion_planner_output.lateral_y_t_spline.set_points(t_vec, y_vec);
  motion_planner_output.lateral_theta_t_spline.set_points(t_vec, theta_vec);
  motion_planner_output.lateral_s_t_spline.set_points(t_vec, s_vec);
  motion_planner_output.lateral_t_s_spline.set_points(s_vec, t_vec);
  motion_planner_output.s_lat_vec = s_vec;
  motion_planner_output.lat_init_flag = true;

  ilqr_solver::ControlVec u_vec;
  u_vec.resize(N);

  // set u_vec to motion_planner_output for warm start
  for (size_t i = 0; i < N; ++i) {
    ilqr_solver::Control u;
    u.resize(1);
    u[0] = omega_vec[i];
    u_vec[i] = u;
  }
  motion_planner_output.u_vec = u_vec;

  // assemble results
  const auto &general_lateral_decider_output =  // result from lat decision
      session_->planning_context().general_lateral_decider_output();

  auto &traj_points = session_->mutable_planning_context()
                          ->mutable_planning_result()
                          .traj_points;
  const auto &reference_path_ptr = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;
  for (size_t i = 0; i < N; i++) {
    traj_points[i].x = x_vec[i + 1];
    traj_points[i].y = y_vec[i + 1];
    traj_points[i].heading_angle = theta_vec[i + 1];
    traj_points[i].curvature = planning_input_.curv_factor() * delta_vec[i + 1];

    traj_points[i].v = general_lateral_decider_output.v_cruise;
    traj_points[i].t = planning_output.time_vec(i);

    // frenet state update
    Point2D cart_pt(traj_points[i].x, traj_points[i].y);
    Point2D frenet_pt;

    if (reference_path_ptr->get_frenet_coord() != nullptr &&
        reference_path_ptr->get_frenet_coord()->XYToSL(cart_pt, frenet_pt)) {
      traj_points[i].s = frenet_pt.x;
      traj_points[i].l = frenet_pt.y;
    } else {
      LOG_DEBUG(
          "XYToSL = FAILED !!!!!!!! index: %ld,  point.s : "
          "%f, point.l: %f ",
          i, traj_points[i].s, traj_points[i].l);
    }
  }
  return true;
}

std::shared_ptr<planning_math::KDPath>
LateralMotionPlanner::ConstructLateralKDPath(const std::vector<double> &x_vec,
                                             const std::vector<double> &y_vec) {
  std::vector<planning_math::PathPoint> lat_path_points;
  lat_path_points.reserve(x_vec.size());
  for (int i = 1; i <= 26; ++i) {
    if (std::isnan(x_vec[i]) || std::isnan(y_vec[i])) {
      LOG_ERROR("skip NaN point");
      continue;
    }
    planning_math::PathPoint path_point{x_vec[i], y_vec[i]};
    lat_path_points.emplace_back(path_point);
    if (!lat_path_points.empty()) {
      auto &last_pt = lat_path_points.back();
      if (planning_math::Vec2d(last_pt.x() - path_point.x(),
                               last_pt.y() - path_point.y())
              .Length() < 1e-3) {
        continue;
      }
    }
  }
  if (lat_path_points.size() <= 2) {
    return nullptr;
  }
  return std::make_shared<planning_math::KDPath>(std::move(lat_path_points));
}

bool LateralMotionPlanner::IsLocatedInSplitArea() {
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
  if (is_exist_ramp_on_road || is_exist_split_on_ramp || is_exist_intersection_split) {
    const auto& current_lane = virtual_lane_manager->get_current_lane();
    if (current_lane != nullptr) {
      double end_s = planning_input_.ref_vel() * 5.0;
      ReferencePathPoint end_point;
      if (current_lane->get_reference_path()
                      ->get_reference_point_by_lon(end_s, end_point)) {
        Point2D end_cart(end_point.path_point.x(), end_point.path_point.y());
        Point2D end_frenet;
        const auto& left_lane = virtual_lane_manager->get_left_lane();
        if (left_lane != nullptr) {
          const auto& left_frenet_coord =
              left_lane->get_reference_path()->get_frenet_coord();
          if (left_frenet_coord->XYToSL(end_cart, end_frenet)) {
            if (end_frenet.y <= 0.1) {
              return is_arrived_split_point;
            }
          }
        }
        const auto& right_lane = virtual_lane_manager->get_right_lane();
        if (right_lane != nullptr) {
          const auto& right_frenet_coord =
              right_lane->get_reference_path()->get_frenet_coord();
          if (right_frenet_coord->XYToSL(end_cart, end_frenet)) {
            if (end_frenet.y <= 0.1) {
              return is_arrived_split_point;
            }
          }
        }
      }
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

void LateralMotionPlanner::SaveDebugInfo() {
  // record input and output
  DebugInfoManager::GetInstance()
      .GetDebugInfoPb()
      ->mutable_lateral_motion_planning_input()
      ->CopyFrom(planning_input_);
  DebugInfoManager::GetInstance()
      .GetDebugInfoPb()
      ->mutable_lateral_motion_planning_output()
      ->CopyFrom(planning_problem_ptr_->GetOutput());
}
}  // namespace planning
