
#include "base_lateral_motion_planner.h"

#include <algorithm>
#include <cstddef>
#include <iostream>
#include <vector>

#include "config/basic_type.h"
#include "debug_info_log.h"
#include "ilqr_define.h"
#include "lateral_motion_planner.pb.h"
#include "lateral_obstacle.h"
#include "math_lib.h"
#include "planning_context.h"
#include "spline.h"
#include "virtual_lane_manager.h"
#include "problem_solver/solver_define.h"


static const double pi_const = 3.141592654;
static const double planning_loop_dt = 0.1;

namespace planning {

BaseLateralMotionPlanner::BaseLateralMotionPlanner(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  config_ = config_builder->cast<LateralMotionPlannerConfig>();
  InitInputAndOutput();
};

void BaseLateralMotionPlanner::InitInputAndOutput() {
  //
  planning_weight_ptr_ =
      std::make_shared<pnc::lateral_planning::BaseWeight>(config_);
  // init input
  const size_t N = config_.horizon + 1;

  planning_input_.mutable_ref_x_vec()->Resize(N, 0.0);
  planning_input_.mutable_ref_y_vec()->Resize(N, 0.0);
  planning_input_.mutable_ref_theta_vec()->Resize(N, 0.0);
  planning_input_.mutable_ref_vel_vec()->Resize(N, 0.0);

  planning_input_.mutable_last_x_vec()->Resize(N, 0.0);
  planning_input_.mutable_last_y_vec()->Resize(N, 0.0);
  planning_input_.mutable_last_theta_vec()->Resize(N, 0.0);

  planning_input_.mutable_front_axis_ref_x_vec()->Resize(N, 0.0);
  planning_input_.mutable_front_axis_ref_y_vec()->Resize(N, 0.0);

  planning_input_.mutable_first_soft_upper_bound_x0_vec()->Resize(N, 0.0);
  planning_input_.mutable_first_soft_upper_bound_y0_vec()->Resize(N, 0.0);
  planning_input_.mutable_first_soft_upper_bound_x1_vec()->Resize(N, 0.0);
  planning_input_.mutable_first_soft_upper_bound_y1_vec()->Resize(N, 0.0);

  planning_input_.mutable_first_soft_lower_bound_x0_vec()->Resize(N, 0.0);
  planning_input_.mutable_first_soft_lower_bound_y0_vec()->Resize(N, 0.0);
  planning_input_.mutable_first_soft_lower_bound_x1_vec()->Resize(N, 0.0);
  planning_input_.mutable_first_soft_lower_bound_y1_vec()->Resize(N, 0.0);

  planning_input_.mutable_second_soft_upper_bound_x0_vec()->Resize(N, 0.0);
  planning_input_.mutable_second_soft_upper_bound_y0_vec()->Resize(N, 0.0);
  planning_input_.mutable_second_soft_upper_bound_x1_vec()->Resize(N, 0.0);
  planning_input_.mutable_second_soft_upper_bound_y1_vec()->Resize(N, 0.0);

  planning_input_.mutable_second_soft_lower_bound_x0_vec()->Resize(N, 0.0);
  planning_input_.mutable_second_soft_lower_bound_y0_vec()->Resize(N, 0.0);
  planning_input_.mutable_second_soft_lower_bound_x1_vec()->Resize(N, 0.0);
  planning_input_.mutable_second_soft_lower_bound_y1_vec()->Resize(N, 0.0);

  planning_input_.mutable_hard_upper_bound_x0_vec()->Resize(N, 0.0);
  planning_input_.mutable_hard_upper_bound_y0_vec()->Resize(N, 0.0);
  planning_input_.mutable_hard_upper_bound_x1_vec()->Resize(N, 0.0);
  planning_input_.mutable_hard_upper_bound_y1_vec()->Resize(N, 0.0);

  planning_input_.mutable_hard_lower_bound_x0_vec()->Resize(N, 0.0);
  planning_input_.mutable_hard_lower_bound_y0_vec()->Resize(N, 0.0);
  planning_input_.mutable_hard_lower_bound_x1_vec()->Resize(N, 0.0);
  planning_input_.mutable_hard_lower_bound_y1_vec()->Resize(N, 0.0);

  planning_input_.mutable_control_vec()->Resize(N, 0.0);
  //
  planning_output_.mutable_time_vec()->Resize(N, 0.0);
  planning_output_.mutable_x_vec()->Resize(N, 0.0);
  planning_output_.mutable_y_vec()->Resize(N, 0.0);
  planning_output_.mutable_theta_vec()->Resize(N, 0.0);
  planning_output_.mutable_delta_vec()->Resize(N, 0.0);
  planning_output_.mutable_omega_vec()->Resize(N, 0.0);
  planning_output_.mutable_acc_vec()->Resize(N, 0.0);
  planning_output_.mutable_jerk_vec()->Resize(N, 0.0);
  //
  CalculateCurvFactor();
  is_uniform_motion_ = true;
  is_need_reverse_ = false;
  is_ref_consistent_ = false;
  valid_continuity_idx_ = 0;
  lane_rel_theta_error_ = 0.0;
  ref_theta_vec_.resize(N, 0.0);
  virtual_ref_x_.reserve(N);
  virtual_ref_y_.reserve(N);
  virtual_ref_theta_.reserve(N);
  // init output
  x_vec_.resize(N + 1, 0.0);
  y_vec_.resize(N + 1, 0.0);
  theta_vec_.resize(N + 1, 0.0);
  delta_vec_.resize(N + 1, 0.0);
  omega_vec_.resize(N + 1, 0.0);
  curv_vec_.resize(N + 1, 0.0);
  d_curv_vec_.resize(N + 1, 0.0);
  s_vec_.resize(N + 1, 0.0);
  t_vec_.resize(N + 1, 0.0);
}

void BaseLateralMotionPlanner::ResetInputAndOutput() {
  valid_continuity_idx_ = 0;
  std::fill(ref_theta_vec_.begin(), ref_theta_vec_.end(), 0.0);
  std::fill(x_vec_.begin(), x_vec_.end(), 0.0);
  std::fill(y_vec_.begin(), y_vec_.end(), 0.0);
  std::fill(theta_vec_.begin(), theta_vec_.end(), 0.0);
  std::fill(delta_vec_.begin(), delta_vec_.end(), 0.0);
  std::fill(omega_vec_.begin(), omega_vec_.end(), 0.0);
  std::fill(curv_vec_.begin(), curv_vec_.end(), 0.0);
  std::fill(d_curv_vec_.begin(), d_curv_vec_.end(), 0.0);
  std::fill(s_vec_.begin(), s_vec_.end(), 0.0);
  std::fill(t_vec_.begin(), t_vec_.end(), 0.0);
  virtual_ref_x_.clear();
  virtual_ref_y_.clear();
  virtual_ref_theta_.clear();
}

bool BaseLateralMotionPlanner::Execute() { return true; }

void BaseLateralMotionPlanner::CalculateCurvFactor() {
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  double c1 = 1 / std::max(vehicle_param.wheel_base, 1e-6);
  double c2 = 0.0;  // neutral
  double vel = 0.0;  // neutral
  curv_factor_ = pnc::mathlib::GetCurvFactor(c1, c2, vel);
}

bool BaseLateralMotionPlanner::HandleReferencePathData() {
  // 1.set init state
  const auto &reference_path_ptr =
      session_->planning_context().lane_change_decider_output().coarse_planning_info.reference_path;
  const auto &planning_init_point =
      reference_path_ptr->get_frenet_ego_state().planning_init_point();
  planning_input_.mutable_init_state()->set_x(
      planning_init_point.lat_init_state.x());
  planning_input_.mutable_init_state()->set_y(
      planning_init_point.lat_init_state.y());
  planning_input_.mutable_init_state()->set_theta(
      planning_init_point.lat_init_state.theta());
  planning_input_.mutable_init_state()->set_delta(
      planning_init_point.lat_init_state.delta());
  JSON_DEBUG_VALUE("init_pos_x1", planning_init_point.lat_init_state.x())
  JSON_DEBUG_VALUE("init_pos_y1", planning_init_point.lat_init_state.y())
  JSON_DEBUG_VALUE("coarse_planning_info_ref_pnts_size",
                   reference_path_ptr->get_points().size())
  JSON_DEBUG_VALUE("coarse_planning_info_ref_line_s",
                   reference_path_ptr->get_points().back().path_point.s())
  // 2.set reference
  const auto &general_lateral_decider_output =  // result from lat decision
      session_->planning_context().general_lateral_decider_output();
  // static const double min_v_cruise = 0.5;
  const double ref_vel =
      std::max(general_lateral_decider_output.v_cruise, config_.min_v_cruise);
  const auto &enu_ref_path = general_lateral_decider_output.enu_ref_path;
  const auto &enu_ref_theta = general_lateral_decider_output.enu_ref_theta;
  std::vector<double> enu_ref_vel = general_lateral_decider_output.enu_ref_vel;
  if (is_uniform_motion_) {
    enu_ref_vel.resize(enu_ref_path.size(), ref_vel);
  }
  // assert(enu_ref_path.size() == enu_ref_theta.size());
  if (enu_ref_path.empty() || enu_ref_theta.empty() || enu_ref_vel.empty() ||
      enu_ref_path.size() != enu_ref_theta.size() || enu_ref_path.size() != enu_ref_vel.size() ||
      !session_->environmental_model().location_valid()) {
    return false;
  }
  if (is_need_reverse_) {
    // set reference velocity
    planning_input_.set_ref_vel(-ref_vel);
    // set back axis reference path
    for (size_t i = 0; i < enu_ref_path.size(); ++i) {
      planning_input_.mutable_ref_x_vec()->Set(i, enu_ref_path[i].first);
      planning_input_.mutable_ref_y_vec()->Set(i, enu_ref_path[i].second);
      double enu_ref_theta_i = enu_ref_theta[i] - M_PI;
      if (enu_ref_theta_i > M_PI) {
        enu_ref_theta_i -= 2.0 * M_PI;
      } else if (enu_ref_theta_i < -M_PI) {
        enu_ref_theta_i += 2.0 * M_PI;
      }
      ref_theta_vec_[i] = enu_ref_theta_i;
      planning_input_.mutable_ref_vel_vec()->Set(i, -enu_ref_vel[i]);
    }
  } else {
    // set reference velocity
    planning_input_.set_ref_vel(ref_vel);
    // set back axis reference path
    for (size_t i = 0; i < enu_ref_path.size(); ++i) {
      planning_input_.mutable_ref_x_vec()->Set(i, enu_ref_path[i].first);
      planning_input_.mutable_ref_y_vec()->Set(i, enu_ref_path[i].second);
      ref_theta_vec_[i] = enu_ref_theta[i];
      planning_input_.mutable_ref_vel_vec()->Set(i, enu_ref_vel[i]);
    }
  }
  // set front axis reference path
  const auto &front_axis_enu_ref_path = general_lateral_decider_output.front_axis_enu_ref_path;
  // assert(front_axis_enu_ref_path.size() == enu_ref_path.size());
  // if (front_axis_enu_ref_path.empty()) {
  //   return false;
  // }
  for (size_t i = 0; i < front_axis_enu_ref_path.size(); ++i) {
    planning_input_.mutable_front_axis_ref_x_vec()->Set(i, front_axis_enu_ref_path[i].first);
    planning_input_.mutable_front_axis_ref_y_vec()->Set(i, front_axis_enu_ref_path[i].second);
  }
  // angle fix of difference between theta and ref_theta, such as [-179deg and
  // 179deg]
  double angle_compensate = 0.0;
  const auto d_theta =
      ref_theta_vec_.front() - planning_init_point.lat_init_state.theta();
  if (d_theta > pi_const) {
    angle_compensate = -2.0 * pi_const;
  } else if (d_theta < -pi_const) {
    angle_compensate = 2.0 * pi_const;
  } else {
    angle_compensate = 0.0;
  }
  // angle fix of ref_theta
  double angle_offset = 0.0;
  for (size_t i = 0; i < ref_theta_vec_.size(); ++i) {
    if (i == 0) {
      planning_input_.mutable_ref_theta_vec()->Set(
          i, ref_theta_vec_[i] + angle_compensate);
    } else {
      const auto delta_theta = ref_theta_vec_[i] - ref_theta_vec_[i - 1];
      if (delta_theta > pi_const) {
        angle_offset -= 2.0 * pi_const;
      } else if (delta_theta < -pi_const) {
        angle_offset += 2.0 * pi_const;
      }
      planning_input_.mutable_ref_theta_vec()->Set(
          i, ref_theta_vec_[i] + angle_offset + angle_compensate);
    }
  }
  // 3.set last trajectory: temporarily same as reference: TODO
  const auto &motion_planner_output =
      session_->planning_context().motion_planner_output();
  const auto& last_path_s_vec = motion_planner_output.s_lat_vec;
  double final_t = 5.0;
  double last_path_length = last_path_s_vec.size() > 0 ? last_path_s_vec.back() : 0.0;
  is_ref_consistent_ = (ref_vel * final_t - last_path_length) <= 2.0;
  if (motion_planner_output.lat_init_flag) {
    Eigen::Vector2d init_point(planning_init_point.lat_init_state.x(),
                              planning_init_point.lat_init_state.y());
    pnc::spline::Projection last_path_projection_spline;
    last_path_projection_spline.CalProjectionPoint(
        motion_planner_output.x_s_spline, motion_planner_output.y_s_spline,
        last_path_s_vec.front(), last_path_s_vec.back(), init_point);
    double last_start_s = last_path_projection_spline.GetOutput().s_proj;
    for (size_t i = 0; i < enu_ref_path.size(); ++i) {
      double last_x = motion_planner_output.x_s_spline(last_start_s);
      double last_y = motion_planner_output.y_s_spline(last_start_s);
      double last_theta = motion_planner_output.theta_s_spline(last_start_s);
      double lateral_ref_theta = planning_input_.ref_theta_vec(i);
      double theta_err = lateral_ref_theta - last_theta;
      const double pi2 = 2.0 * M_PI;
      if (theta_err > M_PI) {
        last_theta += pi2;
      } else if (theta_err < -M_PI) {
        last_theta -= pi2;
      }
      planning_input_.mutable_last_x_vec()->Set(i, last_x);
      planning_input_.mutable_last_y_vec()->Set(i, last_y);
      planning_input_.mutable_last_theta_vec()->Set(i, last_theta);
      if (last_start_s <= last_path_length) {
        valid_continuity_idx_++;
      }
      double ds = ref_vel * config_.delta_t;
      if (!enu_ref_vel.empty()) {
        ds = enu_ref_vel[i] * config_.delta_t;
      }
      last_start_s += ds;
    }
    planning_input_.set_q_continuity(0.0);
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
  //
  // const auto &vehicle_param =
  //     VehicleConfigurationContext::Instance()->get_vehicle_param();
  // double c1 = 1 / std::max(vehicle_param.wheel_base, 1e-6);
  // double c2 = 0.0;  // neutral
  // curv_factor_ = pnc::mathlib::GetCurvFactor(c1, c2, ref_vel);
  planning_input_.set_curv_factor(curv_factor_);
  return true;
}

bool BaseLateralMotionPlanner::HandleLateralBoundData() {
  // set soft and hard bound
  const auto &general_lateral_decider_output =  // result from lat decision
      session_->planning_context().general_lateral_decider_output();
  const auto &first_soft_bounds_cart_point =
      general_lateral_decider_output.first_soft_bounds_cart_point;
  const auto &second_soft_bounds_cart_point =
      general_lateral_decider_output.second_soft_bounds_cart_point;
  const auto &hard_bounds_cart_point =
      general_lateral_decider_output.hard_bounds_cart_point;
  // assert(first_soft_bounds_cart_point.size() == hard_bounds_cart_point.size());
  if (first_soft_bounds_cart_point.empty() || hard_bounds_cart_point.empty() ||
      first_soft_bounds_cart_point.size() != hard_bounds_cart_point.size()) {
    return false;
  }
  // assert(first_soft_bounds_cart_point.size() == second_soft_bounds_cart_point.size());
  if (second_soft_bounds_cart_point.empty() || first_soft_bounds_cart_point.size() != second_soft_bounds_cart_point.size()) {
    return false;
  }
  for (size_t i = 0; i < hard_bounds_cart_point.size(); ++i) {
    size_t index = i;
    size_t next_index = i + 1;

    if (i == hard_bounds_cart_point.size() - 1) {
      index = i - 1;
      next_index = i;
    }

    const auto &first_soft_lower_bound = first_soft_bounds_cart_point[index].first;
    const auto &first_soft_upper_bound = first_soft_bounds_cart_point[index].second;
    const auto &next_first_soft_lower_bound = first_soft_bounds_cart_point[next_index].first;
    const auto &next_first_soft_upper_bound = first_soft_bounds_cart_point[next_index].second;

    planning_input_.mutable_first_soft_lower_bound_x0_vec()->Set(i,
                                                           first_soft_lower_bound.x);
    planning_input_.mutable_first_soft_lower_bound_y0_vec()->Set(i,
                                                           first_soft_lower_bound.y);
    planning_input_.mutable_first_soft_lower_bound_x1_vec()->Set(
        i, next_first_soft_lower_bound.x);
    planning_input_.mutable_first_soft_lower_bound_y1_vec()->Set(
        i, next_first_soft_lower_bound.y);

    planning_input_.mutable_first_soft_upper_bound_x0_vec()->Set(i,
                                                           first_soft_upper_bound.x);
    planning_input_.mutable_first_soft_upper_bound_y0_vec()->Set(i,
                                                           first_soft_upper_bound.y);
    planning_input_.mutable_first_soft_upper_bound_x1_vec()->Set(
        i, next_first_soft_upper_bound.x);
    planning_input_.mutable_first_soft_upper_bound_y1_vec()->Set(
        i, next_first_soft_upper_bound.y);

    const auto &second_soft_lower_bound = second_soft_bounds_cart_point[index].first;
    const auto &second_soft_upper_bound = second_soft_bounds_cart_point[index].second;
    const auto &next_second_soft_lower_bound = second_soft_bounds_cart_point[next_index].first;
    const auto &next_second_soft_upper_bound = second_soft_bounds_cart_point[next_index].second;

    planning_input_.mutable_second_soft_lower_bound_x0_vec()->Set(i,
                                                          second_soft_lower_bound.x);
    planning_input_.mutable_second_soft_lower_bound_y0_vec()->Set(i,
                                                          second_soft_lower_bound.y);
    planning_input_.mutable_second_soft_lower_bound_x1_vec()->Set(
        i, next_second_soft_lower_bound.x);
    planning_input_.mutable_second_soft_lower_bound_y1_vec()->Set(
        i, next_second_soft_lower_bound.y);

    planning_input_.mutable_second_soft_upper_bound_x0_vec()->Set(i,
                                                          second_soft_upper_bound.x);
    planning_input_.mutable_second_soft_upper_bound_y0_vec()->Set(i,
                                                          second_soft_upper_bound.y);
    planning_input_.mutable_second_soft_upper_bound_x1_vec()->Set(
        i, next_second_soft_upper_bound.x);
    planning_input_.mutable_second_soft_upper_bound_y1_vec()->Set(
        i, next_second_soft_upper_bound.y);

    const auto &hard_lower_bound = hard_bounds_cart_point[index].first;
    const auto &hard_upper_bound = hard_bounds_cart_point[index].second;
    const auto &next_hard_lower_bound =
        hard_bounds_cart_point[next_index].first;
    const auto &next_hard_upper_bound =
        hard_bounds_cart_point[next_index].second;

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
  return true;
}

bool BaseLateralMotionPlanner::HandleFeedbackInfoData() {
  const auto &lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto &coarse_planning_info = lane_change_decider_output.coarse_planning_info;
  const auto &reference_path_ptr = coarse_planning_info.reference_path;
  const auto &planning_init_point =
      reference_path_ptr->get_frenet_ego_state().planning_init_point();
  // set init info
  const auto& frenet_coord = reference_path_ptr->get_frenet_coord();
  Point2D cart_ref0(planning_input_.ref_x_vec(0), planning_input_.ref_y_vec(0));
  Point2D frenet_ref0;
  Point2D cart_init(planning_input_.init_state().x(),
                    planning_input_.init_state().y());
  Point2D frenet_init;
  if (frenet_coord != nullptr) {
    if (frenet_coord->XYToSL(cart_ref0, frenet_ref0)) {
    planning_weight_ptr_->SetInitDisToRef((planning_init_point.frenet_state.r - frenet_ref0.y));
    planning_weight_ptr_->SetInitRefThetaError(
          (planning_input_.init_state().theta() - planning_input_.ref_theta_vec(0)) * 57.3);
    } else {
      planning_weight_ptr_->CalculateInitInfo(planning_input_);
    }
    double lane_theta = frenet_coord->GetPathCurveHeading(planning_init_point.frenet_state.s);
    lane_rel_theta_error_ = planning_math::NormalizeAngle(planning_init_point.lat_init_state.theta() - lane_theta);
  } else {
    planning_weight_ptr_->CalculateInitInfo(planning_input_);
  }
  const double ego_v =
      session_->environmental_model().get_ego_state_manager()->ego_v();
  const double ego_l = reference_path_ptr->get_frenet_ego_state().l();
  planning_weight_ptr_->SetEgoVel(ego_v);
  planning_weight_ptr_->SetEgoL(ego_l);
  planning_weight_ptr_->SetRefVel(planning_input_.ref_vel());
  planning_weight_ptr_->SetInitS(planning_init_point.frenet_state.s);
  planning_weight_ptr_->SetInitL(planning_init_point.frenet_state.r);
  planning_weight_ptr_->CalculateLastPathDistToRef(reference_path_ptr, planning_input_);
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double kv2 = curv_factor_ * planning_input_.ref_vel() * planning_input_.ref_vel();
  double steer_ratio = vehicle_param.steer_ratio;
  double max_steer_angle = vehicle_param.max_steer_angle;  // rad
  double max_steer_angle_rate =
      std::min(vehicle_param.max_steer_angle_rate, config_.max_steer_angle_dot / 57.3);
  double max_steer_angle_rate_lc =
      std::min(vehicle_param.max_steer_angle_rate, config_.max_steer_angle_dot_lc / 57.3);
  max_wheel_angle_ = max_steer_angle / steer_ratio;
  max_wheel_angle_rate_ = max_steer_angle_rate / steer_ratio;
  max_wheel_angle_rate_lc_ = max_steer_angle_rate_lc / steer_ratio;
  double max_acc = std::min(max_wheel_angle_ * kv2, 5.0);
  double limit_jerk = max_wheel_angle_rate_ * kv2;
  double limit_jerk_lc = max_wheel_angle_rate_lc_ * kv2;
  std::vector<double> xp_v{4.167, 8.333, 15.0, 25.0};
  std::vector<double> fp_max_jerk{limit_jerk, 1.8, 1.5, 1.4};
  double max_jerk = planning::interp(planning_input_.ref_vel(), xp_v, fp_max_jerk);
  max_jerk = std::min(limit_jerk, max_jerk);
  planning_weight_ptr_->SetMaxAcc(max_acc);
  planning_weight_ptr_->SetMaxJerk(max_jerk);
  planning_weight_ptr_->SetMaxJerkLC(limit_jerk_lc);
  planning_weight_ptr_->SetInitSteerAngle(planning_init_point.lat_init_state.delta() * steer_ratio * 57.3);
  return true;
}

void BaseLateralMotionPlanner::StraightPath() {
  planning_input_.set_q_ref_x(0.0);
  planning_input_.set_q_ref_y(0.0);
  planning_input_.set_q_ref_theta(0.0);
  planning_input_.set_q_continuity(0.0);
  planning_input_.set_q_acc(500.0);
  planning_input_.set_q_jerk(100.0);
  planning_input_.set_q_acc_bound(config_.q_acc_bound);
  planning_input_.set_q_jerk_bound(config_.q_jerk_bound);
  planning_input_.set_q_soft_corridor(0.0);
  planning_input_.set_q_hard_corridor(0.0);
  planning_input_.set_complete_follow(true);
  planning_weight_ptr_->MutablePathWeights().Reset();
}

bool BaseLateralMotionPlanner::HandleOutputData() {
  // assembling planning output proto
  bool is_solver_success = true;
  const size_t N = config_.horizon + 1;
  double s = 0.0;
  double t = 0.0;
  for (size_t i = 0; i < N; ++i) {
    x_vec_[i + 1] = planning_output_.x_vec(i);
    y_vec_[i + 1] = planning_output_.y_vec(i);
    theta_vec_[i + 1] =
        planning_output_.theta_vec(i);  // note that theta cannot be limited with
                                       // [-pi, pi] to avoid incorrect spline
    delta_vec_[i + 1] = planning_output_.delta_vec(i);
    curv_vec_[i + 1] = curv_factor_ * planning_output_.delta_vec(i);

    omega_vec_[i + 1] = planning_output_.omega_vec(i);
    d_curv_vec_[i + 1] = curv_factor_ * omega_vec_[i + 1];

    if (i == 0) {
      s = 0.0;
      t = 0.0;
    } else {
      const double ds =
          std::hypot(x_vec_[i + 1] - x_vec_[i], y_vec_[i + 1] - y_vec_[i]);
      s += std::max(ds, 1e-3);
      t += 0.2;
    }
    s_vec_[i + 1] = s;
    t_vec_[i + 1] = t;
    if (planning_input_.q_continuity() <= 1e-6 && i <= planning_input_.motion_plan_concerned_index()) {
      if (std::fabs(planning_output_.theta_vec(i) -
                    planning_input_.ref_theta_vec(i)) *
              57.3 >
          90.0) {
        is_solver_success = false;
      }
      if (std::hypot(planning_output_.x_vec(i) - planning_input_.ref_x_vec(i),
                    planning_output_.y_vec(i) - planning_input_.ref_y_vec(i)) >
          10.0) {
        is_solver_success = false;
      }
    }
  }
  const auto solver_condition = planning_output_.solver_info().solver_condition();
  if ((!is_solver_success) ||
      (solver_condition >= ilqr_solver::iLqr::BACKWARD_PASS_FAIL)) {
    return false;
  }
  // generate motion planning output into planning_context
  auto &motion_planner_output =
      session_->mutable_planning_context()->mutable_motion_planner_output();
  const auto& reference_path_ptr = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;
  const auto& frenet_coord = reference_path_ptr->get_frenet_coord();
  // append the planning traj anti-direction for decoupling lat & lon replan
  const static double appended_length = config_.path_backward_appended_length;
  motion_planner_output.path_backward_appended_length = appended_length;
  Eigen::Vector2d unit_vector(x_vec_[1] - x_vec_[2], y_vec_[1] - y_vec_[2]);
  unit_vector.normalize();

  s_vec_[0] = -appended_length;
  x_vec_[0] = x_vec_[1] + unit_vector.x() * appended_length;
  y_vec_[0] = y_vec_[1] + unit_vector.y() * appended_length;
  theta_vec_[0] = theta_vec_[1];
  delta_vec_[0] = delta_vec_[1];
  omega_vec_[0] = omega_vec_[1];
  theta_vec_[0] = theta_vec_[1];
  curv_vec_[0] = curv_vec_[1];
  d_curv_vec_[0] = d_curv_vec_[1];
  t_vec_[0] = -0.2;

  const size_t concerned_index = planning_input_.motion_plan_concerned_index();
  // double concerned_dis_to_ref = std::hypot(
  //     x_vec_[concerned_index + 2] - planning_input_.ref_x_vec(concerned_index + 1),
  //     y_vec_[concerned_index + 2] - planning_input_.ref_y_vec(concerned_index + 1));
  if (planning_weight_ptr_->GetLateralMotionScene() ==
      pnc::lateral_planning::LateralMotionScene::RAMP &&
      reference_path_ptr->GetReferencePathCurveInfo().curve_type ==
      ReferencePathCurveInfo::CurveType::BIG_CURVE &&
      !planning_input_.complete_follow() &&
      (concerned_index < N - 1)) {
    size_t end_points_size = concerned_index + 1;
    Point2D frenet_ref_pt, frenet_traj_pt;
    for (size_t i = concerned_index + 1; i < N; ++i) {
      if (frenet_coord->XYToSL(Point2D(planning_input_.ref_x_vec(i), planning_input_.ref_y_vec(i)), frenet_ref_pt) &&
          frenet_coord->XYToSL(Point2D(planning_output_.x_vec(i), planning_output_.y_vec(i)), frenet_traj_pt)) {
        if (std::fabs(frenet_traj_pt.y - frenet_ref_pt.y) >= 0.05) {
          end_points_size = i;
          break;
        }
      }
    }
    std::vector<double> end_x_vec(N);
    std::vector<double> end_y_vec(N);
    std::vector<double> end_s_vec(N);
    for (size_t i = 0; i < N; ++i) {
      if (i >= end_points_size) {
        end_x_vec[i] = planning_input_.ref_x_vec(i);
        end_y_vec[i] = planning_input_.ref_y_vec(i);
        end_s_vec[i] = end_s_vec[i - 1] +
                       std::max(std::hypot(end_x_vec[i] - end_x_vec[i - 1],
                                           end_y_vec[i] - end_y_vec[i - 1]),
                                1e-3);
      } else {
        end_x_vec[i] = planning_output_.x_vec(i);
        end_y_vec[i] = planning_output_.y_vec(i);
        end_s_vec[i] = s_vec_[i + 1];
      }
    }
    pnc::mathlib::spline end_x_s_spline;
    pnc::mathlib::spline end_y_s_spline;
    end_x_s_spline.set_points(end_s_vec, end_x_vec);
    end_y_s_spline.set_points(end_s_vec, end_y_vec);
    double end_ds =
        (end_s_vec.back() - end_s_vec[end_points_size - 1]) /
        (N - end_points_size);
    end_ds = std::max(std::min(end_ds, planning_input_.ref_vel() * 0.2), 1e-3);
    double end_s = end_s_vec[end_points_size - 1];
    for (size_t i = end_points_size; i < N; ++i) {
      end_s += end_ds;
      x_vec_[i + 1] = end_x_s_spline(end_s);
      y_vec_[i + 1] = end_y_s_spline(end_s);
      double next_theta = std::atan2(end_y_s_spline.deriv(1, end_s),
                                     end_x_s_spline.deriv(1, end_s));
      double theta_err = theta_vec_[i] - next_theta;
      const double pi2 = 2.0 * M_PI;
      if (theta_err > M_PI) {
        next_theta += pi2;
      } else if (theta_err < -M_PI) {
        next_theta -= pi2;
      }
      theta_vec_[i + 1] = next_theta;
      s_vec_[i + 1] = end_s;
    }
  }

  // construct lateral kd path
  motion_planner_output.lateral_path_coord =
      ConstructLateralKDPath(x_vec_, y_vec_);

  // set state spline
  motion_planner_output.x_s_spline.set_points(s_vec_, x_vec_);
  motion_planner_output.y_s_spline.set_points(s_vec_, y_vec_);
  motion_planner_output.theta_s_spline.set_points(s_vec_, theta_vec_);
  motion_planner_output.delta_s_spline.set_points(s_vec_, delta_vec_);
  motion_planner_output.omega_s_spline.set_points(s_vec_, omega_vec_);
  motion_planner_output.curv_s_spline.set_points(s_vec_, curv_vec_);
  motion_planner_output.d_curv_s_spline.set_points(s_vec_, d_curv_vec_);
  motion_planner_output.lateral_x_t_spline.set_points(t_vec_, x_vec_);
  motion_planner_output.lateral_y_t_spline.set_points(t_vec_, y_vec_);
  motion_planner_output.lateral_theta_t_spline.set_points(t_vec_, theta_vec_);
  motion_planner_output.lateral_s_t_spline.set_points(t_vec_, s_vec_);
  motion_planner_output.lateral_t_s_spline.set_points(s_vec_, t_vec_);
  motion_planner_output.s_lat_vec = s_vec_;
  motion_planner_output.lat_init_flag = true;
  motion_planner_output.curv_factor = curv_factor_;
  motion_planner_output.lat_valid_end_idx = planning_input_.motion_plan_concerned_index();
  motion_planner_output.concerned_index =
      planning_input_.motion_plan_concerned_index() + 1;

  ilqr_solver::ControlVec u_vec;
  u_vec.resize(N);

  // set u_vec to motion_planner_output for warm start
  for (size_t i = 0; i < N; ++i) {
    ilqr_solver::Control u;
    u.resize(1);
    u[0] = omega_vec_[i];
    u_vec[i] = u;
  }
  motion_planner_output.u_vec = u_vec;

  // assemble results
  const auto& general_lateral_decider_output =
      session_->planning_context().general_lateral_decider_output();
  auto& traj_points = session_->mutable_planning_context()
                              ->mutable_planning_result()
                              .traj_points;
  for (size_t i = 0; i < N; i++) {
    traj_points[i].x = x_vec_[i + 1];
    traj_points[i].y = y_vec_[i + 1];
    traj_points[i].heading_angle = theta_vec_[i + 1];
    traj_points[i].curvature = curv_factor_ * delta_vec_[i + 1];

    traj_points[i].v = general_lateral_decider_output.v_cruise;
    traj_points[i].t = planning_output_.time_vec(i);

    // frenet state update
    Point2D cart_pt(traj_points[i].x, traj_points[i].y);
    Point2D frenet_pt;

    if (reference_path_ptr->get_frenet_coord() != nullptr &&
        reference_path_ptr->get_frenet_coord()->XYToSL(cart_pt, frenet_pt)) {
      traj_points[i].s = frenet_pt.x;
      traj_points[i].l = frenet_pt.y;
    } else {
      ILOG_DEBUG << "XYToSL = FAILED !!!!!!!! index:" << i
                 << ",  point.s : " << traj_points[i].s
                 << ", point.l: " << traj_points[i].l;
    }
  }
  return true;
}

std::shared_ptr<planning_math::KDPath> BaseLateralMotionPlanner::ConstructLateralKDPath(
    const std::vector<double> &x_vec, const std::vector<double> &y_vec) {
  std::vector<planning_math::PathPoint> lat_path_points;
  lat_path_points.reserve(x_vec.size());
  for (int i = 1; i <= config_.horizon + 1; ++i) {
    if (std::isnan(x_vec[i]) || std::isnan(y_vec[i])) {
      ILOG_ERROR << "skip NaN point";
      continue;
    }
    planning_math::PathPoint path_point{x_vec[i], y_vec[i]};
    if (!lat_path_points.empty()) {
      const auto& last_pt = lat_path_points.back();
      if (planning_math::Vec2d(last_pt.x() - path_point.x(),
                               last_pt.y() - path_point.y())
              .Length() < 1e-3) {
        continue;
      }
    }
    lat_path_points.emplace_back(std::move(path_point));
  }
  if (lat_path_points.size() <= planning_math::KDPath::kKDPathMinPathPointSize) {
    return nullptr;
  }
  return std::make_shared<planning_math::KDPath>(std::move(lat_path_points));
}

void BaseLateralMotionPlanner::SaveDebugInfo() {
// record input and output
#ifdef ENABLE_PROTO_LOG
  DebugInfoManager::GetInstance()
      .GetDebugInfoPb()
      ->mutable_lateral_motion_planning_input()
      ->CopyFrom(planning_input_);
  DebugInfoManager::GetInstance()
      .GetDebugInfoPb()
      ->mutable_lateral_motion_planning_output()
      ->CopyFrom(planning_output_);
#endif
}
}  // namespace planning
