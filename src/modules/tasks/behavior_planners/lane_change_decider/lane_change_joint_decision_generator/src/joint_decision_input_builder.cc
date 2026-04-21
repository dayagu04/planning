#include "joint_decision_input_builder.h"

#include <algorithm>
#include <cmath>
#include <memory>

#include "common/config/basic_type.h"
#include "debug_info_log.h"
#include "environmental_model.h"
#include "ifly_time.h"
#include "planning_context.h"
#include "behavior_planners/speed_limit_decider/speed_limit_decider_output.h"
namespace planning {
namespace lane_change_joint_decision {

JointDecisionInputBuilder::JointDecisionInputBuilder(
    const EgoPlanningConfigBuilder* config_builder, framework::Session* session)
    : session_(session) {
  lc_decision_config_ = config_builder->cast<JointDecisionPlannerConfig>();
  speed_planning_config_ = config_builder->cast<SpeedPlannerConfig>();
  obstacles_selector_ =
      std::make_shared<JointDecisionObstaclesSelector>(session);

  comfort_params_.v0 = 33.5;
  comfort_params_.s0 = 2.5;
  comfort_params_.T = lc_decision_config_.lc_thw;
  comfort_params_.a = 2.0;
  comfort_params_.b_max = 2.0;
  comfort_params_.b = 1.0;
  comfort_params_.b_hard = 4.0;
  comfort_params_.delta = 4.0;
  comfort_params_.max_accel_jerk = 3.0;
  comfort_params_.max_decel_jerk = 1.5;
  comfort_params_.cool_factor = 0.99;
  comfort_params_.eps = 1e-3;

  ref_trajectory_.clear();
  ref_trajectory_.reserve(kPlanningTimeSteps);
}

void JointDecisionInputBuilder::SetObstaclesSelector(
    std::shared_ptr<JointDecisionObstaclesSelector> obstacles_selector) {
  obstacles_selector_ = obstacles_selector;
}

void JointDecisionInputBuilder::BuildLaneChangeInput(
    planning::common::JointDecisionPlanningInput& planning_input,
    std::shared_ptr<
        pnc::lane_change_joint_decision::JointDecisionPlanningProblem>
        planning_problem_ptr,
    const LaneChangeDecisionInfo& lc_info) {
  lead_one_id_ = lc_info.gap_front_agent_id;
  JSON_DEBUG_VALUE("lead_one_id", lead_one_id_);

  BuildLaneChangeEgoInfo(planning_input, lc_info);

  BuildLaneChangeWeightInfo(planning_input);

  key_agent_ids_.clear();
  BuildObsInfo(planning_input, lc_info);

  JSON_DEBUG_VECTOR("joint_key_agent_ids", key_agent_ids_, 0);

  BuildRoadInfo(planning_input, planning_problem_ptr);
}

double JointDecisionInputBuilder::CalculateComfortAcceleration(
    const double current_acc, const double current_vel, const double current_s,
    const double front_vel, const double front_s) const {
  double s0 = comfort_params_.s0;
  double v0 = comfort_params_.v0;
  double a = comfort_params_.a;
  double b_max = comfort_params_.b_max;
  double b = comfort_params_.b;
  double b_hard = comfort_params_.b_hard;
  double max_accel_jerk = comfort_params_.max_accel_jerk;
  double max_decel_jerk = comfort_params_.max_decel_jerk;
  double delta = comfort_params_.delta;
  double cool_factor = comfort_params_.cool_factor;
  double eps = comfort_params_.eps;
  double tau = comfort_params_.T;

  double s_alpha = std::max(eps, front_s - current_s);

  double delta_v = current_vel - front_vel;
  double s_star =
      s0 + std::max(0.0, current_vel * tau + (current_vel * delta_v) /
                                                 (2.0 * std::sqrt(a * b_max)));

  double v_target = std::max(eps, v0);

  double z = s_star / s_alpha;

  double a_free;
  if (current_vel <= v_target) {
    a_free = a * (1.0 - std::pow(current_vel / v_target, delta));
  } else {
    a_free = -b * (1.0 - std::pow(v_target / current_vel, a * delta / b));
  }

  double a_idm;
  if (current_vel <= v_target) {
    if (z < 1.0 && std::abs(a_free) > eps) {
      a_idm = a_free * (1.0 - std::pow(z, 2.0 * a / a_free));
    } else {
      a_idm = a * (1.0 - std::pow(z, 2.0));
    }
  } else {
    if (z >= 1.0) {
      a_idm = a_free + a * (1.0 - std::pow(z, 2.0));
    } else {
      a_idm = a_free;
    }
  }

  a_idm = std::max(std::min(a, a_idm), -b_hard);

  double a_cah = (current_vel * current_vel * (-b)) /
                 (front_vel * front_vel - 2 * s_alpha * (-b));

  a_cah = std::max(std::min(a, a_cah), -b_hard);

  double comfort_acc;
  if (a_idm >= a_cah) {
    comfort_acc = a_idm;
  } else {
    comfort_acc = (1.0 - cool_factor) * a_idm +
                  cool_factor * (a_cah - b * tanh((a_idm - a_cah) / (-b)));
  }

  double acc_change = comfort_acc - current_acc;
  if (acc_change > 0 && acc_change > max_accel_jerk * kPlanningTimeStep) {
    comfort_acc = current_acc + max_accel_jerk * kPlanningTimeStep;
  } else if (acc_change < 0 &&
             acc_change < -max_decel_jerk * kPlanningTimeStep) {
    comfort_acc = current_acc - max_decel_jerk * kPlanningTimeStep;
  }

  comfort_acc = std::max(std::min(a, comfort_acc), -b_hard);

  return comfort_acc;
}

void JointDecisionInputBuilder::BuildLaneChangeEgoInfo(
    planning::common::JointDecisionPlanningInput& planning_input,
    const LaneChangeDecisionInfo& lc_info) {
  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  const auto& planning_init_point = ego_state_manager->planning_init_point();

  planning_input.mutable_ego_init_state()->set_x(planning_init_point.x);
  planning_input.mutable_ego_init_state()->set_y(planning_init_point.y);
  planning_input.mutable_ego_init_state()->set_theta(
      planning_init_point.heading_angle);
  planning_input.mutable_ego_init_state()->set_delta(planning_init_point.delta);
  planning_input.mutable_ego_init_state()->set_vel(planning_init_point.v);
  planning_input.mutable_ego_init_state()->set_acc(planning_init_point.a);

  const auto& vehicle_param =
      planning::VehicleConfigurationContext::Instance()->get_vehicle_param();
  planning_input.set_ego_length(vehicle_param.length);
  planning_input.set_ego_width(vehicle_param.width);

  if (lc_info.ego_ref_traj.size() < kPlanningTimeSteps) {
    return;
  }

  ref_trajectory_.clear();
  ref_trajectory_.reserve(kPlanningTimeSteps);
  ref_trajectory_.resize(kPlanningTimeSteps);

  std::vector<double> s_vec, x_vec, y_vec;
  s_vec.reserve(kPlanningTimeSteps);
  x_vec.reserve(kPlanningTimeSteps);
  y_vec.reserve(kPlanningTimeSteps);
  for (size_t i = 0; i < kPlanningTimeSteps; ++i) {
    s_vec.push_back(lc_info.ego_ref_traj[i].s);
    x_vec.push_back(lc_info.ego_ref_traj[i].x);
    y_vec.push_back(lc_info.ego_ref_traj[i].y);
  }

  pnc::mathlib::spline x_s_spline, y_s_spline;
  x_s_spline.set_points(s_vec, x_vec);
  y_s_spline.set_points(s_vec, y_vec);

  const double wheel_base = vehicle_param.wheel_base;
  const double max_steer_angle = vehicle_param.max_steer_angle;
  const double front_edge_to_rear_axle = vehicle_param.front_edge_to_rear_axle;
  const double dt = kPlanningTimeStep;

  const double cruise_speed = ego_state_manager->ego_v_cruise();
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto lane_change_state = lane_change_decider_output.curr_state;
  const bool is_in_lane_change_execution =
      lane_change_state == StateMachineLaneChangeStatus::kLaneChangeExecution ||
      lane_change_state == StateMachineLaneChangeStatus::kLaneChangeComplete ||
      lane_change_state == StateMachineLaneChangeStatus::kLaneChangeHold;

  const auto& speed_limit_decider_output =
      session_->planning_context().speed_limit_decider_output();
  double speed_limit_normal = cruise_speed;
  
  if (is_in_lane_change_execution) {
    speed_limit_normal = std::fmin(
        speed_limit_normal, speed_planning_config_.lane_change_upper_speed_limit_kph / 3.6);
  }

  double speed_limit_ref = std::numeric_limits<double>::max();
  auto speed_limit_type_ref = SpeedLimitType::NONE;
  speed_limit_decider_output.GetSpeedLimit(&speed_limit_ref,
                                           &speed_limit_type_ref);
  double current_lc_limit = speed_limit_ref;
  //限速设置：不超过130，曲率限速不能超， max(自车速度*1.05，巡航限速*1.05)；
  if(speed_limit_type_ref == SpeedLimitType::SHARP_CURVATURE){
    current_lc_limit = speed_limit_ref;
  }else{
    current_lc_limit = std::max(planning_init_point.v, speed_limit_ref) * 1.05; //变道时轻微超速
  }
  double kAbsoluteMaxSpeed = 130.0 / 3.6;
  const double v0 = std::min(current_lc_limit, kAbsoluteMaxSpeed);
  JSON_DEBUG_VALUE("joint_decision_limit_speed", v0);
  comfort_params_.v0 = v0;

  // const auto& lane_change_decider_output =
  //     session_->planning_context().lane_change_decider_output();
  // const int front_agent_node_id =
  //     lane_change_decider_output.lc_gap_info.front_node_id;
  const int front_agent_node_id =
      lc_info.gap_front_agent_id;
  const agent::Agent* lead_one_agent = nullptr;
  std::vector<trajectory::TrajectoryPoint> lead_trajectory;

  if (front_agent_node_id != -1) {
    lead_one_agent =
        session_->environmental_model().get_agent_manager()->GetAgent(
            front_agent_node_id);
      if (lead_one_agent != nullptr) {
        const auto& primary_trajectories =
            lead_one_agent->trajectories_used_by_st_graph();
        const auto& fallback_trajectories = lead_one_agent->trajectories();
        const auto& selected_trajectories = primary_trajectories.empty()
                                                ? fallback_trajectories
                                                : primary_trajectories;
        if (!selected_trajectories.empty() &&
            !selected_trajectories.front().empty()) {
          lead_trajectory = selected_trajectories.front();
        }
    }
  }

  const auto& reference_path_ptr = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;
  if (reference_path_ptr == nullptr) {
    return;
  }

  const auto& ego_lane = session_->environmental_model()
                             .get_virtual_lane_manager()
                             ->get_current_lane();
  if (ego_lane == nullptr || ego_lane->get_lane_frenet_coord() == nullptr) {
    return;
  }
  const auto& ego_lane_coord = ego_lane->get_lane_frenet_coord();

  double ego_s = 0.0, ego_l = 0.0;
  if (!ego_lane_coord->XYToSL(planning_init_point.x, planning_init_point.y,
                              &ego_s, &ego_l)) {
    return;
  }

  double current_s = 0.0;
  double current_v = planning_init_point.v;
  double current_a = planning_init_point.a;
  double current_theta = planning_init_point.heading_angle;
  double current_delta = planning_init_point.delta;

  ref_trajectory_[0].t = 0.0;
  ref_trajectory_[0].s = current_s;
  ref_trajectory_[0].vel = current_v;
  ref_trajectory_[0].acc = current_a;
  ref_trajectory_[0].x = planning_init_point.x;
  ref_trajectory_[0].y = planning_init_point.y;
  ref_trajectory_[0].theta = current_theta;
  ref_trajectory_[0].delta = current_delta;

  double front_s = 250.0;
  double front_vel = v0;
  double front_acc = 0.0;

  if (s_vec.empty()) {
    return;
  }

  double s_min = s_vec.front();
  double s_max = s_vec.back();


  double ref_ego_s = reference_path_ptr->get_frenet_ego_state().s();

  for (int i = 1; i < kPlanningTimeSteps; ++i) {
    double t = i * dt;
    if (!lead_trajectory.empty() && lead_one_agent != nullptr) {
      size_t lead_idx = static_cast<size_t>(i);
      if (lead_idx >= lead_trajectory.size()) {
        lead_idx = lead_trajectory.size() - 1;
      }
      if (lead_idx > 0 && lead_idx <= lead_trajectory.size()) {
        const auto& lead_point = lead_trajectory[lead_idx - 1];
        front_acc = lead_one_agent->accel_fusion();
        double lead_s = 0.0, lead_l = 0.0;
        if (ego_lane_coord->XYToSL(lead_point.x(), lead_point.y(), &lead_s,
                                   &lead_l)) {
          double lead_rear_s = lead_s - lead_one_agent->length() * 0.5;
          double ego_front_s = ego_s + front_edge_to_rear_axle;
          front_s = std::max(0.0, lead_rear_s - ego_front_s);
          auto matched_point = ego_lane_coord->GetPathPointByS(lead_s);
          double heading_diff = planning_math::NormalizeAngle(
              lead_point.theta() - matched_point.theta());
          front_vel = lead_point.vel() * std::cos(heading_diff);
        }
      }
    }

    double next_acc = CalculateComfortAcceleration(
        current_a, current_v, current_s, front_vel, front_s);

    constexpr double kHalf = 0.5;
    const double dt2 = dt * dt;
    double next_v = std::max(0.0, current_v + next_acc * dt);
    double ds = std::max(0.0, current_v * dt + kHalf * next_acc * dt2);
    double next_s = current_s + ds;
    next_s = std::max(current_s, next_s);

    double ref_s = ref_ego_s + next_s;
    ref_s = std::clamp(ref_s, s_min, s_max);
    double ref_x = x_s_spline(ref_s);
    double ref_y = y_s_spline(ref_s);

    double dx_ds = x_s_spline.deriv(1, ref_s);
    double dy_ds = y_s_spline.deriv(1, ref_s);
    double d2x_ds2 = x_s_spline.deriv(2, ref_s);
    double d2y_ds2 = y_s_spline.deriv(2, ref_s);

    double ds_norm = std::sqrt(dx_ds * dx_ds + dy_ds * dy_ds);
    double curvature = 0.0;
    if (ds_norm > 1e-6) {
      curvature =
          (dx_ds * d2y_ds2 - dy_ds * d2x_ds2) / (ds_norm * ds_norm * ds_norm);
    }

    double next_delta = std::atan(wheel_base * curvature);
    next_delta = std::clamp(next_delta, -max_steer_angle, max_steer_angle);

    double next_theta =
        current_theta + (next_v * std::tan(next_delta) / wheel_base) * dt;
    double dtheta = planning_math::NormalizeAngle(next_theta - current_theta);
    next_theta = current_theta + dtheta;

    ref_trajectory_[i].t = t;
    ref_trajectory_[i].s = next_s;
    ref_trajectory_[i].vel = next_v;
    ref_trajectory_[i].acc = next_acc;
    ref_trajectory_[i].x = ref_x;
    ref_trajectory_[i].y = ref_y;
    ref_trajectory_[i].theta = next_theta;
    ref_trajectory_[i].delta = next_delta;

    current_s = next_s;
    current_v = next_v;
    current_a = next_acc;
    current_theta = next_theta;
    current_delta = next_delta;
    ego_s += ds;
  }

  planning_input.mutable_ref_x_vec()->Clear();
  planning_input.mutable_ref_y_vec()->Clear();
  planning_input.mutable_ref_theta_vec()->Clear();
  planning_input.mutable_ref_delta_vec()->Clear();
  planning_input.mutable_ref_vel_vec()->Clear();
  planning_input.mutable_ref_acc_vec()->Clear();
  planning_input.mutable_ref_s_vec()->Clear();

  for (size_t i = 0; i < kPlanningTimeSteps; ++i) {
    planning_input.mutable_ref_x_vec()->Add(ref_trajectory_[i].x);
    planning_input.mutable_ref_y_vec()->Add(ref_trajectory_[i].y);
    planning_input.mutable_ref_theta_vec()->Add(ref_trajectory_[i].theta);
    planning_input.mutable_ref_delta_vec()->Add(ref_trajectory_[i].delta);
    planning_input.mutable_ref_vel_vec()->Add(ref_trajectory_[i].vel);
    planning_input.mutable_ref_acc_vec()->Add(ref_trajectory_[i].acc);
    planning_input.mutable_ref_s_vec()->Add(ref_trajectory_[i].s);
  }
}

void JointDecisionInputBuilder::BuildLaneChangeWeightInfo(
    planning::common::JointDecisionPlanningInput& planning_input) {
  planning_input.set_q_ego_ref_x(lc_decision_config_.q_ego_ref_x);
  planning_input.set_q_ego_ref_y(lc_decision_config_.q_ego_ref_y);
  planning_input.set_q_ego_ref_theta(lc_decision_config_.q_ego_ref_theta);
  planning_input.set_q_ego_ref_delta(lc_decision_config_.q_ego_ref_delta);
  planning_input.set_q_ego_ref_vel(lc_decision_config_.q_ego_ref_vel);
  planning_input.set_q_ego_ref_acc(lc_decision_config_.q_ego_ref_acc);

  planning_input.set_q_obs_ref_x(lc_decision_config_.q_obs_ref_x);
  planning_input.set_q_obs_ref_y(lc_decision_config_.q_obs_ref_y);
  planning_input.set_q_obs_ref_theta(lc_decision_config_.q_obs_ref_theta);
  planning_input.set_q_obs_ref_delta(lc_decision_config_.q_obs_ref_delta);
  planning_input.set_q_obs_ref_vel(lc_decision_config_.q_obs_ref_vel);
  planning_input.set_q_obs_ref_acc(lc_decision_config_.q_obs_ref_acc);

  const auto& vehicle_param =
      planning::VehicleConfigurationContext::Instance()->get_vehicle_param();
  planning_input.mutable_curv_factor_vec()->Clear();
  for (size_t i = 0; i < kPlanningTimeSteps; ++i) {
    double c1 = 1 / std::max(vehicle_param.wheel_base, 1e-6);
    double c2 = 0.0;  // neutral
    double curv_factor =
        pnc::mathlib::GetCurvFactor(c1, c2, ref_trajectory_[i].vel);
    planning_input.mutable_curv_factor_vec()->Add(curv_factor);
  }

  planning_input.set_three_disc_safe_dist(
      lc_decision_config_.three_disc_safe_dist);
  planning_input.set_q_three_disc_safe_dist_weight(
      lc_decision_config_.q_three_disc_safe_dist_weight);

  planning_input.set_road_boundary_safe_dist(
      lc_decision_config_.road_boundary_safe_dist);
  planning_input.set_q_road_boundary_weight(
      lc_decision_config_.q_road_boundary_weight);
  planning_input.set_q_ego_acc_weight(lc_decision_config_.q_ego_acc_weight);
  planning_input.set_q_ego_jerk_weight(lc_decision_config_.q_ego_jerk_weight);
  planning_input.set_q_ego_omega_weight(lc_decision_config_.q_ego_omega_weight);
  planning_input.set_q_ego_delta_weight(lc_decision_config_.q_ego_delta_weight);
  planning_input.set_q_obs_jerk_weight(lc_decision_config_.q_obs_jerk_weight);
  planning_input.set_q_obs_omega_weight(lc_decision_config_.q_obs_omega_weight);

  planning_input.set_q_ego_acc_bound_weight(
      lc_decision_config_.q_ego_acc_bound_weight);
  planning_input.clear_ego_acc_max();
  planning_input.clear_ego_acc_min();

  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  const double current_vel = ego_state_manager->planning_init_point().v;
  double ego_acc_max = lc_decision_config_.ego_acc_max;
  // if (!lc_decision_config_.max_acc_bound_vel_table.empty() &&
  //     !lc_decision_config_.max_acc_bound_acc_table.empty() &&
  //     lc_decision_config_.max_acc_bound_vel_table.size() ==
  //         lc_decision_config_.max_acc_bound_acc_table.size()) {
  //   const auto& vel_table = lc_decision_config_.max_acc_bound_vel_table;
  //   const auto& acc_table = lc_decision_config_.max_acc_bound_acc_table;
  //   ego_acc_max = interp(current_vel, vel_table, acc_table);
  // }
  for (size_t i = 0; i < kPlanningTimeSteps; ++i) {
    planning_input.add_ego_acc_max(ego_acc_max);
    planning_input.add_ego_acc_min(lc_decision_config_.ego_acc_min);
  }

  planning_input.set_q_ego_jerk_bound_weight(
      lc_decision_config_.q_ego_jerk_bound_weight);
  planning_input.clear_ego_jerk_max();
  planning_input.clear_ego_jerk_min();
  for (size_t i = 0; i < kPlanningTimeSteps; ++i) {
    planning_input.add_ego_jerk_max(lc_decision_config_.ego_jerk_max);
    planning_input.add_ego_jerk_min(lc_decision_config_.ego_jerk_min);
  }

  planning_input.set_q_ego_vel_bound_weight(lc_decision_config_.q_ego_vel_bound_weight);
  planning_input.clear_ego_vel_max();
  const double ego_vel_max = comfort_params_.v0;
  for (size_t i = 0; i < kPlanningTimeSteps; ++i) {
    planning_input.add_ego_vel_max(ego_vel_max);
  }
  planning_input.set_q_hard_halfplane_weight(
      lc_decision_config_.q_hard_halfplane_weight);
  planning_input.set_hard_halfplane_dist(
      lc_decision_config_.hard_halfplane_dist);
  planning_input.set_halfplane_cost_allocation_ratio(
      lc_decision_config_.halfplane_cost_allocation_ratio);

  planning_input.set_q_soft_halfplane_weight(
      lc_decision_config_.q_soft_halfplane_weight);
  planning_input.set_soft_halfplane_s0(lc_decision_config_.soft_halfplane_s0);
  planning_input.set_soft_halfplane_tau(lc_decision_config_.soft_halfplane_tau);
  planning_input.set_soft_halfplane_cost_allocation_ratio(
      lc_decision_config_.soft_halfplane_cost_allocation_ratio);
  planning_input.set_halfplane_cost_allocation_ratio_later(
      lc_decision_config_.halfplane_cost_allocation_ratio_later);

  planning_input.set_obs_reaction_decay_time(
      lc_decision_config_.obs_reaction_decay_time);
  planning_input.set_obs_keep_ref_factor(
      lc_decision_config_.obs_keep_ref_factor);
}

void JointDecisionInputBuilder::BuildObsInfo(
    planning::common::JointDecisionPlanningInput& planning_input,
    const LaneChangeDecisionInfo& lc_info) {
  planning_input.clear_obs_init_state();
  planning_input.clear_obs_ref_trajectory();
  if (obstacles_selector_ != nullptr) {
    auto obs_select_start_time = IflyTime::Now_ms();
    obstacles_selector_->SelectLaneChangeObstacles(ref_trajectory_, lc_info);
    auto obs_select_end_time = IflyTime::Now_ms();
    JSON_DEBUG_VALUE("JointDecisionObstacleSelectionTime",
                     obs_select_end_time - obs_select_start_time);

    const auto& key_obstacles = obstacles_selector_->GetKeyObstacles();
    planning_input.set_obs_num(key_obstacles.size());

    for (const auto& obstacle : key_obstacles) {
      //检查 obstacle 完备性
      if (obstacle.ref_x_vec.size() < kPlanningTimeSteps ||
          obstacle.ref_y_vec.size() < kPlanningTimeSteps ||
          obstacle.ref_theta_vec.size() < kPlanningTimeSteps ||
          obstacle.ref_delta_vec.size() < kPlanningTimeSteps ||
          obstacle.ref_vel_vec.size() < kPlanningTimeSteps ||
          obstacle.ref_acc_vec.size() < kPlanningTimeSteps ||
          obstacle.ref_s_vec.size() < kPlanningTimeSteps) {
        continue;
      }
      key_agent_ids_.push_back(obstacle.agent_id);
      auto* obs_state = planning_input.add_obs_init_state();
      obs_state->set_x(obstacle.init_x);
      obs_state->set_y(obstacle.init_y);
      obs_state->set_theta(obstacle.init_theta);
      obs_state->set_delta(obstacle.init_delta);
      obs_state->set_vel(obstacle.init_vel);
      obs_state->set_acc(obstacle.init_acc);

      auto* obs_ref_trajectory = planning_input.add_obs_ref_trajectory();
      obs_ref_trajectory->set_length(obstacle.length);
      obs_ref_trajectory->set_width(obstacle.width);
      obs_ref_trajectory->set_longitudinal_label(obstacle.longitudinal_label);
      obs_ref_trajectory->set_obs_id(obstacle.agent_id);
      obs_ref_trajectory->set_init_s(obstacle.init_s);

      const double obs_wheel_base = obstacle.length * 0.75;
      for (size_t i = 0; i < obstacle.ref_x_vec.size(); ++i) {
        obs_ref_trajectory->add_ref_x_vec(obstacle.ref_x_vec[i]);
        obs_ref_trajectory->add_ref_y_vec(obstacle.ref_y_vec[i]);
        obs_ref_trajectory->add_ref_theta_vec(obstacle.ref_theta_vec[i]);
        obs_ref_trajectory->add_ref_delta_vec(obstacle.ref_delta_vec[i]);
        obs_ref_trajectory->add_ref_vel_vec(obstacle.ref_vel_vec[i]);
        obs_ref_trajectory->add_ref_acc_vec(obstacle.ref_acc_vec[i]);
        obs_ref_trajectory->add_ref_s_vec(obstacle.ref_s_vec[i]);

        double c1 = 1 / std::max(obs_wheel_base, 1e-6);
        double c2 = 0.0;
        double obs_curv_factor =
            pnc::mathlib::GetCurvFactor(c1, c2, obstacle.ref_vel_vec[i]);
        obs_ref_trajectory->add_curv_factor_vec(obs_curv_factor);
      }
    }
  }
}

void JointDecisionInputBuilder::BuildRoadInfo(
    planning::common::JointDecisionPlanningInput& planning_input,
    std::shared_ptr<
        pnc::lane_change_joint_decision::JointDecisionPlanningProblem>
        planning_problem_ptr) {
  // const auto& reference_path_ptr = session_->planning_context()
  //                                      .lane_change_decider_output()
  //                                      .coarse_planning_info.reference_path;
  const auto& reference_path_ptr = session_->environmental_model()
                                  .get_reference_path_manager()
                                  ->get_reference_path_by_current_lane();
  double ego_s = reference_path_ptr->get_frenet_ego_state().s();
  std::vector<planning::planning_math::PathPoint> left_road_points;
  std::vector<planning::planning_math::PathPoint> right_road_points;
  left_road_points.reserve(kPlanningTimeSteps);
  right_road_points.reserve(kPlanningTimeSteps);

  const std::shared_ptr<planning::planning_math::KDPath> frenet_coord =
      reference_path_ptr->get_frenet_coord();

  if (frenet_coord == nullptr) {
    return;
  }

  for (size_t i = 0; i < kPlanningTimeSteps; ++i) {
    double current_s = ego_s + ref_trajectory_[i].s;
    planning::ReferencePathPoint target_ref_point;
    if (!reference_path_ptr->get_reference_point_by_lon(current_s,
                                                        target_ref_point)) {
      continue;
    }

    double x, y;
    double left_road_distance =
        std::min(target_ref_point.distance_to_left_road_border, 20.0);
    if (frenet_coord->SLToXY(current_s, left_road_distance, &x, &y)) {
      left_road_points.emplace_back(planning::planning_math::PathPoint(x, y));
    }

    double right_road_distance =
        std::min(target_ref_point.distance_to_right_road_border, 20.0);
    if (frenet_coord->SLToXY(current_s, -right_road_distance, &x, &y)) {
      right_road_points.emplace_back(planning::planning_math::PathPoint(x, y));
    }
  }

  if (!left_road_points.empty() && !right_road_points.empty()) {
    planning_input.mutable_left_road_boundary()->clear_x_vec();
    planning_input.mutable_left_road_boundary()->clear_y_vec();
    planning_input.mutable_right_road_boundary()->clear_x_vec();
    planning_input.mutable_right_road_boundary()->clear_y_vec();

    for (const auto& point : left_road_points) {
      planning_input.mutable_left_road_boundary()->add_x_vec(point.x());
      planning_input.mutable_left_road_boundary()->add_y_vec(point.y());
    }

    for (const auto& point : right_road_points) {
      planning_input.mutable_right_road_boundary()->add_x_vec(point.x());
      planning_input.mutable_right_road_boundary()->add_y_vec(point.y());
    }

    auto left_boundary_path =
        left_road_points.size() < planning_math::KDPath::kKDPathMinPathPointSize
            ? nullptr
            : std::make_shared<planning::planning_math::KDPath>(
                  std::move(left_road_points));

    auto right_boundary_path =
        right_road_points.size() <
                planning_math::KDPath::kKDPathMinPathPointSize
            ? nullptr
            : std::make_shared<planning::planning_math::KDPath>(
                  std::move(right_road_points));

    planning_problem_ptr->SetBoundaryPaths(left_boundary_path,
                                           right_boundary_path);
  }

}  // namespace lane_change_joint_decision
}  // namespace lane_change_joint_decision
}  // namespace planning