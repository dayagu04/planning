#include "safety_target.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>

#include "behavior_planners/long_ref_path_decider/target_marker/target.h"
#include "behavior_planners/speed_limit_decider/speed_limit_decider_output.h"
#include "behavior_planners/long_ref_path_decider/long_ref_path_decider_output.h"
#include "common/config/basic_type.h"
#include "common/st_graph/st_graph_utils.h"
#include "debug_info_log.h"
#include "environmental_model.h"
#include "math/linear_interpolation.h"
#include "planning_context.h"
#include "trajectory1d/piecewise_jerk_acceleration_trajectory1d.h"
#include "utils/pose2d_utils.h"

namespace planning {

SafetyTarget::SafetyTarget(const SpeedPlannerConfig& config,
                           framework::Session* session)
    : Target(config, session) {
  safety_target_pb_.Clear();

  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  const double cruise_speed = ego_state_manager->ego_v_cruise();

  JSON_DEBUG_VALUE("cruise_speed", cruise_speed);

  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto lane_change_state = lane_change_decider_output.curr_state;
  const auto is_in_lane_change_execution =
      lane_change_state == StateMachineLaneChangeStatus::kLaneChangeExecution ||
      lane_change_state == StateMachineLaneChangeStatus::kLaneChangeComplete ||
      lane_change_state == StateMachineLaneChangeStatus::kLaneChangeCancel;

  const auto& speed_limit_decider_output =
      session_->planning_context().speed_limit_decider_output();

  double speed_limit_normal = cruise_speed;
  const double speed_limit_from_lane_change =
      is_in_lane_change_execution
          ? config_.lane_change_upper_speed_limit_kph / 3.6
          : std::numeric_limits<double>::max();
  speed_limit_normal =
      std::fmin(speed_limit_normal, speed_limit_from_lane_change);

  double speed_limit_ref = std::numeric_limits<double>::max();
  auto speed_limit_type_ref = SpeedLimitType::NONE;
  speed_limit_decider_output.GetSpeedLimit(&speed_limit_ref,
                                           &speed_limit_type_ref);

  const double desired_speed = std::fmin(speed_limit_normal, speed_limit_ref);

  JSON_DEBUG_VALUE("limit_speed", desired_speed);

  idm_params_.v0 = desired_speed;
  idm_params_.s0 = 3.5;
  idm_params_.T = 1.0;
  idm_params_.a = 1.5;
  idm_params_.b = 1.0;
  idm_params_.b_max = 2.0;
  idm_params_.delta = 4.0;
  idm_params_.b_hard = 4.0;
  idm_params_.max_a_jerk = 5.0;
  idm_params_.max_b_jerk = 1.0;
  idm_params_.max_deceleration_jerk_lat_follow = 2.0;
  idm_params_.max_deceleration_jerk_lon_cutin = 4.0;
  idm_params_.virtual_front_s = 200.0;
  idm_params_.cool_factor = 0.99;
  idm_params_.over_speed_factor = 0.3;
  is_lat_follow_ = false;
  is_lon_cutin_ = false;
  idm_params_.follow_consider_distance = 10.0;
  idm_params_.follow_consider_time_headway = 1.5;

  upper_bound_infos_ =
      std::vector<UpperBoundInfo>(plan_points_num_, UpperBoundInfo());

  follow_agent_ids_.clear();

  GenerateUpperBoundInfo();

  acc_values_ = std::vector<double>(plan_points_num_, 0.0);
  std::vector<double> follow_agent_ids_double(follow_agent_ids_.begin(), follow_agent_ids_.end());
  JSON_DEBUG_VECTOR("safety_follow_agent_ids", follow_agent_ids_double, 0);

  GenerateSafetyTarget();

  AddSafetyTargetDataToProto();
}

void SafetyTarget::GenerateUpperBoundInfo() {
  const auto* st_graph = session_->planning_context().st_graph_helper();
  const double virtual_front_s = idm_params_.virtual_front_s;
  const double virtual_front_vel = idm_params_.v0;
  const auto& ego_vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double front_edge_to_rear_axle =
      ego_vehicle_param.front_edge_to_rear_axle;
  const auto& ego_lane = session_->environmental_model()
                             .get_virtual_lane_manager()
                             ->get_current_lane();
  if (ego_lane == nullptr || ego_lane->get_lane_frenet_coord() == nullptr) {
    return;
  }
  const auto& ego_lane_coord = ego_lane->get_lane_frenet_coord();
  if (ego_lane_coord == nullptr) {
    return;
  }

  double ego_s = 0.0, ego_l = 0.0;
  const auto& ego_init_point = session_->environmental_model()
                                   .get_ego_state_manager()
                                   ->planning_init_point();
  if (!ego_lane_coord->XYToSL(ego_init_point.x, ego_init_point.y, &ego_s,
                              &ego_l)) {
    return;
  }

  const auto& lat_obstacle_decision = session_->planning_context()
                                          .lateral_obstacle_decider_output()
                                          .lat_obstacle_decision;
  const auto& lane_borrow_output =
      session_->planning_context().lane_borrow_decider_output();
  const auto& blocked_obs_id = lane_borrow_output.blocked_obs_id;

  std::vector<FollowAgentWithSource> follow_agents;
  std::unordered_set<int32_t> added_agent_ids;
  std::unordered_set<int32_t> forbidden_ids(blocked_obs_id.begin(),
                                            blocked_obs_id.end());

  // 从 lat_obstacle_decision 中收集 FOLLOW 类型的 agents
  for (const auto& [agent_id, decision] : lat_obstacle_decision) {
    if (decision == LatObstacleDecisionType::FOLLOW) {
      if (forbidden_ids.find(agent_id) != forbidden_ids.end() ||
          added_agent_ids.find(agent_id) != added_agent_ids.end()) {
        continue;
      }

      auto agent =
          session_->environmental_model().get_agent_manager()->GetAgent(
              agent_id);
      if (agent != nullptr) {
        follow_agents.push_back(
            {agent, FollowAgentSource::kLatObstacleDecision});
        added_agent_ids.insert(agent_id);
      }
    }
  }

  const auto& agent_longitudinal_decider_output =
      session_->planning_context().agent_longitudinal_decider_output();
  const auto& cutin_ids = agent_longitudinal_decider_output.cutin_agent_ids;

  // 从 cutin_agent_ids 中收集符合条件的 agents
  for (const int32_t cutin_id : cutin_ids) {
    if (forbidden_ids.find(cutin_id) != forbidden_ids.end() ||
        added_agent_ids.find(cutin_id) != added_agent_ids.end()) {
      continue;
    }

    auto agent =
        session_->environmental_model().get_agent_manager()->GetAgent(cutin_id);
    if (agent != nullptr) {
      double agent_s = 0.0, agent_l = 0.0;
      if (!ego_lane_coord->XYToSL(agent->x(), agent->y(), &agent_s, &agent_l)) {
        continue;
      }
      const double dis_relative =
          agent_s - ego_s - 0.5 * agent->length() - front_edge_to_rear_axle;
      const double dis_lon_consider =
          std::max(ego_init_point.v * idm_params_.follow_consider_time_headway,
                   idm_params_.follow_consider_distance);

      if (dis_relative <= dis_lon_consider) {
        follow_agents.push_back({agent, FollowAgentSource::kCutinAgentIds});
        added_agent_ids.insert(cutin_id);
      }
    }
  }

  std::vector<FollowAgentInfo> follow_agent_infos(plan_points_num_);
  for (const auto& agent_with_source : follow_agents) {
    const auto& agent = agent_with_source.agent;
    follow_agent_ids_.push_back(agent->agent_id());
  }

  if (!follow_agents.empty()) {
    for (size_t i = 0; i < plan_points_num_; i++) {
      double min_agent_s = std::numeric_limits<double>::max();
      FollowAgentInfo best_agent_info = {
          0, 0.0, 0.0, FollowAgentSource::kLatObstacleDecision};
      bool found_valid_agent = false;

      for (const auto& agent_with_source : follow_agents) {
        const auto& agent = agent_with_source.agent;
        const auto& agent_trajectories = agent->trajectories_used_by_st_graph();
        if (agent_trajectories.empty()) continue;

        const auto& traj_point = agent_trajectories.front()[i];
        double center_s = 0.0, center_l = 0.0;

        if (!ego_lane_coord->XYToSL(traj_point.x(), traj_point.y(), &center_s,
                                    &center_l)) {
          continue;
        }

        auto matched_point = ego_lane_coord->GetPathPointByS(center_s);
        double heading_diff = traj_point.theta() - matched_point.theta();
        double agent_speed = traj_point.vel() * std::cos(heading_diff);
        double agent_s =
            center_s - ego_s - front_edge_to_rear_axle - agent->length() * 0.5;

        if (agent_s < min_agent_s) {
          min_agent_s = agent_s;
          best_agent_info = {agent->agent_id(), agent_s, agent_speed,
                             agent_with_source.source};
          found_valid_agent = true;
        }
      }

      if (found_valid_agent) {
        follow_agent_infos[i] = best_agent_info;
      }
    }
  }

  for (size_t i = 0; i < plan_points_num_; i++) {
    const double t = i * dt_;

    if (st_graph != nullptr) {
      const auto& upper_bound = st_graph->GetPassCorridorUpperBound(t);
      if (upper_bound.agent_id() != speed::kNoAgentId) {
        upper_bound_infos_[i] = {
            upper_bound.s(),           t,
            upper_bound.velocity(),    TargetType::kSafety,
            upper_bound.agent_id(),    upper_bound.boundary_id(),
            upper_bound.acceleration()};
      } else {
        upper_bound_infos_[i] = {
            virtual_front_s, t,  virtual_front_vel, TargetType::kSafety, 799999,
            799999,          0.0};
      }

      if (!follow_agents.empty() && follow_agent_infos[i].s < upper_bound.s()) {
        if (follow_agent_infos[i].source ==
            FollowAgentSource::kLatObstacleDecision) {
          is_lat_follow_ = true;
        } else if (follow_agent_infos[i].source ==
                   FollowAgentSource::kCutinAgentIds) {
          is_lon_cutin_ = true;
        }

        upper_bound_infos_[i] = {follow_agent_infos[i].s,
                                 t,
                                 follow_agent_infos[i].v,
                                 TargetType::kSafety,
                                 follow_agent_infos[i].agent_id,
                                 899999,
                                 0.0};
      }
    }
  }
}

void SafetyTarget::GenerateSafetyTarget() {
  const double default_t = 0.0;
  const bool default_has_target = false;
  const double default_s_target = 0.0;
  const double default_v_target = 0.0;
  const TargetType default_target_type = TargetType::kNotSet;
  auto default_target_value =
      TargetValue(default_t, default_has_target, default_s_target,
                  default_v_target, default_target_type);
  target_values_ =
      std::vector<TargetValue>(plan_points_num_, default_target_value);

  double current_s = init_lon_state_[0];
  double current_v = init_lon_state_[1];
  double current_a = init_lon_state_[2];

  target_values_[0].set_relative_t(0.0);
  target_values_[0].set_has_target(true);
  target_values_[0].set_s_target_val(current_s);
  target_values_[0].set_v_target_val(current_v);
  target_values_[0].set_target_type(TargetType::kSafety);
  acc_values_[0] = current_a;

  for (int32_t i = 1; i < plan_points_num_; i++) {
    const double t = i * dt_;
    auto& target_value = target_values_[i];
    target_value.set_relative_t(t);

    const double front_s = upper_bound_infos_[i].s;
    const double front_vel = upper_bound_infos_[i].v;

    double tau = idm_params_.T;
    const auto& agents_headway_Info = session_->planning_context()
                                          .agent_headway_decider_output()
                                          .agents_headway_Info();
    const auto* st_graph = session_->planning_context().st_graph_helper();

    if (st_graph != nullptr) {
      const auto& upper_bound = st_graph->GetPassCorridorUpperBound(t);
      if (upper_bound.agent_id() != speed::kNoAgentId) {
        const int32_t lead_id = upper_bound.agent_id();
        auto iter = agents_headway_Info.find(lead_id);
        if (iter != agents_headway_Info.end()) {
          tau = iter->second.current_headway;
        }
      }
    }

    double safety_acc = CalculateSafetyAcceleration(
        current_a, current_v, current_s, front_vel, front_s, tau);

    acc_values_[i] = safety_acc;

    double next_s = current_s + current_v * dt_ + 0.5 * safety_acc * dt_ * dt_;
    double next_v = current_v + safety_acc * dt_;

    next_s = std::max(current_s, next_s);
    next_v = std::max(0.0, next_v);

    target_value.set_has_target(true);
    target_value.set_s_target_val(next_s);
    target_value.set_v_target_val(next_v);
    target_value.set_target_type(TargetType::kSafety);
    
    current_s = next_s;
    current_v = next_v;
    current_a = safety_acc;
  }
}

double SafetyTarget::CalcDesiredVelocity(const double d_rel, const double d_des,
                                         const double v_lead,
                                         const double v_ego) const {
  double v_lead_clip = std::max(v_lead, 0.0);
  const double max_runaway_speed = -2.0;
  double l_slope = interp(v_lead, _L_SLOPE_BP, _L_SLOPE_V);
  double p_slope = interp(v_lead, _P_SLOPE_BP, _P_SLOPE_V);
  double x_linear_to_parabola = p_slope / std::pow(l_slope, 2);
  double x_parabola_offset = p_slope / (2 * std::pow(l_slope, 2));

  double v_rel = v_ego - v_lead;
  double v_rel_des = 0.0;
  double soft_brake_distance = 0.0;
  if (d_rel < d_des) {
    double v_rel_des_1 = (-max_runaway_speed) / d_des * (d_rel - d_des);
    double v_rel_des_2 = (d_rel - d_des) * l_slope / 3.0;
    v_rel_des = std::min(v_rel_des_1, v_rel_des_2);
    v_rel_des = std::max(v_rel_des, max_runaway_speed);
    soft_brake_distance = d_rel;
  } else if (d_rel < d_des + x_linear_to_parabola) {
    v_rel_des = (d_rel - d_des) * l_slope;
    v_rel_des = std::max(v_rel_des, max_runaway_speed);
    soft_brake_distance = v_rel / l_slope + d_des;
  } else {
    v_rel_des = std::sqrt(2 * (d_rel - d_des - x_parabola_offset) * p_slope);
    soft_brake_distance =
        std::pow(v_rel, 2) / (2 * p_slope) + x_parabola_offset + d_des;
  }
  double v_target = v_rel_des + v_lead;
  return v_target;
}

double SafetyTarget::CalculateSafetyAcceleration(
    const double current_acc, const double current_vel, const double current_s,
    const double front_vel, const double front_s, const double tau) const {
  double v0 = idm_params_.v0;
  double a = idm_params_.a;
  double b = idm_params_.b;
  double b_max = idm_params_.b_max;
  double b_hard = idm_params_.b_hard;
  double delta = idm_params_.delta;
  double s0 = idm_params_.s0;
  double cool_factor = idm_params_.cool_factor;
  double over_speed_factor = idm_params_.over_speed_factor;

  double s_alpha = std::max(1e-3, front_s - current_s);
  double delta_v = current_vel - front_vel;

  double s_star =
      s0 + std::max(0.0, current_vel * tau + (current_vel * delta_v) /
                                                 (2.0 * std::sqrt(a * b_max)));

  double s_safe = s0 + current_vel * tau;

  double s_desired = std::max(s0, front_s - s_safe);

  double dynamic_v0 =
      CalcDesiredVelocity(front_s - current_s, s_safe, front_vel, current_vel);

  double desired_v0 = std::min(v0, dynamic_v0);

  double final_v0 = 0.0;

  if (current_s < s_desired) {
    final_v0 = v0;
  } else {
    final_v0 = desired_v0;
  }

  double a_free;
  if (current_vel <= final_v0) {
    a_free = a * (1.0 - std::pow(current_vel / final_v0, delta));
  } else if (current_vel > final_v0 && current_s < s_desired) {
    double s_progress = std::max(0.0, std::min(1.0, current_s / s_desired));
    double over_vel_ratio = current_vel / std::max(final_v0, 1e-6);
    double position_ratio = 1.0 - s_progress;
    double over_speed_ratio = over_vel_ratio - 1.0;
    double over_speed_demand =
        position_ratio - over_speed_ratio * over_speed_factor;
    if (over_speed_demand > 0.1) {
      a_free = a * over_speed_factor * over_speed_demand;
    } else {
      a_free = -b * over_speed_factor * over_speed_ratio;
    }
  } else {
    a_free = -b * (1.0 - std::pow(final_v0 / current_vel, a * delta / b));
  }

  double z = s_star / s_desired;

  double a_idm;
  if (current_vel <= final_v0) {
    if (z >= 1.0) {
      a_idm = a * (1.0 - std::pow(z, 2.0));
    } else {
      if (std::abs(a_free) > 1e-6) {
        a_idm = a_free * (1.0 - std::pow(z, 2.0 * a / a_free));
      } else {
        a_idm = a * (1.0 - std::pow(z, 2.0));
      }
    }
  } else {
    if (z >= 1.0) {
      a_idm = a_free + a * (1.0 - std::pow(z, 2.0));
    } else {
      a_idm = a_free;
    }
  }

  double ds_star = s_alpha - s_star;
  double ds_safe = s_alpha - s_safe;
  double a_cah;
  if (ds_safe < 0.0 && ds_star < 0.0) {
    a_cah = b_hard * ds_star / s_star;
  } else if (ds_safe > 0.0 && ds_star < 0.0) {
    a_cah = b * ds_star / s_star;
  } else {
    a_cah = a_idm;
  }

  double final_acc;
  if (a_idm >= a_cah) {
    double distance_ratio = std::min(current_s / s_desired, 1.0);
    final_acc = a_idm * (1.0 - distance_ratio) + a_cah * distance_ratio;
  } else {
    final_acc = (1.0 - cool_factor) * a_idm +
                cool_factor * (a_cah - b * tanh((a_idm - a_cah) / (-b)));
  }

  double max_decel_jerk = 0.0;
  if (is_lat_follow_) {
    max_decel_jerk = idm_params_.max_deceleration_jerk_lat_follow;
  } else if (is_lon_cutin_) {
    max_decel_jerk = idm_params_.max_deceleration_jerk_lon_cutin;
  } else {
    max_decel_jerk = idm_params_.max_b_jerk;
  }

  double acc_change = final_acc - current_acc;
  if (acc_change > 0 && acc_change > idm_params_.max_a_jerk * dt_) {
    final_acc = current_acc + idm_params_.max_a_jerk * dt_;
  } else if (acc_change < 0 && acc_change < -max_decel_jerk * dt_) {
    final_acc = current_acc - max_decel_jerk * dt_;
  }

  final_acc = std::max(std::min(a, final_acc), -idm_params_.b_hard);

  return final_acc;
}

void SafetyTarget::AddSafetyTargetDataToProto() {
#ifdef ENABLE_PROTO_LOG
  auto& debug_info_pb = DebugInfoManager::GetInstance().GetDebugInfoPb();
  auto mutable_safety_target_data =
      debug_info_pb->mutable_lon_target_s_ref()->mutable_safety_target();

  if (!target_values_.empty()) {
    for (int32_t i = 0; i < plan_points_num_; i++) {
      const auto& value = target_values_[i];
      auto* ptr = safety_target_pb_.add_safety_target_s_ref();
      ptr->set_s(value.s_target_val());
      ptr->set_v(value.v_target_val());
      ptr->set_t(value.relative_t());
      ptr->set_a(acc_values_[i]);
      ptr->set_target_type(static_cast<int32_t>(value.target_type()));
    }
  }

  for (const auto& upper_bound : upper_bound_infos_) {
    auto* ptr = safety_target_pb_.add_upper_bound_velocities();
    ptr->set_t(upper_bound.t);
    ptr->set_v(upper_bound.v);
    ptr->set_a(upper_bound.a);
    ptr->set_agent_id(upper_bound.agent_id);
  }

  mutable_safety_target_data->CopyFrom(safety_target_pb_);
#endif

  auto mutable_lon_ref_path_decider_output =
      session_->mutable_planning_context()
          ->mutable_lon_ref_path_decider_output();

  mutable_lon_ref_path_decider_output->is_safety_target_lat_follow =
      is_lat_follow_;
  mutable_lon_ref_path_decider_output->is_safety_target_lon_cutin =
      is_lon_cutin_;
  mutable_lon_ref_path_decider_output->follow_agent_ids = follow_agent_ids_;
  mutable_lon_ref_path_decider_output->safe_target_upper_bound_infos.clear();
  mutable_lon_ref_path_decider_output->safe_target_upper_bound_infos.reserve(
      upper_bound_infos_.size());
  for (const auto& info : upper_bound_infos_) {
    SafetyTargetUpperBoundInfo output_info;
    output_info.s = info.s;
    output_info.t = info.t;
    output_info.v = info.v;
    output_info.agent_id = info.agent_id;
    output_info.st_boundary_id = info.st_boundary_id;
    mutable_lon_ref_path_decider_output->safe_target_upper_bound_infos
        .push_back(output_info);
  }
}

}  // namespace planning