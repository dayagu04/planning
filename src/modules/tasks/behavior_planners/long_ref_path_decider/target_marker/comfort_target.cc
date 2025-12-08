#include "comfort_target.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "behavior_planners/long_ref_path_decider/long_ref_path_decider_output.h"
#include "behavior_planners/long_ref_path_decider/target_marker/target.h"
#include "behavior_planners/speed_limit_decider/speed_limit_decider_output.h"
#include "common/config/basic_type.h"
#include "debug_info_log.h"
#include "environmental_model.h"
#include "math/math_utils.h"
#include "planning_context.h"

namespace planning {

ComfortTarget::ComfortTarget(const SpeedPlannerConfig& config,
                             framework::Session* session)
    : Target(config, session) {
  comfort_target_pb_.Clear();

  comfort_params_.s0 = 3.5;
  comfort_params_.T = 1.0;
  comfort_params_.a = 1.5;
  comfort_params_.b_max = 2.0;
  comfort_params_.b = 1.0;
  comfort_params_.b_hard = 4.0;
  comfort_params_.delta = 4.0;
  comfort_params_.max_accel_jerk = 3.0;
  comfort_params_.min_decel_jerk = 1.0;
  comfort_params_.max_decel_jerk = 1.5;
  comfort_params_.virtual_front_s = 200.0;
  comfort_params_.cool_factor = 0.99;
  comfort_params_.follow_consider_distance = 15.0;
  comfort_params_.follow_consider_time_headway = 1.5;
  comfort_params_.delay_time_buffer = 0.3;
  comfort_params_.eps = 1e-6;
  comfort_params_.static_speed_threshold = 0.2;

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
      lane_change_state == StateMachineLaneChangeStatus::kLaneChangeHold;

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

  comfort_params_.v0 = desired_speed;

  const auto& cipv_info = session_->planning_context().cipv_decider_output();
  const auto& stop_destination_decider_output =
      session_->planning_context().stop_destination_decider_output();
  if (session_->is_rads_scene() &&
      cipv_info.cipv_id() ==
          stop_destination_decider_output.stop_destination_virtual_agent_id()) {
    comfort_params_.s0 = 0.0;
  }

  upper_bound_infos_ =
      std::vector<UpperBoundInfo>(plan_points_num_, UpperBoundInfo());

  follow_agent_ids_.clear();

  joint_danger_agent_ids_.clear();

  GenerateUpperBoundInfo();

  std::vector<double> follow_agent_ids_double(follow_agent_ids_.begin(),
                                              follow_agent_ids_.end());

  std::vector<double> joint_danger_agent_ids_double(
      joint_danger_agent_ids_.begin(), joint_danger_agent_ids_.end());

  JSON_DEBUG_VECTOR("comfort_follow_agent_ids", follow_agent_ids_double, 0);
  JSON_DEBUG_VECTOR("joint_danger_agent_ids", joint_danger_agent_ids_double, 0);

  acc_values_ = std::vector<double>(plan_points_num_, 0.0);

  GenerateComfortTarget();

  comfort_jerk_min_vec_[0] = comfort_jerk_min_vec_[1];
  JSON_DEBUG_VECTOR("comfort_jerk_min_vec", comfort_jerk_min_vec_, 1);
  comfort_v_target_vec_[0] = comfort_v_target_vec_[1];
  JSON_DEBUG_VECTOR("comfort_v_target_vec", comfort_v_target_vec_, 1);

  auto mutable_lon_ref_path_decider_output =
      session_->mutable_planning_context()
          ->mutable_lon_ref_path_decider_output();

  mutable_lon_ref_path_decider_output->is_comfort_target_lat_follow =
      is_lat_follow_;
  mutable_lon_ref_path_decider_output->is_comfort_target_lon_cutin =
      is_lon_cut_in_;
  mutable_lon_ref_path_decider_output->follow_agent_ids = follow_agent_ids_;
  mutable_lon_ref_path_decider_output->comfort_target_upper_bound_infos.clear();
  mutable_lon_ref_path_decider_output->comfort_target_upper_bound_infos.reserve(
      upper_bound_infos_.size());
  for (const auto& info : upper_bound_infos_) {
    ComfortTargetUpperBoundInfo output_info;
    output_info.s = info.s;
    output_info.t = info.t;
    output_info.v = info.v;
    output_info.agent_id = info.agent_id;
    output_info.st_boundary_id = info.st_boundary_id;
    mutable_lon_ref_path_decider_output->comfort_target_upper_bound_infos
        .push_back(output_info);
  }

  AddComfortTargetDataToProto();
}

void ComfortTarget::GenerateUpperBoundInfo() {
  const auto* st_graph = session_->planning_context().st_graph_helper();
  const double virtual_front_s = comfort_params_.virtual_front_s;
  const double virtual_front_vel = comfort_params_.v0;
  const int32_t virtual_front_agent_id = 799999;
  const int32_t virtual_front_st_boundary_id = 799999;
  const double virtual_front_acc = 0.0;
  const auto& ego_vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double front_edge_to_rear_axle =
      ego_vehicle_param.front_edge_to_rear_axle;
  const double rear_edge_to_front_axle =
      ego_vehicle_param.rear_edge_to_rear_axle;
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
  double ego_v = init_lon_state_[1];
  const auto& ego_init_point = session_->environmental_model()
                                   .get_ego_state_manager()
                                   ->planning_init_point();
  if (!ego_lane_coord->XYToSL(ego_init_point.x, ego_init_point.y, &ego_s,
                              &ego_l)) {
    return;
  }

  const auto& route_info = session_->environmental_model().get_route_info();
  const auto& ego_lane_road_right_output =
  session_->planning_context().ego_lane_road_right_decider_output();
  bool filtering_rear_agent = true;
  if (route_info != nullptr) {
    const auto& route_info_output = route_info->get_route_info_output();
    if (route_info_output.is_closing_merge ||
        route_info_output.is_closing_split ||
        ego_lane_road_right_output.is_merge_region ||
        ego_lane_road_right_output.is_split_region) {
      filtering_rear_agent = false;
    }
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

  const auto& lat_lon_joint_planner_output =
      session_->planning_context().lat_lon_joint_planner_decider_output();
  const auto& danger_ids = lat_lon_joint_planner_output.GetDangerObstacleIds();

  for (const int32_t danger_id : danger_ids) {
    joint_danger_agent_ids_.push_back(danger_id);
  }

  std::vector<int32_t> critical_agent_ids;
  critical_agent_ids.reserve(cutin_ids.size() + danger_ids.size());
  critical_agent_ids.insert(critical_agent_ids.end(), cutin_ids.begin(),
                            cutin_ids.end());
  critical_agent_ids.insert(critical_agent_ids.end(), danger_ids.begin(),
                            danger_ids.end());

  for (const int32_t agent_id : critical_agent_ids) {
    if (forbidden_ids.find(agent_id) != forbidden_ids.end() ||
        added_agent_ids.find(agent_id) != added_agent_ids.end()) {
      continue;
    }

    auto agent =
        session_->environmental_model().get_agent_manager()->GetAgent(agent_id);
    if (agent == nullptr) {
      continue;
    }

    double agent_s = 0.0;
    double agent_l = 0.0;
    if (!ego_lane_coord->XYToSL(agent->x(), agent->y(), &agent_s, &agent_l)) {
      continue;
    }

    if (agent_s < ego_s && filtering_rear_agent) {
      continue;
    }

    const double dis_relative =
        agent_s - ego_s - 0.5 * agent->length() - front_edge_to_rear_axle;
    const double dis_lon_consider = std::max(
        ego_init_point.v * comfort_params_.follow_consider_time_headway,
        comfort_params_.follow_consider_distance);

    const bool is_traffic_control_obstacle =
        agent->type() == agent::AgentType::TRAFFIC_CONE ||
        agent->type() == agent::AgentType::CTASH_BARREL ||
        agent->type() == agent::AgentType::WATER_SAFETY_BARRIER;

    bool is_lateral_left_or_right = false;
    auto lat_decision_iter = lat_obstacle_decision.find(agent_id);
    if (lat_decision_iter != lat_obstacle_decision.end()) {
      const auto& lat_decision = lat_decision_iter->second;
      is_lateral_left_or_right =
          (lat_decision == LatObstacleDecisionType::LEFT ||
           lat_decision == LatObstacleDecisionType::RIGHT);
    }

    if ((agent->is_static() ||
         std::fabs(agent->speed()) < comfort_params_.static_speed_threshold) &&
        !is_traffic_control_obstacle && is_lateral_left_or_right) {
      continue;
    }

    if (dis_relative <= dis_lon_consider) {
      follow_agents.push_back({agent, FollowAgentSource::kCutinAgentIds});
      added_agent_ids.insert(agent_id);
    }
  }

  std::vector<FollowAgentInfo> follow_agent_infos(plan_points_num_);
  std::unordered_set<int32_t> valid_agent_ids;

  if (!follow_agents.empty()) {
    for (size_t i = 0; i < plan_points_num_; i++) {
      double min_agent_s = std::numeric_limits<double>::max();
      FollowAgentInfo best_agent_info = {
          899999, 210.0,  comfort_params_.v0,
          0.0,    899999, FollowAgentSource::kLatObstacleDecision};
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
        double heading_diff = planning_math::NormalizeAngle(
            traj_point.theta() - matched_point.theta());
        double agent_speed = traj_point.vel() * std::cos(heading_diff);
        double agent_acc = traj_point.acc() * std::cos(heading_diff);
        double agent_s = std::max(
            comfort_params_.eps,
            center_s - ego_s - front_edge_to_rear_axle - agent->length() * 0.5);

        if (agent_s < min_agent_s) {
          min_agent_s = agent_s;
          best_agent_info = {
              agent->agent_id(), agent_s, agent_speed,
              agent_acc,         899999,  agent_with_source.source};
          found_valid_agent = true;
        }
      }

      if (found_valid_agent) {
        follow_agent_infos[i] = best_agent_info;
        valid_agent_ids.insert(best_agent_info.agent_id);
      }
    }
  }

  for (size_t i = 0; i < plan_points_num_; i++) {
    const double t = i * dt_;

    if (st_graph != nullptr) {
      const auto& upper_bound = st_graph->GetPassCorridorUpperBound(t);
      if (upper_bound.agent_id() != speed::kNoAgentId) {
        upper_bound_infos_[i] = {upper_bound.s(),
                                 t,
                                 upper_bound.velocity(),
                                 TargetType::kComfort,
                                 upper_bound.agent_id(),
                                 upper_bound.boundary_id(),
                                 upper_bound.acceleration(),
                                 false,
                                 false};
      } else {
        upper_bound_infos_[i] = {virtual_front_s,
                                 t,
                                 virtual_front_vel,
                                 TargetType::kComfort,
                                 virtual_front_agent_id,
                                 virtual_front_st_boundary_id,
                                 virtual_front_acc,
                                 false,
                                 false};
      }

      if (!follow_agents.empty() && follow_agent_infos[i].s < upper_bound.s()) {
        if (follow_agent_infos[i].source ==
            FollowAgentSource::kLatObstacleDecision) {
          is_lat_follow_ = true;
        } else if (follow_agent_infos[i].source ==
                   FollowAgentSource::kCutinAgentIds) {
          is_lon_cut_in_ = true;
        }

        upper_bound_infos_[i] = {
            follow_agent_infos[i].s,
            t,
            follow_agent_infos[i].v,
            TargetType::kComfort,
            follow_agent_infos[i].agent_id,
            follow_agent_infos[i].st_boundary_id,
            follow_agent_infos[i].a,
            follow_agent_infos[i].source ==
                FollowAgentSource::kLatObstacleDecision,
            follow_agent_infos[i].source == FollowAgentSource::kCutinAgentIds};
      }
    }
  }

  follow_agent_ids_.assign(valid_agent_ids.begin(), valid_agent_ids.end());
  std::sort(follow_agent_ids_.begin(), follow_agent_ids_.end());
}

void ComfortTarget::GenerateComfortTarget() {
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
  target_values_[0].set_target_type(TargetType::kComfort);
  acc_values_[0] = current_a;

  comfort_jerk_min_vec_.clear();
  comfort_jerk_min_vec_.resize(plan_points_num_,
                               comfort_params_.min_decel_jerk);

  comfort_v_target_vec_.clear();
  comfort_v_target_vec_.resize(plan_points_num_, comfort_params_.v0);

  for (int32_t i = 1; i < plan_points_num_; i++) {
    const double t = i * dt_;
    auto& target_value = target_values_[i];
    target_value.set_relative_t(t);

    const double front_s = upper_bound_infos_[i - 1].s;
    const double front_vel = upper_bound_infos_[i - 1].v;

    const bool is_follow = upper_bound_infos_[i - 1].is_follow;
    const bool is_cut_in = upper_bound_infos_[i - 1].is_cut_in;

    double tau = comfort_params_.T;
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

    double min_follow_distance =
        comfort_params_.s0 + current_v * comfort_params_.delay_time_buffer;
    double max_follow_distance = comfort_params_.s0 + current_v * tau;

    double max_decel_jerk = (is_cut_in || is_follow)
                                ? comfort_params_.max_decel_jerk
                                : comfort_params_.min_decel_jerk;

    double decel_jerk = comfort_params_.min_decel_jerk;

    if (front_s >= max_follow_distance) {
      decel_jerk = comfort_params_.min_decel_jerk;
    } else if (front_s <= min_follow_distance) {
      decel_jerk = max_decel_jerk;
    } else {
      double distance_diff = max_follow_distance - min_follow_distance;
      if (std::abs(distance_diff) < comfort_params_.eps) {
        decel_jerk = comfort_params_.min_decel_jerk;
      } else {
        double ratio = (front_s - min_follow_distance) / distance_diff;
        double smooth_ratio = 3.0 * ratio * ratio - 2.0 * ratio * ratio * ratio;
        decel_jerk =
            max_decel_jerk +
            smooth_ratio * (comfort_params_.min_decel_jerk - max_decel_jerk);
      }
    }

    comfort_jerk_min_vec_[i] = -decel_jerk;

    double comfort_acc = CalculateComfortAcceleration(
        current_a, current_v, current_s, front_vel, front_s, tau, decel_jerk,
        comfort_v_target_vec_[i]);
    acc_values_[i] = comfort_acc;
    double ds = std::max(0.0, current_v * dt_ + 0.5 * comfort_acc * dt_ * dt_);
    double next_s = current_s + ds;
    double next_v = current_v + comfort_acc * dt_;

    next_v = std::max(0.0, next_v);

    target_value.set_has_target(true);
    target_value.set_s_target_val(next_s);
    target_value.set_v_target_val(next_v);
    target_value.set_target_type(TargetType::kComfort);

    current_s = next_s;
    current_v = next_v;
    current_a = comfort_acc;
  }
}

double ComfortTarget::CalculateComfortAcceleration(
    const double current_acc, const double current_vel, const double current_s,
    const double front_vel, const double front_s, const double tau,
    const double decel_jerk, double& v_target) const {
  double s0 = comfort_params_.s0;
  double v0 = comfort_params_.v0;
  double a = comfort_params_.a;
  double b_max = comfort_params_.b_max;
  double b = comfort_params_.b;
  double b_hard = comfort_params_.b_hard;
  double max_accel_jerk = comfort_params_.max_accel_jerk;
  double delta = comfort_params_.delta;
  double cool_factor = comfort_params_.cool_factor;
  double eps = comfort_params_.eps;

  double s_alpha = std::max(eps, front_s - current_s);

  double delta_v = current_vel - front_vel;
  double s_star =
      s0 + std::max(0.0, current_vel * tau + (current_vel * delta_v) /
                                                 (2.0 * std::sqrt(a * b_max)));

  v_target = std::max(eps, v0);

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
  if (acc_change > 0 && acc_change > max_accel_jerk * dt_) {
    comfort_acc = current_acc + max_accel_jerk * dt_;
  } else if (acc_change < 0 && acc_change < -decel_jerk * dt_) {
    comfort_acc = current_acc - decel_jerk * dt_;
  }

  comfort_acc = std::max(std::min(a, comfort_acc), -b_hard);

  return comfort_acc;
}

void ComfortTarget::AddComfortTargetDataToProto() {
#ifdef ENABLE_PROTO_LOG
  auto& debug_info_pb = DebugInfoManager::GetInstance().GetDebugInfoPb();
  auto mutable_comfort_target_data =
      debug_info_pb->mutable_lon_target_s_ref()->mutable_comfort_target();

  if (!target_values_.empty()) {
    for (int32_t i = 0; i < plan_points_num_; i++) {
      const auto& value = target_values_[i];
      auto* ptr = comfort_target_pb_.add_comfort_target_s_ref();
      ptr->set_s(value.s_target_val());
      ptr->set_v(value.v_target_val());
      ptr->set_t(value.relative_t());
      ptr->set_a(acc_values_[i]);
      ptr->set_target_type(static_cast<int32_t>(value.target_type()));
    }
  }

  for (const auto& upper_bound : upper_bound_infos_) {
    auto* ptr = comfort_target_pb_.add_upper_bound_infos();
    ptr->set_s(upper_bound.s);
    ptr->set_t(upper_bound.t);
    ptr->set_v(upper_bound.v);
    ptr->set_a(upper_bound.a);
    ptr->set_agent_id(upper_bound.agent_id);
    ptr->set_st_boundary_id(upper_bound.st_boundary_id);
  }

  mutable_comfort_target_data->CopyFrom(comfort_target_pb_);
#endif
}

}  // namespace planning