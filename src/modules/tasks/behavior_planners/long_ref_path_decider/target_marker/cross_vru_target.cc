#include "cross_vru_target.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "behavior_planners/long_ref_path_decider/long_ref_path_decider_output.h"
#include "behavior_planners/long_ref_path_decider/target_marker/target.h"
#include "behavior_planners/speed_limit_decider/speed_limit_decider_output.h"
#include "common/config/basic_type.h"
#include "common/st_graph/st_graph_utils.h"
#include "debug_info_log.h"
#include "ego_planning_config.h"
#include "environmental_model.h"
#include "math/linear_interpolation.h"
#include "modules/context/vehicle_config_context.h"
#include "planning_context.h"

namespace planning {

CrossVRUTarget::CrossVRUTarget(const SpeedPlannerConfig& config,
                               framework::Session* session)
    : Target(config, session) {
  cross_vru_target_pb_.Clear();

  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  const double cruise_speed = ego_state_manager->ego_v_cruise();

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

  params_.v0 = desired_speed;
  params_.s0 = 4.0;
  params_.T = 1.2;
  params_.a = 1.5;
  params_.b = 1.0;
  params_.b_max = 2.0;
  params_.delta = 4.0;
  params_.b_hard = 4.0;
  params_.max_a_jerk = 5.0;
  params_.max_b_jerk = 4.0;
  params_.default_front_s = 200;
  params_.cool_factor = 0.99;
  params_.over_speed_factor = 0.3;
  params_.end_time_buffer = 1.0;

  AnalyzeCrossVRUAgentsAndInitialize();

  JSON_DEBUG_VECTOR("cross_vru_agent_ids", cross_vru_agent_ids_, 0);

  GenerateCrossVRUTarget();

  auto mutable_lon_ref_path_decider_output =
      session_->mutable_planning_context()
          ->mutable_lon_ref_path_decider_output();

  mutable_lon_ref_path_decider_output->is_cross_vru_target_pre_handle =
      is_pre_handle_cross_vru_;

  AddCrossVRUTargetDataToProto();
}

void CrossVRUTarget::AnalyzeCrossVRUAgentsAndInitialize() {
  agent_infos_.clear();
  cross_vru_agent_ids_.clear();
  is_pre_handle_cross_vru_ = false;

  target_values_ = std::vector<TargetValue>(
      plan_points_num_, TargetValue(0.0, false, 0.0, 0.0, TargetType::kNotSet));

  const auto* st_graph = session_->planning_context().st_graph_helper();
  if (st_graph == nullptr) return;

  const auto& virtual_lane_mgr =
      session_->environmental_model().get_virtual_lane_manager();
  const auto& ego_lane = virtual_lane_mgr->get_current_lane();
  if (ego_lane == nullptr) {
    return;
  }
  const auto& ego_reference_path = ego_lane->get_reference_path();
  if (ego_reference_path == nullptr) {
    return;
  }
  const auto& ego_lane_coord = ego_reference_path->get_frenet_coord();
  if (ego_lane_coord == nullptr) {
    return;
  }

  const auto& ego_state_mgr =
      session_->environmental_model().get_ego_state_manager();
  const auto& planning_init_point = ego_state_mgr->planning_init_point();

  double ego_s = 0.0;
  double ego_l = 0.0;
  if (!ego_lane_coord->XYToSL(planning_init_point.x, planning_init_point.y,
                              &ego_s, &ego_l)) {
    return;
  }

  const auto& ego_vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double half_ego_width = ego_vehicle_param.width * 0.5;
  const double front_edge_to_rear_axle =
      ego_vehicle_param.front_edge_to_rear_axle;
  const double rear_edge_to_rear_axle =
      ego_vehicle_param.rear_edge_to_rear_axle;

  const auto& relieve_jerk_agent_ids = st_graph->relieve_jerk_agent_ids();

  std::vector<const agent::Agent*> valid_agents;
  for (const auto& agent_id : relieve_jerk_agent_ids) {
    const auto* agent =
        session_->environmental_model().get_agent_manager()->GetAgent(agent_id);
    if (agent == nullptr || agent->is_vru_crossing_virtual_obs()) continue;
    valid_agents.push_back(agent);
  }

  if (valid_agents.empty()) return;

  cross_vru_agent_ids_.reserve(valid_agents.size());

  for (const auto* agent : valid_agents) {
    cross_vru_agent_ids_.emplace_back(static_cast<double>(agent->agent_id()));

    CrossVRUAgentInfo info;
    info.agent_id = agent->agent_id();

    const auto& agent_headway_decider_output =
        session_->planning_context().agent_headway_decider_output();
    const auto& agents_headway_map =
        agent_headway_decider_output.agents_headway_Info();

    auto iter = agents_headway_map.find(agent->agent_id());
    if (iter != agents_headway_map.end()) {
      info.headway_time = iter->second.current_headway;
    } else {
      info.headway_time = params_.T;
    }

    const auto& agent_trajectories = agent->trajectories_used_by_st_graph();
    if (agent_trajectories.empty()) continue;

    const auto* st_graph_helper =
        session_->planning_context().st_graph_helper();
    const auto& agent_st_boundary_id_map =
        st_graph_helper->GetAgentIdSTBoundariesMap();
    if (agent_st_boundary_id_map.find(agent->agent_id()) ==
        agent_st_boundary_id_map.end())
      continue;

    const auto& st_boundary_id = agent_st_boundary_id_map.at(agent->agent_id());
    speed::STBoundary st_boundary;
    if (!st_graph_helper->GetStBoundary(st_boundary_id.front(), &st_boundary))
      continue;

    info.crossing_start_time = st_boundary.min_t();
    info.crossing_end_time = st_boundary.max_t();

    info.agent_traj_v.clear();
    info.agent_traj_v.reserve(plan_points_num_);

    info.agent_traj_s.clear();
    info.agent_traj_s.reserve(plan_points_num_);

    const auto& trajectory = agent_trajectories.front();
    double t = 0.0;
    for (size_t i = 0; i < plan_points_num_; ++i) {
      const auto& traj_point = trajectory[i];
      t = i * dt_;
      double center_s = 0.0;
      double center_l = 0.0;
      if (!ego_lane_coord->XYToSL(traj_point.x(), traj_point.y(), &center_s,
                                  &center_l)) {
        return;
      }
      auto matched_point = ego_lane_coord->GetPathPointByS(center_s);
      double heading_diff = traj_point.theta() - matched_point.theta();
      double agent_s =
          center_s - ego_s - front_edge_to_rear_axle - agent->length() * 0.5;
      double agent_speed = traj_point.vel() * std::cos(heading_diff);
      if(t <= info.crossing_start_time) {
        info.agent_traj_s.push_back(st_boundary.min_s());
        info.agent_traj_v.push_back(agent_speed);
      } else if (t > info.crossing_start_time && t <= info.crossing_end_time + params_.end_time_buffer) {
        info.agent_traj_s.push_back(agent_s);
        info.agent_traj_v.push_back(agent_speed);
      } else {
        info.agent_traj_s.push_back(params_.default_front_s);
        info.agent_traj_v.push_back(params_.v0);
      }
    }

    agent_infos_.emplace_back(info);
  }

  is_pre_handle_cross_vru_ = true;
}

void CrossVRUTarget::GenerateCrossVRUTarget() {
  if (agent_infos_.empty()) return;

  double current_s = init_lon_state_[0];
  double current_v = init_lon_state_[1];
  double current_a = init_lon_state_[2];

  target_values_[0].set_relative_t(0.0);
  target_values_[0].set_has_target(true);
  target_values_[0].set_s_target_val(current_s);
  target_values_[0].set_v_target_val(current_v);
  target_values_[0].set_target_type(TargetType::kCrossVRU);

  for (int32_t i = 1; i < plan_points_num_; i++) {
    auto& target_value = target_values_[i];
    target_value.set_relative_t(i * dt_);

    double cross_vru_acc = CalculateVRUDeceleration(current_v, current_s,
                                                 current_a, i, agent_infos_);

    double next_s =
        current_s + current_v * dt_ + 0.5 * cross_vru_acc * dt_ * dt_;
    double next_v = std::max(0.0, current_v + cross_vru_acc * dt_);
    next_s = std::max(0.0, std::max(current_s, next_s));

    target_value.set_has_target(true);
    target_value.set_s_target_val(next_s);
    target_value.set_v_target_val(next_v);
    target_value.set_target_type(TargetType::kCrossVRU);

    current_s = next_s;
    current_v = next_v;
    current_a = cross_vru_acc;
  }
}

double CrossVRUTarget::CalcDesiredVelocity(const double d_rel,
                                           const double d_des,
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

double CrossVRUTarget::CalculateVRUDecelerationCore(
    const double current_vel, const double current_s, const double front_s,
    const double front_vel, const double headway_time) const {
  double v0 = params_.v0;
  double a = params_.a;
  double b = params_.b;
  double b_max = params_.b_max;
  double b_hard = params_.b_hard;
  double delta = params_.delta;
  double s0 = params_.s0;
  double cool_factor = params_.cool_factor;
  double over_speed_factor = params_.over_speed_factor;

  double s_alpha = std::max(1e-3, front_s - current_s);
  double delta_v = current_vel - front_vel;

  double s_star = s0 + std::max(0.0, current_vel * headway_time +
                                         (current_vel * delta_v) /
                                             (2.0 * std::sqrt(a * b_max)));

  double s_safe = s0 + current_vel * headway_time;

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

  return final_acc;
}

double CrossVRUTarget::CalculateVRUDeceleration(
    const double current_vel, const double current_s, const double current_acc,
    const int32_t index,
    const std::vector<CrossVRUAgentInfo>& agent_infos) const {
  double target_acc = 0.0;
  std::vector<double> candidate_accelerations;

  for (const auto& agent_info : agent_infos) {
    double front_s = agent_info.agent_traj_s[index];
    double front_vel = agent_info.agent_traj_v[index];

    double agent_target_acc = CalculateVRUDecelerationCore(
        current_vel, current_s, front_s, front_vel, agent_info.headway_time);

    double acc_change = agent_target_acc - current_acc;
    if (acc_change > 0 && acc_change > params_.max_a_jerk * dt_) {
      agent_target_acc = current_acc + params_.max_a_jerk * dt_;
    } else if (acc_change < 0 && acc_change < -params_.max_b_jerk * dt_) {
      agent_target_acc = current_acc - params_.max_b_jerk * dt_;
    }

    agent_target_acc =
        std::max(std::min(params_.a, agent_target_acc), -params_.b_hard);
    candidate_accelerations.push_back(agent_target_acc);
  }

  if (!candidate_accelerations.empty()) {
    target_acc = *std::min_element(candidate_accelerations.begin(),
                                   candidate_accelerations.end());
  }

  return target_acc;
}

void CrossVRUTarget::AddCrossVRUTargetDataToProto() {
#ifdef ENABLE_PROTO_LOG
  auto& debug_info_pb = DebugInfoManager::GetInstance().GetDebugInfoPb();
  auto mutable_cross_vru_target_data =
      debug_info_pb->mutable_lon_target_s_ref()->mutable_cross_vru_target();

  if (!target_values_.empty()) {
    for (const auto& value : target_values_) {
      auto* ptr = cross_vru_target_pb_.add_cross_vru_target_s_ref();
      ptr->set_s(value.s_target_val());
      ptr->set_t(value.relative_t());
      ptr->set_target_type(static_cast<int32_t>(value.target_type()));
    }
  }
  mutable_cross_vru_target_data->CopyFrom(cross_vru_target_pb_);
#endif
}

}  // namespace planning