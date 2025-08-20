#include "cross_vru_target.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>

#include "behavior_planners/long_ref_path_decider/bound_maker/bound_maker.h"
#include "behavior_planners/long_ref_path_decider/long_ref_path_decider_output.h"
#include "behavior_planners/long_ref_path_decider/target_marker/target.h"
#include "behavior_planners/speed_limit_decider/speed_limit_decider_output.h"
#include "common/config/basic_type.h"
#include "common/st_graph/st_graph_utils.h"
#include "debug_info_log.h"
#include "ego_planning_config.h"
#include "environmental_model.h"
#include "math/linear_interpolation.h"
#include "math/math_utils.h"
#include "modules/context/vehicle_config_context.h"
#include "planning_context.h"
#include "utils/pose2d_utils.h"

namespace planning {

namespace {
constexpr double kOvertakeAccelValue = 3.0;
constexpr double kSafetyDistance = 3.5;
}  // namespace

CrossVRUTarget::CrossVRUTarget(const SpeedPlannerConfig& config,
                               framework::Session* session)
    : Target(config, session) {
  cross_vru_target_pb_.Clear();

  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  params_.v0 = ego_state_manager->ego_v_cruise();
  params_.s0 = 3.5;
  params_.T = 0.5;
  params_.a = 1.5;
  params_.b = 2.0;
  params_.delta = 4.0;
  params_.b_hard = 4.0;
  params_.max_a_jerk = 4.0;
  params_.max_b_jerk = 1.5;
  params_.default_front_s = 200;
  params_.cool_factor = 0.99;

  AnalyzeCrossVRUAgentsAndInitialize();

  JSON_DEBUG_VECTOR("cross_vru_agent_ids", cross_vru_agent_ids_, 0);

  GenerateCrossVRUTarget();

  AddCrossVRUTargetDataToProto();
}

void CrossVRUTarget::AnalyzeCrossVRUAgentsAndInitialize() {
  agent_infos_.clear();
  cross_vru_agent_ids_.clear();

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

    bool is_need_overtake = false;
    MakeYieldOrOvertakeDecision(agent, st_boundary.min_t(), ego_lane_coord,
                                rear_edge_to_rear_axle, planning_init_point,
                                &is_need_overtake);

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
      double agent_speed = traj_point.vel() * std::cos(heading_diff);
      double overtake_speed = planning_init_point.v + kOvertakeAccelValue;
      if (!is_need_overtake) {
        if (t <= info.crossing_start_time) {
          info.agent_traj_s.push_back(st_boundary.min_s());
          info.agent_traj_v.push_back(agent_speed);
        } else if (t <= info.crossing_end_time) {
          int32_t index =
              static_cast<int32_t>((t - info.crossing_start_time) / dt_);
          info.agent_traj_s.push_back(st_boundary.lower_points()[index].s());
          info.agent_traj_v.push_back(agent_speed);
        } else {
          info.agent_traj_s.push_back(params_.default_front_s);
          info.agent_traj_v.push_back(params_.v0);
        }
      } else {
        const double safety_dis = ego_vehicle_param.length + kSafetyDistance;
        if (t <= info.crossing_start_time) {
          info.agent_traj_s.push_back(st_boundary.max_s() + safety_dis);
          info.agent_traj_v.push_back(overtake_speed);
        } else if (t <= info.crossing_end_time) {
          int32_t index =
              static_cast<int32_t>((t - info.crossing_start_time) / dt_);
          info.agent_traj_s.push_back(st_boundary.upper_points()[index].s());
          info.agent_traj_v.push_back(overtake_speed);
        } else {
          info.agent_traj_s.push_back(params_.default_front_s);
          info.agent_traj_v.push_back(params_.v0);
        }
      }
    }

    agent_infos_.emplace_back(info);
  }
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

    double target_acc = CalculateVRUDeceleration(current_v, current_s,
                                                 current_a, i, agent_infos_);

    current_a = target_acc;
    current_v = std::max(0.0, current_v + target_acc * dt_);
    current_s = std::max(
        current_s, current_s + current_v * dt_ + 0.5 * target_acc * dt_ * dt_);

    target_value.set_has_target(true);
    target_value.set_s_target_val(current_s);
    target_value.set_v_target_val(current_v);
    target_value.set_target_type(TargetType::kCrossVRU);
  }
}

double CrossVRUTarget::CalculateVRUDecelerationCore(
    const double current_vel, const double current_s, const double front_s,
    const double front_vel, const double headway_time) const {
  double v0 = params_.v0;
  double a = params_.a;
  double b = params_.b;
  double b_hard = params_.b_hard;
  double delta = params_.delta;
  double s0 = params_.s0;
  double cool_factor = params_.cool_factor;

  double s_alpha = std::max(1e-3, front_s - current_s);
  double delta_v = current_vel - front_vel;

  double s_star = s0 + current_vel * headway_time +
                  (current_vel * delta_v) / (2.0 * std::sqrt(a * b));

  double s_safe = s0 + current_vel * headway_time;

  double a_free;
  if (current_vel <= v0) {
    a_free = a * (1.0 - std::pow(current_vel / v0, delta));
  } else {
    a_free = -b * (1.0 - std::pow(v0 / current_vel, a * delta / b));
  }

  double z = s_star / s_alpha;

  double a_idm;
  if (current_vel <= v0) {
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
  if (ds_safe > 0.0 && ds_star < 0.0 || ds_safe < 0.0 && ds_star < 0.0) {
    a_cah = b_hard * ds_star / s_star;
  } else if (ds_safe > 0.0 && ds_star < 0.0) {
    a_cah = b * ds_star / s_star;
  } else {
    a_cah = a_free;
  }

  double final_acc;
  if (a_idm >= a_cah) {
    final_acc = a_idm;
  } else {
    final_acc = (1.0 - cool_factor) * a_idm +
                cool_factor * (a_cah - b * tanh((a_idm - a_cah) / (-b)));
  }

  final_acc = std::max(std::min(a, final_acc), -params_.b_hard);

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

void CrossVRUTarget::MakeYieldOrOvertakeDecision(
    const agent::Agent* agent, const double interaction_t,
    const std::shared_ptr<planning_math::KDPath>& ego_lane_coord,
    const double rear_edge_to_rear_axle,
    const PlanningInitPoint& planning_init_point, bool* is_overtake) {
  if (agent->is_reverse_relieve_agent()) {
    *is_overtake = false;
  }
  double ego_s = 0.0, ego_l = 0.0;
  if (!ego_lane_coord->XYToSL(planning_init_point.x, planning_init_point.y,
                              &ego_s, &ego_l)) {
    return;
  }
  const auto& agent_corners = agent->box().GetAllCorners();
  double obs_min_l = std::numeric_limits<double>::max(),
         obs_max_l = -std::numeric_limits<double>::max(),
         obs_min_s = std::numeric_limits<double>::max(),
         obs_max_s = -std::numeric_limits<double>::max();
  for (const auto& agent_corner : agent_corners) {
    double agent_corner_s = 0.0, agent_corner_l = 0.0;
    if (ego_lane_coord->XYToSL(agent_corner.x(), agent_corner.y(),
                               &agent_corner_s, &agent_corner_l)) {
      obs_min_l = std::min(obs_min_l, agent_corner_l);
      obs_max_l = std::max(obs_max_l, agent_corner_l);
      obs_min_s = std::min(obs_min_s, agent_corner_s);
      obs_max_s = std::max(obs_max_s, agent_corner_s);
    }
  }

  std::vector<double> overtake_traj;
  double v_target = planning_init_point.v + kOvertakeAccelValue;
  const auto ego_overtake_traj =
      GenerateOvertakeTrajByJLT(v_target, planning_init_point);
  const double interaction_s = ego_s +
                               ego_overtake_traj.Evaluate(0, interaction_t) -
                               rear_edge_to_rear_axle;
  if (interaction_s > obs_max_s + kSafetyDistance) {
    *is_overtake = true;
  }
}

SecondOrderTimeOptimalTrajectory CrossVRUTarget::GenerateOvertakeTrajByJLT(
    const double v_target, const PlanningInitPoint& planning_init_point) {
  LonState init_state;
  init_state.p = 0.0;
  init_state.v = planning_init_point.v;
  init_state.a = planning_init_point.a;

  StateLimit state_limit;
  state_limit.v_end = v_target;
  state_limit.a_min = -2.0;
  state_limit.a_max = 2.0;
  state_limit.j_min = -2.0;
  state_limit.j_max = 2.0;

  return SecondOrderTimeOptimalTrajectory(init_state, state_limit);
}

void CrossVRUTarget::AddCrossVRUTargetDataToProto() {
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

  auto mutable_lon_ref_path_decider_output =
      session_->mutable_planning_context()
          ->mutable_lon_ref_path_decider_output();
  if (mutable_lon_ref_path_decider_output) {
    mutable_lon_ref_path_decider_output->vru_agent_infos.clear();
    for (const auto& agent_info : agent_infos_) {
      VRUAgentInfo vru_info;
      vru_info.agent_id = agent_info.agent_id;
      mutable_lon_ref_path_decider_output->vru_agent_infos.push_back(vru_info);
    }
  }

  mutable_cross_vru_target_data->CopyFrom(cross_vru_target_pb_);
}

}  // namespace planning