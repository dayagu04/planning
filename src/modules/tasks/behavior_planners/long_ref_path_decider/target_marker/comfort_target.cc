#include "comfort_target.h"

#include <algorithm>
#include <cmath>

#include "behavior_planners/long_ref_path_decider/long_ref_path_decider.h"
#include "behavior_planners/long_ref_path_decider/long_ref_path_decider_output.h"
#include "behavior_planners/speed_limit_decider/speed_limit_decider_output.h"
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
  comfort_params_.max_accel_jerk = 5.0;
  comfort_params_.min_decel_jerk = 1.0;
  comfort_params_.max_decel_jerk = 1.5;
  comfort_params_.emergency_decel_jerk = 5.0;
  comfort_params_.virtual_front_s = 200.0;
  comfort_params_.cool_factor = 0.99;
  comfort_params_.delay_time_buffer = 0.3;
  comfort_params_.eps = 1e-3;
  comfort_params_.static_speed_threshold = 0.2;
  comfort_params_.emergency_ttc_threshold = 1.5;
  comfort_params_.confluence_headway = 10.0;
  comfort_params_.cipv_decel_threshold = -3.0;
  comfort_params_.min_speed_diff_for_emergency = 0.1;
  comfort_params_.virtual_front_agent_id = 799999;
  comfort_params_.default_follow_agent_id = 899999;
  comfort_params_.default_follow_st_boundary_id = 899999;
  comfort_params_.default_follow_agent_s = 210.0;
  comfort_params_.kinematic_half_coefficient = 0.5;
  comfort_params_.agent_length_half_coefficient = 0.5;

  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  const double cruise_speed = ego_state_manager->ego_v_cruise();
  JSON_DEBUG_VALUE("cruise_speed", cruise_speed);

  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto lane_change_state = lane_change_decider_output.curr_state;
  const bool is_in_lane_change_execution =
      lane_change_state == StateMachineLaneChangeStatus::kLaneChangeExecution ||
      lane_change_state == StateMachineLaneChangeStatus::kLaneChangeComplete ||
      lane_change_state == StateMachineLaneChangeStatus::kLaneChangeHold;
  JSON_DEBUG_VALUE("lane_change_state", static_cast<int>(lane_change_state));

  const auto& speed_limit_decider_output =
      session_->planning_context().speed_limit_decider_output();
  double speed_limit_normal = cruise_speed;
  if (is_in_lane_change_execution) {
    speed_limit_normal = std::fmin(
        speed_limit_normal, config_.lane_change_upper_speed_limit_kph / 3.6);
  }

  double speed_limit_ref = std::numeric_limits<double>::max();
  auto speed_limit_type_ref = SpeedLimitType::NONE;
  speed_limit_decider_output.GetSpeedLimit(&speed_limit_ref,
                                           &speed_limit_type_ref);
  comfort_params_.v0 = std::fmin(speed_limit_normal, speed_limit_ref);
  JSON_DEBUG_VALUE("limit_speed", comfort_params_.v0);

  const auto& route_info = session_->environmental_model().get_route_info();
  const auto& ego_lane_road_right_output =
      session_->planning_context().ego_lane_road_right_decider_output();
  is_confluence_area_ = ego_lane_road_right_output.is_merge_region ||
                        ego_lane_road_right_output.is_split_region;

  if (!is_confluence_area_ && route_info) {
    const auto& ego_state =
        session_->environmental_model().get_ego_state_manager();
    const double dis_threshold =
        ego_state->ego_v() * comfort_params_.confluence_headway;
    const auto& route_info_output = route_info->get_route_info_output();
    const auto& mlc_info = route_info_output.mlc_decider_scene_type_info;
    is_confluence_area_ =
        ((mlc_info.mlc_scene_type == SPLIT_SCENE ||
          mlc_info.mlc_scene_type == MERGE_SCENE) &&
         mlc_info.dis_to_link_topo_change_point < dis_threshold);
  }
  JSON_DEBUG_VALUE("is_confluence_area", is_confluence_area_);

  if (session_->is_rads_scene()) {
    const auto& cipv_info = session_->planning_context().cipv_decider_output();
    const auto& stop_destination_decider_output =
        session_->planning_context().stop_destination_decider_output();
    const auto agent_manager =
        session_->environmental_model().get_agent_manager();
    int32_t cipv_id = cipv_info.cipv_id();

    if (cipv_id ==
        stop_destination_decider_output.stop_destination_virtual_agent_id()) {
      comfort_params_.s0 = 0.0;
    } else if (auto agent = agent_manager->GetAgent(cipv_id)) {
      comfort_params_.s0 = agent->is_static()
                               ? config_.rads_comfort_param_static_s0
                               : config_.rads_comfort_param_dynamic_s0;
    }
  }

  upper_bound_infos_.resize(plan_points_num_);
  acc_values_.resize(plan_points_num_, 0.0);

  GenerateUpperBoundInfo();

  JSON_DEBUG_VECTOR("upper_bound_agent_ids",
                    std::vector<double>(upper_bound_agent_ids_.begin(),
                                        upper_bound_agent_ids_.end()),
                    0)
  JSON_DEBUG_VECTOR(
      "comfort_follow_agent_ids",
      std::vector<double>(follow_agent_ids_.begin(), follow_agent_ids_.end()),
      0);
  JSON_DEBUG_VECTOR("joint_danger_agent_ids",
                    std::vector<double>(joint_danger_agent_ids_.begin(),
                                        joint_danger_agent_ids_.end()),
                    0);
  JSON_DEBUG_VECTOR("rule_base_cutin_agent_ids",
                    std::vector<double>(rule_base_cutin_agent_ids_.begin(),
                                        rule_base_cutin_agent_ids_.end()),
                    0);

  GenerateComfortTarget();

  comfort_jerk_min_vec_[0] = comfort_jerk_min_vec_[1];
  JSON_DEBUG_VECTOR("comfort_jerk_min_vec", comfort_jerk_min_vec_, 1);
  comfort_v_target_vec_[0] = comfort_v_target_vec_[1];
  JSON_DEBUG_VECTOR("comfort_v_target_vec", comfort_v_target_vec_, 1);

  auto mutable_lon_ref_path_decider_output =
      session_->mutable_planning_context()
          ->mutable_lon_ref_path_decider_output();
  mutable_lon_ref_path_decider_output->is_lat_follow = is_lat_follow_;
  mutable_lon_ref_path_decider_output->is_lon_cutin = is_lon_cut_in_;
  mutable_lon_ref_path_decider_output->is_joint_danger = is_joint_danger_;
  mutable_lon_ref_path_decider_output->is_lon_cipv_emergency_stop =
      is_lon_cipv_emergency_stop_;
  mutable_lon_ref_path_decider_output->is_joint_danger_emergency_stop =
      is_joint_danger_emergency_stop_;
  JSON_DEBUG_VALUE("lon_cipv_emergency_stop", is_lon_cipv_emergency_stop_);
  JSON_DEBUG_VALUE("joint_danger_emergency_stop",
                   is_joint_danger_emergency_stop_);

  mutable_lon_ref_path_decider_output->parallel_overtake_agent_id =
      parallel_overtake_agent_id_;
  mutable_lon_ref_path_decider_output->st_overtake_agent_ids =
      lower_bound_agent_ids_;

  auto& upper_bound_infos_out =
      mutable_lon_ref_path_decider_output->comfort_target_upper_bound_infos;
  upper_bound_infos_out.clear();
  upper_bound_infos_out.reserve(upper_bound_infos_.size());
  for (const auto& info : upper_bound_infos_) {
    upper_bound_infos_out.push_back(
        {info.s, info.t, info.v, info.a, info.agent_id, info.st_boundary_id});
  }

  auto& comfort_target_out =
      mutable_lon_ref_path_decider_output->comfort_target;
  comfort_target_out.clear();
  comfort_target_out.reserve(target_values_.size());
  for (const auto& target_val : target_values_) {
    comfort_target_out.push_back(
        {target_val.s_target_val(), target_val.v_target_val()});
  }

  AddComfortTargetDataToProto();
}

bool ComfortTarget::CheckEmergencyCondition(double ego_v, double obs_s,
                                            double obs_v, double obs_a) const {
  double delta_v = ego_v - obs_v;
  if (delta_v < comfort_params_.min_speed_diff_for_emergency) {
    return false;
  }

  double distance_threshold = comfort_params_.s0 + comfort_params_.T * ego_v;
  double ttc = obs_s / delta_v;

  return (ttc < comfort_params_.emergency_ttc_threshold ||
          obs_a < comfort_params_.cipv_decel_threshold) &&
         obs_s < distance_threshold;
}

bool ComfortTarget::CheckCipvEmergencyBraking(double ego_v) {
  const auto& cipv_info = session_->planning_context().cipv_decider_output();

  if (cipv_info.cipv_id() == speed::kNoAgentId) {
    return false;
  }

  const auto agent_manager =
      session_->environmental_model().get_agent_manager();
  const auto* cipv_agent = agent_manager->GetAgent(cipv_info.cipv_id());
  if (cipv_agent && cipv_agent->is_reverse()) {
    return false;
  }

  return CheckEmergencyCondition(ego_v, cipv_info.relative_s(),
                                 cipv_info.v_fusion_frenet(),
                                 cipv_info.acceleration_fusion());
}

bool ComfortTarget::CheckJointDangerEmergencyBraking(double ego_v,
                                                     int32_t agent_id) {
  if (agent_id == speed::kNoAgentId ||
      agent_id == comfort_params_.virtual_front_agent_id) {
    return false;
  }

  bool is_agent_in_joint_and_upper =
      std::find(joint_danger_agent_ids_.begin(), joint_danger_agent_ids_.end(),
                agent_id) != joint_danger_agent_ids_.end() &&
      upper_bound_agent_ids_.find(agent_id) != upper_bound_agent_ids_.end();

  if (!is_agent_in_joint_and_upper || upper_bound_infos_.empty() ||
      upper_bound_infos_[0].s <= comfort_params_.eps) {
    return false;
  }

  return CheckEmergencyCondition(ego_v, upper_bound_infos_[0].s,
                                 upper_bound_infos_[0].v,
                                 upper_bound_infos_[0].a);
}

void ComfortTarget::GenerateUpperBoundInfo() {
  const auto* st_graph = session_->planning_context().st_graph_helper();
  const double virtual_front_s = comfort_params_.virtual_front_s;
  const double virtual_front_vel = comfort_params_.v0;
  const int32_t virtual_front_agent_id = comfort_params_.virtual_front_agent_id;
  const int32_t virtual_front_st_boundary_id =
      comfort_params_.virtual_front_agent_id;
  const double virtual_front_acc = 0.0;

  const auto& ego_vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double front_edge_to_rear_axle =
      ego_vehicle_param.front_edge_to_rear_axle;

  const auto& ego_lane = session_->environmental_model()
                             .get_virtual_lane_manager()
                             ->get_current_lane();
  if (!ego_lane || !ego_lane->get_lane_frenet_coord()) return;

  const auto& ego_lane_coord = ego_lane->get_lane_frenet_coord();
  double ego_s = 0.0, ego_l = 0.0;
  double ego_v = init_lon_state_[1];
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
  std::unordered_set<int32_t> forbidden_ids(
      lane_borrow_output.blocked_obs_id.begin(),
      lane_borrow_output.blocked_obs_id.end());

  const auto& parallel_longitudinal_avoid_output =
      session_->planning_context().parallel_longitudinal_avoid_decider_output();
  const bool is_parallel_overtake =
      parallel_longitudinal_avoid_output
          .is_need_parallel_longitudinal_avoid() &&
      parallel_longitudinal_avoid_output.is_parallel_overtake();
  parallel_overtake_agent_id_ =
      is_parallel_overtake
          ? parallel_longitudinal_avoid_output.parallel_target_agent_id()
          : -1;

  if (st_graph) {
    for (size_t i = 0; i < plan_points_num_; i++) {
      const auto& upper_bound = st_graph->GetPassCorridorUpperBound(i * dt_);
      if (upper_bound.agent_id() != speed::kNoAgentId) {
        upper_bound_agent_ids_.insert(upper_bound.agent_id());
      }

      const auto& lower_bound = st_graph->GetPassCorridorLowerBound(i * dt_);
      if (lower_bound.agent_id() != speed::kNoAgentId) {
        lower_bound_agent_ids_.push_back(lower_bound.agent_id());
      }
    }
  }

  std::vector<FollowAgentWithSource> follow_agents;
  std::unordered_set<int32_t> added_agent_ids;
  const auto agent_manager =
      session_->environmental_model().get_agent_manager();

  const auto& agent_longitudinal_decider_output =
      session_->planning_context().agent_longitudinal_decider_output();
  const auto& cutin_ids = agent_longitudinal_decider_output.cutin_agent_ids;
  rule_base_cutin_agent_ids_.assign(cutin_ids.begin(), cutin_ids.end());

  const auto& lat_lon_joint_planner_output =
      session_->planning_context().lat_lon_joint_planner_decider_output();
  const auto& danger_ids = lat_lon_joint_planner_output.GetDangerObstacleIds();
  joint_danger_agent_ids_.assign(danger_ids.begin(), danger_ids.end());

  ProcessCutinAgents(danger_ids, FollowAgentSource::kJointDangerAgentIds,
                     forbidden_ids, added_agent_ids, follow_agents);
  ProcessCutinAgents(cutin_ids, FollowAgentSource::kLonCutinAgentIds,
                     forbidden_ids, added_agent_ids, follow_agents);

  for (const auto& [agent_id, decision] : lat_obstacle_decision) {
    if (decision == LatObstacleDecisionType::FOLLOW &&
        forbidden_ids.find(agent_id) == forbidden_ids.end() &&
        added_agent_ids.find(agent_id) == added_agent_ids.end() &&
        agent_id != parallel_overtake_agent_id_) {
      if (auto agent = agent_manager->GetAgent(agent_id)) {
        follow_agents.push_back(
            {agent, FollowAgentSource::kLatObstacleDecision});
        added_agent_ids.insert(agent_id);
      }
    }
  }

  std::vector<FollowAgentInfo> follow_agent_infos(plan_points_num_);
  std::unordered_set<int32_t> valid_agent_ids;
  const auto& spatio_temporal_follow_info =
      session_->planning_context()
          .lateral_obstacle_decider_output()
          .spatio_temporal_follow_obstacle_info;

  if (!follow_agents.empty()) {
    for (size_t i = 0; i < plan_points_num_; i++) {
      const double t = i * dt_;
      double min_agent_s = std::numeric_limits<double>::max();
      FollowAgentInfo best_agent_info = {
          comfort_params_.default_follow_agent_id,
          comfort_params_.default_follow_agent_s,
          comfort_params_.v0,
          0.0,
          comfort_params_.default_follow_st_boundary_id,
          FollowAgentSource::kLatObstacleDecision};
      bool found_valid_agent = false;

      for (const auto& agent_with_source : follow_agents) {
        const auto& agent = agent_with_source.agent;
        const int32_t agent_id = agent->agent_id();

        if (agent_with_source.source ==
            FollowAgentSource::kLatObstacleDecision) {
          auto it =
              spatio_temporal_follow_info.find(static_cast<uint32_t>(agent_id));
          if (it == spatio_temporal_follow_info.end()) continue;
          const auto* follow_window = it->second.QueryByTime(t);
          if (!follow_window || follow_window->lateral_decision !=
                                    LatObstacleDecisionType::FOLLOW) {
            continue;
          }
        }

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
        double agent_acc = agent->accel_fusion();
        double agent_s =
            std::max(comfort_params_.eps,
                     center_s - ego_s - front_edge_to_rear_axle -
                         agent->length() *
                             comfort_params_.agent_length_half_coefficient);

        if (agent_s < min_agent_s) {
          min_agent_s = agent_s;
          best_agent_info = {agent->agent_id(),
                             agent_s,
                             agent_speed,
                             agent_acc,
                             comfort_params_.default_follow_st_boundary_id,
                             agent_with_source.source};
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
    upper_bound_infos_[i] = {virtual_front_s,
                             virtual_front_vel,
                             virtual_front_acc,
                             t,
                             TargetType::kComfort,
                             virtual_front_agent_id,
                             virtual_front_st_boundary_id,
                             false,
                             false,
                             false};

    if (st_graph) {
      const auto& upper_bound = st_graph->GetPassCorridorUpperBound(t);
      if (upper_bound.agent_id() != speed::kNoAgentId) {
        auto agent = agent_manager->GetAgent(upper_bound.agent_id());
        if (agent) {
          double upper_bound_v = upper_bound.velocity();
          double upper_bound_a = agent->is_static()
                                     ? agent->accel_fusion()
                                     : upper_bound.acceleration();
          double confidence =
              LongRefPathDecider::CalcUpperBoundConfidence(upper_bound.s());
          upper_bound_infos_[i] = {confidence * upper_bound.s() +
                                       (1.0 - confidence) * virtual_front_s,
                                   confidence * upper_bound.velocity() +
                                       (1.0 - confidence) * virtual_front_vel,
                                   upper_bound_a,
                                   t,
                                   TargetType::kComfort,
                                   upper_bound.agent_id(),
                                   upper_bound.boundary_id(),
                                   false,
                                   false,
                                   false};
        }
      }

      if (!follow_agents.empty()) {
        double follow_confidence = LongRefPathDecider::CalcUpperBoundConfidence(
            follow_agent_infos[i].s);
        double effective_s = follow_confidence * follow_agent_infos[i].s +
                             (1.0 - follow_confidence) * virtual_front_s;
        double effective_v = follow_confidence * follow_agent_infos[i].v +
                             (1.0 - follow_confidence) * virtual_front_vel;

        if (effective_s <= upper_bound_infos_[i].s) {
          const auto source = follow_agent_infos[i].source;
          is_lat_follow_ |= (source == FollowAgentSource::kLatObstacleDecision);
          is_lon_cut_in_ |= (source == FollowAgentSource::kLonCutinAgentIds);
          is_joint_danger_ |=
              (source == FollowAgentSource::kJointDangerAgentIds);

          upper_bound_infos_[i] = {
              effective_s,
              effective_v,
              follow_agent_infos[i].a,
              t,
              TargetType::kComfort,
              follow_agent_infos[i].agent_id,
              follow_agent_infos[i].st_boundary_id,
              source == FollowAgentSource::kLatObstacleDecision,
              source == FollowAgentSource::kLonCutinAgentIds,
              source == FollowAgentSource::kJointDangerAgentIds};
        }
      }

      if (i == 0) {
        is_lon_cipv_emergency_stop_ = CheckCipvEmergencyBraking(ego_v);
        is_joint_danger_emergency_stop_ = CheckJointDangerEmergencyBraking(
            ego_v, upper_bound_infos_[i].agent_id);
      }
    }
  }

  follow_agent_ids_.assign(valid_agent_ids.begin(), valid_agent_ids.end());
  std::sort(follow_agent_ids_.begin(), follow_agent_ids_.end());
}

void ComfortTarget::ProcessCutinAgents(
    const std::vector<int32_t>& agent_ids, FollowAgentSource source,
    const std::unordered_set<int32_t>& forbidden_ids,
    std::unordered_set<int32_t>& added_agent_ids,
    std::vector<FollowAgentWithSource>& follow_agents) {
  const auto& ego_vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const auto& lat_obstacle_decision = session_->planning_context()
                                          .lateral_obstacle_decider_output()
                                          .lat_obstacle_decision;
  const auto& ego_lane = session_->environmental_model()
                             .get_virtual_lane_manager()
                             ->get_current_lane();
  if (!ego_lane || !ego_lane->get_lane_frenet_coord()) return;

  const auto& ego_lane_coord = ego_lane->get_lane_frenet_coord();
  double ego_s = 0.0, ego_l = 0.0;
  const auto& ego_init_point = session_->environmental_model()
                                   .get_ego_state_manager()
                                   ->planning_init_point();
  if (!ego_lane_coord->XYToSL(ego_init_point.x, ego_init_point.y, &ego_s,
                              &ego_l)) {
    return;
  }

  const auto agent_manager =
      session_->environmental_model().get_agent_manager();
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto curr_state = lane_change_decider_output.curr_state;
  int32_t gap_rear_agent_id = -1;

  if (curr_state == StateMachineLaneChangeStatus::kLaneChangeExecution ||
      curr_state == StateMachineLaneChangeStatus::kLaneChangeHold ||
      curr_state == StateMachineLaneChangeStatus::kLaneChangeComplete ||
      curr_state == StateMachineLaneChangeStatus::kLaneChangePropose) {
    const auto dynamic_world =
        session_->environmental_model().get_dynamic_world();
    if (dynamic_world) {
      const auto node_ptr = dynamic_world->GetNode(
          lane_change_decider_output.lc_gap_info.rear_node_id);
      if (node_ptr) gap_rear_agent_id = node_ptr->node_agent_id();
    }
  }

  for (const int32_t agent_id : agent_ids) {
    if (forbidden_ids.find(agent_id) != forbidden_ids.end() ||
        added_agent_ids.find(agent_id) != added_agent_ids.end() ||
        agent_id == gap_rear_agent_id ||
        (parallel_overtake_agent_id_ != -1 &&
         agent_id == parallel_overtake_agent_id_ &&
         source != FollowAgentSource::kLonCutinAgentIds)) {
      continue;
    }

    auto agent = agent_manager->GetAgent(agent_id);
    if (!agent) continue;

    double agent_s = 0.0, agent_l = 0.0;
    if (!ego_lane_coord->XYToSL(agent->x(), agent->y(), &agent_s, &agent_l) ||
        (agent_s < ego_s &&
         !(is_confluence_area_ && upper_bound_agent_ids_.find(agent_id) !=
                                      upper_bound_agent_ids_.end()))) {
      continue;
    }

    const bool is_traffic_control_obstacle =
        agent->type() == agent::AgentType::TRAFFIC_CONE ||
        agent->type() == agent::AgentType::CTASH_BARREL ||
        agent->type() == agent::AgentType::WATER_SAFETY_BARRIER;

    bool is_lateral_left_or_right = false;
    auto lat_decision_iter =
        lat_obstacle_decision.find(static_cast<uint32_t>(agent_id));
    if (lat_decision_iter != lat_obstacle_decision.end()) {
      is_lateral_left_or_right =
          (lat_decision_iter->second == LatObstacleDecisionType::LEFT ||
           lat_decision_iter->second == LatObstacleDecisionType::RIGHT);
    }

    if ((agent->is_static() ||
         std::fabs(agent->speed()) < comfort_params_.static_speed_threshold) &&
        !is_traffic_control_obstacle && is_lateral_left_or_right) {
      continue;
    }

    follow_agents.push_back({agent, source});
    added_agent_ids.insert(agent_id);
  }
}

void ComfortTarget::GenerateComfortTarget() {
  target_values_.resize(plan_points_num_,
                        TargetValue(0.0, false, 0.0, 0.0, TargetType::kNotSet));
  comfort_jerk_min_vec_.resize(plan_points_num_,
                               comfort_params_.min_decel_jerk);
  comfort_v_target_vec_.resize(plan_points_num_, comfort_params_.v0);

  double current_s = init_lon_state_[0];
  double current_v = init_lon_state_[1];
  double current_a = init_lon_state_[2];

  target_values_[0] =
      TargetValue(0.0, true, current_s, current_v, TargetType::kComfort);
  acc_values_[0] = current_a;

  const auto* st_graph = session_->planning_context().st_graph_helper();
  const auto& agents_headway_Info = session_->planning_context()
                                        .agent_headway_decider_output()
                                        .agents_headway_Info();

  for (int32_t i = 1; i < plan_points_num_; i++) {
    const double t = i * dt_;
    const double front_s = upper_bound_infos_[i - 1].s;
    const double front_vel = upper_bound_infos_[i - 1].v;
    const bool is_lat_follow = upper_bound_infos_[i - 1].is_lat_follow;
    const bool is_lon_cut_in = upper_bound_infos_[i - 1].is_lon_cut_in;
    const bool is_joint_danger = upper_bound_infos_[i - 1].is_joint_danger;

    double tau = comfort_params_.T;
    if (st_graph) {
      const auto& upper_bound = st_graph->GetPassCorridorUpperBound(t);
      if (upper_bound.agent_id() != speed::kNoAgentId) {
        auto iter = agents_headway_Info.find(upper_bound.agent_id());
        if (iter != agents_headway_Info.end()) {
          tau = iter->second.current_headway;
        }
      }
    }

    double min_follow_distance =
        comfort_params_.s0 + current_v * comfort_params_.delay_time_buffer;
    double max_follow_distance = comfort_params_.s0 + current_v * tau;
    double max_decel_jerk = (is_lon_cut_in || is_lat_follow || is_joint_danger)
                                ? comfort_params_.max_decel_jerk
                                : comfort_params_.min_decel_jerk;

    double decel_jerk = comfort_params_.min_decel_jerk;
    if (front_s <= min_follow_distance) {
      decel_jerk = max_decel_jerk;
    } else if (front_s < max_follow_distance) {
      double distance_diff = max_follow_distance - min_follow_distance;
      if (distance_diff >= comfort_params_.eps) {
        double ratio = (front_s - min_follow_distance) / distance_diff;
        ratio = std::clamp(ratio, 0.0, 1.0);
        double smooth_ratio = 3.0 * ratio * ratio - 2.0 * ratio * ratio * ratio;
        decel_jerk =
            max_decel_jerk +
            smooth_ratio * (comfort_params_.min_decel_jerk - max_decel_jerk);
      }
    }

    if (is_lon_cipv_emergency_stop_ || is_joint_danger_emergency_stop_) {
      decel_jerk = comfort_params_.emergency_decel_jerk;
    }

    comfort_jerk_min_vec_[i] = -decel_jerk;

    double comfort_acc = CalculateComfortAcceleration(
        current_a, current_v, current_s, front_vel, front_s, tau, decel_jerk,
        comfort_v_target_vec_[i]);
    acc_values_[i] = comfort_acc;

    double ds = std::max(
        0.0, current_v * dt_ + comfort_params_.kinematic_half_coefficient *
                                   comfort_acc * dt_ * dt_);
    double next_s = current_s + ds;
    double next_v = std::max(0.0, current_v + comfort_acc * dt_);

    target_values_[i] =
        TargetValue(t, true, next_s, next_v, TargetType::kComfort);
    current_s = next_s;
    current_v = next_v;
    current_a = comfort_acc;
  }
}

double ComfortTarget::CalculateComfortAcceleration(
    const double current_acc, const double current_vel, const double current_s,
    const double front_vel, const double front_s, const double tau,
    const double decel_jerk, double& v_target) const {
  const auto& p = comfort_params_;
  double s_alpha = std::max(p.eps, front_s - current_s);
  double delta_v = current_vel - front_vel;
  double s_star =
      p.s0 +
      std::max(0.0, current_vel * tau + (current_vel * delta_v) /
                                            (2.0 * std::sqrt(p.a * p.b_max)));
  v_target = std::max(p.eps, p.v0);
  double z = s_star / s_alpha;

  double a_free = (current_vel <= v_target)
                      ? p.a * (1.0 - std::pow(current_vel / v_target, p.delta))
                      : -p.b * (1.0 - std::pow(v_target / current_vel,
                                               p.a * p.delta / p.b));

  double a_idm;
  if (current_vel <= v_target) {
    a_idm = (z < 1.0 && std::abs(a_free) > p.eps)
                ? a_free * (1.0 - std::pow(z, 2.0 * p.a / a_free))
                : p.a * (1.0 - std::pow(z, 2.0));
  } else {
    a_idm = (z >= 1.0) ? a_free + p.a * (1.0 - std::pow(z, 2.0)) : a_free;
  }

  a_idm = std::clamp(a_idm, -p.b_hard, p.a);

  double denominator = front_vel * front_vel - 2 * s_alpha * (-p.b);
  double a_cah;
  if (std::abs(denominator) < p.eps) {
    a_cah = -p.b_max;
  } else {
    a_cah = (current_vel * current_vel * (-p.b)) / denominator;
  }
  a_cah = std::clamp(a_cah, -p.b_hard, p.a);

  double comfort_acc =
      (a_idm >= a_cah)
          ? a_idm
          : (1.0 - p.cool_factor) * a_idm +
                p.cool_factor * (a_cah - p.b * tanh((a_idm - a_cah) / (-p.b)));

  double acc_change = comfort_acc - current_acc;
  if (acc_change > p.max_accel_jerk * dt_) {
    comfort_acc = current_acc + p.max_accel_jerk * dt_;
  } else if (acc_change < -decel_jerk * dt_) {
    comfort_acc = current_acc - decel_jerk * dt_;
  }

  return std::clamp(comfort_acc, -p.b_hard, p.a);
}

void ComfortTarget::AddComfortTargetDataToProto() {
#ifdef ENABLE_PROTO_LOG
  auto& debug_info_pb = DebugInfoManager::GetInstance().GetDebugInfoPb();
  auto mutable_comfort_target_data =
      debug_info_pb->mutable_lon_target_s_ref()->mutable_comfort_target();

  for (int32_t i = 0; i < plan_points_num_; i++) {
    const auto& value = target_values_[i];
    auto* ptr = comfort_target_pb_.add_comfort_target_s_ref();
    ptr->set_s(value.s_target_val());
    ptr->set_v(value.v_target_val());
    ptr->set_t(value.relative_t());
    ptr->set_a(acc_values_[i]);
    ptr->set_target_type(static_cast<int32_t>(value.target_type()));
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
