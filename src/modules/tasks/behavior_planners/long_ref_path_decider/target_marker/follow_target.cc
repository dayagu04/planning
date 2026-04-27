#include "follow_target.h"

#include <cmath>
#include <cstdint>

#include "behavior_planners/long_ref_path_decider/long_ref_path_decider.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "ego_planning_config.h"
#include "environmental_model.h"
#include "planning_context.h"
#include "target.h"

namespace planning {

namespace {
constexpr double kLargeAgentLengthM = 8.0;
constexpr double default_headway = 1.5;
constexpr double min_follow_distance_gap_cut_in = 0.8;
constexpr double kVirtualFrontS = 250.0;
constexpr double kVirtualFrontV = 33.5;
}  // namespace

FollowTarget::FollowTarget(const SpeedPlannerConfig config,
                           framework::Session* session)
    : Target(config, session) {
  follow_target_pb_.Clear();
  upper_bound_infos_ =
      std::vector<UpperBoundInfo>(plan_points_num_, UpperBoundInfo());
  GenerateUpperBoundInfo();

  MakeMinFollowDistance();

  if (session_->is_rads_scene()) {
    GenerateRadsFollowTarget();
  } else if (session_->is_hpp_scene()) {
    GenerateHppFollowTarget();
  } else {
    GenerateFollowTarget();
  }

  AddFollowTargetDataToProto();
}

void FollowTarget::GenerateUpperBoundInfo() {
  const auto* st_graph = session_->planning_context().st_graph_helper();
  const auto& cipv_decider_output =
      session_->planning_context().cipv_decider_output();
  if (st_graph == nullptr) {
    return;
  }
  for (size_t i = 0; i < plan_points_num_; i++) {
    const double t = i * dt_;
    const auto& upper_bound = st_graph->GetPassCorridorUpperBound(t);
    if (upper_bound.agent_id() != speed::kNoAgentId) {
      if (i == 0) {
        cipv_info_.agent_id = upper_bound.agent_id();
        cipv_info_.upper_bound_s = upper_bound.s();
        cipv_info_.vel = upper_bound.velocity();
        auto* agent =
            session_->environmental_model().get_agent_manager()->GetAgent(
                cipv_info_.agent_id);
        if (agent != nullptr) {
          cipv_info_.is_large_vehicle =
              agent->type() == agent::AgentType::BUS ||
              agent->type() == agent::AgentType::TRUCK ||
              agent->type() == agent::AgentType::TRAILER ||
              agent->length() > kLargeAgentLengthM ||
              cipv_decider_output.is_large();
          cipv_info_.type = agent->type();
          cipv_info_.is_tfl_virtual_obs = agent->is_tfl_virtual_obs();
          cipv_info_.is_turnstile_virtual_obs =
              cipv_decider_output.is_turnstile_virtual_obs();
          cipv_info_.is_lane_borrow_obs = agent->is_lane_borrow_virtual_obs();
        }
      }
      const double confidence = LongRefPathDecider::CalcUpperBoundConfidence(
          upper_bound.agent_id(), upper_bound.s());
      upper_bound_infos_[i].s =
          confidence * upper_bound.s() + (1.0 - confidence) * kVirtualFrontS;
      upper_bound_infos_[i].t = t;
      upper_bound_infos_[i].v = confidence * upper_bound.velocity() +
                                (1.0 - confidence) * kVirtualFrontV;
      upper_bound_infos_[i].target_type = TargetType::kFollow;
      upper_bound_infos_[i].agent_id = upper_bound.agent_id();
      upper_bound_infos_[i].st_boundary_id = upper_bound.boundary_id();
    }
  }
  const auto& lon_ref_path_decider_output =
      session_->planning_context().lon_ref_path_decider_output();
  if (lon_ref_path_decider_output.is_lon_cipv_emergency_stop ||
      lon_ref_path_decider_output.is_lon_cutin_emergency_stop) {
    for (size_t i = 0; i < plan_points_num_ &&
                       i < lon_ref_path_decider_output
                               .comfort_target_upper_bound_infos.size();
         i++) {
      const auto& comfort_upper_bound_info =
          lon_ref_path_decider_output.comfort_target_upper_bound_infos[i];
      upper_bound_infos_[i].s = comfort_upper_bound_info.s;
      upper_bound_infos_[i].v = comfort_upper_bound_info.v;
      upper_bound_infos_[i].agent_id = comfort_upper_bound_info.agent_id;
    }
  }
}

void FollowTarget::MakeMinFollowDistance() {
  const double large_vehicle_min_follow_distance =
      config_.large_vehicle_min_follow_distance_gap;
  double min_follow_distance_lower =
      config_.lower_speed_min_follow_distance_gap;
  double cone_min_follow_distance = config_.cone_min_follow_distance_gap;
  const double traffic_light_min_follow_distance_gap =
      config_.traffic_light_min_follow_distance_gap;
  if (cipv_info_.agent_id != -1 && cipv_info_.is_large_vehicle) {
    min_follow_distance_m_ = large_vehicle_min_follow_distance;
    return;
  }
  if (cipv_info_.agent_id != -1 &&
      cipv_info_.type == agent::AgentType::TRAFFIC_CONE) {
    min_follow_distance_m_ = cone_min_follow_distance;
    return;
  }
  if (cipv_info_.agent_id != -1 &&
      (cipv_info_.is_tfl_virtual_obs ||
       cipv_info_.is_turnstile_virtual_obs)) {
    min_follow_distance_m_ = traffic_light_min_follow_distance_gap;
    return;
  }
  const double min_follow_distance_upper =
      config_.high_speed_min_follow_distance_gap;
  const double low_speed_threshold = config_.low_speed_threshold_kmph / 3.6;
  const double high_speed_threshold = config_.high_speed_threshold_kmph / 3.6;

  min_follow_distance_m_ = planning_math::LerpWithLimit(
      min_follow_distance_lower, low_speed_threshold, min_follow_distance_upper,
      high_speed_threshold, init_lon_state_[1]);
}

void FollowTarget::GenerateFollowTarget() {
  auto default_target_value =
      TargetValue(0.0, false, 0.0, 0.0, 0.0, TargetType::kNotSet);
  target_values_ =
      std::vector<TargetValue>(plan_points_num_, default_target_value);

  const auto& agent_headway_decider_output =
      session_->planning_context().agent_headway_decider_output();
  const auto& agents_headway_map =
      agent_headway_decider_output.agents_headway_Info();
  const auto& agent_mgr = session_->environmental_model().get_agent_manager();

  for (int32_t i = 0; i < plan_points_num_; i++) {
    const double t = i * dt_;
    const double vel = virtual_zero_acc_curve_->Evaluate(1, t);
    const double acc = virtual_zero_acc_curve_->Evaluate(2, t);
    const int32_t agent_id = upper_bound_infos_[i].agent_id;
    double upper_bound_s = kVirtualFrontS;
    double follow_time_gap = min_follow_distance_gap_cut_in;

    if (agent_id != speed::kNoAgentId) {
      auto iter = agents_headway_map.find(agent_id);
      if (iter != agents_headway_map.end()) {
        follow_time_gap = iter->second.current_headway;
      }
      upper_bound_s = upper_bound_infos_[i].s;
    }

    double target_s_distance =
        min_follow_distance_m_ + std::max(0.0, vel * follow_time_gap);
    double target_s = std::max(upper_bound_s - target_s_distance, 0.0);

    target_values_[i] =
        TargetValue(t, true, target_s, vel, acc, TargetType::kFollow);

    const auto* agent = agent_mgr->GetAgent(agent_id);
    if (agent && agent->is_lane_borrow_virtual_obs()) {
      target_values_[i].set_s_target_val(upper_bound_infos_[i].s);
    }
  }
}


void FollowTarget::GenerateHppFollowTarget() {
  double matched_desired_headway = default_headway;
  const double default_t = 0.0;
  const bool default_has_target = false;
  const double default_s_target = 0.0;
  const double default_v_target = 0.0;
  const TargetType default_target_type = TargetType::kNotSet;
  auto default_target_value =
      TargetValue(default_t, default_has_target, default_s_target,
                  default_v_target, 0.0, default_target_type);
  target_values_ =
      std::vector<TargetValue>(plan_points_num_, default_target_value);

  const auto& agent_headway_decider_output =
      session_->planning_context().agent_headway_decider_output();
  const auto& agents_headway_map =
      agent_headway_decider_output.agents_headway_Info();
  const auto& cipv_info = session_->planning_context().cipv_decider_output();
  const auto& stop_destination_decider_output =
      session_->planning_context().stop_destination_decider_output();

  for (int32_t i = 0; i < plan_points_num_; i++) {
    const double t = i * dt_;
    auto& target_value = target_values_[i];
    target_value.set_relative_t(t);
    if (upper_bound_infos_[i].target_type == TargetType::kNotSet) {
      target_values_[i] = target_value;
      double s_value = target_value.s_target_val();
      if (MakeSValueWithTargetFollowCurve(i, false, &s_value)) {
        target_value.set_has_target(true);
        target_value.set_s_target_val(s_value);
        target_value.set_target_type(TargetType::kFollow);
      }
      continue;
    }
    target_value.set_has_target(true);
    const double vel = virtual_zero_acc_curve_->Evaluate(1, t);
    const auto& agent_id = upper_bound_infos_[i].agent_id;
    double follow_time_gap = min_follow_distance_gap_cut_in;
    auto iter = agents_headway_map.find(agent_id);
    if (iter != agents_headway_map.end()) {
      follow_time_gap = iter->second.current_headway;
    }

    double target_s_disatnce =
        min_follow_distance_m_ + std::max(0.0, vel * follow_time_gap);

    double upper_bound_s =
        std::max(upper_bound_infos_[i].s - min_follow_distance_m_, 0.0);
    double target_s =
        std::max(upper_bound_infos_[i].s - target_s_disatnce, 0.0);
    double s_target_value = std::min(upper_bound_s, target_s);

    if (cipv_info.cipv_id() ==
            stop_destination_decider_output.stop_destination_virtual_agent_id() ||
        cipv_info.is_turnstile_virtual_obs()) {
      target_s_disatnce = vel * follow_time_gap;
      s_target_value =
          std::max(upper_bound_infos_[i].s - target_s_disatnce, 0.0);
    }

    target_value.set_s_target_val(s_target_value);
    target_value.set_v_target_val(vel);
    target_value.set_target_type(upper_bound_infos_[i].target_type);
  }
}

void FollowTarget::GenerateRadsFollowTarget() {
  double matched_desired_headway = default_headway;
  const double default_t = 0.0;
  const bool default_has_target = false;
  const double default_s_target = 0.0;
  const double default_v_target = 0.0;
  const double default_a_target = 0.0;
  const TargetType default_target_type = TargetType::kNotSet;
  auto default_target_value =
      TargetValue(default_t, default_has_target, default_s_target,
                  default_v_target, default_a_target, default_target_type);
  target_values_ =
      std::vector<TargetValue>(plan_points_num_, default_target_value);

  const auto& agent_headway_decider_output =
      session_->planning_context().agent_headway_decider_output();
  const auto& agents_headway_map =
      agent_headway_decider_output.agents_headway_Info();
  const auto& cipv_info = session_->planning_context().cipv_decider_output();
  const auto& stop_destination_decider_output =
      session_->planning_context().stop_destination_decider_output();
  const auto agent_manager =
      session_->environmental_model().get_agent_manager();

  for (int32_t i = 0; i < plan_points_num_; i++) {
    const double t = i * dt_;
    auto& target_value = target_values_[i];
    target_value.set_relative_t(t);
    if (upper_bound_infos_[i].target_type == TargetType::kNotSet) {
      target_values_[i] = target_value;
      double s_value = target_value.s_target_val();
      if (MakeSValueWithTargetFollowCurve(i, false, &s_value)) {
        target_value.set_has_target(true);
        target_value.set_s_target_val(s_value);
        target_value.set_target_type(TargetType::kFollow);
      }
      continue;
    }
    target_value.set_has_target(true);
    const double vel = virtual_zero_acc_curve_->Evaluate(1, t);
    const double acc = virtual_zero_acc_curve_->Evaluate(2, t);
    const auto& agent_id = upper_bound_infos_[i].agent_id;
    double follow_time_gap = min_follow_distance_gap_cut_in;
    auto iter = agents_headway_map.find(agent_id);
    if (iter != agents_headway_map.end()) {
      follow_time_gap = iter->second.current_headway;
    }
    const auto* agent = agent_manager->GetAgent(agent_id);
    double rads_obs_follow_distance_buffer =
        config_.rads_follow_distance_buffer_dynamic;
    if (agent != nullptr && agent->is_static()) {
      rads_obs_follow_distance_buffer =
          config_.rads_follow_distance_buffer_static;
    }
    double target_s_distance =
        std::max(vel * follow_time_gap + rads_obs_follow_distance_buffer,
                 rads_obs_follow_distance_buffer);

    double upper_bound_s = std::max(
        upper_bound_infos_[i].s - rads_obs_follow_distance_buffer, 0.0);
    double target_s =
        std::max(upper_bound_infos_[i].s - target_s_distance, 0.0);
    double s_target_value = std::min(upper_bound_s, target_s);

    if (cipv_info.cipv_id() ==
        stop_destination_decider_output.stop_destination_virtual_agent_id()) {
      target_s_distance = vel * follow_time_gap;
      s_target_value =
          std::max(upper_bound_infos_[i].s - target_s_distance, 0.0);
    }

    target_value.set_s_target_val(s_target_value);
    target_value.set_v_target_val(vel);
    target_value.set_a_target_val(acc);
    target_value.set_target_type(upper_bound_infos_[i].target_type);
  }
}

bool FollowTarget::MakeSValueWithTargetFollowCurve(
    const int32_t index, const bool has_valid_s_value,
    double* const target_s_value) const {
  if (target_follow_curve_ == nullptr) {
    return false;
  }
  auto st_points = target_follow_curve_->get_target_st_curve().get_st_points();
  if (st_points.size() - 1 < index) {
    return false;
  }

  double target_follow_value = st_points[index].s;
  if (target_follow_value < *target_s_value || !has_valid_s_value) {
    *target_s_value = target_follow_value;
    return true;
  }
  return false;
}

double FollowTarget::MakeSlowerFollowSTarget(const double speed,
                                             const double upper_bound_s,
                                             const double time_gap) const {
  constexpr double kFollowDistanceBuffer = 2.0;
  double follow_target_distance = std::fmax(
      speed * time_gap + kFollowDistanceBuffer, min_follow_distance_m_);
  double s_target = upper_bound_s - follow_target_distance;
  return s_target;
}

void FollowTarget::AddFollowTargetDataToProto() {
#ifdef ENABLE_PROTO_LOG
  auto& debug_info_pb = DebugInfoManager::GetInstance().GetDebugInfoPb();
  auto mutable_follow_target_data =
      debug_info_pb->mutable_lon_target_s_ref()->mutable_follow_target();
  if (!target_values_.empty()) {
    for (const auto& value : target_values_) {
      auto* ptr = follow_target_pb_.add_follow_target_s_ref();
      ptr->set_s(value.s_target_val());
      ptr->set_v(value.v_target_val());
      ptr->set_t(value.relative_t());
      ptr->set_target_type(static_cast<int32_t>(value.target_type()));
    }
  }
  mutable_follow_target_data->CopyFrom(follow_target_pb_);
#endif
}

}  // namespace planning