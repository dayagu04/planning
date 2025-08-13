#include "safety_target.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>

#include "behavior_planners/long_ref_path_decider/target_marker/target.h"
#include "behavior_planners/speed_limit_decider/speed_limit_decider_output.h"
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
  idm_params_.b_hard = 2.0;
  idm_params_.front_b_hard = 5.0;
  idm_params_.max_jerk = 1.0;
  idm_params_.virtual_front_s = 200.0;
  idm_params_.min_distance = 0.5;

  upper_bound_infos_ =
      std::vector<UpperBoundInfo>(plan_points_num_, UpperBoundInfo());

  GenerateUpperBoundInfo();

  GenerateSafetyTarget();

  AddSafetyTargetDataToProto();
}

void SafetyTarget::GenerateUpperBoundInfo() {
  const auto* st_graph = session_->planning_context().st_graph_helper();

  const double virtual_front_s = idm_params_.virtual_front_s;
  const double virtual_front_vel = idm_params_.v0;

  for (size_t i = 0; i < plan_points_num_; i++) {
    const double t = i * dt_;

    if (st_graph != nullptr) {
      const auto& upper_bound = st_graph->GetPassCorridorUpperBound(t);
      if (upper_bound.agent_id() != speed::kNoAgentId) {
        upper_bound_infos_[i].s = upper_bound.s();
        upper_bound_infos_[i].t = t;
        upper_bound_infos_[i].v = upper_bound.velocity();
        upper_bound_infos_[i].target_type = TargetType::kSafety;
        upper_bound_infos_[i].agent_id = upper_bound.agent_id();
        upper_bound_infos_[i].st_boundary_id = upper_bound.boundary_id();
        continue;
      }
    }

    upper_bound_infos_[i].s = virtual_front_s;
    upper_bound_infos_[i].t = t;
    upper_bound_infos_[i].v = virtual_front_vel;
    upper_bound_infos_[i].target_type = TargetType::kSafety;
    upper_bound_infos_[i].agent_id = 799999;
    upper_bound_infos_[i].st_boundary_id = 799999;
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

  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  double ego_vel = ego_state_manager->ego_v();
  double ego_acc = ego_state_manager->ego_acc();
  double ego_s = 0.0;

  double current_s = ego_s;
  double current_v = ego_vel;
  double current_a = ego_acc;

  target_values_[0].set_relative_t(0.0);
  target_values_[0].set_has_target(true);
  target_values_[0].set_s_target_val(current_s);
  target_values_[0].set_v_target_val(current_v);
  target_values_[0].set_target_type(TargetType::kSafety);

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
          tau = std::min(iter->second.current_headway, idm_params_.T);
        }
      }
    }

    double final_acc = CalculateSafetyAcceleration(
        current_v, front_s, front_vel, current_s, current_a, tau);

    double next_s = current_s + current_v * dt_ + 0.5 * final_acc * dt_ * dt_;
    double next_v = current_v + final_acc * dt_;

    next_s = std::max(current_s, next_s);
    next_v = std::max(0.0, next_v);

    target_value.set_has_target(true);
    target_value.set_s_target_val(next_s);
    target_value.set_v_target_val(next_v);
    target_value.set_target_type(TargetType::kSafety);

    current_s = next_s;
    current_v = next_v;
    current_a = final_acc;
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
    const double current_vel, const double front_s, const double front_vel,
    const double current_s, const double current_acc, const double tau) const {
  const double v0 = idm_params_.v0;
  const double s0 = idm_params_.s0;
  const double T = idm_params_.T;
  const double a = idm_params_.a;
  const double b = idm_params_.b;
  const double b_max = idm_params_.b_max;
  const double b_hard = idm_params_.b_hard;
  const double front_b_hard = idm_params_.front_b_hard;
  const double delta = idm_params_.delta;

  double delta_v = std::max(0.0, current_vel - front_vel);
  double s_star =
      s0 + std::max(0.0, current_vel * current_vel / (2 * b_max) -
                             front_vel * front_vel / (2 * front_b_hard) +
                             current_vel * tau);

  double s_desired = std::max(0.0, front_s - (s0 + current_vel * tau));
  double dynamic_v0 =
      CalcDesiredVelocity(current_s, s_desired, front_vel, current_vel);
  JSON_DEBUG_VALUE("safety_dynamic_vel", dynamic_v0);
  double final_v0 = std::max(front_vel, std::min(v0, dynamic_v0));
  JSON_DEBUG_VALUE("safety_target_vel", final_v0);
  double s_alpha = s_desired;
  double v0_safe = std::max(1e-3, final_v0);

  double a_free;
  if (current_vel <= final_v0) {
    a_free = a * (1 - std::pow(current_vel / v0_safe, delta));
  } else if (current_vel > final_v0 && current_s < s_desired) {
    double s_progress = std::max(0.0, std::min(1.0, current_s / s_desired));
    double vel_ratio = current_vel / std::max(final_v0, 1e-6);
    double position_demand = 1.0 - s_progress;
    double speed_control = vel_ratio - 1.0;
    double net_demand = position_demand - speed_control * 0.3;
    if (net_demand > 0.1) {
      a_free = a * 0.3 * net_demand;
    } else {
      a_free = -b * 0.3 * speed_control;
    }
  } else {
    a_free = -b * (1 - std::pow(final_v0 / v0, a * delta / b));
  }

  double z = s_star / s_alpha;
  double acc_idm;
  if (current_vel <= v0_safe) {
    if (z >= 1.0) {
      acc_idm = a * (1 - z * z);
    } else {
      if (std::abs(a_free) > 1e-6) {
        acc_idm = a_free * (1 - std::pow(z, 2.0 * a / a_free));
      } else {
        acc_idm = a * (1 - z * z);
      }
    }
  } else {
    if (z >= 1.0) {
      acc_idm = a_free + a * (1 - z * z);
    } else {
      acc_idm = a_free;
    }
  }

  double ds = front_s - s_star;
  double acc_cah;
  if (ds > idm_params_.min_distance) {
    acc_cah = std::min(0.0, -(current_vel * delta_v) / (2 * s_star));
  } else {
    acc_cah = b * (ds / (s_star - idm_params_.s0));
  }

  double final_acc;
  if (acc_idm >= acc_cah) {
    double distance_ratio = std::min(front_s / s_star, 2.0);
    double transition_factor = std::max(0.0, distance_ratio - 1.0);
    final_acc =
        acc_idm * transition_factor + acc_cah * (1.0 - transition_factor);
  } else {
    final_acc = acc_cah;
  }

  double kMaxJerk = idm_params_.max_jerk;
  double kMaxAccChange = kMaxJerk * dt_;
  double acc_change = final_acc - current_acc;
  if (std::abs(acc_change) > kMaxAccChange) {
    if (acc_change > 0) {
      final_acc = current_acc + kMaxAccChange;
    } else {
      final_acc = current_acc - kMaxAccChange;
    }
  }

  final_acc = std::max(std::min(idm_params_.a, final_acc), -idm_params_.b_hard);

  return final_acc;
}

void SafetyTarget::AddSafetyTargetDataToProto() {
  auto& debug_info_pb = DebugInfoManager::GetInstance().GetDebugInfoPb();
  auto mutable_safety_target_data =
      debug_info_pb->mutable_lon_target_s_ref()->mutable_safety_target();

  if (!target_values_.empty()) {
    for (const auto& value : target_values_) {
      auto* ptr = safety_target_pb_.add_safety_target_s_ref();
      ptr->set_s(value.s_target_val());
      ptr->set_t(value.relative_t());
      ptr->set_target_type(static_cast<int32_t>(value.target_type()));
    }
  }

  mutable_safety_target_data->CopyFrom(safety_target_pb_);
}

}  // namespace planning