#include "weight_maker.h"

#include <cstdint>

#include "debug_info_log.h"
#include "environmental_model.h"
#include "planning_context.h"
#include "status/status.h"
#include "trajectory1d/piecewise_jerk_acceleration_trajectory1d.h"

namespace planning {

WeightMaker::WeightMaker(const SpeedPlannerConfig& speed_planning_config,
                         framework::Session* session)
    : speed_planning_config_(speed_planning_config), session_(session) {
  dt_ = speed_planning_config_.dt;
  plan_time_ = speed_planning_config_.planning_time;
  plan_points_num_ = static_cast<int32_t>(plan_time_ / dt_) + 1;
}

common::Status WeightMaker::Run(const TargetMaker& target_maker,
                                const BoundMaker& bound_maker) {
  ILOG_INFO << "=======LongRefPathDecider: WeightMaker=======";
  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  const auto& init_point = ego_state_manager->planning_init_point();
  init_lon_state_ = {0, init_point.v, init_point.a};

  MakeSWeight(target_maker, bound_maker);

  MakeVWeight(target_maker);

  MakeAccWeight(target_maker);

  MakeJerkWeight();

  CollectDataToProto(target_maker);

  return common::Status::OK();
}

void WeightMaker::MakeSWeight(const TargetMaker& target_maker,
                              const BoundMaker& bound_maker) {
  const auto& lon_ref_path_decider_output =
      session_->planning_context().lon_ref_path_decider_output();

  const bool is_emergency_scenario =
      lon_ref_path_decider_output.is_cross_vru_pre_handle ||
      lon_ref_path_decider_output.is_lon_cipv_emergency_stop ||
      lon_ref_path_decider_output.is_lon_cutin_emergency_stop;

  s_weight_ = std::vector<double>(plan_points_num_, kDefaultSWeight);

  if (is_emergency_scenario) {
    s_weight_ = std::vector<double>(plan_points_num_, kEmergencySWeight);
    return;
  }

  const auto& comfort_target = lon_ref_path_decider_output.comfort_target;

  is_urgent_ = false;
  for (size_t i = 0; i < plan_points_num_; ++i) {
    double relative_t = i * dt_;
    const double s_upper = bound_maker.s_upper_bound(relative_t);
    const double ego_s = comfort_target[i].first;
    const double ego_v = comfort_target[i].second;

    const double s_diff = s_upper - ego_s;
    if (s_diff < ego_v * kDelayTimeBuffer + kBaseSafeDistance) {
      is_urgent_ = true;
      break;
    }
  }

  if (is_urgent_) {
    s_weight_ = std::vector<double>(plan_points_num_, kEmergencySWeight);
  } else {
    for (size_t i = 0; i < plan_points_num_; ++i) {
      double relative_t = i * dt_;
      auto target_value = target_maker.target_value(relative_t);

      if (target_value.target_type() == TargetType::kFollow) {
        constexpr double k = 1.0;
        s_weight_[i] = kDefaultSWeight * (1.0 - std::exp(-k * relative_t));
      } else {
        s_weight_[i] = kDefaultSWeight;
      }
    }
  }
}

void WeightMaker::MakeVWeight(const TargetMaker& target_maker) {
  const double default_v_weight =
      speed_planning_config_.weight_maker_config.v_weight;
  const double cruise_v_weight =
      speed_planning_config_.weight_maker_config.cruise_v_weight;
  v_weight_ = std::vector<double>(plan_points_num_, default_v_weight);
  int kCruiseSpeedCount = 0;
  for (size_t i = 0; i < plan_points_num_; ++i) {
    double relative_t = i * dt_;
    auto target_value = target_maker.target_value(relative_t);
    if (target_value.target_type() == TargetType::kCruiseSpeed) {
      v_weight_[i] = cruise_v_weight;
      ++kCruiseSpeedCount;
    }
  }
  if (kCruiseSpeedCount == plan_points_num_) {
    for (size_t i = 0; i < plan_points_num_; ++i) {
      v_weight_[i] = cruise_v_weight + KDefaultVWeightIncrement;
    }
  }
}

bool WeightMaker::IsNeedAWeight(const TargetMaker& target_maker) {
  int kCruiseSpeedCount = 0;
  for (size_t i = 0; i < plan_points_num_; ++i) {
    double relative_t = i * dt_;
    auto target_value = target_maker.target_value(relative_t);
    if (target_value.target_type() == TargetType::kCruiseSpeed) {
      ++kCruiseSpeedCount;
      if (kCruiseSpeedCount == plan_points_num_) {
        return true;
      }
    }
  }
  return false;
}

void WeightMaker::MakeAccWeight(const TargetMaker& target_maker) {
  const double default_a_weight =
      speed_planning_config_.weight_maker_config.a_weight;
  acc_weight_ = std::vector<double>(plan_points_num_, default_a_weight);
  if (IsNeedAWeight(target_maker)) {
    a_weight_ = std::vector<double>(
        plan_points_num_, default_a_weight + KDefaultAWeightIncrement);
  } else {
    a_weight_ = std::vector<double>(plan_points_num_, 0.0);
  }
}

void WeightMaker::MakeJerkWeight() {
  const double default_jerk_weight =
      speed_planning_config_.weight_maker_config.jerk_weight;
  jerk_weight_ = std::vector<double>(plan_points_num_, default_jerk_weight);
}

double WeightMaker::s_weight(const double t) const {
  int32_t index = static_cast<int32_t>(std::round(t / dt_));
  return s_weight_[index];
}

double WeightMaker::v_weight(const double t) const {
  int32_t index = static_cast<int32_t>(std::round(t / dt_));
  return v_weight_[index];
}

double WeightMaker::a_weight(const double t) const {
  int32_t index = static_cast<int32_t>(std::round(t / dt_));
  return acc_weight_[index];
}

double WeightMaker::a_new_weight(const double t) const {
  int32_t index = static_cast<int32_t>(std::round(t / dt_));
  return a_weight_[index];
}

double WeightMaker::jerk_weight(const double t) const {
  int32_t index = static_cast<int32_t>(std::round(t / dt_));
  return jerk_weight_[index];
}

void WeightMaker::Reset() {
  s_weight_.clear();
  v_weight_.clear();
  a_weight_.clear();
  acc_weight_.clear();
  jerk_weight_.clear();
  weight_maker_replay_info_.Clear();
}

void WeightMaker::CollectDataToProto(const TargetMaker& target_maker) {
#ifdef ENABLE_PROTO_LOG
  auto& debug_info_pb = DebugInfoManager::GetInstance().GetDebugInfoPb();
  auto mutable_weight_data =
      debug_info_pb->mutable_weight_maker()->mutable_weight_maker_replay_info();
  mutable_weight_data->set_is_urgent(is_urgent_);
  for (size_t i = 0; i < plan_points_num_; ++i) {
    double relative_t = i * dt_;
    auto target_value = target_maker.target_value(relative_t);
    auto* ptr = weight_maker_replay_info_.add_target_point();
    ptr->set_s(target_value.s_target_val());
    ptr->set_t(target_value.relative_t());
    ptr->set_target_type(static_cast<int32_t>(target_value.target_type()));
    ptr->set_s_weight(s_weight_[i]);
  }
  mutable_weight_data->CopyFrom(weight_maker_replay_info_);
#endif
}

}  // namespace planning