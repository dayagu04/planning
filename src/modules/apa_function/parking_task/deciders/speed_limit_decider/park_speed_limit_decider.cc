#include "park_speed_limit_decider.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "apa_param_config.h"
#include "debug_info_log.h"
#include "log_glog.h"
#include "speed/apa_speed_decision.h"

namespace planning {
namespace apa_planner {
#define DECIDER_DEBUG (0)

ParkSpeedLimitDecider::ParkSpeedLimitDecider() {}

void ParkSpeedLimitDecider::Process(
    const std::vector<pnc::geometry_lib::PathPoint>& path,
    SpeedDecisions* speed_decisions) {
  if (path.size() <= 1) {
    return;
  }

  UpdateConfig();

  AddSpeedLimitDecisions(path, speed_decisions);

  PublishDebugInfo();

  return;
}

void ParkSpeedLimitDecider::AddSpeedLimitDecisions(
    const std::vector<pnc::geometry_lib::PathPoint>& path,
    SpeedDecisions* speed_decisions) {
  double speed_limit;
  SpeedLimitDecision speed_limit_decision;

  size_t path_point_size = path.size();

  for (size_t i = 0; i < path_point_size; ++i) {
    const double path_s = path.at(i).s;
    const pnc::geometry_lib::PathPoint& point = path[i];
    speed_limit = default_cruise_speed_;

    if (i + 1 < path_point_size) {
      const pnc::geometry_lib::PathPoint& next_point = path[i + 1];

      // speed limit from path curvature
      if (std::fabs(point.kappa - next_point.kappa) >
          kappa_gap_in_path_point_) {
        speed_limit = std::min(speed_limit, kappa_gap_speed_limit_);

        speed_limit_decision.advised_speed = speed_limit;
        speed_limit_decision.reason_code = SpeedLimitReason::PATH_KAPPA_CHANGE;
        speed_limit_decision.path_s = path_s;

        speed_decisions->speed_limit_decisions.emplace_back(
            speed_limit_decision);
      }
    }

    // speed limit from nudge obstacles
    if (point.dist_to_obs < obs_dist_thresh_) {
      speed_limit = std::min(speed_limit, obs_dist_speed_limit_);

      speed_limit_decision.advised_speed = speed_limit;
      speed_limit_decision.reason_code = SpeedLimitReason::CLOSE_TO_OBSTACLE;
      speed_limit_decision.path_s = path_s;

      speed_decisions->speed_limit_decisions.emplace_back(speed_limit_decision);
    }

    // speed limit from path kappa
    if (point.kappa > kappa_thresh_ || point.kappa < -kappa_thresh_) {
      speed_limit = std::min(speed_limit, kappa_speed_limit_);
      speed_limit_decision.advised_speed = speed_limit;
      speed_limit_decision.reason_code = SpeedLimitReason::PATH_KAPPA;
      speed_limit_decision.path_s = path_s;

      speed_decisions->speed_limit_decisions.emplace_back(speed_limit_decision);
    }

    speed_limit_profile_.AppendSpeedLimit(path_s, speed_limit);
  }

  return;
}

StopDecision* ParkSpeedLimitDecider::GetCloseStopDecision(
    SpeedDecisions* speed_decisions) {
  if (speed_decisions->stop_decisions.empty()) {
    return nullptr;
  }

  StopDecision* stop_decision = nullptr;
  for (size_t i = 0; i < speed_decisions->stop_decisions.size(); i++) {
    if (stop_decision == nullptr) {
      stop_decision = &speed_decisions->stop_decisions[i];
    } else if (speed_decisions->stop_decisions[i].path_s <
               stop_decision->path_s) {
      stop_decision = &speed_decisions->stop_decisions[i];
    }

#if DECIDER_DEBUG
    ILOG_INFO << "s = " << stop_decision->path_s
              << ",reason = " << static_cast<int>(stop_decision->reason_code);
#endif
  }

  return stop_decision;
}

void ParkSpeedLimitDecider::PublishDebugInfo() {
  auto& debug_ = DebugInfoManager::GetInstance().GetDebugInfoPb();
  common::ApaSpeedDebug* speed_debug = debug_->mutable_apa_speed_debug();
  speed_debug->set_ref_cruise_speed(default_cruise_speed_);

  common::SVGraphSpeedConstraint* dp_speed_constraint_debug =
      speed_debug->mutable_dp_speed_constraint();

  const std::vector<std::pair<double, double>>& points =
      speed_limit_profile_.SpeedLimitPoints();

  for (size_t i = 0; i < points.size(); i++) {
    dp_speed_constraint_debug->add_s(points[i].first);
    dp_speed_constraint_debug->add_v_upper_bound(points[i].second);
    dp_speed_constraint_debug->add_a_upper_bound(acc_upper_);
    dp_speed_constraint_debug->add_a_lower_bound(acc_lower_);
    dp_speed_constraint_debug->add_jerk_upper_bound(jerk_upper_);
    dp_speed_constraint_debug->add_jerk_lower_bound(jerk_lower_);

#if DECIDER_DEBUG

    ILOG_INFO << "i = " << i << ",s = " << points[i].first
              << ",v up = " << points[i].second << ",jerk upper = "
              << dp_speed_constraint_debug->jerk_upper_bound(i);
#endif
  }

  return;
}

const SpeedLimitDecision* ParkSpeedLimitDecider::GetSpeedLimitDecisionBySRange(
    const SpeedDecisions* speed_decisions, const double start_s,
    const double end_s) const {
  if (speed_decisions == nullptr) {
    return nullptr;
  }

  if (speed_decisions->speed_limit_decisions.empty()) {
    return nullptr;
  }

  const SpeedLimitDecision* decision = nullptr;

  for (size_t i = 0; i < speed_decisions->speed_limit_decisions.size(); i++) {
    if (speed_decisions->speed_limit_decisions[i].path_s > end_s) {
      continue;
    }

    if (speed_decisions->speed_limit_decisions[i].path_s < start_s) {
      continue;
    }

    if (decision == nullptr) {
      decision = &speed_decisions->speed_limit_decisions[i];
    } else if (decision->advised_speed >
               speed_decisions->speed_limit_decisions[i].advised_speed) {
      decision = &speed_decisions->speed_limit_decisions[i];
    }
  }

  if (decision != nullptr) {
    ILOG_INFO << " speed limit = " << decision->advised_speed
              << ",s = " << decision->path_s << ",start_s = " << start_s;
  }

  return decision;
}

void ParkSpeedLimitDecider::UpdateConfig() {
  const apa_planner::ParkingSpeedConfig& speed_config =
      apa_param.GetParam().speed_config;

  default_cruise_speed_ = speed_config.default_cruise_speed;
  min_cruise_speed_ = speed_config.min_cruise_speed;

  acc_upper_ = speed_config.acc_upper;
  acc_lower_ = speed_config.acc_lower;

  jerk_upper_ = speed_config.jerk_upper;
  jerk_lower_ = speed_config.jerk_lower;

  // update path point kappa gap
  // If front wheel change 0.8 ratio, add speed limit.
  double front_wheel =
      std::atan(apa_param.GetParam().wheel_base /
                std::max(0.001, apa_param.GetParam().min_turn_radius));
  double front_wheel_gap = 0.8 * front_wheel;
  double kappa = std::tan(front_wheel_gap) / apa_param.GetParam().wheel_base;
  kappa_gap_in_path_point_ = kappa;
  kappa_gap_speed_limit_ = min_cruise_speed_;

  // kappa limit speed
  kappa_thresh_ = kappa;
  kappa_speed_limit_ = 0.8;

  // obs distance related
  obs_dist_thresh_ = 0.3;
  obs_dist_speed_limit_ = speed_config.obs_dist_for_speed_limit;

  ILOG_INFO << "kappa_gap_in_path_point = " << kappa_gap_in_path_point_
            << ",kappa_thresh = " << kappa_thresh_;

  return;
}

const double ParkSpeedLimitDecider::CalcRefSpeedBySpeedLimitDecision(
    const double ego_v, const double ego_s,
    const SpeedLimitDecision* decision) {
  double abs_ego_v = std::fabs(ego_v);
  if (abs_ego_v < decision->advised_speed) {
    return decision->advised_speed;
  }

  double delta_s = decision->path_s - ego_s;

  // obstacle is over, no need speed limit
  if (delta_s < 0) {
    return abs_ego_v;
  }

  double advised_acc = -0.2;
  double ref_v = abs_ego_v + advised_acc * 0.1;
  if (ref_v <= decision->advised_speed) {
    return decision->advised_speed;
  }

  return ref_v;
}
}  // namespace apa_planner
}  // namespace planning
