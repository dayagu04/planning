#include "park_speed_limit_decider.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "apa_param_config.h"
#include "log_glog.h"
#include "speed/apa_speed_decision.h"
#include "debug_info_log.h"

namespace planning {

#define DECIDER_DEBUG (0)

ParkSpeedLimitDecider::ParkSpeedLimitDecider() {}

void ParkSpeedLimitDecider::Process(
    std::shared_ptr<apa_planner::ApaObstacleManager> obs_manager,
    const std::vector<pnc::geometry_lib::PathPoint>& path,
    SpeedDecisions* speed_decisions) {
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

  StopDecision* stop_decision = GetCloseStopDecision(speed_decisions);

  for (size_t i = 0; i < path.size(); ++i) {
    const double path_s = path.at(i).s;
    const pnc::geometry_lib::PathPoint& point = path[i];
    speed_limit = default_cruise_speed_;

    if (i + 1 < path.size()) {
      const pnc::geometry_lib::PathPoint& next_point = path[i + 1];

      // speed limit from path curvature
      if (std::fabs(point.kappa - next_point.kappa) >
          kappa_gap_in_path_point_) {
        speed_limit = std::min(speed_limit, min_cruise_speed_);

        speed_limit_decision.advised_speed = 0.3;
        speed_limit_decision.reason_code = SpeedLimitReason::PATH_KAPPA_CHANGE;
        speed_limit_decision.path_s = path_s;

        speed_decisions->speed_limit_decisions.emplace_back(
            speed_limit_decision);
      }
    }

    // speed limit from nudge obstacles
    if (point.dist_to_obs < obs_dist_for_speed_limit_) {
      speed_limit = std::min(speed_limit, min_cruise_speed_);

      speed_limit_decision.advised_speed = 0.3;
      speed_limit_decision.reason_code = SpeedLimitReason::CLOSE_TO_OBSTACLE;
      speed_limit_decision.path_s = path_s;

      speed_decisions->speed_limit_decisions.emplace_back(speed_limit_decision);
    }

    if (stop_decision != nullptr && path_s > (stop_decision->path_s - 1e-3)) {
      speed_limit = 0;
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
  common::SVGraphSpeedConstraint *speed_limit_debug =
      speed_debug->mutable_speed_limit();

  const std::vector<std::pair<double, double>>& points =
      speed_limit_profile_.SpeedLimitPoints();

  for (size_t i = 0; i < points.size(); i++) {

    speed_limit_debug->add_s(points[i].first);
    speed_limit_debug->add_v_upper_bound(points[i].second);

#if DECIDER_DEBUG

    ILOG_INFO << "i = " << i << ",s = " << points[i].first
              << ",v up = " << points[i].second;
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
  obs_dist_for_speed_limit_ = speed_config.obs_dist_for_speed_limit;

  return;
}

const double ParkSpeedLimitDecider::CalcRefSpeedBySpeedLimitDecision(
    const double ego_v, const double ego_s,
    const SpeedLimitDecision* decision) {
  double abs_ego_v = std::fabs(ego_v);
  if (abs_ego_v < decision->advised_speed) {
    return decision->advised_speed;
  }

  double delta_s  = decision->path_s - ego_s;

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

}  // namespace planning
