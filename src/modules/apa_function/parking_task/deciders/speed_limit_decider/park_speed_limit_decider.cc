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

void ParkSpeedLimitDecider::Execute(
    std::vector<pnc::geometry_lib::PathPoint>& path,
    SpeedDecisions* speed_decisions) {
  if (path.size() <= 1) {
    return;
  }

  config_.Init();

  Pose2D ego_pose = measure_data_ptr_->GetPose();
  col_det_interface_ptr_->GetPathSafeCheckPtr()->Excute(
      ego_pose, PathCheckRequest::DISTANCE_CHECK, 0.0, 0.0, path);

  AddSpeedLimitDecisions(path, speed_decisions);

  PublishDebugInfo(path);

#if DECIDER_DEBUG
  TaskDebug(path);
#endif

  return;
}

void ParkSpeedLimitDecider::AddSpeedLimitDecisions(
    const std::vector<pnc::geometry_lib::PathPoint>& path,
    SpeedDecisions* speed_decisions) {
  double speed_limit;
  ParkLonDecision speed_limit_decision;
  speed_limit_decision.decision_type = LonDecisionType::CAUTION;

  speed_limit_profile_.Clear();
  size_t path_point_size = path.size();
  speed_limit_profile_.Reverse(path_point_size);
  for (size_t i = 0; i < path_point_size; ++i) {
    const double path_s = path.at(i).s;
    const pnc::geometry_lib::PathPoint& point = path[i];
    speed_limit = config_.default_cruise_speed;

    if (i + 1 < path_point_size) {
      const pnc::geometry_lib::PathPoint& next_point = path[i + 1];

      // speed limit from path curvature switch
      if (std::fabs(point.kappa - next_point.kappa) >
          config_.kappa_switch_thresh) {
        speed_limit =
            std::min(speed_limit, config_.speed_limit_by_kappa_switch);

        speed_limit_decision.decision_speed = speed_limit;
        speed_limit_decision.reason_code = LonDecisionReason::PATH_KAPPA_SWITCH;
        speed_limit_decision.path_s = path_s;

        speed_decisions->decisions.emplace_back(speed_limit_decision);
      }
    }

    // speed limit from nudge obstacles
    if (point.dist_to_obs < config_.obs_dist_upper) {
      double dist = std::max(0.0, point.dist_to_obs);
      if (dist < config_.obs_dist_lower) {
        speed_limit = config_.speed_limit_lower_by_obs;
      } else {
        speed_limit = config_.zero_order_param_by_obs +
                      config_.first_order_param_by_obs * dist;
      }

      speed_limit_decision.decision_speed = speed_limit;
      speed_limit_decision.reason_code = LonDecisionReason::CLOSE_TO_OBSTACLE;
      speed_limit_decision.path_s = path_s;

      speed_decisions->decisions.emplace_back(speed_limit_decision);
    }

    // speed limit from path kappa
    if (point.kappa > config_.kappa_thresh ||
        point.kappa < -config_.kappa_thresh) {
      speed_limit = std::min(speed_limit, config_.speed_limit_lower_by_kappa);
      speed_limit_decision.decision_speed = speed_limit;
      speed_limit_decision.reason_code = LonDecisionReason::PATH_KAPPA;
      speed_limit_decision.path_s = path_s;

      speed_decisions->decisions.emplace_back(speed_limit_decision);
    }

    speed_limit_profile_.AppendSpeedLimit(path_s, speed_limit);
  }

  return;
}

void ParkSpeedLimitDecider::PublishDebugInfo(
    const std::vector<pnc::geometry_lib::PathPoint>& path) {
  auto& debug_ = DebugInfoManager::GetInstance().GetDebugInfoPb();
  common::ApaSpeedDebug* speed_debug = debug_->mutable_apa_speed_debug();
  speed_debug->set_ref_cruise_speed(config_.default_cruise_speed);

  return;
}

const ParkLonDecision* ParkSpeedLimitDecider::GetSpeedLimitDecisionBySRange(
    const SpeedDecisions* speed_decisions, const double start_s,
    const double end_s) const {
  if (speed_decisions == nullptr) {
    return nullptr;
  }

  if (speed_decisions->decisions.empty()) {
    return nullptr;
  }

  const ParkLonDecision* decision = nullptr;

  for (size_t i = 0; i < speed_decisions->decisions.size(); i++) {
    if (speed_decisions->decisions[i].decision_type !=
        LonDecisionType::CAUTION) {
      continue;
    }

    if (speed_decisions->decisions[i].path_s > end_s) {
      continue;
    }

    if (speed_decisions->decisions[i].path_s < start_s) {
      continue;
    }

    if (decision == nullptr) {
      decision = &speed_decisions->decisions[i];
    } else if (decision->decision_speed >
               speed_decisions->decisions[i].decision_speed) {
      decision = &speed_decisions->decisions[i];
    }
  }

  if (decision != nullptr) {
    ILOG_INFO << "speed limit = " << decision->decision_speed
              << ", s = " << decision->path_s << ",start_s = " << start_s;
  }

  return decision;
}

const double ParkSpeedLimitDecider::CalcRefSpeedBySpeedLimitDecision(
    const double ego_v, const double ego_s, const ParkLonDecision* decision) {
  double abs_ego_v = std::fabs(ego_v);
  if (abs_ego_v < decision->decision_speed) {
    return decision->decision_speed;
  }

  double delta_s = decision->path_s - ego_s;

  // obstacle is over, no need speed limit
  if (delta_s < 0) {
    return abs_ego_v;
  }

  double advised_acc = -0.2;
  double ref_v = abs_ego_v + advised_acc * 0.1;
  if (ref_v <= decision->decision_speed) {
    return decision->decision_speed;
  }

  return ref_v;
}

void ParkSpeedLimitDecider::TaskDebug(
    const std::vector<pnc::geometry_lib::PathPoint>& path) {
  for (auto& point : path) {
    ILOG_INFO << "s = " << point.s << ", kappa = " << point.kappa;
  }

  return;
}

}  // namespace apa_planner
}  // namespace planning
