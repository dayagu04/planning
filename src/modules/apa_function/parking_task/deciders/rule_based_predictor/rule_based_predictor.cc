#include "rule_based_predictor.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "apa_obstacle.h"
#include "apa_param_config.h"
#include "debug_info_log.h"
#include "geometry_math.h"
#include "log_glog.h"
#include "math/vec2d.h"
#include "pose2d.h"
#include "predictor_config.h"

namespace planning {
namespace apa_planner {

#define DEBUG_TASK (0)

void RuleBasedPredictor::Execute(
    std::shared_ptr<ApaObstacleManager>& obs_manager) {
  for (auto& obj : obs_manager->GetMutableObstaclesODTracking()) {
    ApaObstacle& obstacle = obj.second;

    if (obstacle.GetObsAttributeType() != ApaObsAttributeType::FUSION_POLYGON) {
      continue;
    }

    if (obstacle.GetObsMovementType() != ApaObsMovementType::MOTION) {
      obstacle.ClearPredictTraj();
      continue;
    }

    if (obstacle.Speed() < 0.3) {
      obstacle.ClearPredictTraj();
      continue;
    }

    if (obstacle.GetObsScemanticType() == ApaObsScemanticType::PEOPLE) {
      PredictByCV(obstacle);
    } else {
      PredictByCTRV(obstacle);
    }
  }

  RecordDebugInfo(obs_manager);

  return;
}

void RuleBasedPredictor::PredictByCV(ApaObstacle& obs) {
  trajectory::Trajectory predict_traj;
  trajectory::TrajectoryPoint traj_point;
  RuleBasedPredictorConfig config;
  config.predict_time = 4.0;
  config.time_resolution = 0.2;
  double dist_resolution = obs.Speed() * config.time_resolution;
  double current_time = 0.0;
  double current_dist = 0.0;

  const Eigen::Vector2d& dir = obs.GetSpeedHeading();
  Eigen::Vector2d start = obs.GetCenterPose().pos;
  double obs_heading = obs.GetCenterPose().heading;

  Eigen::Vector2d point;
  int size = std::ceil(config.predict_time / config.time_resolution);
  predict_traj.reserve(size);
  for (int i = 0; i < size; i++) {
    point = start + dir * current_dist;

    traj_point.set_absolute_time(current_time);
    traj_point.set_vel(obs.Speed());
    traj_point.set_x(point.x());
    traj_point.set_y(point.y());
    traj_point.set_theta(obs_heading);

    predict_traj.emplace_back(traj_point);

    current_dist += dist_resolution;
    current_time += config.time_resolution;
  }

  obs.SetPredictTraj(predict_traj);

  return;
}

void RuleBasedPredictor::PredictByCTRV(ApaObstacle& obs) {
  obs.ClearPredictTraj();

  double speed = obs.Speed();
  if (speed < 0.3) {
    return;
  }

  trajectory::Trajectory predict_traj;
  trajectory::TrajectoryPoint traj_point;

  RuleBasedPredictorConfig config;
  config.predict_time = 4.0;
  config.time_resolution = 0.2;

  double current_time = 0.0;

  Eigen::Vector2d current_pos = obs.GetCenterPose().pos;
  double current_heading = obs.GetCenterPose().heading;

  obs.UpdateObstacleHistoryPositions(current_pos);
  const auto& history = obs.GetHistoryTraejctory();

  // ---------- 1. estimate ω ----------
  const double omega0 = PredictApaObstacleSignedOmega(
      history, config.time_resolution, speed, obs);

  // ---------- 2. predict ----------
  int size = std::ceil(config.predict_time / config.time_resolution);
  predict_traj.reserve(size);

  const double omega_tau = 1.2;  // decay time constant

  for (int i = 0; i < size; i++) {
    double dt = config.time_resolution;

    // ω attenuation (to prevent dead circles)
    double omega = omega0 * std::exp(-current_time / omega_tau);

    if (std::fabs(omega) > 1e-4) {
      double new_heading = current_heading + omega * dt;
      current_pos.x() +=
          (speed / omega) * (std::sin(new_heading) - std::sin(current_heading));
      current_pos.y() += (speed / omega) *
                         (-std::cos(new_heading) + std::cos(current_heading));
      current_heading = new_heading;
    } else {
      current_pos.x() += speed * std::cos(current_heading) * dt;
      current_pos.y() += speed * std::sin(current_heading) * dt;
    }

    traj_point.set_absolute_time(current_time);
    traj_point.set_vel(speed);
    traj_point.set_x(current_pos.x());
    traj_point.set_y(current_pos.y());
    traj_point.set_theta(current_heading);

    predict_traj.emplace_back(traj_point);
    current_time += dt;
  }

  obs.SetPredictTraj(predict_traj);

  return;
}

void RuleBasedPredictor::RecordDebugInfo(
    std::shared_ptr<ApaObstacleManager>& obs_manager) {
  auto& debug = DebugInfoManager::GetInstance().GetDebugInfoPb();
  common::ApaSpeedDebug* speed_debug = debug->mutable_apa_speed_debug();
  speed_debug->clear_predict_traj_set();

  common::ParkPredictTraj* debug_traj;
  common::TrajectoryPoint point;
  for (auto& obj : obs_manager->GetObstaclesODTracking()) {
    const ApaObstacle& obstacle = obj.second;

    if (obstacle.GetPredictTraj().size() <= 0) {
      continue;
    }

    debug_traj = speed_debug->mutable_predict_traj_set()->add_trajs();
    debug_traj->Clear();

    const trajectory::Trajectory& pred_traj = obstacle.GetPredictTraj();
    for (size_t i = 0; i < pred_traj.size(); i++) {
      point.set_x(pred_traj[i].x());
      point.set_y(pred_traj[i].y());
      point.set_heading_angle(pred_traj[i].theta());

      debug_traj->add_point()->CopyFrom(point);
    }
    debug_traj->set_obs_id(obstacle.GetId());

#if DEBUG_TASK
    debug_traj->DebugString();
#endif
  }

  return;
}

const double RuleBasedPredictor::PredictApaObstacleSignedOmega(
    const std::deque<Eigen::Vector2d>& history, double history_dt, double speed,
    ApaObstacle& obs) {
  constexpr double kFreezeSpeed = 0.15;
  constexpr double kMinReliableOmega = 0.08;
  constexpr double kEnterTurnOmega = 0.15;
  constexpr double kExitTurnOmega = 0.01;
  constexpr double kMaxOmega = 0.5;
  constexpr double alpha = 0.35;

  double omega_raw_signed =
      EstimateOmegaByCurvature(history, history_dt, speed);

  double omega_raw_mag = std::fabs(omega_raw_signed);

  // low-pass filtering (only filtering amplitude values)
  double omega_mag =
      alpha * omega_raw_mag + (1.0 - alpha) * obs.GetLastOmegaMagnitude();

  // turn to evidence
  DynamicObsTurnDirection evidence = DynamicObsTurnDirection::STRAIGHT;

  evidence = (omega_raw_signed > 0.0) ? DynamicObsTurnDirection::LEFT
                                      : DynamicObsTurnDirection::RIGHT;

  // direction update
  DynamicObsTurnDirection dir = obs.GetTurnDirection();

  switch (dir) {
    case DynamicObsTurnDirection::STRAIGHT:
      if (omega_mag > kEnterTurnOmega) {
        dir = evidence;
      }
      break;

    case DynamicObsTurnDirection::LEFT:
      if (omega_mag < kExitTurnOmega ||
          omega_raw_signed * obs.GetLastOmegaSigned() < 0.0) {
        dir = DynamicObsTurnDirection::STRAIGHT;
      }
      break;

    case DynamicObsTurnDirection::RIGHT:
      if (omega_mag < kExitTurnOmega ||
          omega_raw_signed * obs.GetLastOmegaSigned() < 0.0) {
        dir = DynamicObsTurnDirection::STRAIGHT;
      }
      break;
  }
  // set status
  obs.SetTurnDirection(dir);
  obs.SetLastOmegaMagnitude(omega_mag);
  obs.SetLastOmegaSigned(static_cast<int>(dir) * omega_mag);

  // weighted calculation ω
  return static_cast<int>(dir) * omega_mag;
}

const double RuleBasedPredictor::EstimateOmegaByCurvature(
    const std::deque<Eigen::Vector2d>& history, double history_dt,
    double speed) {
  constexpr double kFreezeSpeed = 0.3;
  if (history.size() < 3 || speed < kFreezeSpeed) return 0.0;

  const auto& p0 = history[history.size() - 3];
  const auto& p1 = history[history.size() - 2];
  const auto& p2 = history[history.size() - 1];

  Eigen::Vector2d v0 = p1 - p0;
  Eigen::Vector2d v1 = p2 - p1;
  Eigen::Vector2d v2 = p2 - p0;

  const double a = v0.norm();
  const double b = v1.norm();
  const double c = v2.norm();

  constexpr double kMinLen = 0.2;
  if (a < kMinLen || b < kMinLen || c < kMinLen) return 0.0;

  // signed area * 2
  double cross = v0.x() * v1.y() - v0.y() * v1.x();
  double area2 = cross;

  // curvature κ = 2 * area / (abc)
  double curvature = 2.0 * area2 / (a * b * c);

  // ω = v * κ
  double omega = speed * curvature;

  constexpr double kMaxOmega = 0.5;
  omega = std::clamp(omega, -kMaxOmega, kMaxOmega);
  return omega;
}

}  // namespace apa_planner
}  // namespace planning