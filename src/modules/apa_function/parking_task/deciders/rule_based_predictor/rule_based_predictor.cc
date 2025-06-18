#include "rule_based_predictor.h"

#include <cmath>
#include <cstddef>

#include "apa_param_config.h"
#include "geometry_math.h"
#include "log_glog.h"
#include "math/vec2d.h"
#include "pose2d.h"
#include "debug_info_log.h"

namespace planning {
namespace apa_planner {

#define DEBUG_TASK (1)

void RuleBasedPredictor::Execute(
    std::shared_ptr<ApaObstacleManager> obs_manager) {
  for (auto& obj : obs_manager->GetMutableObstacles()) {
    ApaObstacle& obstacle = obj.second;

    if (obstacle.GetObsAttributeType() !=
        ApaObsAttributeType::FUSION_POLYGON) {
      continue;
    }

    if (obstacle.Speed() < 0.01) {
      continue;
    }

    Predict(obstacle);
  }

  RecordDebugInfo(obs_manager);

  return;
}

void RuleBasedPredictor::Predict(ApaObstacle& obs) {
  trajectory::Trajectory predict_traj;
  trajectory::TrajectoryPoint traj_point;
  double predict_time = 5.0;
  double delta_time = 1.0;
  double max_dist = obs.Speed() * predict_time + 0.1;
  double delta_dist = obs.Speed() * delta_time;

  double current_time = 0.0;
  double current_dist = 0.0;

  const Eigen::Vector2d& dir = obs.GetSpeedHeading();
  Eigen::Vector2d start = obs.GetCenterPose().pos;
  double obs_heading = obs.GetCenterPose().heading;

  Eigen::Vector2d point;
  while (current_dist < max_dist) {
    point = start + dir * current_dist;

    traj_point.set_absolute_time(current_time);
    traj_point.set_vel(obs.Speed());
    traj_point.set_x(point.x());
    traj_point.set_y(point.y());
    traj_point.set_theta(obs_heading);

    predict_traj.emplace_back(traj_point);

    current_dist += delta_dist;
    current_time += delta_time;
  }

  obs.SetPredictTraj(predict_traj);

  return;
}

void RuleBasedPredictor::RecordDebugInfo(
    std::shared_ptr<ApaObstacleManager> obs_manager) {
  auto& debug = DebugInfoManager::GetInstance().GetDebugInfoPb();
  common::ApaSpeedDebug* speed_debug = debug->mutable_apa_speed_debug();
  speed_debug->clear_predict_traj_set();

  common::ParkPredictTraj debug_traj;
  common::TrajectoryPoint point;
  for (auto& obj : obs_manager->GetMutableObstacles()) {
    ApaObstacle& obstacle = obj.second;

    if (obstacle.GetPredictTraj().size() <= 0) {
      continue;
    }

    debug_traj.Clear();

    const trajectory::Trajectory& pred_traj = obstacle.GetPredictTraj();
    for (size_t i = 0; i < pred_traj.size(); i++) {
      point.set_x(pred_traj[i].x());
      point.set_y(pred_traj[i].y());
      point.set_heading_angle(pred_traj[i].theta());

      debug_traj.add_point()->CopyFrom(point);
    }

    speed_debug->mutable_predict_traj_set()->add_trajs()->CopyFrom(debug_traj);
  }

  return;
}

}  // namespace apa_planner
}  // namespace planning