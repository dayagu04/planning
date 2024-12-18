#include "parking_stop_decider.h"
#include <cstddef>
#include "collision_detection/path_safe_checker.h"
#include "obstacle_manager.h"
#include "debug_info_log.h"

namespace planning {

#define DECIDER_DEBUG (0)

void ParkingStopDecider::Process(
    const ParkObstacleList& obstacles,
    const std::shared_ptr<apa_planner::ApaWorld> apa_world_ptr,
    const double tracking_path_collision_dist,
    std::vector<pnc::geometry_lib::PathPoint>& path,
    SpeedDecisions* speed_decisions) {
  tracking_path_collision_dist_ = tracking_path_collision_dist;
  const Pose2D pose = apa_world_ptr->GetMeasureDataManagerPtr()->GetPose();

  PathSafeChecker safe_checker;
  safe_checker.Excute(&obstacles, pose, PathCheckRequest::DISTANCE_CHECK, 0.08,
                      0.08, path);
  ego_project_s_ = safe_checker.GetEgoPathProjectS();

  AddStopDecisionByPlanningPath(path, speed_decisions);

  AddStopDecisionByPredictedPath(path, tracking_path_collision_dist,
                                 speed_decisions);

  AddDebugInfo(path);

  return;
}

void ParkingStopDecider::AddStopDecisionByPlanningPath(
    std::vector<pnc::geometry_lib::PathPoint>& path,
    SpeedDecisions* speed_decisions) {
  if (path.empty()) {
    return;
  }

  StopDecision stop_decision;

  for (auto& point : path) {
    if (point.col_flag) {
      stop_decision.stop_point.x = point.pos.x();
      stop_decision.stop_point.y = point.pos.y();
      stop_decision.stop_point.theta = point.heading;
      stop_decision.reason_code = StopDecisionReason::OCC_COLLISION;
      stop_decision.path_s = point.s - 0.2;

      speed_decisions->stop_decisions.emplace_back(stop_decision);

      break;
    }
  }

  // add stop decision by path finish
  auto& point = path.back();

  stop_decision.stop_point.x = point.pos.x();
  stop_decision.stop_point.y = point.pos.y();
  stop_decision.stop_point.theta = point.heading;
  stop_decision.reason_code = StopDecisionReason::PATH_FINISH;
  stop_decision.path_s = point.s;

  speed_decisions->stop_decisions.emplace_back(stop_decision);

  return;
}

void ParkingStopDecider::AddDebugInfo(
    const std::vector<pnc::geometry_lib::PathPoint>& path) {
  auto& debug_ = DebugInfoManager::GetInstance().GetDebugInfoPb();
  common::ApaSpeedDebug* speed_debug = debug_->mutable_apa_speed_debug();
  speed_debug->Clear();
  common::SVGraphSpeedConstraint *speed_limit_debug =
      speed_debug->mutable_speed_limit();

  for (size_t i = 0; i < path.size(); i++) {

    speed_limit_debug->add_obs_dist(path[i].dist_to_obs);

#if DECIDER_DEBUG

    ILOG_INFO << "i = " << i << ",dist to obs = " << path[i].dist_to_obs;
#endif
  }

  return;
}

void ParkingStopDecider::AddStopDecisionByPredictedPath(
    const std::vector<pnc::geometry_lib::PathPoint>& path,
    const double tracking_path_collision_dist, SpeedDecisions* speed_decisions) {
  if (path.empty()) {
    return;
  }
  StopDecision stop_decision;
  stop_decision.reason_code =
      StopDecisionReason::PREDICTED_TRACKING_PATH_COLLISION;
  stop_decision.path_s = ego_project_s_ + tracking_path_collision_dist;

  double dist;
  double nearest_dist = 10000.0;
  size_t nearest_id = 0;
  for (size_t i = 0; i < path.size(); i++) {
    dist = std::fabs(path[i].s - stop_decision.path_s);

    if (dist < nearest_dist) {
      nearest_dist = dist;
      nearest_id = i;
    }
  }

  auto& point = path[nearest_id];

  stop_decision.stop_point.x = point.pos.x();
  stop_decision.stop_point.y = point.pos.y();
  stop_decision.stop_point.theta = point.heading;

  speed_decisions->stop_decisions.emplace_back(stop_decision);

  return;
}

void ParkingStopDecider::TaskDebug() { return; }

}  // namespace planning