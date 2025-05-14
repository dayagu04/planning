#include "parking_stop_decider.h"

#include <algorithm>
#include <cstddef>
#include <vector>

#include "apa_obstacle.h"
#include "collision_detection/path_safe_checker.h"
#include "debug_info_log.h"
#include "log_glog.h"
#include "obstacle_manager.h"
#include "pose2d.h"
#include "speed/apa_speed_decision.h"
#include "task_basic_types.h"

namespace planning {
namespace apa_planner {

#define DECIDER_DEBUG (1)

bool ParkingStopDecider::IsVehComponentCollision(const Polygon2D* polygon,
                                                 const ApaObstacle& obs) {
  bool is_collision = false;

  // envelop box check
  gjk_interface_.PolygonCollisionByCircleCheck(
      &is_collision, &obs.GetPolygon2DGlobal(), polygon, 0.01);

  if (!is_collision) {
    // ILOG_INFO << "size = " << obstacle.points.size() << " box no
    // collision";
    return false;
  }

  // internal points
  const std::vector<Eigen::Vector2d>& points = obs.GetPtClout2dGlobal();

  for (size_t j = 0; j < points.size(); j++) {
    gjk_interface_.PolygonPointCollisionDetect(
        polygon, Eigen::Vector2f(points[j][0], points[j][1]), &is_collision);

    if (is_collision) {
      return true;
    }
  }

  return false;
}

bool ParkingStopDecider::IsVehCollision(const PolygonFootPrint& foot_print,
                                        const ApaObstacle& obs) {
  Polygon2D veh_global_polygon;
  bool is_collision = false;

  is_collision = IsVehComponentCollision(&foot_print.max_polygon, obs);
  if (obs.GetObsMovementType() == ApaObsMovementType::MOTION) {
    return is_collision;
  }

  // static agent need more accurate check.
  if (!is_collision) {
    return false;
  }

  is_collision = IsVehComponentCollision(&foot_print.body, obs);
  if (is_collision) {
    return true;
  }

  is_collision = IsVehComponentCollision(&foot_print.mirror_left, obs);
  if (is_collision) {
    return true;
  }

  is_collision = IsVehComponentCollision(&foot_print.mirror_right, obs);
  if (is_collision) {
    return true;
  }

  return false;
}

bool ParkingStopDecider::GetPathCollisionByObstacle(int* collision_index,
                                                    const ApaObstacle& obs) {
  *collision_index = -1;

  bool is_collision = false;

  std::vector<PolygonFootPrint>* path_polygons;
  if (obs.GetObsMovementType() == ApaObsMovementType::STATIC) {
    path_polygons = &little_buffer_path_polygons_;
  } else {
    path_polygons = &big_buffer_path_polygons_;
  }

  int point_size = path_polygons->size();
  for (int i = 0; i < point_size; i++) {
    const PolygonFootPrint& footprint = path_polygons->at(i);

    if (IsVehCollision(footprint, obs)) {
      is_collision = true;
      *collision_index = i;
      break;
    }
  }

  return is_collision;
}

void ParkingStopDecider::ExtendPath(
    const double extend_length,
    std::vector<pnc::geometry_lib::PathPoint>& path) {
  if (path.size() <= 2) {
    return;
  }

  size_t path_point_size = path.size();
  Eigen::Vector2d path_end_global = path.back().pos;

  Eigen::Vector2d the_last_but_one = path[path_point_size - 2].pos;
  Eigen::Vector2d unit_line_vec =
      Eigen::Vector2d(path_end_global[0] - the_last_but_one[0],
                      path_end_global[1] - the_last_but_one[1]);
  if (unit_line_vec.norm() < 0.01) {
    return;
  }
  unit_line_vec.normalize();

  double extend_s = 0.05;
  double ds = 0.05;

  Eigen::Vector2d point;
  pnc::geometry_lib::PathPoint global_point;
  double phi = path.back().heading;
  double total_s = path.back().s;
  while (extend_s <= extend_length) {
    point = path_end_global + extend_s * unit_line_vec;
    global_point.Set(point, phi);
    global_point.kappa = 0.0;
    global_point.s = total_s + extend_s;

    path.emplace_back(global_point);

    extend_s += ds;
  }

  return;
}

void ParkingStopDecider::GeneratePathFootPrint(
    const std::vector<pnc::geometry_lib::PathPoint>& path) {
  double lon_buffer = 0.01;

  // generate veh local polygon
  PolygonFootPrint foot_print_little_buffer;
  PolygonFootPrint foot_print_big_buffer;
  GenerateVehCompactPolygon(config_.static_agent_lat_buffer, lon_buffer,
                            config_.static_agent_lat_buffer,
                            &foot_print_little_buffer);
  GenerateVehCompactPolygon(config_.dynamic_agent_lat_buffer, lon_buffer,
                            config_.dynamic_agent_lat_buffer,
                            &foot_print_big_buffer);

  little_buffer_path_polygons_.clear();
  big_buffer_path_polygons_.clear();
  PolygonFootPrint global_polygon;
  Pose2D global_pose;
  Transform2d tf;

  size_t path_end_id = path.size() - 1;
  for (size_t i = 0; i <= path_end_id; ++i) {
    global_pose.x = path[i].pos[0];
    global_pose.y = path[i].pos[1];
    global_pose.theta = path[i].heading;
    tf.SetBasePose(global_pose);

    FootPrintLocalToGlobal(tf, &foot_print_little_buffer, &global_polygon);
    little_buffer_path_polygons_.emplace_back(global_polygon);

    FootPrintLocalToGlobal(tf, &foot_print_big_buffer, &global_polygon);
    big_buffer_path_polygons_.emplace_back(global_polygon);
  }

  return;
}

bool ParkingStopDecider::GetOverlapBoundaryPoints(
    const std::vector<pnc::geometry_lib::PathPoint>& path,
    const ApaObstacle& obstacle, std::vector<STPoint>* upper_points,
    std::vector<STPoint>* lower_points) {
  if (path.empty()) {
    return false;
  }

  upper_points->clear();
  lower_points->clear();

  double end_s = path.back().s;
  double max_time = 7.0;

  // For those with no predicted trajectories, just map the obstacle's
  // current position to ST-graph and always assume it's static.
  int collision_index;
  bool is_collision = GetPathCollisionByObstacle(&collision_index, obstacle);

  if (is_collision && collision_index >= 0 && collision_index < path.size()) {
    const auto& curr_point = path[collision_index];
    double low_s = std::fmax(0.0, curr_point.s);
    double high_s = end_s;

    lower_points->emplace_back(low_s, 0.0);
    lower_points->emplace_back(low_s, max_time);
    upper_points->emplace_back(high_s, 0.0);
    upper_points->emplace_back(high_s, max_time);

    ILOG_INFO << "collision_index = " << collision_index << ", s = " << low_s;
  }

  return (lower_points->size() > 1 && upper_points->size() > 1);
}

void ParkingStopDecider::ComputeSTBoundary(
    std::vector<pnc::geometry_lib::PathPoint>& path, ApaObstacle& obstacle) {
  ParkLonDecision decision;
  decision.decision_type = LonDecisionType::NONE;

  std::vector<STPoint> lower_points;
  std::vector<STPoint> upper_points;

  if (!GetOverlapBoundaryPoints(path, obstacle, &upper_points, &lower_points)) {
    obstacle.SetLonDecision(decision);
    return;
  }

  auto boundary = STBoundary::CreateInstance(lower_points, upper_points);
  boundary.set_id(obstacle.GetId());

  decision.decision_speed = 0.0;
  decision.decision_type = LonDecisionType::STOP;
  if (obstacle.GetObsMovementType() == ApaObsMovementType::STATIC) {
    decision.lon_decision_buffer = config_.static_agent_lon_buffer;
    decision.reason_code = LonDecisionReason::STATIC_OCC_COLLISION;
  } else {
    decision.lon_decision_buffer = config_.dynamic_agent_lon_buffer;
    decision.reason_code = LonDecisionReason::DYNAMIC_OCC_COLLISION;
  }
  decision.path_s = lower_points.front().s();
  decision.perception_id = obstacle.GetId();

  // todo: generate emergency brake profile.
  double hard_brake_s = init_point_.v * init_point_.v / 2.0 / 2.0;
  decision.lon_decision_buffer =
      std::min(decision.lon_decision_buffer, decision.path_s - hard_brake_s);

  // if emergency brake, todo:
  if (decision.lon_decision_buffer < 0.05) {
    ILOG_INFO << "emergency brake, lon buffer is not safe, buffer = "
              << decision.lon_decision_buffer;

    decision.lon_decision_buffer = 0.05;
  }

  boundary.SetCharacteristicLength(decision.lon_decision_buffer);

  obstacle.SetPathSTBoundary(boundary);
  obstacle.SetLonDecision(decision);

  ILOG_INFO << "obs id = " << decision.perception_id
            << ",s = " << decision.path_s
            << ", type = " << static_cast<int>(obstacle.GetObsScemanticType());

  return;
}

void ParkingStopDecider::AddDecisionByObstacle(
    const std::vector<pnc::geometry_lib::PathPoint>& path) {
  // check
  if (obs_manager_ == nullptr || path.size() <= 0 ||
      obs_manager_->GetObstacles().size() == 0) {
    return;
  }

  std::vector<pnc::geometry_lib::PathPoint> extend_path = path;
  ExtendPath(0.3, extend_path);

  GeneratePathFootPrint(extend_path);

  // Go through every obstacle.
  double min_stop_s = std::numeric_limits<double>::max();
  ParkLonDecision stop_decision;
  stop_decision.Clear();

  for (auto& obj : obs_manager_->GetMutableObstacles()) {
    ApaObstacle& obstacle = obj.second;

    ComputeSTBoundary(extend_path, obstacle);

    const auto& decision = obstacle.LongitudinalDecision();
    if (decision.decision_type == LonDecisionType::STOP) {
      // Store the stop fence info by in-path obstacle.
      if (decision.path_s - decision.lon_decision_buffer < min_stop_s) {
        stop_obstacle_ = &obstacle;
        min_stop_s = decision.path_s - decision.lon_decision_buffer;
        stop_decision = decision;
      }
    }
  }

  if (stop_decision.decision_type == LonDecisionType::STOP &&
      min_stop_s < stop_decision_.path_s - stop_decision_.lon_decision_buffer) {
    stop_decision_ = stop_decision;
  }

  return;
}

void ParkingStopDecider::Execute(
    const SVPoint& init_point,
    const std::vector<pnc::geometry_lib::PathPoint>& path) {
  // init
  stop_obstacle_ = nullptr;
  stop_decision_.Clear();
  init_point_ = init_point;
  config_.Init();

  AddDecisionByPathTargetPoint(path);

  AddDecisionByObstacle(path);

  // AddStopDecisionByControlPath(path, 20.0, speed_decisions);

#if DECIDER_DEBUG
  TaskDebug();
#endif

  return;
}

void ParkingStopDecider::AddDecisionByPathTargetPoint(
    const std::vector<pnc::geometry_lib::PathPoint>& path) {
  if (path.empty()) {
    return;
  }

  stop_decision_.decision_type = LonDecisionType::STOP;

  // add stop decision by path finish
  auto& point = path.back();

  stop_decision_.path_interaction_point.x = point.pos.x();
  stop_decision_.path_interaction_point.y = point.pos.y();
  stop_decision_.path_interaction_point.theta = point.heading;
  stop_decision_.reason_code = LonDecisionReason::PATH_FINISH;
  stop_decision_.path_s = point.s;
  stop_decision_.lon_decision_buffer = 0.0;

  return;
}

void ParkingStopDecider::AddStopDecisionByControlPath(
    const std::vector<pnc::geometry_lib::PathPoint>& path,
    const double stop_fence_s, SpeedDecisions* speed_decisions) {
  if (path.empty()) {
    return;
  }
  ParkLonDecision stop_decision;
  stop_decision.reason_code = LonDecisionReason::CONTROL_PATH_COLLISION;
  stop_decision.decision_type = LonDecisionType::STOP;
  stop_decision.path_s = stop_fence_s;

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

  stop_decision.path_interaction_point.x = point.pos.x();
  stop_decision.path_interaction_point.y = point.pos.y();
  stop_decision.path_interaction_point.theta = point.heading;
  stop_decision.lon_decision_buffer = 0.2;

  speed_decisions->decisions.emplace_back(stop_decision);

  return;
}

void ParkingStopDecider::TaskDebug() {
  ILOG_INFO << "stop decision, reason = "
            << static_cast<int>(stop_decision_.reason_code)
            << ", lon buffer = " << stop_decision_.lon_decision_buffer
            << ", s = " << stop_decision_.path_s
            << ", id = " << stop_decision_.perception_id;

  return;
}

void ParkingStopDecider::AddStopDecisionByDistance(
    const double stop_s, const LonDecisionReason decision_reason) {
  ParkLonDecision stop_decision;
  if (decision_reason == LonDecisionReason::STATIC_OCC_COLLISION) {
    stop_decision.lon_decision_buffer = config_.static_agent_lon_buffer;
  } else if (decision_reason == LonDecisionReason::DYNAMIC_OCC_COLLISION) {
    stop_decision.lon_decision_buffer = config_.dynamic_agent_lon_buffer;
  } else {
    stop_decision.lon_decision_buffer = 0;
  }

  stop_decision.reason_code = decision_reason;
  stop_decision.decision_type = LonDecisionType::STOP;
  stop_decision.path_s = stop_s;

  if (stop_decision.decision_type == LonDecisionType::STOP &&
      stop_decision.path_s - stop_decision.lon_decision_buffer <
          stop_decision_.path_s - stop_decision_.lon_decision_buffer) {
    stop_decision_ = stop_decision;
  }

  return;
}

}  // namespace apa_planner
}  // namespace planning