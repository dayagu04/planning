#include "parking_stop_decider.h"

#include <algorithm>
#include <cstddef>
#include <vector>

#include "apa_debug_data.pb.h"
#include "apa_obstacle.h"
#include "collision_detection/path_safe_checker.h"
#include "debug_info_log.h"
#include "log_glog.h"
#include "obstacle_manager.h"
#include "path/discretized_path.h"
#include "pose2d.h"
#include "speed/apa_speed_decision.h"
#include "task_basic_types.h"
#include "vec2d.h"

namespace planning {
namespace apa_planner {

#define DECIDER_DEBUG (1)

void ParkingStopDecider::Execute(
    const SVPoint& init_point,
    const std::vector<pnc::geometry_lib::PathPoint>& lateral_path,
    const std::vector<pnc::geometry_lib::PathPoint>& control_path,
    const pnc::geometry_lib::PathSegGear gear) {
  // init
  stop_obstacle_ = nullptr;
  stop_decision_.Clear();
  init_point_ = init_point;
  gear_ = gear;
  config_.Init();

  AddDecisionByPathTargetPoint(lateral_path);

  // todo: use lateral path and control path
  if (!apa_param.GetParam().speed_config.use_remain_dist) {
    AddDecisionByObstacle(control_path, false);
    RecordDebugInfo(lateral_path);
  }

#if DECIDER_DEBUG
  TaskDebug();
#endif

  return;
}

void ParkingStopDecider::RecordDebugInfo(
    const std::vector<pnc::geometry_lib::PathPoint>& lateral_path) {
  auto& debug = DebugInfoManager::GetInstance().GetDebugInfoPb();
  common::ApaSpeedDebug* speed_debug = debug->mutable_apa_speed_debug();
  speed_debug->clear_stop_signs();

  DiscretizedPath path_data;
  planning_math::PathPoint path_point;
  for (size_t i = 0; i < lateral_path.size(); ++i) {
    path_point.set_x(lateral_path[i].pos.x());
    path_point.set_y(lateral_path[i].pos.y());
    path_point.set_theta(lateral_path[i].heading);
    path_point.set_s(lateral_path[i].s);
    path_point.set_kappa(lateral_path[i].kappa);

    path_data.push_back(path_point);
  }

  if (stop_decision_.decision_type == LonDecisionType::STOP) {
    double s = stop_decision_.path_s - stop_decision_.lon_decision_buffer;
    path_point = path_data.Evaluate(s);

    planning_math::Vec2d base(path_point.x(), path_point.y());
    planning_math::Vec2d vec =
        planning_math::Vec2d::CreateUnitVec2d(path_point.theta());
    planning_math::Vec2d stop_sign_pose;

    const auto& config = apa_param.GetParam();
    if (gear_ == pnc::geometry_lib::PathSegGear::SEG_GEAR_REVERSE) {
      stop_sign_pose = base - vec * config.rear_overhanging;
    } else {
      stop_sign_pose =
          base + vec * (config.front_overhanging + config.wheel_base);
    }

    common::ApaStopSign stop_sign;
    stop_sign.mutable_stop_pose()->set_x(stop_sign_pose.x());
    stop_sign.mutable_stop_pose()->set_y(stop_sign_pose.y());
    stop_sign.mutable_stop_pose()->set_theta(path_point.theta());

    speed_debug->add_stop_signs()->CopyFrom(stop_sign);
  }

  ILOG_INFO << "stop sign size = " << speed_debug->stop_signs_size();

  return;
}

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
    gjk_interface_.PolygonPointCollisionDetect(polygon, points[j],
                                               &is_collision);

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
    path_polygons = &small_buffer_path_polygons_;
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
  GenerateVehCompactPolygon(config_.lat_buffer_to_static_agent, lon_buffer,
                            config_.lat_buffer_to_static_agent,
                            &foot_print_little_buffer);
  GenerateVehCompactPolygon(config_.lat_buffer_to_dynamic_agent, lon_buffer,
                            config_.lat_buffer_to_dynamic_agent,
                            &foot_print_big_buffer);

  small_buffer_path_polygons_.clear();
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
    small_buffer_path_polygons_.emplace_back(global_polygon);

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

  // For those with no predicted trajectories, just map the obstacle's
  // current position to ST-graph and always assume it's static.
  if (obstacle.GetObsAttributeType() != ApaObsAttributeType::FUSION_POLYGON) {
    double max_time = 7.0;
    int collision_index = 0;
    bool is_collision = GetPathCollisionByObstacle(&collision_index, obstacle);

    if (is_collision && collision_index >= 0 && collision_index < path.size()) {
      collision_index -= 1;
      collision_index = std::max(0, collision_index);

      const auto& curr_point = path[collision_index];
      double low_s = std::fmax(0.0, curr_point.s);
      double high_s = end_s;

      lower_points->emplace_back(low_s, 0.0);
      lower_points->emplace_back(low_s, max_time);
      upper_points->emplace_back(high_s, 0.0);
      upper_points->emplace_back(high_s, max_time);

      ILOG_INFO << "collision_index = " << collision_index << ", s = " << low_s;
    }
  } else {
    Polygon2D global_polygon = obstacle.GetPolygon2DGlobal();
    const pnc::geometry_lib::PathPoint&  center=obstacle. GetCenterPose();
    Transform2d tf(Pose2D(center.pos[0], center.pos[1], center.heading));
    Polygon2D local_polygon;
    GlobalPolygonToULFLocal(&global_polygon, tf, &local_polygon);

    bool is_collision;
    int first_collision_index;
    int end_collision_index;

    const trajectory::Trajectory& traj = obstacle.GetPredictTraj();

    // 2. Go through every point of the predicted obstacle trajectory.
    for (int i = 0; i < traj.size(); ++i) {
      const trajectory::TrajectoryPoint& trajectory_point = traj[i];
      double trajectory_point_time = trajectory_point.absolute_time();
      if (trajectory_point_time  > 7.0) {
        continue;
      }

      tf.SetBasePose(Pose2D(trajectory_point.x(), trajectory_point.y(),
                            trajectory_point.theta()));
      ULFLocalPolygonToGlobal(&global_polygon, &local_polygon, tf);

      is_collision = CheckCollisionByODObject(
          big_buffer_path_polygons_, global_polygon, &first_collision_index,
          &end_collision_index);

      if (is_collision) {
        double low_s = std::fmax(0.0, path[first_collision_index].s);
        double high_s = std::fmin(end_s, path[end_collision_index].s);

        lower_points->emplace_back(low_s,  trajectory_point_time);
        upper_points->emplace_back(high_s, trajectory_point_time);
      }
    }
  }

  return (lower_points->size() > 1 && upper_points->size() > 1);
}

void ParkingStopDecider::ComputeSTBoundary(
    std::vector<pnc::geometry_lib::PathPoint>& path, ApaObstacle& obstacle) {
  ParkLonDecision decision;
  decision.decision_type = LonDecisionType::NONE;

  std::vector<STPoint> lower_points;
  std::vector<STPoint> upper_points;

  // compute od,occ,ground line st boundary.
  if (!GetOverlapBoundaryPoints(path, obstacle, &upper_points, &lower_points)) {
    obstacle.SetLonDecision(decision);
    return;
  }

  auto boundary = STBoundary::CreateInstance(lower_points, upper_points);
  boundary.set_id(obstacle.GetId());

  decision.decision_speed = 0.0;
  decision.decision_type = LonDecisionType::STOP;
  if (obstacle.GetObsMovementType() == ApaObsMovementType::STATIC) {
    decision.lon_decision_buffer = config_.lon_buffer_to_static_agent;
    decision.reason_code = LonDecisionReason::STATIC_OCC_COLLISION;
  } else {
    decision.lon_decision_buffer = config_.lon_buffer_to_dynamic_agent;
    decision.reason_code = LonDecisionReason::DYNAMIC_OCC_COLLISION;
  }
  decision.path_s = boundary.min_s();
  decision.perception_id = obstacle.GetId();

  // todo: generate emergency brake profile.
  double hard_brake_s = init_point_.v * init_point_.v / 2.0 / 2.0;
  decision.lon_decision_buffer =
      std::min(decision.lon_decision_buffer, decision.path_s - hard_brake_s);

  // if emergency brake, todo:
  if (decision.lon_decision_buffer < config_.min_lon_buffer) {
    ILOG_INFO << "emergency brake, lon buffer is not safe, buffer = "
              << decision.lon_decision_buffer;

    decision.lon_decision_buffer = config_.min_lon_buffer;
  }

  boundary.SetCharacteristicLength(decision.lon_decision_buffer);

  obstacle.SetPathSTBoundary(boundary);
  obstacle.SetLonDecision(decision);

  ILOG_INFO << "obs id = " << decision.perception_id
            << ",s = " << decision.path_s
            << ", type = " << static_cast<int>(obstacle.GetObsScemanticType());

  if (obstacle.GetObsAttributeType() == ApaObsAttributeType::FUSION_POLYGON) {
    boundary.DebugString();
  }

  return;
}

void ParkingStopDecider::AddDecisionByObstacle(
    const std::vector<pnc::geometry_lib::PathPoint>& path,
    const bool check_extend_path) {
  // check
  if (obs_manager_ == nullptr || path.size() <= 0 ||
      obs_manager_->GetObstacles().size() == 0) {
    return;
  }

  std::vector<pnc::geometry_lib::PathPoint> extend_path = path;
  if (check_extend_path) {
    ExtendPath(config_.extra_check_dist, extend_path);
  }

  GeneratePathFootPrint(extend_path);

  // Go through every obstacle.
  double min_stop_s =
      stop_decision_.path_s - stop_decision_.lon_decision_buffer;

  for (auto& obj : obs_manager_->GetMutableObstacles()) {
    ApaObstacle& obstacle = obj.second;

    ComputeSTBoundary(extend_path, obstacle);

    const auto& decision = obstacle.LongitudinalDecision();
    if (decision.decision_type == LonDecisionType::STOP) {
      // Store the stop fence info by in-path obstacle.
      if (decision.path_s - decision.lon_decision_buffer < min_stop_s) {
        stop_obstacle_ = &obstacle;
        min_stop_s = decision.path_s - decision.lon_decision_buffer;
        stop_decision_ = decision;
      }
    }
  }

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

  terminal_decision_ = stop_decision_;

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
            << ", id = " << stop_decision_.perception_id
            << ", terminal s = " << terminal_decision_.path_s;

  return;
}

void ParkingStopDecider::AddStopDecisionByDistance(
    const double stop_s, const LonDecisionReason decision_reason,
    const std::vector<pnc::geometry_lib::PathPoint>& lateral_path) {
  ParkLonDecision stop_decision;
  if (decision_reason == LonDecisionReason::STATIC_OCC_COLLISION) {
    stop_decision.lon_decision_buffer = config_.lon_buffer_to_static_agent;
  } else if (decision_reason == LonDecisionReason::DYNAMIC_OCC_COLLISION) {
    stop_decision.lon_decision_buffer = config_.lon_buffer_to_dynamic_agent;
  } else {
    stop_decision.lon_decision_buffer = 0;
  }

  stop_decision.reason_code = decision_reason;
  stop_decision.decision_type = LonDecisionType::STOP;
  stop_decision.path_s = std::max(0.0, stop_s);

  if (stop_decision.path_s - stop_decision.lon_decision_buffer <
      stop_decision_.path_s - stop_decision_.lon_decision_buffer) {
    stop_decision_ = stop_decision;
  }

  // todo: need delete
  RecordDebugInfo(lateral_path);

#if DECIDER_DEBUG
  TaskDebug();
#endif

  return;
}

void ParkingStopDecider::PathDebug(
    const std::vector<pnc::geometry_lib::PathPoint>& path) {
  for (auto& point : path) {
    ILOG_INFO << "s = " << point.s << ",x = " << point.pos.x()
              << ", y = " << point.pos.y() << ", theta = " << point.heading;
  }
  return;
}

const double ParkingStopDecider::GetStopDecisionS() {
  // sanity check
  double total_s = 20.0;
  if (stop_decision_.decision_type == LonDecisionType::STOP) {
    total_s = std::min(
        total_s, stop_decision_.path_s - stop_decision_.lon_decision_buffer);
  }

  return total_s;
}

const double ParkingStopDecider::GetTerminalS() {
  return terminal_decision_.path_s;
}

bool ParkingStopDecider::IsVehCollisionByOD(const PolygonFootPrint& foot_print,
                                            const Polygon2D& obs) {
  bool is_collision = false;
  gjk_interface_.PolygonCollisionByCircleCheck(
      &is_collision, &foot_print.max_polygon, &obs, 0.1);

  return is_collision;
}

bool ParkingStopDecider::CheckCollisionByODObject(
    const std::vector<PolygonFootPrint>& polygon_path,
    const Polygon2D& obs_polygon, int* start_collision_index,
    int* end_collision_index) {
  *start_collision_index = -1;
  *end_collision_index = -1;

  bool is_collision = false;
  bool find_collision = false;

  int point_size = polygon_path.size();
  for (int i = 0; i < point_size; i++) {
    const PolygonFootPrint& point_polygon = polygon_path[i];
    is_collision = IsVehCollisionByOD(point_polygon, obs_polygon);

    if (is_collision) {
      // find collision
      if (!find_collision) {
        find_collision = true;
        *start_collision_index = i;
      }

      // check ego path
      *end_collision_index = i;
    }
  }

  return find_collision;
}

}  // namespace apa_planner
}  // namespace planning