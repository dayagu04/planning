#include "path_safe_checker.h"

#include <cstddef>

#include "ad_common/math/vec2d.h"
#include "log_glog.h"
#include "polygon_base.h"
#include "pose2d.h"

namespace planning {
#define DEBUG_PATH_CHECKER (0)
#define NOT_CARE_SAFE_DIST (0.5)

void PathSafeChecker::Excute(const Pose2D& ego_pose,
                             const PathCheckRequest requst,
                             const double lat_buffer, const double lon_buffer,
                             std::vector<pnc::geometry_lib::PathPoint>& path) {
  lat_buffer_ = lat_buffer;
  lon_buffer_ = lon_buffer;
  ego_project_s_ = 0;

  if (requst == PathCheckRequest::COLLISION_CHECK) {
    ExcuteCollisionCheck(path, ego_pose);
  } else if (requst == PathCheckRequest::DISTANCE_CHECK) {
    ExcuteDistanceCheck(ego_pose, path);
  }

  return;
}

void PathSafeChecker::ExcuteDistanceCheck(
    const Pose2D& ego_pose, std::vector<pnc::geometry_lib::PathPoint>& path) {
  // init
  is_path_collision_ = false;
  is_ego_collision_ = false;

  for (auto& point : path) {
    point.col_flag = false;
    point.dist_to_obs = 20.0;
  }

  // check
  if (obs_manager_ == nullptr || path.size() <= 0) {
    return;
  }

  if (obs_manager_->GetObstacles().size() == 0) {
    return;
  }

  // get closest point
  path_nearest_idx_ = GetNearestPathPoint(path, ego_pose);
  Pose2D global_pose;
  bool is_collision = false;
  Transform2d tf;

  global_pose = ego_pose;
  tf.SetBasePose(global_pose);
  VehCollisionPosition collision_component = VehCollisionPosition::NONE;

  // generate veh local polygon
  GenerateVehCompactPolygon(lat_buffer_, lon_buffer_, lat_buffer_,
                            &polygon_foot_print_);

  // check path
  size_t collision_index = 100000;
  size_t path_end_id = path.size() - 1;
  double dist = 10;
  for (size_t i = path_nearest_idx_; i <= path_end_id; ++i) {
    if (!is_collision) {
      global_pose.x = path[i].pos[0];
      global_pose.y = path[i].pos[1];
      global_pose.theta = path[i].heading;
      tf.SetBasePose(global_pose);

      dist = GetVehicleDistance(tf, &polygon_foot_print_, &collision_component);

      if (dist < 1e-3) {
        is_collision = true;
        collision_index = i;

        if (i == path_nearest_idx_) {
          is_ego_collision_ = true;
        }
      }
    }

    if (is_collision) {
      path[i].col_flag = true;
      path[i].dist_to_obs = 0;
    } else {
      path[i].col_flag = false;
      // 这里的距离包含buffer.
      path[i].dist_to_obs = dist;
    }

#if DEBUG_PATH_CHECKER
    ILOG_INFO << "id = " << i
              << ", collision info = " << static_cast<int>(collision_component);
#endif
  }

  path_collision_idx_ = std::min(collision_index, path.size());
  is_path_collision_ = is_collision;

#if DEBUG_PATH_CHECKER
  DebugCollisionInfo(path_end_id, collision_component, ego_pose, path);
#endif

  return;
}

void PathSafeChecker::ExcuteCollisionCheck(
    const std::vector<pnc::geometry_lib::PathPoint>& path,
    const Pose2D& ego_pose) {
  is_path_collision_ = false;
  is_ego_collision_ = false;

  if (obs_manager_ == nullptr || path.size() <= 0) {
    return;
  }

  if (obs_manager_->GetObstacles().size() == 0) {
    return;
  }

  // get closest point
  path_nearest_idx_ = GetNearestPathPoint(path, ego_pose);
  Pose2D global_pose;
  bool is_collision = false;
  Transform2d tf;

  global_pose = ego_pose;
  tf.SetBasePose(global_pose);
  VehCollisionPosition collision_component = VehCollisionPosition::NONE;

  // generate veh local polygon
  GenerateVehCompactPolygon(lat_buffer_, lon_buffer_, lat_buffer_,
                            &polygon_foot_print_);

  // check path
  size_t collision_index = 100000;
  cdl::AABB path_point_aabb;
  size_t path_end_id = path.size() - 1;
  for (size_t i = path_nearest_idx_; i <= path_end_id; ++i) {
    global_pose.x = path[i].pos[0];
    global_pose.y = path[i].pos[1];
    global_pose.theta = path[i].heading;
    tf.SetBasePose(global_pose);

    is_collision =
        IsVehicleCollision(tf, &polygon_foot_print_, &collision_component);

    if (i == path_nearest_idx_ && is_collision) {
      is_ego_collision_ = true;
    }

#if DEBUG_PATH_CHECKER
    ILOG_INFO << "id = " << i
              << ", collision info = " << static_cast<int>(collision_component);
#endif

    if (!is_collision) {
      continue;
    } else {
      collision_index = i;
      break;
    }
  }

  path_collision_idx_ = std::min(collision_index, path.size());
  is_path_collision_ = is_collision;

#if DEBUG_PATH_CHECKER
  DebugCollisionInfo(path_end_id, collision_component, ego_pose, path);
#endif

  return;
}

void PathSafeChecker::GenerateVehBox(const double lateral_safe_buffer,
                                     const double lon_safe_buffer,
                                     const double max_bbox_lat_buffer) {
  const apa_planner::ApaParameters& config = apa_param.GetParam();

  GetUpLeftCoordinatePolygonByParam(
      &polygon_foot_print_.body, config.rear_overhanging + lon_safe_buffer,
      config.wheel_base + config.front_overhanging + lon_safe_buffer,
      config.car_width / 2.0 + lateral_safe_buffer);

  // left mirror
  Position2D center;
  center.x = config.footprint_circle_x[6];
  center.y = config.footprint_circle_y[6] + lateral_safe_buffer;
  double radius = std::fabs(config.footprint_circle_r[6]);
  GenerateMirrorPolygon(&polygon_foot_print_.mirror_left, 0.3, radius * 2,
                        center);

  // right mirror
  center.x = config.footprint_circle_x[3];
  center.y = config.footprint_circle_y[3] - lateral_safe_buffer;
  GenerateMirrorPolygon(&polygon_foot_print_.mirror_right, 0.3, radius * 2,
                        center);

  GetUpLeftCoordinatePolygonByParam(
      &polygon_foot_print_.max_polygon,
      config.rear_overhanging + lon_safe_buffer,
      config.wheel_base + config.front_overhanging + lon_safe_buffer,
      config.max_car_width / 2.0 + max_bbox_lat_buffer);

  return;
}

void PathSafeChecker::UpdatePathValidDist(
    const std::vector<pnc::geometry_lib::PathPoint>& path,
    const Pose2D& ego_pose) {
  if (!is_path_collision_) {
    return;
  }

  path_valid_dist_ = 0.0;
  if (path.size() <= 0) {
    return;
  }

  if (path_nearest_idx_ >= path.size()) {
    return;
  }

  // ILOG_INFO << "path size = " << path.size();

  double path_dist = 0.0;
  double dist;
  Pose2D pose;
  Pose2D next_pose;
  for (size_t i = path_nearest_idx_; i < path_collision_idx_; i++) {
    pose.x = path[i].pos[0];
    pose.y = path[i].pos[1];
    pose.theta = path[i].heading;

    if (i + 1 >= path.size()) {
      break;
    }
    next_pose.x = path[i + 1].pos[0];
    next_pose.y = path[i + 1].pos[1];
    next_pose.theta = path[i + 1].heading;

    dist = pose.DistanceTo(next_pose);

    path_dist += dist;
  }

  // ego projection
  ad_common::math::Vec2d path_line(
      path[path_nearest_idx_ + 1].pos[0] - path[path_nearest_idx_].pos[0],
      path[path_nearest_idx_ + 1].pos[1] - path[path_nearest_idx_].pos[1]);

  if (path_line.Length() > 0.01) {
    path_line.Normalize();

    ad_common::math::Vec2d ego_vector(
        ego_pose.x - path[path_nearest_idx_].pos[0],
        ego_pose.y - path[path_nearest_idx_].pos[1]);

    double projection_dist = path_line.InnerProd(ego_vector);

    path_dist -= projection_dist;
  }

  path_valid_dist_ = path_dist;

  return;
}

size_t PathSafeChecker::GetNearestPathPoint(
    const std::vector<pnc::geometry_lib::PathPoint>& path, const Pose2D& pose) {
  double nearest_dist = 1000000.0;
  double dist;
  Pose2D global_pose;
  size_t nearest_id = 0;

  if (path.size() <= 1) {
    return 0;
  }

  for (size_t i = 0; i < path.size(); i++) {
    global_pose.x = path[i].pos[0];
    global_pose.y = path[i].pos[1];
    global_pose.theta = path[i].heading;
    dist = pose.DistanceSquareTo(&global_pose);

    if (i == 0) {
      nearest_dist = dist;
      nearest_id = i;

      continue;
    }

    if (dist < nearest_dist) {
      nearest_dist = dist;
      nearest_id = i;
    }

#if DEBUG_PATH_CHECKER
    ILOG_INFO << " i = " << i << ",dist = " << dist
              << " ,nearest_dist= " << nearest_dist;
#endif
  }

  GenerateEgoS(nearest_id, path, pose);

  return nearest_id;
}

const bool PathSafeChecker::IsPolygonCollision(const Polygon2D* car) {
  bool is_collision = false;
  if (obs_manager_ == nullptr) {
    return is_collision;
  }

  for (const auto& pair : obs_manager_->GetObstacles()) {
    // envelop box check
    gjk_interface_.PolygonCollisionByCircleCheck(
        &is_collision, &pair.second.GetPolygon2DGlobal(), car, 0.01);

    if (!is_collision) {
      // ILOG_INFO << "size = " << obstacle.points.size() << " box no
      // collision";
      continue;
    }

    // internal points
    const std::vector<Eigen::Vector2d>& points =
        pair.second.GetPtClout2dGlobal();

    for (size_t j = 0; j < points.size(); j++) {
      gjk_interface_.PolygonPointCollisionDetect(
          car, Eigen::Vector2f(points[j][0], points[j][1]), &is_collision);

      if (is_collision) {
        // ILOG_INFO << "size = " << obstacle.points.size() << " j =" << j;
        return true;
      }

      // ILOG_INFO << "point no collision";
    }
  }

  return false;
}

void PathSafeChecker::GenerateMirrorPolygon(Polygon2D* box,
                                            const double x_length,
                                            const double y_length,
                                            const Position2D& center) {
  box->vertexes[0].x = center.x + x_length / 2;
  box->vertexes[0].y = center.y - y_length / 2;

  box->vertexes[1].x = center.x + x_length / 2;
  box->vertexes[1].y = center.y + y_length / 2;

  box->vertexes[2].x = center.x - x_length / 2;
  box->vertexes[2].y = center.y + y_length / 2;

  box->vertexes[3].x = center.x - x_length / 2;
  box->vertexes[3].y = center.y - y_length / 2;

  box->vertex_num = 4;

  box->shape = PolygonShape::box;
  UpdatePolygonValue(box, NULL, 0, false, POLYGON_MAX_RADIUS);

  box->min_tangent_radius = std::min(y_length / 2, x_length / 2);

  return;
}

void PathSafeChecker::GetCompactCarPolygonByParam(Polygon2D* box,
                                                  const double lat_buffer,
                                                  const double lon_buffer) {
  const apa_planner::ApaParameters& config = apa_param.GetParam();

  box->vertexes[0].x = config.car_vertex_x_vec[2] + lon_buffer;
  box->vertexes[0].y = config.car_vertex_y_vec[2] + lat_buffer;

  box->vertexes[1].x = config.car_vertex_x_vec[0] + lon_buffer;
  box->vertexes[1].y = config.car_vertex_y_vec[0] + lat_buffer;

  box->vertexes[2].x = config.car_vertex_x_vec[15] - lon_buffer;
  box->vertexes[2].y = config.car_vertex_y_vec[15] + lat_buffer;

  box->vertexes[3].x = config.car_vertex_x_vec[13] - lon_buffer;
  box->vertexes[3].y = config.car_vertex_y_vec[13] + lat_buffer;

  box->vertexes[4].x = config.car_vertex_x_vec[12] - lon_buffer;
  box->vertexes[4].y = config.car_vertex_y_vec[12] - lat_buffer;

  box->vertexes[5].x = config.car_vertex_x_vec[10] - lon_buffer;
  box->vertexes[5].y = config.car_vertex_y_vec[10] - lat_buffer;

  box->vertexes[6].x = config.car_vertex_x_vec[5] + lon_buffer;
  box->vertexes[6].y = config.car_vertex_y_vec[5] - lat_buffer;

  box->vertexes[7].x = config.car_vertex_x_vec[3] + lon_buffer;
  box->vertexes[7].y = config.car_vertex_y_vec[3] - lat_buffer;

  box->vertex_num = 8;

  box->shape = PolygonShape::multi_edge;
  UpdatePolygonValue(box, NULL, 0, false, POLYGON_MAX_RADIUS);

  box->min_tangent_radius = config.car_width / 2 + lat_buffer;

  return;
}

const bool PathSafeChecker::IsVehicleCollision(
    const Transform2d& tf, PolygonFootPrint* foot_print,
    VehCollisionPosition* collision_info) {
  Polygon2D veh_global_polygon;
  bool is_collision = false;
  ULFLocalPolygonToGlobal(&veh_global_polygon, &foot_print->max_polygon, tf);

  is_collision = IsPolygonCollision(&veh_global_polygon);
  if (!is_collision) {
    *collision_info = VehCollisionPosition::MAX_POLYGON_NO_COLLISION;
    return false;
  }

  ULFLocalPolygonToGlobal(&veh_global_polygon, &foot_print->body, tf);

  is_collision = IsPolygonCollision(&veh_global_polygon);
  if (is_collision) {
    *collision_info = VehCollisionPosition::BODY;
    return true;
  }

  ULFLocalPolygonToGlobal(&veh_global_polygon, &foot_print->mirror_left, tf);

  // ILOG_INFO << "left mirror check";
  is_collision = IsPolygonCollision(&veh_global_polygon);
  if (is_collision) {
    *collision_info = VehCollisionPosition::LEFT_MIRROR;
    return true;
  }

  ULFLocalPolygonToGlobal(&veh_global_polygon, &foot_print->mirror_right, tf);

  is_collision = IsPolygonCollision(&veh_global_polygon);
  if (is_collision) {
    *collision_info = VehCollisionPosition::RIGHT_MIRROR;
    return true;
  }

  *collision_info = VehCollisionPosition::NONE;
  return false;
}

bool PathSafeChecker::CalcEgoCollision(const Pose2D& ego_pose,
                                       const double lat_buffer,
                                       const double lon_buffer) {
  is_path_collision_ = false;
  is_ego_collision_ = false;

  if (obs_manager_ == nullptr) {
    return false;
  }

  if (obs_manager_->GetObstacles().size() == 0) {
    return false;
  }

  bool is_collision = false;
  Transform2d tf;
  tf.SetBasePose(ego_pose);

  VehCollisionPosition collision_component = VehCollisionPosition::NONE;

  // generate veh local polygon
  GenerateVehBox(lat_buffer, lon_buffer, lat_buffer);

  is_collision =
      IsVehicleCollision(tf, &polygon_foot_print_, &collision_component);

  return is_collision;
}

void PathSafeChecker::DebugCollisionInfo(
    const size_t path_end_id, const VehCollisionPosition collision_component,
    const Pose2D& ego_pose,
    const std::vector<pnc::geometry_lib::PathPoint>& path) const {
  ILOG_INFO << "path_collision_idx_= " << path_collision_idx_
            << " ,is_path_collision_= " << is_path_collision_
            << " ,path_nearest_idx_= " << path_nearest_idx_
            << " ,path_end_id= " << path_end_id
            << " , component=" << static_cast<int>(collision_component)
            << ",lat safe buffer = " << lat_buffer_;

  ego_pose.DebugString();

  PolygonDebugString(&polygon_foot_print_.body, "body");
  PolygonDebugString(&polygon_foot_print_.max_polygon, "max bounding box");
  PolygonDebugString(&polygon_foot_print_.mirror_left, "left mirror");

  Pose2D tmp;
  for (size_t i = 0; i < path.size(); i++) {
    tmp = Pose2D(path[i].pos[0], path[i].pos[1], path[i].heading);

    ILOG_INFO << "id = " << i;
    tmp.DebugString();
  }

  return;
}

const double PathSafeChecker::GetVehicleDistance(
    const Transform2d& tf, PolygonFootPrint* foot_print,
    VehCollisionPosition* collision_info) {
  Polygon2D veh_global_polygon;
  bool is_collision = false;
  double min_dist = 10.0;

  ULFLocalPolygonToGlobal(&veh_global_polygon, &foot_print->max_polygon, tf);
  is_collision = GetPolygonDistance(&veh_global_polygon, &min_dist);
  if (!is_collision) {
    *collision_info = VehCollisionPosition::MAX_POLYGON_NO_COLLISION;

    if (min_dist >= NOT_CARE_SAFE_DIST) {
      return min_dist;
    }
  }

  ULFLocalPolygonToGlobal(&veh_global_polygon, &foot_print->body, tf);
  is_collision = GetPolygonDistance(&veh_global_polygon, &min_dist);
  if (is_collision) {
    *collision_info = VehCollisionPosition::BODY;
    return 0;
  }

  // ILOG_INFO << "left mirror check";
  ULFLocalPolygonToGlobal(&veh_global_polygon, &foot_print->mirror_left, tf);
  is_collision = GetPolygonDistance(&veh_global_polygon, &min_dist);
  if (is_collision) {
    *collision_info = VehCollisionPosition::LEFT_MIRROR;
    return 0;
  }

  ULFLocalPolygonToGlobal(&veh_global_polygon, &foot_print->mirror_right, tf);
  is_collision = GetPolygonDistance(&veh_global_polygon, &min_dist);
  if (is_collision) {
    *collision_info = VehCollisionPosition::RIGHT_MIRROR;
    return 0;
  }

  *collision_info = VehCollisionPosition::NONE;
  return min_dist;
}

const bool PathSafeChecker::GetPolygonDistance(const Polygon2D* polygon,
                                               double* min_dist) {
  bool is_collision = false;
  double dist = 10;

  for (const auto& pair : obs_manager_->GetObstacles()) {
    // envelop box check
    gjk_interface_.PolygonDistanceByThresh(
        &is_collision, &dist, polygon, &pair.second.GetPolygon2DGlobal(), 1.0);

    // distance is big, no need accurate check.
    if (dist >= 1.0) {
      *min_dist = std::min(*min_dist, dist);
      continue;
    }

    // internal points
    const std::vector<Eigen::Vector2d>& points =
        pair.second.GetPtClout2dGlobal();
    for (size_t j = 0; j < points.size(); j++) {
      gjk_interface_.PolygonDistanceByThresh(
          polygon, Position2D(points[j][0], points[j][1]), 1.0, &is_collision,
          &dist);

      if (is_collision) {
        // ILOG_INFO << "size = " << points.size() << " j =" << j;
        return true;
      }

      *min_dist = std::min(*min_dist, dist);

      // ILOG_INFO << "point no collision";
    }
  }

  return is_collision;
}

void PathSafeChecker::GenerateEgoS(
    const size_t nearest_id,
    const std::vector<pnc::geometry_lib::PathPoint>& path, const Pose2D& pose) {
  ad_common::math::Vec2d p0;
  ad_common::math::Vec2d p1;
  double p0_s;

  if (nearest_id == path.size() - 1) {
    p0.set_x(path[nearest_id - 1].pos[0]);
    p0.set_y(path[nearest_id - 1].pos[1]);
    p0_s = path[nearest_id - 1].s;

    p1.set_x(path[nearest_id].pos[0]);
    p1.set_y(path[nearest_id].pos[1]);
  } else {
    p0.set_x(path[nearest_id].pos[0]);
    p0.set_y(path[nearest_id].pos[1]);
    p0_s = path[nearest_id].s;

    p1.set_x(path[nearest_id + 1].pos[0]);
    p1.set_y(path[nearest_id + 1].pos[1]);
  }
  ad_common::math::Vec2d p0_p1(p1.x() - p0.x(), p1.y() - p0.y());
  p0_p1.Normalize();

  ad_common::math::Vec2d p0_ego;
  p0_ego.set_x(pose.x - p0.x());
  p0_ego.set_x(pose.y - p0.y());

  double project_s = p0_p1.InnerProd(p0_ego);
  ego_project_s_ = p0_s + project_s;

  return;
}

}  // namespace planning