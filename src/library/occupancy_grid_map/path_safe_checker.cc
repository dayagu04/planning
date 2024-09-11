#include "path_safe_checker.h"

#include <cstddef>

#include "Eigen/src/Core/Matrix.h"
#include "ad_common/math/vec2d.h"
#include "log_glog.h"
#include "modules/adas_function/display_state_types.h"
#include "polygon_base.h"
#include "pose2d.h"

namespace planning {
#define DEBUG_PATH_CHECKER (1)

void PathSafeChecker::Excute(
    const std::vector<pnc::geometry_lib::PathPoint>& path,
    const pnc::geometry_lib::PathSegGear gear, const ParkObstacleList* obs,
    const Pose2D& ego_pose) {
  obs_ = obs;
  is_path_collision_ = false;
  is_ego_collision_ = false;

  if (obs_ == nullptr) {
    return;
  }

  if (path.size() <= 0) {
    return;
  }

  size_t path_end_id = path.size() - 1;

  if (obs_->point_cloud_list.empty() && obs_->virtual_obs.empty()) {
    return;
  }

  hierarchy_lateral_safe_buffer_.clear();
  // hierarchy_lateral_safe_buffer_.emplace_back(0.2);
  // hierarchy_lateral_safe_buffer_.emplace_back(0.1);
  hierarchy_lateral_safe_buffer_.emplace_back(0.05);
  advised_lon_buffer_ = 0.2;

  // get closest point
  size_t path_nearest_idx_ = GetNearestPathPoint(path, ego_pose);
  Pose2D global_pose;

  bool is_collision = false;
  Transform2d tf;
  PolygonFootPrint* tmp_foot_print;

  // 做安全检查时，使用不同安全宽度.
  double advised_lat_buffer = 0.2;
  global_pose = ego_pose;
  tf.SetBasePose(global_pose);
  // generate vehicle polygon
  tmp_foot_print = &foot_print_;
  VehCollisionPosition collision_component = VehCollisionPosition::none;

  // get safe width
  size_t lat_buffer_size = hierarchy_lateral_safe_buffer_.size();
  for (size_t i = 0; i < lat_buffer_size; i++) {
    advised_lat_buffer = hierarchy_lateral_safe_buffer_[i];

    // generate veh local polygon
    GenerateVehCompactPolygon(gear, advised_lat_buffer, advised_lon_buffer_);

    is_collision =
        IsFootPrintPolygonCollision(tf, tmp_foot_print, &collision_component);
    if (!is_collision) {
      break;
    }

    // collision
    if (i == lat_buffer_size - 1) {
      is_ego_collision_ = true;
    }
  }
  advised_lat_buffer_ = advised_lat_buffer;

  // check path
  size_t collision_index = 100000;
  cdl::AABB path_point_aabb;

  for (size_t i = path_nearest_idx_; i <= path_end_id; ++i) {
    global_pose.x = path[i].pos[0];
    global_pose.y = path[i].pos[1];
    global_pose.theta = path[i].heading;
    tf.SetBasePose(global_pose);

    // generate vehicle polygon
    if (i == path_end_id) {
      tmp_foot_print = &path_end_foot_print_;
    } else {
      tmp_foot_print = &foot_print_;
    }

    is_collision =
        IsFootPrintPolygonCollision(tf, tmp_foot_print, &collision_component);
    if (!is_collision) {
      continue;
    } else {
      collision_index = i;
      break;
    }
  }

  path_collision_idx_ = collision_index;
  is_path_collision_ = is_collision;

#if DEBUG_PATH_CHECKER
  ILOG_INFO << "path_collision_idx_= " << path_collision_idx_
            << " ,is_path_collision_= " << is_path_collision_
            << " ,path_nearest_idx_= " << path_nearest_idx_
            << " ,path_end_id= " << path_end_id
            << " , component=" << static_cast<int>(collision_component)
            << ",lat safe buffer = " << advised_lat_buffer_;

  PolygonDebugString(&foot_print_.body);

  if (tmp_foot_print != nullptr) {
    PolygonDebugString(&tmp_foot_print->body);
  }

  ego_pose.DebugString();

  Pose2D tmp;
  for (size_t i = 0; i < path.size(); i++) {
    tmp = Pose2D(path[i].pos[0], path[i].pos[1], path[i].heading);

    ILOG_INFO << "id = " << i;
    tmp.DebugString();
  }

#endif

  return;
}

void PathSafeChecker::GenerateVehBox(const pnc::geometry_lib::PathSegGear gear,
                                     const double lateral_safe_buffer,
                                     const double lon_safe_buffer) {
  const apa_planner::ApaParameters& config = apa_param.GetParam();

  GetUpLeftCoordinatePolygonByParam(
      &foot_print_.body, config.rear_overhanging,
      config.wheel_base + config.front_overhanging,
      config.car_width / 2.0 + lateral_safe_buffer);

  // left mirror
  Position2D center;
  center.x = config.footprint_circle_x[3];
  center.y = config.footprint_circle_y[3] + lateral_safe_buffer;
  GenerateMirrorPolygon(&foot_print_.mirror_left, 0.1,
                        config.footprint_circle_r[3], center);

  // right mirror
  center.x = config.footprint_circle_x[6];
  center.y = config.footprint_circle_y[6] - lateral_safe_buffer;
  GenerateMirrorPolygon(&foot_print_.mirror_right, 0.1,
                        config.footprint_circle_r[6], center);

  GetUpLeftCoordinatePolygonByParam(
      &foot_print_.max_polygon, config.rear_overhanging,
      config.wheel_base + config.front_overhanging,
      config.max_car_width / 2.0 + lateral_safe_buffer);

  // if path point is end, add extra lon buffer for safe check.
  GetUpLeftCoordinatePolygonByParam(
      &path_end_foot_print_.body, config.rear_overhanging + lon_safe_buffer,
      config.wheel_base + config.front_overhanging + lon_safe_buffer,
      config.car_width / 2.0 + lateral_safe_buffer);

  path_end_foot_print_.mirror_left = foot_print_.mirror_left;
  path_end_foot_print_.mirror_right = foot_print_.mirror_right;
  GetUpLeftCoordinatePolygonByParam(
      &path_end_foot_print_.max_polygon,
      config.rear_overhanging + lon_safe_buffer,
      config.wheel_base + config.front_overhanging + lon_safe_buffer,
      config.max_car_width / 2.0 + lateral_safe_buffer);

  return;
}

void PathSafeChecker::GenerateVehCompactPolygon(
    const pnc::geometry_lib::PathSegGear gear, const double lateral_safe_buffer,
    const double lon_safe_buffer) {
  const apa_planner::ApaParameters& config = apa_param.GetParam();

  if (config.car_vertex_x_vec.size() != 20) {
    ILOG_ERROR << "config invalid";
    GenerateVehBox(gear, lateral_safe_buffer, lon_safe_buffer);
    return;
  }

  GetCompactCarPolygonByParam(&foot_print_.body, lateral_safe_buffer, 0.0);

  // left mirror
  Position2D center;
  center.x = config.footprint_circle_x[3];
  center.y = config.footprint_circle_y[3] + lateral_safe_buffer;
  GenerateMirrorPolygon(&foot_print_.mirror_left, 0.1,
                        config.footprint_circle_r[3], center);

  // right mirror
  center.x = config.footprint_circle_x[6];
  center.y = config.footprint_circle_y[6] - lateral_safe_buffer;
  GenerateMirrorPolygon(&foot_print_.mirror_right, 0.1,
                        config.footprint_circle_r[6], center);

  GetUpLeftCoordinatePolygonByParam(
      &foot_print_.max_polygon, config.rear_overhanging,
      config.wheel_base + config.front_overhanging,
      config.max_car_width / 2.0 + lateral_safe_buffer);

  // if path point is end, add extra lon buffer for safe check.
  GetCompactCarPolygonByParam(&path_end_foot_print_.body, lateral_safe_buffer,
                              0.1);

  path_end_foot_print_.mirror_left = foot_print_.mirror_left;
  path_end_foot_print_.mirror_right = foot_print_.mirror_right;
  GetUpLeftCoordinatePolygonByParam(
      &path_end_foot_print_.max_polygon,
      config.rear_overhanging + lon_safe_buffer,
      config.wheel_base + config.front_overhanging + lon_safe_buffer,
      config.max_car_width / 2.0 + lateral_safe_buffer);

  return;
}

void PathSafeChecker::UpdatePathValidDist(
    const std::vector<pnc::geometry_lib::PathPoint>& path,
    const Pose2D& ego_pose) {
  if (!is_path_collision_) {
    return;
  }

  if (path_nearest_idx_ >= path.size() - 1) {
    return;
  }

  double path_dist = 0.0;
  double dist;
  Pose2D pose;
  Pose2D next_pose;
  for (size_t i = path_nearest_idx_; i < path_collision_idx_ - 1; i++) {
    pose.x = path[i].pos[0];
    pose.y = path[i].pos[1];
    pose.theta = path[i].heading;

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

  path_line.Normalize();

  ad_common::math::Vec2d ego_vector(
      ego_pose.x - path[path_nearest_idx_].pos[0],
      ego_pose.y - path[path_nearest_idx_].pos[1]);

  double projection_dist = path_line.InnerProd(ego_vector);

  path_dist -= projection_dist;

  path_valid_dist_ = path_dist;

  return;
}

size_t PathSafeChecker::GetNearestPathPoint(
    const std::vector<pnc::geometry_lib::PathPoint>& path, const Pose2D& pose) {
  double nearest_dist = 1000000.0;
  double dist;
  Pose2D global_pose;
  size_t nearest_id = 0;

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

  return nearest_id;
}

const bool PathSafeChecker::IsPolygonCollision(const Polygon2D* car) {
  bool is_collision = false;
  for (const auto& obstacle : obs_->point_cloud_list) {
    // envelop box check
    gjk_interface_.PolygonCollisionByCircleCheck(
        &is_collision, &obstacle.envelop_polygon, car, 0.01);

    if (!is_collision) {
      continue;
    }

    // internal points
    for (size_t j = 0; j < obstacle.points.size(); j++) {
      gjk_interface_.PolygonPointCollisionDetect(&is_collision, car,
                                                 obstacle.points[j]);

      if (is_collision) {
        // ILOG_INFO << "size = " << obstacle.points.size() << " j =" << j;
        return true;
      }
    }
  }

  return false;
}

int PathSafeChecker::GenerateMirrorPolygon(Polygon2D* box,
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

  return 0;
}

int PathSafeChecker::GetCompactCarPolygonByParam(Polygon2D* box,
                                                 const double lat_buffer,
                                                 const double lon_buffer) {
  const apa_planner::ApaParameters& config = apa_param.GetParam();

  box->vertexes[0].x = config.car_vertex_x_vec[2] + lon_buffer;
  box->vertexes[0].y = config.car_vertex_y_vec[2] + lat_buffer;

  box->vertexes[1].x = config.car_vertex_x_vec[0] + lon_buffer;
  box->vertexes[1].y = config.car_vertex_y_vec[0] + lat_buffer;

  box->vertexes[6].x = config.car_vertex_x_vec[5] + lon_buffer;
  box->vertexes[6].y = config.car_vertex_y_vec[5] - lat_buffer;

  box->vertexes[7].x = config.car_vertex_x_vec[3] + lon_buffer;
  box->vertexes[7].y = config.car_vertex_y_vec[3] - lat_buffer;

  box->vertexes[2].x = config.car_vertex_x_vec[15] - lon_buffer - 0.15;
  box->vertexes[2].y = config.car_vertex_y_vec[15] + lat_buffer;

  box->vertexes[3].x = config.car_vertex_x_vec[13] - lon_buffer;
  box->vertexes[3].y = config.car_vertex_y_vec[13] + lat_buffer + 0.15;

  box->vertexes[4].x = config.car_vertex_x_vec[12] - lon_buffer;
  box->vertexes[4].y = config.car_vertex_y_vec[12] - lat_buffer - 0.15;

  box->vertexes[5].x = config.car_vertex_x_vec[10] - lon_buffer - 0.15;
  box->vertexes[5].y = config.car_vertex_y_vec[10] - lat_buffer;

  box->vertex_num = 8;

  box->shape = PolygonShape::multi_edge;
  UpdatePolygonValue(box, NULL, 0, false, POLYGON_MAX_RADIUS);

  box->min_tangent_radius = config.car_width / 2 + lat_buffer;

  return 0;
}

const bool PathSafeChecker::IsFootPrintPolygonCollision(
    const Transform2d& tf, PolygonFootPrint* foot_print,
    VehCollisionPosition* collision_info) {
  Polygon2D veh_global_polygon;
  bool is_collision = false;
  ULFLocalPolygonToGlobal(&veh_global_polygon, &foot_print->max_polygon, tf);

  is_collision = IsPolygonCollision(&veh_global_polygon);
  if (!is_collision) {
    return false;
  }

  ULFLocalPolygonToGlobal(&veh_global_polygon, &foot_print->body, tf);

  is_collision = IsPolygonCollision(&veh_global_polygon);
  if (is_collision) {
    *collision_info = VehCollisionPosition::body;
    return true;
  }

  ULFLocalPolygonToGlobal(&veh_global_polygon, &foot_print->mirror_left, tf);

  is_collision = IsPolygonCollision(&veh_global_polygon);
  if (is_collision) {
    *collision_info = VehCollisionPosition::left_mirror;
    return true;
  }

  ULFLocalPolygonToGlobal(&veh_global_polygon, &foot_print->mirror_right, tf);

  is_collision = IsPolygonCollision(&veh_global_polygon);
  if (is_collision) {
    *collision_info = VehCollisionPosition::right_mirror;
    return true;
  }

  return false;
}

}  // namespace planning