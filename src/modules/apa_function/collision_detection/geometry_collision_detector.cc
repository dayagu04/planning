#include "geometry_collision_detector.h"

#include "geometry_math.h"

namespace planning {
namespace apa_planner {

void GeometryCollisionDetector::Reset() {}

const ColResult GeometryCollisionDetector::Update(
    const geometry_lib::PathSegment &path_seg, const double lat_buffer,
    const double lon_buffer) {
  col_res_.Reset();
  if (obs_manager_ == nullptr || obs_manager_->GetObstacles().empty()) {
    return col_res_;
  }

  col_res_.remain_car_dist = path_seg.Getlength();
  col_res_.remain_dist = path_seg.Getlength();

  UpdateSafeBuffer(lat_buffer, lon_buffer);

  CalPathSegBound(path_seg);

  if (path_seg.seg_type == geometry_lib::SEG_TYPE_LINE) {
    Update(path_seg.line_seg);
  } else {
    Update(path_seg.arc_seg);
  }

  return col_res_;
}

void GeometryCollisionDetector::Update(
    const geometry_lib::LineSegment &line_seg) {
  geometry_lib::LocalToGlobalTf l2g_tf;
  l2g_tf.Init(line_seg.pA, line_seg.heading);
  geometry_lib::LineSegment line;
  Eigen::Vector2d pA;
  Eigen::Vector2d pB;

  // 根据顶点构建基于车位坐标系的自车多边形
  // 后续可以根据障碍物属性来选择使用哪种多边形
  std::vector<geometry_lib::LineSegment> car_with_mirror_line_vec;
  car_with_mirror_line_vec.clear();
  car_with_mirror_line_vec.reserve(
      car_with_mirror_polygon_vertex_with_buffer_.size());
  for (size_t i = 0; i < car_with_mirror_polygon_vertex_with_buffer_.size();
       ++i) {
    pA = car_with_mirror_polygon_vertex_with_buffer_[i];
    if (i < car_with_mirror_polygon_vertex_with_buffer_.size() - 1) {
      pB = car_with_mirror_polygon_vertex_with_buffer_[i + 1];
    } else {
      pB = car_with_mirror_polygon_vertex_with_buffer_[0];
    }
    line.SetPoints(l2g_tf.GetPos(pA), l2g_tf.GetPos(pB));
    car_with_mirror_line_vec.emplace_back(line);
  }

  std::vector<geometry_lib::LineSegment> car_without_mirror_line_vec;
  car_without_mirror_line_vec.clear();
  car_without_mirror_line_vec.reserve(
      car_without_mirror_polygon_vertex_with_buffer_.size());
  for (size_t i = 0; i < car_without_mirror_polygon_vertex_with_buffer_.size();
       ++i) {
    pA = car_without_mirror_polygon_vertex_with_buffer_[i];
    if (i < car_without_mirror_polygon_vertex_with_buffer_.size() - 1) {
      pB = car_without_mirror_polygon_vertex_with_buffer_[i + 1];
    } else {
      pB = car_without_mirror_polygon_vertex_with_buffer_[0];
    }
    line.SetPoints(l2g_tf.GetPos(pA), l2g_tf.GetPos(pB));
    car_without_mirror_line_vec.emplace_back(line);
  }

  std::vector<geometry_lib::LineSegment> chassis_line_vec;
  chassis_line_vec.clear();
  chassis_line_vec.reserve(chassis_vertex_with_buffer_.size());
  for (size_t i = 0; i < chassis_vertex_with_buffer_.size(); ++i) {
    pA = chassis_vertex_with_buffer_[i];
    if (i < chassis_vertex_with_buffer_.size() - 1) {
      pB = chassis_vertex_with_buffer_[i + 1];
    } else {
      pB = chassis_vertex_with_buffer_[0];
    }
    line.SetPoints(l2g_tf.GetPos(pA), l2g_tf.GetPos(pB));
    chassis_line_vec.emplace_back(line);
  }

  double min_obs_move_dist = 33.3;
  const Eigen::Vector2d AB = line_seg.pB - line_seg.pA;
  const Eigen::Vector2d unit_obs_move_line = -AB.normalized();

  const std::unordered_map<size_t, ApaObstacle> &obs_map =
      obs_manager_->GetObstacles();

  std::vector<geometry_lib::LineSegment> *car_line_vec;
  geometry_lib::LineSegment obs_move_line;
  Eigen::Vector2d cross_point;
  for (const auto &obs_pair : obs_map) {
    const ApaObstacle obs = obs_pair.second;
    switch (obs.GetObsHeightType()) {
      case ApaObsHeightType::RUN_OVER:
        continue;
      case ApaObsHeightType::LOW:
        car_line_vec = &chassis_line_vec;
        break;
      case ApaObsHeightType::MID:
        car_line_vec = &car_without_mirror_line_vec;
        break;
      default:
        car_line_vec = &car_with_mirror_line_vec;
        break;
    }

    const std::vector<Eigen::Vector2d> &pt_clout_2d = obs.GetPtClout2dLocal();

    for (const Eigen::Vector2d &pt : pt_clout_2d) {
      if (!col_res_.path_rectangle_bound.IsPtInRectangleBound(pt)) {
        continue;
      }
      for (geometry_lib::LineSegment &line : *car_line_vec) {
        obs_move_line.SetPoints(pt,
                                pt + min_obs_move_dist * unit_obs_move_line);
        if (geometry_lib::CalcTwoLineSegIntersection(cross_point, line,
                                                     obs_move_line)) {
          const double dist_CP = (cross_point - obs_move_line.pA).norm();
          if (dist_CP < min_obs_move_dist) {
            min_obs_move_dist = dist_CP;
          }
        }
      }
    }
  }

  col_res_.remain_obs_dist = min_obs_move_dist;

  col_res_.remain_dist =
      std::min(col_res_.remain_obs_dist, col_res_.remain_car_dist);

  col_res_.col_flag =
      (col_res_.remain_obs_dist < col_res_.remain_car_dist + lon_buffer_);
}

void GeometryCollisionDetector::Update(const geometry_lib::Arc &arc_seg) {
  geometry_lib::LocalToGlobalTf l2g_tf;
  l2g_tf.Init(arc_seg.pA, arc_seg.headingA);
  geometry_lib::LineSegment line;
  Eigen::Vector2d pA;
  Eigen::Vector2d pB;

  // 根据顶点构建基于车位坐标系的自车多边形
  // 后续可以根据障碍物属性来选择使用哪种多边形
  std::vector<geometry_lib::LineSegment> car_with_mirror_line_vec;
  car_with_mirror_line_vec.clear();
  car_with_mirror_line_vec.reserve(
      car_with_mirror_polygon_vertex_with_buffer_.size());
  for (size_t i = 0; i < car_with_mirror_polygon_vertex_with_buffer_.size();
       ++i) {
    pA = car_with_mirror_polygon_vertex_with_buffer_[i];
    if (i < car_with_mirror_polygon_vertex_with_buffer_.size() - 1) {
      pB = car_with_mirror_polygon_vertex_with_buffer_[i + 1];
    } else {
      pB = car_with_mirror_polygon_vertex_with_buffer_[0];
    }
    line.SetPoints(l2g_tf.GetPos(pA), l2g_tf.GetPos(pB));
    car_with_mirror_line_vec.emplace_back(line);
  }

  std::vector<geometry_lib::LineSegment> car_without_mirror_line_vec;
  car_without_mirror_line_vec.clear();
  car_without_mirror_line_vec.reserve(
      car_without_mirror_polygon_vertex_with_buffer_.size());
  for (size_t i = 0; i < car_without_mirror_polygon_vertex_with_buffer_.size();
       ++i) {
    pA = car_without_mirror_polygon_vertex_with_buffer_[i];
    if (i < car_without_mirror_polygon_vertex_with_buffer_.size() - 1) {
      pB = car_without_mirror_polygon_vertex_with_buffer_[i + 1];
    } else {
      pB = car_without_mirror_polygon_vertex_with_buffer_[0];
    }
    line.SetPoints(l2g_tf.GetPos(pA), l2g_tf.GetPos(pB));
    car_without_mirror_line_vec.emplace_back(line);
  }

  std::vector<geometry_lib::LineSegment> chassis_line_vec;
  chassis_line_vec.clear();
  chassis_line_vec.reserve(chassis_vertex_with_buffer_.size());
  for (size_t i = 0; i < chassis_vertex_with_buffer_.size(); ++i) {
    pA = chassis_vertex_with_buffer_[i];
    if (i < chassis_vertex_with_buffer_.size() - 1) {
      pB = chassis_vertex_with_buffer_[i + 1];
    } else {
      pB = chassis_vertex_with_buffer_[0];
    }
    line.SetPoints(l2g_tf.GetPos(pA), l2g_tf.GetPos(pB));
    chassis_line_vec.emplace_back(line);
  }

  const Eigen::Vector2d v_OA = arc_seg.pA - arc_seg.circle_info.center;
  const Eigen::Vector2d v_OB = arc_seg.pB - arc_seg.circle_info.center;
  const double car_rot_angle = geometry_lib::GetAngleFromTwoVec(v_OA, v_OB);

  const std::unordered_map<size_t, ApaObstacle> &obs_map =
      obs_manager_->GetObstacles();

  std::vector<geometry_lib::LineSegment> *car_line_vec;
  // obstacle rotates around the the car rotation center to form a circle
  // The minimum angle allowed for obstacle rotation
  auto min_obs_rot_limit_angle = 5.0;
  geometry_lib::Circle obs_rot_circle;
  // the cross points of obstacle circle and single car polygon line seg
  std::vector<Eigen::Vector2d> cross_points;
  for (const auto &obs_pair : obs_map) {
    const ApaObstacle obs = obs_pair.second;
    switch (obs.GetObsHeightType()) {
      case ApaObsHeightType::RUN_OVER:
        car_line_vec = &chassis_line_vec;
        break;
      case ApaObsHeightType::LOW:
        car_line_vec = &car_without_mirror_line_vec;
        break;
      case ApaObsHeightType::MID:
        car_line_vec = &car_with_mirror_line_vec;
        break;
      default:
        car_line_vec = &car_with_mirror_line_vec;
    }

    const std::vector<Eigen::Vector2d> pt_clout_2d = obs.GetPtClout2dLocal();

    for (const Eigen::Vector2d &pt : pt_clout_2d) {
      if (!col_res_.path_rectangle_bound.IsPtInRectangleBound(pt)) {
        continue;
      }
      for (geometry_lib::LineSegment &line : *car_line_vec) {
        obs_rot_circle.center = arc_seg.circle_info.center;
        obs_rot_circle.radius = (pt - obs_rot_circle.center).norm();
        const size_t num = geometry_lib::CalcLineSegAndCircleIntersection(
            line, obs_rot_circle, cross_points);

        if (num == 0) {
          // if num == 0, no cross points, contiue
          continue;
        }

        const Eigen::Vector2d v_OC = pt - arc_seg.circle_info.center;
        Eigen::Vector2d D;
        double obs_rot_angle = 0.0;
        if (num == 1) {
          D = cross_points.front();
          const Eigen::Vector2d v_OD = D - arc_seg.circle_info.center;
          obs_rot_angle = geometry_lib::GetAngleFromTwoVec(v_OC, v_OD);
        } else if (num == 2) {
          const Eigen::Vector2d v_OD1 =
              cross_points.front() - arc_seg.circle_info.center;
          const Eigen::Vector2d v_OD2 =
              cross_points.back() - arc_seg.circle_info.center;

          const double obs_rot_angle1 =
              geometry_lib::GetAngleFromTwoVec(v_OC, v_OD1);

          const double obs_rot_angle2 =
              geometry_lib::GetAngleFromTwoVec(v_OC, v_OD2);

          if (std::fabs(obs_rot_angle1) < std::fabs(obs_rot_angle2)) {
            D = cross_points.front();
            obs_rot_angle = obs_rot_angle1;
          } else {
            D = cross_points.back();
            obs_rot_angle = obs_rot_angle2;
          }
        }

        if (obs_rot_angle * car_rot_angle < 0.0 &&
            fabs(obs_rot_angle) < fabs(min_obs_rot_limit_angle)) {
          // the rotation direction of obstacles and car must be opposite
          min_obs_rot_limit_angle = obs_rot_angle;
        }
      }
    }
  }

  col_res_.remain_obs_dist =
      std::fabs(min_obs_rot_limit_angle) * arc_seg.circle_info.radius;

  col_res_.remain_dist =
      std::min(col_res_.remain_obs_dist, col_res_.remain_car_dist);

  col_res_.col_flag =
      (col_res_.remain_obs_dist < col_res_.remain_car_dist + lon_buffer_);
}

void GeometryCollisionDetector::CalPathSegBound(
    const geometry_lib::PathSegment &path_seg) {
  std::vector<geometry_lib::PathPoint> pose_vec;
  pose_vec.reserve(4);
  if (path_seg.seg_type == geometry_lib::SEG_TYPE_LINE) {
    pose_vec.emplace_back(path_seg.GetStartPose());
    pose_vec.emplace_back(path_seg.GetEndPose());
  } else {
    geometry_lib::PathPoint pose;
    double length = 0.0;
    while (length < path_seg.Getlength() - 3e-2) {
      CalPtFromPathSeg(pose, path_seg, length);
      pose_vec.emplace_back(pose);
      length += 30 * kDeg2Rad * path_seg.arc_seg.circle_info.radius;
    }
    pose_vec.emplace_back(path_seg.GetEndPose());
  }
  std::vector<Eigen::Vector2d> car_vertex_vec;
  Eigen::Vector2d car_vertex;
  car_vertex_vec.clear();
  car_vertex_vec.reserve(pose_vec.size() * 4 + 4);
  for (const geometry_lib::PathPoint &pose : pose_vec) {
    geometry_lib::LocalToGlobalTf l2g_tf(pose.pos, pose.heading);
    for (size_t i = 0; i < car_with_mirror_rectangle_vertex_.size(); ++i) {
      car_vertex = l2g_tf.GetPos(car_with_mirror_rectangle_vertex_[i]);
      car_vertex_vec.emplace_back(car_vertex);
    }
  }

  double x_min = 0.0, y_min = 0.0, x_max = 0.0, y_max = 0.0;
  geometry_lib::GetPolygonBound(&x_min, &x_max, &y_min, &y_max, car_vertex_vec);
  x_min -= 0.5;
  y_min -= 0.5;
  x_max += 0.5;
  y_max += 0.5;

  col_res_.path_rectangle_bound.Set(x_min, y_min, x_max, y_max);
}

}  // namespace apa_planner
}  // namespace planning