#include "collision_detection.h"

#include <math.h>

#include <cmath>

#include "apa_param_setting.h"
#include "geometry_math.h"

namespace planning {

// car data
static const std::vector<double> g_car_local_vertex_x_vec = {
    3.187342,  3.424531,  3.593071,  3.593071,  3.424531,  3.187342,
    2.177994,  1.916421,  1.96496,   -0.476357, -0.798324, -0.879389,
    -0.879389, -0.798324, -0.476357, 1.96496,   1.916421,  2.177994};

static const std::vector<double> g_car_local_vertex_y_vec = {
    0.887956,  0.681712, 0.334651,  -0.334651, -0.681712, -0.887956,
    -0.887956, -1.06715, -0.887956, -0.887956, -0.706505, -0.334845,
    0.334845,  0.706505, 0.887956,  0.887956,  1.06715,   0.887956};

void CollisionDetector::Init() {
  car_line_local_vec_.clear();
  car_line_local_vec_.reserve(g_car_local_vertex_x_vec.size());

  std::vector<double> inflated_car_local_vertex_y_vec;
  inflated_car_local_vertex_y_vec.clear();
  inflated_car_local_vertex_y_vec.reserve(g_car_local_vertex_y_vec.size());
  for (size_t i = 0; i < g_car_local_vertex_y_vec.size(); ++i) {
    inflated_car_local_vertex_y_vec[i] = g_car_local_vertex_y_vec[i];
    if (inflated_car_local_vertex_y_vec[i] > 0) {
      inflated_car_local_vertex_y_vec[i] += param_.lat_inflation;
    } else {
      inflated_car_local_vertex_y_vec[i] -= param_.lat_inflation;
    }
  }

  pnc::geometry_lib::LineSegment car_line;
  Eigen::Vector2d p1;
  Eigen::Vector2d p2;

  for (size_t i = 0; i < g_car_local_vertex_x_vec.size(); ++i) {
    p1 << g_car_local_vertex_x_vec[i], inflated_car_local_vertex_y_vec[i];
    if (i < g_car_local_vertex_x_vec.size() - 1) {
      p2 << g_car_local_vertex_x_vec[i + 1],
          inflated_car_local_vertex_y_vec[i + 1];
    } else {
      p2 << g_car_local_vertex_x_vec[0], inflated_car_local_vertex_y_vec[0];
    }
    car_line.SetPoints(p1, p2);
    car_line_local_vec_.emplace_back(car_line);
  }
}

void CollisionDetector::SetParam(Paramters param) {
  param_ = param;
  SetLatInflation();
}

void CollisionDetector::SetLatInflation() { Init(); }

void CollisionDetector::Reset() {
  obstacle_global_vec_.clear();
  param_.Reset();
}

void CollisionDetector::SetObstacles(
    const std::vector<Eigen::Vector2d> &obstacle_global_vec) {
  obstacle_global_vec_.clear();
  obstacle_global_vec_.reserve(obstacle_global_vec.size());
  obstacle_global_vec_ = obstacle_global_vec;
}

void CollisionDetector::AddObstacles(
    const std::vector<Eigen::Vector2d> &obstacle_global_vec) {
  obstacle_global_vec_.reserve(obstacle_global_vec_.size() +
                               obstacle_global_vec.size());
  obstacle_global_vec_.insert(obstacle_global_vec_.end(),
                              obstacle_global_vec.begin(),
                              obstacle_global_vec.end());
}

void CollisionDetector::AddObstacles(const Eigen::Vector2d &obstacle_global) {
  obstacle_global_vec_.reserve(obstacle_global_vec_.size() + 1);
  obstacle_global_vec_.emplace_back(obstacle_global);
}

const CollisionDetector::CollisionResult CollisionDetector::Update(
    const pnc::geometry_lib::LineSegment &line_seg,
    const double heading_start) {
  if (obstacle_global_vec_.size() < 1) {
    CollisionResult tmp_result;
    return tmp_result;
  }

  // car line segment
  std::vector<pnc::geometry_lib::LineSegment> car_line_global_vec;
  car_line_global_vec.clear();
  car_line_global_vec.reserve(car_line_local_vec_.size());
  pnc::geometry_lib::LineSegment car_line_global;
  pnc::geometry_lib::LocalToGlobalTf l2g_tf;
  l2g_tf.Init(line_seg.pA, heading_start);
  for (const auto &car_line_local : car_line_local_vec_) {
    car_line_global.pA = l2g_tf.GetPos(car_line_local.pA);
    car_line_global.pB = l2g_tf.GetPos(car_line_local.pB);
    car_line_global_vec.emplace_back(car_line_global);
  }

  // detect if there is intersection(point_P) between obstacle and car line
  // segment
  double min_obs_move_dist = 33.3;
  pnc::geometry_lib::LineSegment obs_move_line;
  // the cross points of obstacle lin seg and single car polygon line seg
  Eigen::Vector2d cross_point;
  // collision points between car movement along trajectory and obstacles
  Eigen::Vector2d collision_point(0.0, 0.0);

  const Eigen::Vector2d AB = line_seg.pB - line_seg.pA;
  const Eigen::Vector2d unit_obs_move_line = -AB.normalized();
  size_t i = 0;
  size_t j = 0;
  for (const auto &obstacle_global : obstacle_global_vec_) {
    for (auto &car_line_global : car_line_global_vec) {
      obs_move_line.pA = obstacle_global;
      // obs_move_line.pB = obs_move_line.pA - AB;
      obs_move_line.pB =
          obs_move_line.pA + min_obs_move_dist * unit_obs_move_line;
      if (GetIntersectionFromTwoLineSeg(cross_point, car_line_global,
                                        obs_move_line)) {
        const auto dist_CP = (cross_point - obs_move_line.pA).norm();
        if (dist_CP < min_obs_move_dist) {
          collision_point = cross_point;
          min_obs_move_dist = dist_CP;
          i = j;
        }
      }
    }
    j++;
  }

  CollisionResult result;

  result.remain_car_dist = AB.norm();
  result.remain_obstacle_dist = min_obs_move_dist;
  result.remain_dist =
      std::min(result.remain_obstacle_dist, result.remain_car_dist);

  result.collision_flag =
      (result.remain_obstacle_dist <= result.remain_car_dist);

  std::cout << "collision_point = " << collision_point.transpose()
            << "  obstacle_global = " << obstacle_global_vec_[i].transpose()
            << std::endl;

  result.collision_point = collision_point;

  return result;
}

const CollisionDetector::CollisionResult CollisionDetector::Update(
    const pnc::geometry_lib::Arc &arc, const double heading_start) {
  if (obstacle_global_vec_.size() < 1) {
    CollisionResult tmp_result;
    return tmp_result;
  }
  // car line segment
  std::vector<pnc::geometry_lib::LineSegment> car_line_global_vec;
  car_line_global_vec.clear();
  car_line_global_vec.reserve(car_line_local_vec_.size());

  pnc::geometry_lib::LineSegment car_line_global;
  pnc::geometry_lib::LocalToGlobalTf l2g_tf;
  l2g_tf.Init(arc.pA, heading_start);
  for (const auto &car_line_local : car_line_local_vec_) {
    car_line_global.pA = l2g_tf.GetPos(car_line_local.pA);
    car_line_global.pB = l2g_tf.GetPos(car_line_local.pB);
    car_line_global_vec.emplace_back(car_line_global);
  }

  // obstacle arc segment
  const auto v_OA = arc.pA - arc.circle_info.center;
  const auto v_OB = arc.pB - arc.circle_info.center;
  const auto car_rot_angle = pnc::geometry_lib::GetAngleFromTwoVec(v_OA, v_OB);

  // obstacle rotates around the the car rotation center to form a circle
  // The minimum angle allowed for obstacle rotation
  auto min_obs_rot_limit_angle = 5.0;
  pnc::geometry_lib::Circle obs_rot_circle;
  // the cross points of obstacle circle and single car polygon line seg
  std::vector<Eigen::Vector2d> cross_points;
  // collision points between car movement along trajectory and obstacles
  Eigen::Vector2d collision_point;
  collision_point.setZero();
  size_t i = 0;
  size_t j = 0;
  for (const auto &obstacle_global : obstacle_global_vec_) {
    for (const auto &car_line_global : car_line_global_vec) {
      obs_rot_circle.center = arc.circle_info.center;
      obs_rot_circle.radius = (obstacle_global - obs_rot_circle.center).norm();

      const auto num = pnc::geometry_lib::CalcCrossPointsOfLineSegAndCircle(
          car_line_global, obs_rot_circle, cross_points);

      if (num != 1) {
        // if num == 0, no cross points, contiue
        // if num == 2, no possioble due to car motion limit, continue
        continue;
      }

      const auto v_OC = obstacle_global - arc.circle_info.center;
      const auto v_OD = cross_points.front() - arc.circle_info.center;

      const auto obs_rot_angle =
          pnc::geometry_lib::GetAngleFromTwoVec(v_OC, v_OD);

      if (obs_rot_angle * car_rot_angle < 0.0 &&
          fabs(obs_rot_angle) < fabs(min_obs_rot_limit_angle)) {
        // the rotation direction of obstacles and car must be opposite
        min_obs_rot_limit_angle = obs_rot_angle;
        collision_point = cross_points.front();
        i = j;
      }
    }
    j++;
  }

  CollisionResult result;

  result.remain_car_dist = fabs(car_rot_angle) * arc.circle_info.radius;
  // result.remain_obstacle_dist =
  //     fabs(min_obs_rot_limit_angle) * obs_rot_circle.radius; //err cal method
  result.remain_obstacle_dist =
      fabs(min_obs_rot_limit_angle) * arc.circle_info.radius;

  result.collision_flag =
      (result.remain_obstacle_dist <= result.remain_car_dist);

  std::cout << "collision_point = " << collision_point.transpose()
            << "  obstacle_global = " << obstacle_global_vec_[i].transpose()
            << std::endl;

  result.remain_dist =
      std::min(result.remain_obstacle_dist, result.remain_car_dist);

  result.collision_point = collision_point;

  return result;
}

const CollisionDetector::CollisionResult CollisionDetector::Update(
    const pnc::geometry_lib::LineSegment &line_seg, const double heading_start,
    const std::vector<Eigen::Vector2d> &obstacle_global_vec) {
  const auto tmp_obstacle_global_vec = obstacle_global_vec_;
  obstacle_global_vec_ = obstacle_global_vec;
  CollisionResult result = Update(line_seg, heading_start);
  obstacle_global_vec_ = tmp_obstacle_global_vec;
  return result;
}

const CollisionDetector::CollisionResult CollisionDetector::Update(
    const pnc::geometry_lib::Arc &arc, const double heading_start,
    const std::vector<Eigen::Vector2d> &obstacle_global_vec) {
  const auto tmp_obstacle_global_vec = obstacle_global_vec_;
  obstacle_global_vec_ = obstacle_global_vec;
  CollisionResult result = Update(arc, heading_start);
  obstacle_global_vec_ = tmp_obstacle_global_vec;
  return result;
}

const double CollisionDetector::CalMinDistObs2Car(
    const Eigen::Vector2d &obs_pos,
    const pnc::geometry_lib::PathPoint &ego_pose) {
  // car line segment
  std::vector<pnc::geometry_lib::LineSegment> car_line_global_vec;
  car_line_global_vec.clear();
  car_line_global_vec.reserve(car_line_local_vec_.size());
  pnc::geometry_lib::LineSegment car_line_global;
  pnc::geometry_lib::LocalToGlobalTf l2g_tf;
  l2g_tf.Init(ego_pose.pos, ego_pose.heading);
  for (const auto &car_line_local : car_line_local_vec_) {
    car_line_global.pA = l2g_tf.GetPos(car_line_local.pA);
    car_line_global.pB = l2g_tf.GetPos(car_line_local.pB);
    car_line_global_vec.emplace_back(car_line_global);
  }
  double min_dist = 166.6;
  // std::cout << "ego_pose = " << ego_pose.pos << std::endl;
  for (const auto &car_line_global : car_line_global_vec) {
    min_dist = std::min(min_dist, pnc::geometry_lib::CalPoint2LineSegDist(
                                      obs_pos, car_line_global));
    // std::cout << "min_dist = " << min_dist << "\n";
    // std::cout << "car_line_global.pA = " << car_line_global.pA.transpose()
    //           << "car_line_global.pB = " << car_line_global.pB.transpose()
    //           << "\n";
  }
  return min_dist;
}

const bool CollisionDetector::IsObstacleInCar(
    const Eigen::Vector2d &obs_pos,
    const pnc::geometry_lib::PathPoint &ego_pose) {
  // car line segment
  std::vector<pnc::geometry_lib::LineSegment> car_line_global_vec;
  car_line_global_vec.clear();
  car_line_global_vec.reserve(car_line_local_vec_.size());
  pnc::geometry_lib::LineSegment car_line_global;
  pnc::geometry_lib::LocalToGlobalTf l2g_tf;
  l2g_tf.Init(ego_pose.pos, ego_pose.heading);
  std::vector<Eigen::Vector2d> car_polygon;
  car_polygon.clear();
  car_polygon.reserve(g_car_local_vertex_x_vec.size());
  for (const auto &car_line_local : car_line_local_vec_) {
    car_line_global.pA = l2g_tf.GetPos(car_line_local.pA);
    car_line_global.pB = l2g_tf.GetPos(car_line_local.pB);
    car_line_global_vec.emplace_back(car_line_global);
    car_polygon.emplace_back(car_line_global.pA);
  }

  if (pnc::geometry_lib::IsPointInPolygon(car_polygon, obs_pos)) {
    return true;
  }

  double min_dist = 166.6;
  for (const auto &car_line_global : car_line_global_vec) {
    min_dist = std::min(min_dist, pnc::geometry_lib::CalPoint2LineSegDist(
                                      obs_pos, car_line_global));
  }
  if (min_dist < apa_param.GetParam().max_obs2car_dist) {
    return true;
  }

  return false;
}

}  // namespace planning