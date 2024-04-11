#include "collision_detection.h"

#include <math.h>

#include <cmath>

#include "geometry_math.h"

namespace planning {

void CollisionDetector::Init() {
  car_line_local_vec_.clear();
  car_line_local_vec_.reserve(apa_param.GetParam().car_vertex_x_vec.size());
  car_local_vertex_vec_.clear();
  car_local_vertex_vec_.reserve(apa_param.GetParam().car_vertex_x_vec.size());

  std::vector<double> inflated_car_local_vertex_y_vec;
  inflated_car_local_vertex_y_vec.clear();
  inflated_car_local_vertex_y_vec.reserve(
      apa_param.GetParam().car_vertex_y_vec.size());
  for (size_t i = 0; i < apa_param.GetParam().car_vertex_y_vec.size(); ++i) {
    inflated_car_local_vertex_y_vec[i] =
        apa_param.GetParam().car_vertex_y_vec[i];
    if (inflated_car_local_vertex_y_vec[i] > 0) {
      inflated_car_local_vertex_y_vec[i] += param_.lat_inflation;
    } else {
      inflated_car_local_vertex_y_vec[i] -= param_.lat_inflation;
    }
  }

  pnc::geometry_lib::LineSegment car_line;
  Eigen::Vector2d p1;
  Eigen::Vector2d p2;

  for (size_t i = 0; i < apa_param.GetParam().car_vertex_x_vec.size(); ++i) {
    p1 << apa_param.GetParam().car_vertex_x_vec[i],
        inflated_car_local_vertex_y_vec[i];
    if (i < apa_param.GetParam().car_vertex_x_vec.size() - 1) {
      p2 << apa_param.GetParam().car_vertex_x_vec[i + 1],
          inflated_car_local_vertex_y_vec[i + 1];
    } else {
      p2 << apa_param.GetParam().car_vertex_x_vec[0],
          inflated_car_local_vertex_y_vec[0];
    }
    car_line.SetPoints(p1, p2);
    car_line_local_vec_.emplace_back(car_line);

    car_local_vertex_vec_.emplace_back(p1);
  }
}

void CollisionDetector::SetParam(Paramters param) {
  param_ = param;
  SetLatInflation();
}

void CollisionDetector::SetLatInflation() { Init(); }

void CollisionDetector::Reset() {
  obs_pt_global_vec_.clear();
  param_.Reset();
}

void CollisionDetector::SetObstacles(
    const std::vector<Eigen::Vector2d> &obs_pt_global_vec) {
  obs_pt_global_vec_.clear();
  obs_pt_global_vec_.reserve(obs_pt_global_vec.size());
  obs_pt_global_vec_ = obs_pt_global_vec;
}

void CollisionDetector::AddObstacles(
    const std::vector<Eigen::Vector2d> &obs_pt_global_vec) {
  obs_pt_global_vec_.reserve(obs_pt_global_vec_.size() +
                             obs_pt_global_vec.size());
  obs_pt_global_vec_.insert(obs_pt_global_vec_.end(), obs_pt_global_vec.begin(),
                            obs_pt_global_vec.end());
}

void CollisionDetector::AddObstacles(const Eigen::Vector2d &obs_pt_global) {
  obs_pt_global_vec_.reserve(obs_pt_global_vec_.size() + 1);
  obs_pt_global_vec_.emplace_back(obs_pt_global);
}

const CollisionDetector::CollisionResult CollisionDetector::Update(
    const pnc::geometry_lib::LineSegment &line_seg,
    const double heading_start) {
  if (obs_pt_global_vec_.size() < 1) {
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
  for (const auto &obs_pt_global : obs_pt_global_vec_) {
    for (auto &car_line_global : car_line_global_vec) {
      obs_move_line.pA = obs_pt_global;
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

  // std::cout << "collision_point = " << collision_point.transpose()
  //           << "  obs_pt_global = " << obs_pt_global_vec_[i].transpose()
  //           << std::endl;

  result.collision_point = collision_point;

  return result;
}

const CollisionDetector::CollisionResult CollisionDetector::Update(
    const pnc::geometry_lib::Arc &arc, const double heading_start) {
  if (obs_pt_global_vec_.size() < 1) {
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
  for (const auto &obs_pt_global : obs_pt_global_vec_) {
    for (const auto &car_line_global : car_line_global_vec) {
      obs_rot_circle.center = arc.circle_info.center;
      obs_rot_circle.radius = (obs_pt_global - obs_rot_circle.center).norm();

      const auto num = pnc::geometry_lib::CalcCrossPointsOfLineSegAndCircle(
          car_line_global, obs_rot_circle, cross_points);

      if (num == 0) {
        // if num == 0, no cross points, contiue
        continue;
      }

      const auto v_OC = obs_pt_global - arc.circle_info.center;
      Eigen::Vector2d D;
      if (num == 1) {
        D = cross_points.front();
      } else if (num == 2) {
        const auto v_OD1 = cross_points.front() - arc.circle_info.center;
        const auto v_OD2 = cross_points.back() - arc.circle_info.center;

        const auto obs_rot_angle1 =
            pnc::geometry_lib::GetAngleFromTwoVec(v_OC, v_OD1);

        const auto obs_rot_angle2 =
            pnc::geometry_lib::GetAngleFromTwoVec(v_OC, v_OD2);

        if (std::fabs(obs_rot_angle1) < std::fabs(obs_rot_angle2)) {
          D = cross_points.front();
        } else {
          D = cross_points.back();
        }
      }
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

  // std::cout << "collision_point = " << collision_point.transpose()
  //           << "  obs_pt_global = " << obs_pt_global_vec_[i].transpose()
  //           << std::endl;

  result.remain_dist =
      std::min(result.remain_obstacle_dist, result.remain_car_dist);

  result.collision_point = collision_point;
  result.collision_point_global = obs_pt_global_vec_[i];
  return result;
}

const CollisionDetector::CollisionResult CollisionDetector::Update(
    const pnc::geometry_lib::LineSegment &line_seg, const double heading_start,
    const std::vector<Eigen::Vector2d> &obs_pt_global_vec) {
  const auto tmp_obs_pt_global_vec = obs_pt_global_vec_;
  obs_pt_global_vec_ = obs_pt_global_vec;
  CollisionResult result = Update(line_seg, heading_start);
  obs_pt_global_vec_ = tmp_obs_pt_global_vec;
  return result;
}

const CollisionDetector::CollisionResult CollisionDetector::Update(
    const pnc::geometry_lib::Arc &arc, const double heading_start,
    const std::vector<Eigen::Vector2d> &obs_pt_global_vec) {
  const auto tmp_obs_pt_global_vec = obs_pt_global_vec_;
  obs_pt_global_vec_ = obs_pt_global_vec;
  CollisionResult result = Update(arc, heading_start);
  obs_pt_global_vec_ = tmp_obs_pt_global_vec;
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
  return IsObstacleInCar(obs_pos, ego_pose,
                         apa_param.GetParam().max_obs2car_dist_in_slot);
}

const bool CollisionDetector::IsObstacleInCar(
    const Eigen::Vector2d &obs_pos,
    const pnc::geometry_lib::PathPoint &ego_pose, double safe_dist) {
  // car line segment
  std::vector<pnc::geometry_lib::LineSegment> car_line_global_vec;
  car_line_global_vec.clear();
  car_line_global_vec.reserve(car_line_local_vec_.size());
  pnc::geometry_lib::LineSegment car_line_global;
  pnc::geometry_lib::LocalToGlobalTf l2g_tf;
  l2g_tf.Init(ego_pose.pos, ego_pose.heading);
  std::vector<Eigen::Vector2d> car_polygon;
  car_polygon.clear();
  car_polygon.reserve(apa_param.GetParam().car_vertex_x_vec.size());
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
  if (min_dist < safe_dist) {
    return true;
  }

  return false;
}

const CollisionDetector::CollisionResult CollisionDetector::UpdateByLineObs(
    const pnc::geometry_lib::LineSegment &line_seg,
    const double heading_start) {
  std::cout << "UpdateByLineObs\n";
  if (obs_line_global_vec_.empty()) {
    CollisionResult tmp_result;
    return tmp_result;
  }

  pnc::geometry_lib::LocalToGlobalTf l2g_tf;
  l2g_tf.Init(line_seg.pA, heading_start);

  std::vector<Eigen::Vector2d> car_global_vertex_vec_;
  car_global_vertex_vec_.clear();
  car_global_vertex_vec_.reserve(car_local_vertex_vec_.size());
  for (const auto &car_local_vertex : car_local_vertex_vec_) {
    const auto car_global_vertex = l2g_tf.GetPos(car_local_vertex);
    car_global_vertex_vec_.emplace_back(std::move(car_global_vertex));
  }

  // gen car vertex trajectory
  std::vector<pnc::geometry_lib::LineSegment> car_line_global_vec;
  car_line_global_vec.clear();
  car_line_global_vec.reserve(car_global_vertex_vec_.size());
  double traj_dist = 33.3;
  const auto unit_line = (line_seg.pB - line_seg.pA).normalized();
  for (const auto &car_global_vertex : car_global_vertex_vec_) {
    pnc::geometry_lib::LineSegment car_global_line;
    car_global_line.pA = car_global_vertex;
    car_global_line.pB = car_global_vertex + traj_dist * unit_line;
    car_line_global_vec.emplace_back(std::move(car_global_line));
  }
  Eigen::Vector2d collision_point(0.0, 0.0);
  // detect car traj and obs line is or not col
  for (auto &obs_line_global : obs_line_global_vec_) {
    for (auto &car_line_global : car_line_global_vec) {
      Eigen::Vector2d intersection;
      if (GetIntersectionFromTwoLineSeg(intersection, car_line_global,
                                        obs_line_global)) {
        const double dist = (car_line_global.pA - intersection).norm();
        if (dist < traj_dist) {
          traj_dist = dist;
          collision_point = car_line_global.pA;
        }
      }
    }
  }

  CollisionResult result;
  result.remain_car_dist = (line_seg.pB - line_seg.pA).norm();
  result.remain_obstacle_dist = traj_dist;
  result.remain_dist =
      std::min(result.remain_obstacle_dist, result.remain_car_dist);

  result.collision_flag =
      (result.remain_obstacle_dist <= result.remain_car_dist);

  result.collision_point = collision_point;
  return result;
}

const CollisionDetector::CollisionResult CollisionDetector::UpdateByLineObs(
    const pnc::geometry_lib::Arc &arc, const double heading_start) {
  if (obs_line_global_vec_.empty()) {
    CollisionResult tmp_result;
    return tmp_result;
  }

  pnc::geometry_lib::LocalToGlobalTf l2g_tf;
  l2g_tf.Init(arc.pA, heading_start);

  std::vector<Eigen::Vector2d> car_global_vertex_vec_;
  car_global_vertex_vec_.clear();
  car_global_vertex_vec_.reserve(car_local_vertex_vec_.size());
  for (const auto &car_local_vertex : car_local_vertex_vec_) {
    const auto car_global_vertex = l2g_tf.GetPos(car_local_vertex);
    car_global_vertex_vec_.emplace_back(std::move(car_global_vertex));
  }

  // gen car vertex trajectory
  std::vector<pnc::geometry_lib::Arc> car_arc_global_vec;
  car_arc_global_vec.clear();
  car_arc_global_vec.reserve(car_global_vertex_vec_.size());
  double traj_angle = 5.0;

  double sign = 1.0;
  if (!arc.is_anti_clockwise) {
    sign = -1.0;
  }
  const auto rot_matrix =
      pnc::geometry_lib::GetRotm2dFromTheta(traj_angle * sign);

  for (const auto &car_global_vertex : car_global_vertex_vec_) {
    pnc::geometry_lib::Arc car_arc_global;
    car_arc_global.pA = car_global_vertex;
    car_arc_global.circle_info.center = arc.circle_info.center;
    car_arc_global.circle_info.radius =
        (car_arc_global.pA - car_arc_global.circle_info.center).norm();
    car_arc_global.pB =
        car_arc_global.circle_info.center +
        rot_matrix * (car_arc_global.pA - car_arc_global.circle_info.center);
    car_arc_global.is_anti_clockwise = arc.is_anti_clockwise;
    car_arc_global_vec.emplace_back(std::move(car_arc_global));
  }

  const auto v_OA = arc.pA - arc.circle_info.center;
  const auto v_OB = arc.pB - arc.circle_info.center;
  const auto car_rot_angle = pnc::geometry_lib::GetAngleFromTwoVec(v_OA, v_OB);

  // detect car traj and obs line is or not col
  for (auto &obs_line_global : obs_line_global_vec_) {
    for (auto &car_arc_global : car_arc_global_vec) {
      std::pair<Eigen::Vector2d, Eigen::Vector2d> intersections;
      const size_t num = GetArcLineIntersection(intersections, car_arc_global,
                                                obs_line_global);
      if (num == 0) {
        continue;
      }
      const auto v_OC = car_arc_global.pA - car_arc_global.circle_info.center;
      double rot_angle = 0.0;
      if (num == 1) {
        const auto v_OD =
            intersections.first - car_arc_global.circle_info.center;

        rot_angle = pnc::geometry_lib::GetAngleFromTwoVec(v_OC, v_OD);

        bool anti_clock = (rot_angle > 0.0);
        if (anti_clock != car_arc_global.is_anti_clockwise) {
          rot_angle = 2 * 3.1415926 - std::fabs(rot_angle);
        }
      } else if (num == 2) {
        const auto v_OD =
            intersections.first - car_arc_global.circle_info.center;

        double rot_angle_1 = pnc::geometry_lib::GetAngleFromTwoVec(v_OC, v_OD);

        bool anti_clock_1 = (rot_angle_1 > 0.0);
        if (anti_clock_1 != car_arc_global.is_anti_clockwise) {
          rot_angle_1 = 2 * 3.1415926 - std::fabs(rot_angle_1);
        }

        const auto v_OE =
            intersections.second - car_arc_global.circle_info.center;

        double rot_angle_2 = pnc::geometry_lib::GetAngleFromTwoVec(v_OC, v_OE);
        bool anti_clock_2 = (rot_angle_2 > 0.0);
        if (anti_clock_2 != car_arc_global.is_anti_clockwise) {
          rot_angle_2 = 2 * 3.1415926 - std::fabs(rot_angle_2);
        }
        rot_angle = std::min(rot_angle_1, rot_angle_2);
      }

      traj_angle = std::min(traj_angle, std::fabs(rot_angle));
    }
  }

  CollisionResult result;
  result.remain_car_dist = std::fabs(car_rot_angle) * arc.circle_info.radius;
  result.remain_obstacle_dist = std::fabs(traj_angle) * arc.circle_info.radius;
  result.remain_dist =
      std::min(result.remain_obstacle_dist, result.remain_car_dist);

  result.collision_flag =
      (result.remain_obstacle_dist <= result.remain_car_dist);
  return result;
}

const CollisionDetector::CollisionResult CollisionDetector::UpdateByLineObs(
    const pnc::geometry_lib::LineSegment &line_seg, const double heading_start,
    const std::vector<pnc::geometry_lib::LineSegment> &obs_line_global_vec) {
  const auto tmp_obs_line_global_vec = obs_line_global_vec_;
  obs_line_global_vec_ = obs_line_global_vec;
  CollisionResult result = Update(line_seg, heading_start);
  obs_line_global_vec_ = tmp_obs_line_global_vec;
  return result;
}

const CollisionDetector::CollisionResult CollisionDetector::UpdateByLineObs(
    const pnc::geometry_lib::Arc &arc, const double heading_start,
    const std::vector<pnc::geometry_lib::LineSegment> &obs_line_global_vec) {
  const auto tmp_obs_line_global_vec = obs_line_global_vec_;
  obs_line_global_vec_ = obs_line_global_vec;
  CollisionResult result = Update(arc, heading_start);
  obs_line_global_vec_ = tmp_obs_line_global_vec;
  return result;
}

void CollisionDetector::SetLineObstacles(
    const std::vector<pnc::geometry_lib::LineSegment> &obs_line_global_vec) {
  obs_line_global_vec_.clear();
  obs_line_global_vec_.reserve(obs_line_global_vec.size());
  obs_line_global_vec_ = obs_line_global_vec;
}

void CollisionDetector::AddLineObstacles(
    const std::vector<pnc::geometry_lib::LineSegment> &obs_line_global_vec) {
  obs_line_global_vec_.reserve(obs_line_global_vec_.size() +
                               obs_line_global_vec.size());
  obs_line_global_vec_.insert(obs_line_global_vec_.end(),
                              obs_line_global_vec.begin(),
                              obs_line_global_vec.end());
}

void CollisionDetector::AddLineObstacles(
    const pnc::geometry_lib::LineSegment &obs_line_global) {
  obs_line_global_vec_.reserve(obs_line_global_vec_.size() + 1);
  obs_line_global_vec_.emplace_back(obs_line_global);
}

}  // namespace planning