#include "collision_detection.h"

#include <math.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

#include "apa_param_setting.h"
#include "debug_info_log.h"
#include "geometry_math.h"
#include "ifly_time.h"

namespace planning {
namespace apa_planner {
const bool box = true;
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
  obs_pt_global_map_.clear();
}

void CollisionDetector::ClearObstacles() {
  obs_pt_global_vec_.clear();
  obs_pt_global_map_.clear();
}

void CollisionDetector::DeleteObstacles(const size_t obs_type) {
  obs_pt_global_map_.erase(obs_type);
}

void CollisionDetector::SetObstacles(
    const std::vector<Eigen::Vector2d> &obs_pt_global_vec) {
  obs_pt_global_vec_.clear();
  obs_pt_global_vec_.reserve(obs_pt_global_vec.size());
  obs_pt_global_vec_ = obs_pt_global_vec;
}

void CollisionDetector::SetObstacles(
    const std::vector<Eigen::Vector2d> &obs_pt_global_vec,
    const size_t obs_type) {
  obs_pt_global_map_[obs_type] = obs_pt_global_vec;
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

void CollisionDetector::AddObstacles(
    const std::vector<Eigen::Vector2d> &obs_pt_global_vec,
    const size_t obs_type) {
  if (obs_pt_global_map_.count(obs_type) == 0) {
    obs_pt_global_map_[obs_type] = obs_pt_global_vec;
  } else {
    obs_pt_global_map_[obs_type].reserve(obs_pt_global_map_[obs_type].size() +
                                         obs_pt_global_vec.size());
    obs_pt_global_map_[obs_type].insert(obs_pt_global_map_[obs_type].end(),
                                        obs_pt_global_vec.begin(),
                                        obs_pt_global_vec.end());
  }
}

void CollisionDetector::AddObstacles(const Eigen::Vector2d &obs_pt_global,
                                     const size_t obs_type) {
  if (obs_pt_global_map_.count(obs_type) == 0) {
    std::vector<Eigen::Vector2d> obs_pt_global_vec;
    obs_pt_global_vec.clear();
    obs_pt_global_vec.reserve(1);
    obs_pt_global_vec.emplace_back(obs_pt_global);
    obs_pt_global_map_[obs_type] = obs_pt_global_vec;
  } else {
    obs_pt_global_map_[obs_type].reserve(obs_pt_global_map_[obs_type].size() +
                                         1);
    obs_pt_global_map_[obs_type].emplace_back(obs_pt_global);
  }
}

const CollisionDetector::CollisionResult CollisionDetector::Update(
    const pnc::geometry_lib::LineSegment &line_seg,
    const double heading_start) {
  if (obs_pt_global_vec_.size() < 1) {
    CollisionResult tmp_result;
    tmp_result.remain_car_dist = line_seg.length;
    tmp_result.remain_dist = line_seg.length;
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
  Eigen::Vector2d col_pt_ego_global(0.0, 0.0);

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
          col_pt_ego_global = cross_point;
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

  // DEBUG_PRINT("collision_point = " << collision_point.transpose()
  //           << "  obs_pt_global = " << obs_pt_global_vec_[i].transpose()
  //          );

  result.col_pt_ego_global = col_pt_ego_global;
  result.col_pt_obs_global = obs_pt_global_vec_[i];

  return result;
}

const CollisionDetector::CollisionResult CollisionDetector::UpdateByObsMap(
    const pnc::geometry_lib::LineSegment &line_seg,
    const double heading_start) {
  if (obs_pt_global_map_.empty()) {
    CollisionResult tmp_result;
    tmp_result.remain_car_dist = line_seg.length;
    tmp_result.remain_dist = line_seg.length;
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

  std::vector<Eigen::Vector2d> traj_bound;
  pnc::geometry_lib::PathPoint start_pose(line_seg.pA, line_seg.heading);
  pnc::geometry_lib::PathPoint target_pose(line_seg.pB, line_seg.heading);
  // CalTrajBound(traj_bound, start_pose, target_pose, true);
  traj_bound = CalTrajBound(start_pose, target_pose);

  // detect if there is intersection(point_P) between obstacle and car line
  // segment
  double min_obs_move_dist = 33.3;
  pnc::geometry_lib::LineSegment obs_move_line;
  // the cross points of obstacle lin seg and single car polygon line seg
  Eigen::Vector2d cross_point;
  // collision points between car movement along trajectory and obstacles
  Eigen::Vector2d col_pt_ego_global(0.0, 0.0);

  const Eigen::Vector2d AB = line_seg.pB - line_seg.pA;
  const Eigen::Vector2d unit_obs_move_line = -AB.normalized();

  // record final collision point info
  bool col_flag = false;
  ObsType col_obs_type = CHANNEL_OBS;
  size_t col_obs_index = 0;
  size_t col_car_line_order = 0;
  for (const auto &obs_pt_pair : obs_pt_global_map_) {
    for (size_t i = 0; i < obs_pt_pair.second.size(); ++i) {
      const Eigen::Vector2d obs_pt_global = obs_pt_pair.second[i];
      if (!pnc::geometry_lib::IsPointInPolygon(traj_bound, obs_pt_global) &&
          box) {
        continue;
      }
      for (size_t j = 0; j < car_line_global_vec.size(); ++j) {
        car_line_global = car_line_global_vec[j];
        obs_move_line.pA = obs_pt_global;
        obs_move_line.pB =
            obs_move_line.pA + min_obs_move_dist * unit_obs_move_line;
        if (GetIntersectionFromTwoLineSeg(cross_point, car_line_global,
                                          obs_move_line)) {
          const auto dist_CP = (cross_point - obs_move_line.pA).norm();
          if (dist_CP < min_obs_move_dist) {
            col_pt_ego_global = cross_point;
            min_obs_move_dist = dist_CP;
            col_flag = true;
            col_obs_type = static_cast<ObsType>(obs_pt_pair.first);
            col_obs_index = i;
            col_car_line_order = j;
          }
        }
      }
    }
  }

  CollisionResult result;

  result.remain_car_dist = AB.norm();
  result.remain_obstacle_dist = min_obs_move_dist;
  result.remain_dist =
      std::min(result.remain_obstacle_dist, result.remain_car_dist);

  result.collision_flag =
      (result.remain_obstacle_dist <=
       result.remain_car_dist + apa_param.GetParam().col_obs_safe_dist_normal +
           1e-3);

  result.col_pt_ego_global = col_pt_ego_global;
  pnc::geometry_lib::GlobalToLocalTf g2l_tf;
  g2l_tf.Init(line_seg.pA, heading_start);
  result.col_pt_ego_local = g2l_tf.GetPos(col_pt_ego_global);
  result.col_pt_obs_global.setZero();
  result.car_line_order = -1;
  if (col_flag && result.collision_flag) {
    result.col_pt_obs_global = obs_pt_global_map_[col_obs_type][col_obs_index];
    result.car_line_order = col_car_line_order;
    result.obs_type = col_obs_type;
  }
  result.traj_bound = traj_bound;
  return result;
}

const CollisionDetector::CollisionResult CollisionDetector::Update(
    const pnc::geometry_lib::Arc &arc, const double heading_start) {
  if (obs_pt_global_vec_.size() < 1) {
    CollisionResult tmp_result;
    tmp_result.remain_car_dist = arc.length;
    tmp_result.remain_dist = arc.length;
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
  Eigen::Vector2d col_pt_ego_global;
  col_pt_ego_global.setZero();
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
        col_pt_ego_global = cross_points.front();
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

  // DEBUG_PRINT("collision_point = " << collision_point.transpose()
  //           << "  obs_pt_global = " << obs_pt_global_vec_[i].transpose()
  //          );

  result.remain_dist =
      std::min(result.remain_obstacle_dist, result.remain_car_dist);

  result.col_pt_ego_global = col_pt_ego_global;
  result.col_pt_obs_global = obs_pt_global_vec_[i];
  return result;
}

const CollisionDetector::CollisionResult CollisionDetector::UpdateByObsMap(
    const pnc::geometry_lib::Arc &arc, const double heading_start) {
  if (obs_pt_global_map_.empty()) {
    CollisionResult tmp_result;
    tmp_result.remain_car_dist = arc.length;
    tmp_result.remain_dist = arc.length;
    return tmp_result;
  }

  // car line segment
  std::vector<pnc::geometry_lib::LineSegment> car_line_global_vec;
  car_line_global_vec.clear();
  car_line_global_vec.reserve(car_line_local_vec_.size());
  pnc::geometry_lib::LineSegment car_line_global;
  pnc::geometry_lib::LocalToGlobalTf l2g_tf;
  l2g_tf.Init(arc.pA, arc.headingA);
  for (const auto &car_line_local : car_line_local_vec_) {
    car_line_global.pA = l2g_tf.GetPos(car_line_local.pA);
    car_line_global.pB = l2g_tf.GetPos(car_line_local.pB);
    car_line_global_vec.emplace_back(car_line_global);
  }

  std::vector<Eigen::Vector2d> traj_bound;
  pnc::geometry_lib::PathPoint start_pose(arc.pA, arc.headingA);
  pnc::geometry_lib::PathPoint target_pose(arc.pB, arc.headingB);
  // CalTrajBound(traj_bound, start_pose, target_pose, false);
  traj_bound = CalTrajBound(start_pose, target_pose, arc);

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
  Eigen::Vector2d col_pt_ego_global(0.0, 0.0);

  // record final collision point info
  bool col_flag = false;
  ObsType col_obs_type = TLANE_OBS;
  size_t col_obs_index = 0;
  size_t col_car_line_order = 0;
  for (const auto &obs_pt_pair : obs_pt_global_map_) {
    for (size_t i = 0; i < obs_pt_pair.second.size(); ++i) {
      const Eigen::Vector2d obs_pt_global = obs_pt_pair.second[i];
      if (!pnc::geometry_lib::IsPointInPolygon(traj_bound, obs_pt_global) &&
          box) {
        continue;
      }
      for (size_t j = 0; j < car_line_global_vec.size(); ++j) {
        car_line_global = car_line_global_vec[j];
        obs_rot_circle.center = arc.circle_info.center;
        obs_rot_circle.radius = (obs_pt_global - obs_rot_circle.center).norm();
        const double num = pnc::geometry_lib::CalcCrossPointsOfLineSegAndCircle(
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
        const auto v_OD = D - arc.circle_info.center;

        const auto obs_rot_angle =
            pnc::geometry_lib::GetAngleFromTwoVec(v_OC, v_OD);

        if (obs_rot_angle * car_rot_angle < 0.0 &&
            fabs(obs_rot_angle) < fabs(min_obs_rot_limit_angle)) {
          // the rotation direction of obstacles and car must be opposite
          min_obs_rot_limit_angle = obs_rot_angle;
          col_pt_ego_global = D;
          col_flag = true;
          col_obs_type = static_cast<ObsType>(obs_pt_pair.first);
          col_obs_index = i;
          col_car_line_order = j;
        }
      }
    }
  }

  CollisionResult result;
  result.remain_car_dist = fabs(car_rot_angle) * arc.circle_info.radius;
  // result.remain_obstacle_dist =
  //     fabs(min_obs_rot_limit_angle) * obs_rot_circle.radius; //err cal method
  result.remain_obstacle_dist =
      fabs(min_obs_rot_limit_angle) * arc.circle_info.radius;

  result.remain_dist =
      std::min(result.remain_car_dist, result.remain_obstacle_dist);

  result.collision_flag =
      (result.remain_obstacle_dist <=
       result.remain_car_dist + apa_param.GetParam().col_obs_safe_dist_normal +
           1e-3);

  result.col_pt_ego_global = col_pt_ego_global;
  pnc::geometry_lib::GlobalToLocalTf g2l_tf;
  g2l_tf.Init(arc.pA, arc.headingA);
  result.col_pt_ego_local = g2l_tf.GetPos(col_pt_ego_global);
  result.col_pt_obs_global.setZero();
  result.car_line_order = -1;
  if (col_flag && result.collision_flag) {
    result.col_pt_obs_global = obs_pt_global_map_[col_obs_type][col_obs_index];
    result.car_line_order = col_car_line_order;
    result.obs_type = col_obs_type;
  }
  result.traj_bound = traj_bound;
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
  // DEBUG_PRINT("ego_pose = " << ego_pose.pos);
  for (const auto &car_line_global : car_line_global_vec) {
    min_dist = std::min(min_dist, pnc::geometry_lib::CalPoint2LineSegDist(
                                      obs_pos, car_line_global));
    // DEBUG_PRINT("min_dist = " << min_dist << "");
    // DEBUG_PRINT("car_line_global.pA = " << car_line_global.pA.transpose()
    //           << "car_line_global.pB = " << car_line_global.pB.transpose()
    //           << "");
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
  car_polygon.reserve(apa_param.GetParam().car_vertex_x_vec.size());
  for (const auto &car_line_local : car_line_local_vec_) {
    car_line_global.pA = l2g_tf.GetPos(car_line_local.pA);
    car_line_global.pB = l2g_tf.GetPos(car_line_local.pB);
    car_line_global_vec.emplace_back(car_line_global);
    car_polygon.emplace_back(car_line_global.pA);
  }

  return pnc::geometry_lib::IsPointInPolygon(car_polygon, obs_pos);
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
  DEBUG_PRINT("UpdateByLineObs");
  if (obs_line_global_vec_.empty()) {
    CollisionResult tmp_result;
    tmp_result.remain_car_dist = line_seg.length;
    tmp_result.remain_dist = line_seg.length;
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
  Eigen::Vector2d col_pt_ego_global(0.0, 0.0);
  // detect car traj and obs line is or not col
  for (auto &obs_line_global : obs_line_global_vec_) {
    for (auto &car_line_global : car_line_global_vec) {
      Eigen::Vector2d intersection;
      if (GetIntersectionFromTwoLineSeg(intersection, car_line_global,
                                        obs_line_global)) {
        const double dist = (car_line_global.pA - intersection).norm();
        if (dist < traj_dist) {
          traj_dist = dist;
          col_pt_ego_global = car_line_global.pA;
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

  result.col_pt_ego_global = col_pt_ego_global;
  return result;
}

const CollisionDetector::CollisionResult CollisionDetector::UpdateByLineObs(
    const pnc::geometry_lib::Arc &arc, const double heading_start) {
  if (obs_line_global_vec_.empty()) {
    CollisionResult tmp_result;
    tmp_result.remain_car_dist = arc.length;
    tmp_result.remain_dist = arc.length;
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

const std::vector<Eigen::Vector2d> CollisionDetector::CalTrajBound(
    const pnc::geometry_lib::PathPoint &start_pose,
    const pnc::geometry_lib::PathPoint &target_pose) {
  pnc::geometry_lib::LocalToGlobalTf l2g_tf_start;
  l2g_tf_start.Init(start_pose.pos, start_pose.heading);
  pnc::geometry_lib::LocalToGlobalTf l2g_tf_end;
  l2g_tf_end.Init(target_pose.pos, target_pose.heading);

  Eigen::Vector2d car_vertex;
  std::vector<Eigen::Vector2d> car_vertex_vec;
  car_vertex_vec.clear();
  car_vertex_vec.reserve(2 * car_line_local_vec_.size());
  for (size_t i = 0; i < car_line_local_vec_.size(); ++i) {
    car_vertex = l2g_tf_start.GetPos(car_line_local_vec_[i].pA);
    car_vertex_vec.emplace_back(car_vertex);
    car_vertex = l2g_tf_end.GetPos(car_line_local_vec_[i].pA);
    car_vertex_vec.emplace_back(car_vertex);
  }

  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  for (const Eigen::Vector2d &car_vertex : car_vertex_vec) {
    if (car_vertex.x() < min_x) {
      min_x = car_vertex.x();
    }
    if (car_vertex.y() < min_y) {
      min_y = car_vertex.y();
    }
    if (car_vertex.x() > max_x) {
      max_x = car_vertex.x();
    }
    if (car_vertex.y() > max_y) {
      max_y = car_vertex.y();
    }
  }

  const double bound_expand =
      apa_param.GetParam().col_obs_safe_dist_strict + 0.01;
  min_x -= bound_expand;
  min_y -= bound_expand;
  max_x += bound_expand;
  max_y += bound_expand;

  std::vector<Eigen::Vector2d> traj_bound;
  traj_bound.resize(4, Eigen::Vector2d(0.0, 0.0));
  traj_bound[0] << max_x, max_y;
  traj_bound[1] << max_x, min_y;
  traj_bound[2] << min_x, min_y;
  traj_bound[3] << min_x, max_y;

  return traj_bound;
}

const std::vector<Eigen::Vector2d> CollisionDetector::CalTrajBound(
    const pnc::geometry_lib::PathPoint &start_pose,
    const pnc::geometry_lib::PathPoint &target_pose,
    const pnc::geometry_lib::Arc &arc) {
  pnc::geometry_lib::LocalToGlobalTf l2g_tf_start;
  l2g_tf_start.Init(start_pose.pos, start_pose.heading);
  pnc::geometry_lib::LocalToGlobalTf l2g_tf_end;
  l2g_tf_end.Init(target_pose.pos, target_pose.heading);

  Eigen::Vector2d car_vertex;
  std::vector<Eigen::Vector2d> car_vertex_vec;
  std::vector<pnc::geometry_lib::Arc> arc_vec;
  arc_vec.reserve(car_line_local_vec_.size());
  pnc::geometry_lib::Arc temp_arc = arc;
  car_vertex_vec.clear();
  car_vertex_vec.reserve(2 * car_line_local_vec_.size());
  for (size_t i = 0; i < car_line_local_vec_.size(); ++i) {
    car_vertex = l2g_tf_start.GetPos(car_line_local_vec_[i].pA);
    car_vertex_vec.emplace_back(car_vertex);
    temp_arc.pA = car_vertex;
    car_vertex = l2g_tf_end.GetPos(car_line_local_vec_[i].pA);
    car_vertex_vec.emplace_back(car_vertex);
    temp_arc.pB = car_vertex;
    temp_arc.circle_info.radius =
        (temp_arc.pB - temp_arc.circle_info.center).norm();
    arc_vec.emplace_back(temp_arc);
  }

  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  for (const Eigen::Vector2d &car_vertex : car_vertex_vec) {
    if (car_vertex.x() < min_x) {
      min_x = car_vertex.x();
    }
    if (car_vertex.y() < min_y) {
      min_y = car_vertex.y();
    }
    if (car_vertex.x() > max_x) {
      max_x = car_vertex.x();
    }
    if (car_vertex.y() > max_y) {
      max_y = car_vertex.y();
    }
  }

  double bound_expand = 2.0;
  min_x -= bound_expand;
  min_y -= bound_expand;
  max_x += bound_expand;
  max_y += bound_expand;

  pnc::geometry_lib::LineSegment line_seg;
  std::pair<Eigen::Vector2d, Eigen::Vector2d> intersections;

  bool flag = true;
  uint8_t i = 0;
  const uint8_t count = 9;
  const double dx_dy = 0.3;

  do {
    i++;
    if (i > count) {
      break;
    }
    line_seg.pA << max_x, max_y;
    line_seg.pB << max_x, min_y;
    for (const auto &arc : arc_vec) {
      if (pnc::geometry_lib::GetArcLineIntersection(intersections, arc,
                                                    line_seg) > 0) {
        flag = false;
        break;
      }
    }
    if (flag) {
      max_x -= dx_dy;
    } else {
      max_x += dx_dy;
    }
  } while (flag);

  flag = true;
  i = 0;
  do {
    i++;
    if (i > count) {
      break;
    }
    line_seg.pA << max_x, max_y;
    line_seg.pB << min_x, max_y;
    for (const auto &arc : arc_vec) {
      if (pnc::geometry_lib::GetArcLineIntersection(intersections, arc,
                                                    line_seg) > 0) {
        flag = false;
        break;
      }
    }
    if (flag) {
      max_y -= dx_dy;
    } else {
      max_y += dx_dy;
    }
  } while (flag);

  flag = true;
  i = 0;
  do {
    i++;
    if (i > count) {
      break;
    }
    line_seg.pA << min_x, min_y;
    line_seg.pB << min_x, max_y;
    for (const auto &arc : arc_vec) {
      if (pnc::geometry_lib::GetArcLineIntersection(intersections, arc,
                                                    line_seg) > 0) {
        flag = false;
        break;
      }
    }
    if (flag) {
      min_x += dx_dy;
    } else {
      min_x -= dx_dy;
    }
  } while (flag);

  flag = true;
  i = 0;
  do {
    i++;
    if (i > count) {
      break;
    }
    line_seg.pA << min_x, min_y;
    line_seg.pB << max_x, min_y;
    for (const auto &arc : arc_vec) {
      if (pnc::geometry_lib::GetArcLineIntersection(intersections, arc,
                                                    line_seg) > 0) {
        flag = false;
        break;
      }
    }
    if (flag) {
      min_y += dx_dy;
    } else {
      min_y -= dx_dy;
    }
  } while (flag);

  // DEBUG_PRINT("max_x = " << max_x << "  min_x = " << min_x
  //                        << "  max_y = " << max_y << "  min_y = " << min_y);

  bound_expand = apa_param.GetParam().col_obs_safe_dist_strict + 0.01;

  min_x -= bound_expand;
  min_y -= bound_expand;
  max_x += bound_expand;
  max_y += bound_expand;

  std::vector<Eigen::Vector2d> traj_bound;
  traj_bound.resize(4, Eigen::Vector2d(0.0, 0.0));
  traj_bound[0] << max_x, max_y;
  traj_bound[1] << max_x, min_y;
  traj_bound[2] << min_x, min_y;
  traj_bound[3] << min_x, max_y;

  return traj_bound;
}

const bool CollisionDetector::CalTrajBound(
    std::vector<Eigen::Vector2d> &traj_bound,
    const pnc::geometry_lib::PathPoint &start_pose,
    const pnc::geometry_lib::PathPoint &target_pose, bool is_line) {
  pnc::geometry_lib::LocalToGlobalTf l2g_tf_start;
  l2g_tf_start.Init(start_pose.pos, start_pose.heading);
  pnc::geometry_lib::LocalToGlobalTf l2g_tf_end;
  l2g_tf_end.Init(target_pose.pos, target_pose.heading);

  Eigen::Vector2d car_vertex;
  std::vector<Eigen::Vector2d> car_vertex_vec;
  car_vertex_vec.clear();
  car_vertex_vec.reserve(2 * car_line_local_vec_.size());
  for (size_t i = 0; i < car_line_local_vec_.size(); ++i) {
    car_vertex = l2g_tf_start.GetPos(car_line_local_vec_[i].pA);
    car_vertex_vec.emplace_back(car_vertex);
    car_vertex = l2g_tf_end.GetPos(car_line_local_vec_[i].pA);
    car_vertex_vec.emplace_back(car_vertex);
  }

  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  for (const Eigen::Vector2d &car_vertex : car_vertex_vec) {
    if (car_vertex.x() < min_x) {
      min_x = car_vertex.x();
    }
    if (car_vertex.y() < min_y) {
      min_y = car_vertex.y();
    }
    if (car_vertex.x() > max_x) {
      max_x = car_vertex.x();
    }
    if (car_vertex.y() > max_y) {
      max_y = car_vertex.y();
    }
  }
  const double bound_expand = is_line ? 0.168 : param_.bound_expand;
  min_x -= bound_expand;
  min_y -= bound_expand;
  max_x += bound_expand;
  max_y += bound_expand;

  traj_bound.resize(4, Eigen::Vector2d(0.0, 0.0));
  traj_bound[0] << max_x, max_y;
  traj_bound[1] << max_x, min_y;
  traj_bound[2] << min_x, min_y;
  traj_bound[3] << min_x, max_y;

  return true;
}

const CollisionDetector::ObsSlotType CollisionDetector::GetObsSlotType(
    const Eigen::Vector2d &obs,
    const std::pair<Eigen::Vector2d, Eigen::Vector2d> &slot_pt,
    const bool is_left_side, const bool is_vertical_slot) {
  if (is_vertical_slot) {
    // the slot origin pt is on the line(pt2->pt3)
    Eigen::Vector2d slot_left_pt = slot_pt.first;
    Eigen::Vector2d slot_right_pt = slot_pt.second;
    if (slot_left_pt.y() < slot_right_pt.y()) {
      std::swap(slot_left_pt, slot_right_pt);
    }
    const double max_obs_lat_invasion_slot_dist =
        apa_param.GetParam().max_obs_lat_invasion_slot_dist;
    const double max_obs_lon_invasion_slot_dist =
        apa_param.GetParam().max_obs_lon_invasion_slot_dist;
    const Eigen::Vector2d unit_01_vec =
        (slot_left_pt - slot_right_pt).normalized();
    const Eigen::Vector2d unit_02_vec(-1.0, 0.0);
    std::vector<Eigen::Vector2d> area_vec;
    area_vec.resize(4);
    const double max_x = std::max(slot_left_pt.x(), slot_right_pt.x());

    const Eigen::Vector2d pt_1 = slot_left_pt -
                                 max_obs_lat_invasion_slot_dist * unit_01_vec -
                                 1.068 * unit_02_vec;

    const Eigen::Vector2d pt_0 = slot_right_pt +
                                 max_obs_lat_invasion_slot_dist * unit_01_vec -
                                 1.068 * unit_02_vec;

    const Eigen::Vector2d pt_2 = slot_right_pt + max_x * unit_02_vec +
                                 max_obs_lat_invasion_slot_dist * unit_01_vec -
                                 max_obs_lon_invasion_slot_dist * unit_02_vec;

    const Eigen::Vector2d pt_3 = slot_left_pt + max_x * unit_02_vec -
                                 max_obs_lat_invasion_slot_dist * unit_01_vec -
                                 max_obs_lon_invasion_slot_dist * unit_02_vec;

    area_vec[0] = pt_1;
    area_vec[1] = pt_0;
    area_vec[2] = pt_2;
    area_vec[3] = pt_3;

    if (pnc::geometry_lib::IsPointInPolygon(area_vec, obs)) {
      return ObsSlotType::SLOT_IN_OBS;
    }

    area_vec[0] = pt_3;
    area_vec[1] = pt_2;
    area_vec[2] = pt_2 + 0.68 * unit_02_vec;
    area_vec[3] = pt_3 + 0.68 * unit_02_vec;

    if (pnc::geometry_lib::IsPointInPolygon(area_vec, obs)) {
      return ObsSlotType::SLOT_DIRECTLY_BEHIND_OBS;
    }

    area_vec[0] = pt_1 + 0.86 * unit_01_vec - 3.468 * unit_02_vec;
    area_vec[1] = pt_0 - 0.86 * unit_01_vec - 3.468 * unit_02_vec;
    area_vec[2] = pt_0 - 0.86 * unit_01_vec;
    area_vec[3] = pt_1 + 0.86 * unit_01_vec;

    if (pnc::geometry_lib::IsPointInPolygon(area_vec, obs)) {
      return ObsSlotType::SLOT_ENTRANCE_OBS;
    }

    area_vec[0] = pt_0;
    area_vec[1] = pt_0 - 0.86 * unit_01_vec;
    area_vec[2] = pt_2 - 0.86 * unit_01_vec;
    area_vec[3] = pt_2;

    if (pnc::geometry_lib::IsPointInPolygon(area_vec, obs)) {
      if (is_left_side) {
        return ObsSlotType::SLOT_OUTSIDE_OBS;
      } else {
        return ObsSlotType::SLOT_INSIDE_OBS;
      }
    }

    area_vec[0] = pt_1 + 0.86 * unit_01_vec;
    area_vec[1] = pt_1;
    area_vec[2] = pt_3;
    area_vec[3] = pt_3 + 0.86 * unit_01_vec;

    if (pnc::geometry_lib::IsPointInPolygon(area_vec, obs)) {
      if (is_left_side) {
        return ObsSlotType::SLOT_INSIDE_OBS;
      } else {
        return ObsSlotType::SLOT_OUTSIDE_OBS;
      }
    }

    return ObsSlotType::SLOT_OUT_OBS;
  }

  return ObsSlotType::OBS_INVALID;
}
}  // namespace apa_planner
}  // namespace planning