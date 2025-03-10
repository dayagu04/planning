#include "apa_trajectory_stitcher.h"

#include <cmath>
#include <cstddef>

#include "apa_param_config.h"
#include "math/vec2d.h"
#include "pose2d.h"
#include "src/library/reeds_shepp/rs_path_interpolate.h"

#include <cmath>
#include <cstddef>

#include "apa_param_config.h"
#include "math/vec2d.h"
#include "pose2d.h"
#include "src/library/reeds_shepp/rs_path_interpolate.h"

namespace planning {

void ApaTrajectoryStitcher::Process(
    const Pose2D& ego_pose,
    const std::vector<pnc::geometry_lib::PathPoint>& path, const double ego_v,
    const double front_wheel_angle) {
  double kappa = std::tan(front_wheel_angle) / apa_param.GetParam().wheel_base;
  Pose2D predict_pose = ComputeTrajPointByPrediction(ego_pose, ego_v, kappa);

  size_t min_dist_id = 0;
  size_t min_dist_neioghbour_id = 0;
  if (QueryNearestPoint(predict_pose, path, &min_dist_id,
                        &min_dist_neioghbour_id)) {
    EvaluateStitchTraj(predict_pose, path, min_dist_id, min_dist_neioghbour_id);
  } else {
    GeneTrajPointFromVehicleState(ego_pose);
  }

  return;
}

void ApaTrajectoryStitcher::GeneTrajPointFromVehicleState(
    const Pose2D& ego_pose) {
  ClearSticthPoint();
  stitch_point_ = pnc::geometry_lib::PathPoint(
      Eigen::Vector2d(ego_pose.x, ego_pose.y), ego_pose.theta);

  drived_distance_ = 0.0;

  return;
}

void ApaTrajectoryStitcher::GeneTrajPointFromPath(
    const pnc::geometry_lib::PathPoint& point) {
  ClearSticthPoint();
  stitch_point_ = point;

  return;
}

Pose2D ApaTrajectoryStitcher::ComputeTrajPointByPrediction(
    const Pose2D& ego_pose, const double ego_v, const double kappa) {
  Pose2D predict_pose;
  double dist = std::fabs(ego_v * 0.1);

  if (std::fabs(ego_v) < 1e-2) {
    predict_pose = ego_pose;
  } else if (std::fabs(kappa) < 1e-4) {
    PredictByLine(ego_pose, dist, ego_v > 0.0 ? true : false, &predict_pose);
  } else {
    double radius = 1 / kappa;
    PredictByCircle(ego_pose, dist, radius, ego_v > 0.0 ? true : false,
                    &predict_pose);
  }

  return predict_pose;
}

void ApaTrajectoryStitcher::PredictByLine(const Pose2D& ego,
                                          const double move_dist,
                                          const bool is_forward,
                                          Pose2D* new_pose) {
  double dist;
  if (is_forward) {
    dist = move_dist;
  } else {
    dist = -move_dist;
  }

  // get unit vector
  Pose2D unit_vector;
  unit_vector.x = std::cos(ego.GetPhi());
  unit_vector.y = std::sin(ego.GetPhi());

  new_pose->x = ego.x + dist * unit_vector.x;
  new_pose->y = ego.y + dist * unit_vector.y;
  new_pose->theta = ego.theta;

  return;
}

void ApaTrajectoryStitcher::PredictByCircle(const Pose2D& ego, const double arc,
                                            const double radius,
                                            const bool is_forward,
                                            Pose2D* new_pose) {
  // get vehicle circle
  VehicleCircle veh_circle;
  AstarPathGear gear;
  if (is_forward) {
    gear = AstarPathGear::DRIVE;
  } else {
    gear = AstarPathGear::REVERSE;
  }

  GetVehCircleByPose(&veh_circle, &ego, radius, gear);
  double inverse_radius = std::fabs(1.0 / radius);

  GetCirclePoint(&veh_circle, &ego, std::fabs(arc), inverse_radius, new_pose);

  return;
}

void ApaTrajectoryStitcher::GetVehCircleByPose(VehicleCircle* veh_circle,
                                               const Pose2D* pose,
                                               const double radius,
                                               const AstarPathGear gear) {
  veh_circle->radius = radius;
  veh_circle->gear = gear;

  veh_circle->center.x = pose->x - radius * std::sin(pose->theta);
  veh_circle->center.y = pose->y + radius * std::cos(pose->theta);

  return;
}

void ApaTrajectoryStitcher::GetCirclePoint(const VehicleCircle* veh_circle,
                                           const Pose2D* start_pose,
                                           const double arc,
                                           const double inverse_radius,
                                           Pose2D* pose) {
  double delta_theta, theta;
  delta_theta = arc * inverse_radius;

  // left turn
  if (veh_circle->radius > 0.0) {
    if (veh_circle->gear == AstarPathGear::REVERSE) {
      delta_theta = -delta_theta;
    }
  } else {
    // right turn, gear is d
    if (veh_circle->gear == AstarPathGear::DRIVE) {
      delta_theta = -delta_theta;
    }
  }

  // update next point theta
  theta = start_pose->theta + delta_theta;
  double radius = veh_circle->radius;
  pose->x = veh_circle->center.x + radius * std::sin(theta);
  pose->y = veh_circle->center.y - radius * std::cos(theta);
  pose->theta = IflyUnifyTheta(theta, M_PI);

  return;
}

// todo: consider gear
bool ApaTrajectoryStitcher::QueryNearestPoint(
    const Pose2D& pose, const std::vector<pnc::geometry_lib::PathPoint>& path,
    size_t* index) const {
  if (path.size() < 2) {
    return false;
  }

  double dist_sqr_min = std::numeric_limits<double>::max();
  size_t index_min = 0;
  double dist_sqr;

  for (size_t i = 0; i < path.size(); ++i) {
    dist_sqr = pose.DistanceSquareTo(path[i].pos);

    if (dist_sqr < dist_sqr_min + 1e-3) {
      dist_sqr_min = dist_sqr;
      index_min = i;
    }
  }

  // 规划起点应当选择未来点
  ad_common::math::Vec2d path_vector;
  ad_common::math::Vec2d path_ego_vector;
  if (index_min + 1 < path.size()) {
    path_vector.set_x(path[index_min + 1].pos.x() - path[index_min].pos.x());
    path_vector.set_y(path[index_min + 1].pos.y() - path[index_min].pos.y());

  } else {
    path_vector.set_x(path[index_min].pos.x() - path[index_min - 1].pos.x());
    path_vector.set_y(path[index_min].pos.y() - path[index_min - 1].pos.y());
  }

  path_ego_vector.set_x(pose.x - path[index_min].pos.x());
  path_ego_vector.set_y(pose.y - path[index_min].pos.y());

  size_t expect_index;
  if (path_vector.InnerProd(path_ego_vector) > 0.0) {
    expect_index = index_min + 1;
  } else {
    expect_index = index_min;
  }

  // 车辆超过轨迹末端
  if (expect_index >= path.size()) {
    return false;
  }

  *index = expect_index;

  return true;
}

void ApaTrajectoryStitcher::ClearSticthPoint() {
  stitch_point_.Reset();
  return;
}

bool ApaTrajectoryStitcher::QueryNearestPoint(
    const Pose2D& ego_pose,
    const std::vector<pnc::geometry_lib::PathPoint>& path,
    size_t* min_dist_point, size_t* min_dist_point_neighbor) {
  if (path.size() < 2) {
    return false;
  }

  // min dist point
  double dist_sqr_min = std::numeric_limits<double>::max();
  size_t index_min = 0;
  double dist_sqr;

  for (size_t i = 0; i < path.size(); ++i) {
    dist_sqr = ego_pose.DistanceSquareTo(path[i].pos);

    if (dist_sqr < dist_sqr_min + 1e-3) {
      dist_sqr_min = dist_sqr;
      index_min = i;
    }
  }

  // check neighbour
  size_t neighbour_id = 0;
  if (index_min == 0) {
    neighbour_id = 1;
  } else if (index_min + 1 == path.size()) {
    neighbour_id = index_min - 1;
  } else {
    double left_neighbour_dist =
        ego_pose.DistanceSquareTo(path[index_min - 1].pos);
    double right_neighbour_dist =
        ego_pose.DistanceSquareTo(path[index_min + 1].pos);
    if (left_neighbour_dist < right_neighbour_dist) {
      neighbour_id = index_min - 1;
    } else {
      neighbour_id = index_min + 1;
    }
  }

  *min_dist_point = index_min;
  *min_dist_point_neighbor = neighbour_id;

  return true;
}

void ApaTrajectoryStitcher::EvaluateStitchTraj(
    const Pose2D& ego_pose,
    const std::vector<pnc::geometry_lib::PathPoint>& path,
    const size_t min_dist_point_id, const size_t min_dist_point_neighbor_id) {
  ad_common::math::Vec2d predecessor;
  ad_common::math::Vec2d successor;
  ad_common::math::Vec2d base_vector;
  size_t predecessor_point_id;
  size_t successor_point_id;

  if (min_dist_point_id < min_dist_point_neighbor_id) {
    predecessor_point_id = min_dist_point_id;
    successor_point_id = min_dist_point_neighbor_id;
  } else {
    predecessor_point_id = min_dist_point_neighbor_id;
    successor_point_id = min_dist_point_id;
  }

  predecessor.set_x(path[predecessor_point_id].pos.x());
  predecessor.set_y(path[predecessor_point_id].pos.y());
  successor.set_x(path[successor_point_id].pos.x());
  successor.set_y(path[successor_point_id].pos.y());

  base_vector = successor - predecessor;
  base_vector.Normalize();

  ad_common::math::Vec2d predecessor_to_ego(ego_pose.x - predecessor.x(),
                                            ego_pose.y - predecessor.y());

  // evaluate stitch point
  ad_common::math::Vec2d evaluate_point;
  int global_path_stitch_start_id;
  double dot = predecessor_to_ego.InnerProd(base_vector);
  if (dot < 0.0) {
    global_path_stitch_start_id = predecessor_point_id;
  } else if (dot > 1.0) {
    global_path_stitch_start_id = successor_point_id + 1;
  } else {
    global_path_stitch_start_id = successor_point_id;
  }

  evaluate_point = predecessor + base_vector * dot;

  stitch_point_ = path[min_dist_point_id];
  stitch_point_.pos[0] = evaluate_point.x();
  stitch_point_.pos[1] = evaluate_point.y();

  // evaluate stitch traj
  trajectory_.clear();
  trajectory_.emplace_back(stitch_point_);
  for (size_t i = global_path_stitch_start_id; i<path.size(); i++) {
    trajectory_.emplace_back(path[i]);
  }

    // get path lengh
  double accumulated_s = 0.0;
  auto last_x = trajectory_.front().pos.x();
  auto last_y = trajectory_.front().pos.y();
  double x_diff;
  double y_diff;

  for (size_t i = 0; i < trajectory_.size(); ++i) {
    x_diff = trajectory_[i].pos[0] - last_x;
    y_diff = trajectory_[i].pos[1] - last_y;
    accumulated_s += std::sqrt(x_diff * x_diff + y_diff * y_diff);
    trajectory_[i].s = accumulated_s;

    last_x = trajectory_[i].pos[0];
    last_y = trajectory_[i].pos[1];
  }

  drived_distance_ = path.back().s - trajectory_.back().s;

  return;
}

}  // namespace planning