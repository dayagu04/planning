#include "apa_trajectory_stitcher.h"

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
    const double front_wheel_angle, trajectory::Trajectory* trajectory) {
  double kappa = std::tan(front_wheel_angle) / apa_param.GetParam().wheel_base;
  Pose2D predict_pose = ComputeTrajPointByPrediction(ego_pose, ego_v, kappa);

  size_t stitch_id = 0;
  if (QueryNearestPoint(predict_pose, path, &stitch_id)) {
    GeneTrajPointFromPath(path[stitch_id]);
  } else {
    GeneTrajPointFromVehicleState(ego_pose);
  }

  return;
}

void ApaTrajectoryStitcher::GeneTrajPointFromVehicleState(
    const Pose2D& ego_pose) {
  ClearSticthPoint();
  stitch_point_.x = ego_pose.x;
  stitch_point_.y = ego_pose.y;
  stitch_point_.heading_angle = ego_pose.theta;

  return;
}

void ApaTrajectoryStitcher::GeneTrajPointFromPath(
    const pnc::geometry_lib::PathPoint& point) {
  ClearSticthPoint();
  stitch_point_.x = point.pos.x();
  stitch_point_.y = point.pos.y();
  stitch_point_.heading_angle = point.heading;

  return;
}

Pose2D ApaTrajectoryStitcher::ComputeTrajPointByPrediction(
    const Pose2D& ego_pose, const double ego_v, const double kappa) {
  Pose2D predict_pose;
  double dist = std::fabs(ego_v * 0.1);

  if (std::fabs(ego_v) < 1e-2) {
    predict_pose = ego_pose;
  } else if (std::fabs(kappa) < 1e-4) {
    PredictByLine(ego_pose, dist, true, &predict_pose);
  } else {
    double radius = 1 / kappa;
    PredictByCircle(ego_pose, dist, radius, true, &predict_pose);
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
  stitch_point_.s = 0;
  stitch_point_.l = 0;
  stitch_point_.x = 0;
  stitch_point_.y = 0;
  stitch_point_.heading_angle = 0;
  stitch_point_.curvature = 0;
  stitch_point_.dkappa = 0;
  stitch_point_.ddkappa = 0;
  stitch_point_.t = 0;
  stitch_point_.v = 0;
  stitch_point_.a = 0;
  stitch_point_.jerk = 0;
  stitch_point_.frenet_valid = false;

  return;
}

}  // namespace planning