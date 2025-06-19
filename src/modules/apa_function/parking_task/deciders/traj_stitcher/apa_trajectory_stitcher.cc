#include "apa_trajectory_stitcher.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "apa_param_config.h"
#include "dp_speed_common.h"
#include "geometry_math.h"
#include "log_glog.h"
#include "math/vec2d.h"
#include "path/discretized_path.h"
#include "pose2d.h"
#include "src/common/utils.h"

namespace planning {
namespace apa_planner {

#define DEBUG_TASK (1)

void ApaTrajectoryStitcher::Execute(
    const Pose2D& ego_pose,
    const std::vector<pnc::geometry_lib::PathPoint>& lateral_path,
    const double front_wheel_angle, const SVPoint& ego_lon_point,
    const double predict_horizon,
    const trajectory::Trajectory& history_trajectory,
    const pnc::geometry_lib::PathSegGear gear) {
  ego_lon_state_ = ego_lon_point;
  gear_ = gear;
  double kappa = std::tan(front_wheel_angle) / apa_param.GetParam().wheel_base;
  double sign_v = std::abs(ego_lon_point.v);
  if (gear_ == pnc::geometry_lib::PathSegGear::SEG_GEAR_REVERSE) {
    sign_v = -sign_v;
  }

  Pose2D predict_pose =
      ComputeTrajPointByPrediction(ego_pose, sign_v, kappa, predict_horizon);

  // evaluate path
  size_t min_dist_id = 0;
  size_t min_dist_neioghbour_id = 0;
  if (QueryNearestPoint(predict_pose, lateral_path, &min_dist_id,
                        &min_dist_neioghbour_id)) {
    EvaluateStitchPath(predict_pose, lateral_path, min_dist_id,
                       min_dist_neioghbour_id);
  } else {
    GenePathPointFromVehicleState(ego_pose);
  }

  // evaluate speed
  bool same_gear = false;
  if (history_trajectory.GetGear() == 1 &&
      gear_ == pnc::geometry_lib::PathSegGear::SEG_GEAR_REVERSE) {
    same_gear = true;
  } else if (history_trajectory.GetGear() == 2 &&
             gear_ == pnc::geometry_lib::PathSegGear::SEG_GEAR_DRIVE) {
    same_gear = true;
  }

  bool lon_stitch_success = false;
  if (same_gear) {
    lon_stitch_success = history_trajectory.QueryNearestPointWithBuffer(
        planning_math::Vec2d(predict_pose.x, predict_pose.y), 0.0,
        &lon_stitch_point_);
  }

  if (!lon_stitch_success || lon_stitch_point_.s() < 0.0 ||
      lon_stitch_point_.vel() <= 0.01) {
    ILOG_INFO << "lon_stitch_success = " << lon_stitch_success
              << " lon stitch v = " << lon_stitch_point_.vel()
              << ", a = " << lon_stitch_point_.acc()
              << ", s = " << lon_stitch_point_.s();
    GeneSpeedPointFromVehicleState(ego_lon_point);
  }

  SmoothLonDelay();

#if DEBUG_TASK
  TaskDebug(lateral_path, history_trajectory);
#endif

  return;
}

void ApaTrajectoryStitcher::SmoothLonDelay() {
  double v_error = lon_stitch_point_.vel() - ego_lon_state_.v;
  double acc_error = lon_stitch_point_.acc() - ego_lon_state_.acc;

  if (v_error > 0.1) {
    lon_stitch_point_.set_vel(ego_lon_state_.v + 0.1);
  } else if (v_error < -0.1) {
    lon_stitch_point_.set_vel(ego_lon_state_.v - 0.1);
  }

  double v = lon_stitch_point_.vel();
  v = std::max(0.0, v);
  lon_stitch_point_.set_vel(v);

  if (acc_error > 0.1) {
    lon_stitch_point_.set_acc(ego_lon_state_.acc + 0.1);
  } else if (acc_error < -0.1) {
    lon_stitch_point_.set_acc(ego_lon_state_.acc - 0.1);
  }
  return;
}

void ApaTrajectoryStitcher::GenePathPointFromVehicleState(
    const Pose2D& ego_pose) {
  ClearSticthPoint();
  stitch_path_point_ = pnc::geometry_lib::PathPoint(
      Eigen::Vector2d(ego_pose.x, ego_pose.y), ego_pose.theta);
  stitch_path_.push_back(stitch_path_point_);

  return;
}

void ApaTrajectoryStitcher::GeneSpeedPointFromVehicleState(
    const SVPoint& init_point) {
  lon_stitch_point_.set_x(stitch_path_point_.pos[0]);
  lon_stitch_point_.set_y(stitch_path_point_.pos[1]);
  lon_stitch_point_.set_theta(stitch_path_point_.heading);
  lon_stitch_point_.set_kappa(0.0);
  lon_stitch_point_.set_dkappa(0.0);
  lon_stitch_point_.set_s(0);

  lon_stitch_point_.set_absolute_time(0);
  lon_stitch_point_.set_vel(init_point.v);

  // because of measure error, limit acc range.
  double acc = init_point.acc;
  acc = std::max(acc, -0.3);
  acc = std::min(acc, 0.3);
  if (init_point.v < 0.1) {
    acc = 0.0;
  }

  lon_stitch_point_.set_acc(acc);
  lon_stitch_point_.set_jerk(0);

  return;
}

Pose2D ApaTrajectoryStitcher::ComputeTrajPointByPrediction(
    const Pose2D& ego_pose, const double ego_v, const double kappa,
    const double predict_horizon) {
  Pose2D predict_pose;
  double dist = std::fabs(ego_v * predict_horizon);

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
    dist_sqr =
        pose.DistanceSquareTo(Eigen::Vector2f(path[i].pos[0], path[i].pos[1]));

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
  stitch_path_point_.Reset();
  stitch_path_.clear();
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
    dist_sqr = ego_pose.DistanceSquareTo(
        Eigen::Vector2f(path[i].pos[0], path[i].pos[1]));

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
    double left_neighbour_dist = ego_pose.DistanceSquareTo(Eigen::Vector2f(
        path[index_min - 1].pos[0], path[index_min - 1].pos[1]));
    double right_neighbour_dist = ego_pose.DistanceSquareTo(Eigen::Vector2f(
        path[index_min + 1].pos[0], path[index_min + 1].pos[1]));
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

void ApaTrajectoryStitcher::EvaluateStitchPath(
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
  ad_common::math::Vec2d evaluate_point;
  double base_vector_length = base_vector.Length();
  int next_stitch_id;
  if (base_vector_length < 1e-5) {
    evaluate_point = successor;
    next_stitch_id = successor_point_id;
  } else {
    base_vector.Normalize();
    ad_common::math::Vec2d predecessor_to_ego(ego_pose.x - predecessor.x(),
                                              ego_pose.y - predecessor.y());

    // evaluate stitch point
    double dot = predecessor_to_ego.InnerProd(base_vector);
    evaluate_point = predecessor + base_vector * dot;

    if (dot < 0.0) {
      next_stitch_id = predecessor_point_id;
    } else if (dot > base_vector_length) {
      next_stitch_id = successor_point_id + 1;
    } else {
      next_stitch_id = successor_point_id;
    }
  }

  stitch_path_point_ = path[min_dist_point_id];
  stitch_path_point_.pos[0] = evaluate_point.x();
  stitch_path_point_.pos[1] = evaluate_point.y();
  stitch_path_point_.s = 0;

  // evaluate stitch traj
  stitch_path_.clear();
  stitch_path_.emplace_back(stitch_path_point_);

  ILOG_INFO << "path size = " << path.size()
            << ", next stitch id = " << next_stitch_id;
  for (size_t i = next_stitch_id; i < path.size(); i++) {
    stitch_path_.emplace_back(path[i]);
  }

  // get path lengh
  double accumulated_s = 0.0;
  auto last_x = stitch_path_.front().pos.x();
  auto last_y = stitch_path_.front().pos.y();
  double x_diff;
  double y_diff;

  for (size_t i = 0; i < stitch_path_.size(); ++i) {
    x_diff = stitch_path_[i].pos[0] - last_x;
    y_diff = stitch_path_[i].pos[1] - last_y;
    accumulated_s += std::sqrt(x_diff * x_diff + y_diff * y_diff);
    stitch_path_[i].s = accumulated_s;

    last_x = stitch_path_[i].pos[0];
    last_y = stitch_path_[i].pos[1];
  }

  return;
}

const double ApaTrajectoryStitcher::GetStitchPathLength() const {
  if (stitch_path_.empty()) {
    return 0.0;
  }

  return stitch_path_.back().s;
}

const std::vector<pnc::geometry_lib::PathPoint>&
ApaTrajectoryStitcher::GetConstStitchPath() const {
  return stitch_path_;
}

const trajectory::Trajectory& ApaTrajectoryStitcher::GetConstCombinedTraj()
    const {
  return trajectory_;
}

std::vector<pnc::geometry_lib::PathPoint>&
ApaTrajectoryStitcher::GetMutableStitchPath() {
  return stitch_path_;
}

const pnc::geometry_lib::PathPoint*
ApaTrajectoryStitcher::GetStitchPathPointPtr() {
  return &stitch_path_point_;
}

const pnc::geometry_lib::PathPoint& ApaTrajectoryStitcher::GetStitchPathPoint()
    const {
  return stitch_path_point_;
}

void ApaTrajectoryStitcher::CombineTrajBasedOnTime(
    const SpeedData& speed_profile) {
  trajectory_.Clear();

  // sanity check
  if (speed_profile.empty()) {
    ILOG_INFO << "speed profile is empty";
    return;
  }

  DiscretizedPath path_data;
  planning_math::PathPoint path_point;
  for (size_t i = 0; i < stitch_path_.size(); ++i) {
    path_point.set_x(stitch_path_[i].pos.x());
    path_point.set_y(stitch_path_[i].pos.y());
    path_point.set_theta(stitch_path_[i].heading);
    path_point.set_s(stitch_path_[i].s);
    path_point.set_kappa(stitch_path_[i].kappa);

    path_data.push_back(path_point);
  }

  const SpeedPoint* speed_point = nullptr;
  trajectory::TrajectoryPoint traj_point;
  bool is_forward = (gear_ == pnc::geometry_lib::SEG_GEAR_DRIVE ? true : false);
  bool is_overshoot;
  for (size_t i = 0; i < speed_profile.size(); i++) {
    speed_point = &speed_profile[i];

    if (speed_point->s > path_data.back().s()) {
      is_overshoot = true;
      path_point = path_data.EvaluateOvershootPoint(speed_point->s, is_forward);
    } else {
      is_overshoot = false;
      path_point = path_data.Evaluate(speed_point->s);
    }

    // check same point
    if (trajectory_.size() > 0) {
      if (common::DistanceXY(trajectory_.back(), path_point) < 1e-5) {
        continue;
      }
    }

    traj_point.set_absolute_time(speed_point->t);
    traj_point.set_s(speed_point->s);

    traj_point.set_vel(speed_point->v);
    traj_point.set_acc(speed_point->a);
    traj_point.set_x(path_point.x());
    traj_point.set_y(path_point.y());
    traj_point.set_theta(path_point.theta());
    traj_point.set_kappa(path_point.kappa());
    traj_point.set_jerk(speed_point->da);

    trajectory_.push_back(traj_point);
  }

  if (!is_overshoot && trajectory_.back().s() < path_data.back().s()) {
    double extend_total_s = path_data.back().s() - trajectory_.back().s();
    double extend_point_size = std::ceil(extend_total_s / 0.1);
    double cur_s;

    // if traj point speed is not computed in optimizer, default value is 0.0.
    // double extend_point_v = trajectory_.back().vel();
    double extend_point_v = 0;
    for (size_t i = 0; i < extend_point_size; i++) {
      cur_s = trajectory_.back().s() + 0.1;
      cur_s = std::min(cur_s, path_data.back().s());
      path_point = path_data.Evaluate(cur_s);

      // check same point
      if (trajectory_.size() > 0) {
        if (common::DistanceXY(trajectory_.back(), path_point) < 1e-5) {
          continue;
        }
      }

      traj_point.set_absolute_time(trajectory_.back().absolute_time() + 0.1);
      traj_point.set_s(cur_s);
      traj_point.set_vel(extend_point_v);
      traj_point.set_acc(-0.1);
      traj_point.set_jerk(0);
      traj_point.set_x(path_point.x());
      traj_point.set_y(path_point.y());
      traj_point.set_theta(path_point.theta());
      traj_point.set_kappa(path_point.kappa());

      trajectory_.push_back(traj_point);
    }
  }

  if (gear_ == pnc::geometry_lib::PathSegGear::SEG_GEAR_DRIVE) {
    trajectory_.SetGear(2);
  } else if (gear_ == pnc::geometry_lib::PathSegGear::SEG_GEAR_REVERSE) {
    trajectory_.SetGear(1);
  } else {
    trajectory_.SetGear(0);
  }

  if (trajectory_.size() < 3) {
    trajectory_.ExtendTraj(0.1);
  }

  ILOG_INFO << "is overshoot = " << is_overshoot << ", gear = " << is_forward
            << ", traj back s = " << trajectory_.back().s()
            << ", speed back s = " << speed_profile.back().s;

  return;
}

void ApaTrajectoryStitcher::CombineTrajBasedOnPath() { return; }

void ApaTrajectoryStitcher::TaskDebug(
    const std::vector<pnc::geometry_lib::PathPoint>& path,
    const trajectory::Trajectory& trajectory) {
  if (!stitch_path_.empty()) {
    ILOG_INFO << "stitch path s = " << stitch_path_.back().s
              << ", size = " << stitch_path_.size();
  }
  if (!path.empty()) {
    ILOG_INFO << "origin path s = " << path.back().s;
  }

  ILOG_INFO << "gear = " << static_cast<int>(gear_)
            << ", ego v = " << ego_lon_state_.v
            << ", ego a = " << ego_lon_state_.acc;
  ILOG_INFO << "lon stitch v = " << lon_stitch_point_.vel()
            << ", a = " << lon_stitch_point_.acc()
            << ", s = " << lon_stitch_point_.s();

  ILOG_INFO << "traj size = " << trajectory.size();

  // for (auto& point : path) {
  //   ILOG_INFO << "s = " << point.s << ",x = " << point.pos.x()
  //             << ", y = " << point.pos.y() << ", kappa = " << point.kappa;
  // }

  return;
}

const SVPoint ApaTrajectoryStitcher::GetStitchSpeed() const {
  SVPoint point;
  point.s = 0.0;
  point.t = 0.0;
  point.v = lon_stitch_point_.vel();
  point.acc = lon_stitch_point_.acc();

  return point;
}

}  // namespace apa_planner
}  // namespace planning