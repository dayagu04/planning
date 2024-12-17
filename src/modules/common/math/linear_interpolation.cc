#include "math/linear_interpolation.h"

#include <cmath>
#include <iostream>

#include "assert.h"
#include "common.h"
#include "math/math_utils.h"

namespace planning {
namespace planning_math {

double slerp(const double a0, const double t0, const double a1, const double t1,
             const double t) {
  if (std::abs(t1 - t0) <= kMathEpsilon) {
    LOG_DEBUG("input time difference is too small");
    return NormalizeAngle(a0);
  }
  const double a0_n = NormalizeAngle(a0);
  const double a1_n = NormalizeAngle(a1);
  double d = a1_n - a0_n;
  if (d > M_PI) {
    d = d - 2 * M_PI;
  } else if (d < -M_PI) {
    d = d + 2 * M_PI;
  }

  const double r = (t - t0) / (t1 - t0);
  const double a = a0_n + d * r;
  return NormalizeAngle(a);
}

SLPoint InterpolateUsingLinearApproximation(const SLPoint &p0,
                                            const SLPoint &p1, const double w) {
  assert(w > 0.0);

  SLPoint p;
  p.s = (1 - w) * p0.s + w * p1.s;
  p.l = (1 - w) * p0.l + w * p1.l;
  return p;
}

PathPoint InterpolateUsingLinearApproximation(const PathPoint &p0,
                                              const PathPoint &p1,
                                              const double s) {
  double s0 = p0.s();
  double s1 = p1.s();

  PathPoint path_point;
  double weight = (s - s0) / (s1 - s0);
  double x = (1 - weight) * p0.x() + weight * p1.x();
  double y = (1 - weight) * p0.y() + weight * p1.y();
  double theta = slerp(p0.theta(), p0.s(), p1.theta(), p1.s(), s);
  double kappa = (1 - weight) * p0.kappa() + weight * p1.kappa();
  double dkappa = (1 - weight) * p0.dkappa() + weight * p1.dkappa();
  double ddkappa = (1 - weight) * p0.ddkappa() + weight * p1.ddkappa();
  path_point.set_x(x);
  path_point.set_y(y);
  path_point.set_theta(theta);
  path_point.set_kappa(kappa);
  path_point.set_dkappa(dkappa);
  path_point.set_ddkappa(ddkappa);
  path_point.set_s(s);
  return path_point;
}

PncTrajectoryPoint InterpolateUsingLinearApproximation(
    const PncTrajectoryPoint &tp0, const PncTrajectoryPoint &tp1,
    const double t) {
  //   if (!tp0.has_path_point() || !tp1.has_path_point()) {
  //     PncTrajectoryPoint p;
  //     p.path_point = PathPoint();
  //     return p;
  //   }
  const PathPoint pp0 = tp0.path_point;
  const PathPoint pp1 = tp1.path_point;
  double t0 = tp0.relative_time;
  double t1 = tp1.relative_time;

  PncTrajectoryPoint tp;
  tp.v = lerp(tp0.v, t0, tp1.v, t1, t);
  tp.a = lerp(tp0.a, t0, tp1.a, t1, t);
  tp.sigma_x = lerp(tp0.sigma_x, t0, tp1.sigma_x, t1, t);
  tp.sigma_y = lerp(tp0.sigma_y, t0, tp1.sigma_y, t1, t);
  tp.prediction_prob =
      lerp(tp0.prediction_prob, t0, tp1.prediction_prob, t1, t);
  tp.velocity_direction =
      // std::abs(t -t0) < std::abs(t -t1) ? tp0.velocity_direction :
      // tp1.velocity_direction;
      slerp(tp0.velocity_direction, t0, tp1.velocity_direction, t1, t);
  tp.relative_time = t;
  // tp.steer = slerp(tp0.steer, t0, tp1.steer, t1, t);

  PathPoint *path_point = &tp.path_point;
  path_point->set_x(lerp(pp0.x(), t0, pp1.x(), t1, t));
  path_point->set_y(lerp(pp0.y(), t0, pp1.y(), t1, t));
  path_point->set_theta(slerp(pp0.theta(), t0, pp1.theta(), t1, t));
  path_point->set_kappa(lerp(pp0.kappa(), t0, pp1.kappa(), t1, t)) ;
  path_point->set_dkappa(lerp(pp0.dkappa(), t0, pp1.dkappa(), t1, t));
  path_point->set_ddkappa(lerp(pp0.ddkappa(), t0, pp1.ddkappa(), t1, t));
  path_point->set_s(lerp(pp0.s(), t0, pp1.s(), t1, t));
  tp.relative_ego_x = lerp(tp0.relative_ego_x, t0, tp1.relative_ego_x, t1, t);
  tp.relative_ego_y = lerp(tp0.relative_ego_y, t0, tp1.relative_ego_y, t1, t);
  tp.relative_ego_yaw =
      lerp(tp0.relative_ego_yaw, t0, tp1.relative_ego_yaw, t1, t);
  tp.relative_ego_speed =
      lerp(tp0.relative_ego_speed, t0, tp1.relative_ego_speed, t1, t);
  tp.relative_ego_std_dev_x =
      lerp(tp0.relative_ego_std_dev_x, t0, tp1.relative_ego_std_dev_x, t1, t);
  tp.relative_ego_std_dev_y =
      lerp(tp0.relative_ego_std_dev_y, t0, tp1.relative_ego_std_dev_y, t1, t);
  tp.relative_ego_std_dev_yaw = lerp(tp0.relative_ego_std_dev_yaw, t0,
                                     tp1.relative_ego_std_dev_yaw, t1, t);
  tp.relative_ego_std_dev_speed = lerp(tp0.relative_ego_std_dev_speed, t0,
                                       tp1.relative_ego_std_dev_speed, t1, t);
  return tp;
}

PredictionTrajectoryPoint InterpolateUsingLinearApproximation(
    const PredictionTrajectoryPoint &tp0, const PredictionTrajectoryPoint &tp1,
    const double t) {
  double t0 = tp0.relative_time;
  double t1 = tp1.relative_time;

  PredictionTrajectoryPoint tp;
  tp.relative_time = t;
  tp.x = lerp(tp0.x, t0, tp1.x, t1, t);
  tp.y = lerp(tp0.y, t0, tp1.y, t1, t);
  tp.yaw =
      // std::abs(t -t0) < std::abs(t -t1) ? tp0.velocity_direction :
      // tp1.velocity_direction;
      slerp(tp0.yaw, t0, tp1.yaw, t1, t);
  tp.speed = lerp(tp0.speed, t0, tp1.speed, t1, t);
  tp.theta =
      // std::abs(t -t0) < std::abs(t -t1) ? tp0.velocity_direction :
      // tp1.velocity_direction;
      slerp(tp0.theta, t0, tp1.theta, t1, t);
  tp.prob = lerp(tp0.prob, t0, tp1.prob, t1, t);
  tp.std_dev_x = lerp(tp0.std_dev_x, t0, tp1.std_dev_x, t1, t);
  tp.std_dev_y = lerp(tp0.std_dev_y, t0, tp1.std_dev_y, t1, t);
  tp.std_dev_yaw = lerp(tp0.std_dev_yaw, t0, tp1.std_dev_yaw, t1, t);
  tp.std_dev_speed = lerp(tp0.std_dev_speed, t0, tp1.std_dev_speed, t1, t);

  // tp.steer = slerp(tp0.steer, t0, tp1.steer, t1, t);
  tp.relative_ego_x = lerp(tp0.relative_ego_x, t0, tp1.relative_ego_x, t1, t);
  tp.relative_ego_y = lerp(tp0.relative_ego_y, t0, tp1.relative_ego_y, t1, t);
  tp.relative_ego_yaw =
      lerp(tp0.relative_ego_yaw, t0, tp1.relative_ego_yaw, t1, t);
  tp.relative_ego_speed =
      lerp(tp0.relative_ego_speed, t0, tp1.relative_ego_speed, t1, t);
  tp.relative_ego_std_dev_x =
      lerp(tp0.relative_ego_std_dev_x, t0, tp1.relative_ego_std_dev_x, t1, t);
  tp.relative_ego_std_dev_y =
      lerp(tp0.relative_ego_std_dev_y, t0, tp1.relative_ego_std_dev_y, t1, t);
  tp.relative_ego_std_dev_yaw = lerp(tp0.relative_ego_std_dev_yaw, t0,
                                     tp1.relative_ego_std_dev_yaw, t1, t);
  tp.relative_ego_std_dev_speed = lerp(tp0.relative_ego_std_dev_speed, t0,
                                       tp1.relative_ego_std_dev_speed, t1, t);
  return tp;
}

iflyauto::TrajectoryPoint InterpolateUsingLinearApproximation(
    const iflyauto::TrajectoryPoint &p0, const iflyauto::TrajectoryPoint &p1,
    const double s) {
  if (std::fabs(p0.distance - p1.distance) < kMathEpsilon) {
    return p0;
  }

  iflyauto::TrajectoryPoint trajectory_point;
  const double weight0 = (s - p0.distance) / (p1.distance - p0.distance);
  const double weight1 = 1.0 - weight0;
  const double x = weight1 * p0.x + weight0 * p1.x;
  const double y = weight1 * p0.y + weight0 * p1.y;
  const double heading_yaw =
      slerp(p0.heading_yaw, p0.distance, p1.heading_yaw, p1.distance, s);
  const double curvature = weight1 * p0.curvature + weight0 * p1.curvature;
  const double t = weight1 * p0.t + weight0 * p1.t;
  const double v = weight1 * p0.v + weight0 * p1.v;
  const double a = weight1 * p0.a + weight0 * p1.a;
  const double distance = weight1 * p0.distance + weight0 * p1.distance;
  const double jerk = weight1 * p0.jerk + weight0 * p1.jerk;

  trajectory_point.x = x;
  trajectory_point.y = y;
  trajectory_point.heading_yaw = heading_yaw;
  trajectory_point.curvature = curvature;
  trajectory_point.t = t;
  trajectory_point.v = v;
  trajectory_point.a = a;
  trajectory_point.distance = distance;
  trajectory_point.jerk = jerk;

  return trajectory_point;
}

TrajectoryPoint InterpolateUsingLinearApproximation(const TrajectoryPoint &p0,
                                                    const TrajectoryPoint &p1,
                                                    const double t) {
  if (std::fabs(p0.t - p1.t) < kMathEpsilon) {
    return p0;
  }

  TrajectoryPoint trajectory_point;
  const double weight0 = (t - p0.t) / (p1.t - p0.t);
  const double weight1 = 1.0 - weight0;
  const double x = weight1 * p0.x + weight0 * p1.x;
  const double y = weight1 * p0.y + weight0 * p1.y;
  const double heading_angle =
      slerp(p0.heading_angle, p0.t, p1.heading_angle, p1.t, t);
  const double curvature = weight1 * p0.curvature + weight0 * p1.curvature;
  // const double t = weight1 * p0.t + weight0 * p1.t;
  const double v = weight1 * p0.v + weight0 * p1.v;
  const double a = weight1 * p0.a + weight0 * p1.a;
  const double s = weight1 * p0.s + weight0 * p1.s;
  const double jerk = weight1 * p0.jerk + weight0 * p1.jerk;

  trajectory_point.x = x;
  trajectory_point.y = y;
  trajectory_point.heading_angle = heading_angle;
  trajectory_point.curvature = curvature;
  trajectory_point.t = t;
  trajectory_point.v = v;
  trajectory_point.a = a;
  trajectory_point.s = s;
  trajectory_point.jerk = jerk;

  return trajectory_point;
}

}  // namespace planning_math
}  // namespace planning