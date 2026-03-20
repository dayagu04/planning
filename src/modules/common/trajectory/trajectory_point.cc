#include "trajectory_point.h"

namespace planning {
namespace trajectory {

TrajectoryPoint InterpolateUsingLinearApproximation(const TrajectoryPoint &p0,
                                                    const TrajectoryPoint &p1,
                                                    const double s) {
  if (std::fabs(p0.s() - p1.s()) < 1e-5) {
    return p0;
  }

  TrajectoryPoint trajectory_point;
  const double weight0 = (s - p0.s()) / (p1.s() - p0.s());
  const double weight1 = 1.0 - weight0;
  const double x = weight1 * p0.x() + weight0 * p1.x();
  const double y = weight1 * p0.y() + weight0 * p1.y();
  const double curvature = weight1 * p0.kappa() + weight0 * p1.kappa();
  double theta =
      planning_math::slerp(p0.theta(), p0.s(), p1.theta(), p1.s(), s);
  double dkappa = (1 - weight0) * p0.dkappa() + weight0 * p1.dkappa();

  const double t = weight1 * p0.absolute_time() + weight0 * p1.absolute_time();
  const double v = weight1 * p0.vel() + weight0 * p1.vel();
  const double a = weight1 * p0.acc() + weight0 * p1.acc();
  const double jerk = weight1 * p0.jerk() + weight0 * p1.jerk();

  trajectory_point.set_x(x);
  trajectory_point.set_y(y);
  trajectory_point.set_theta(theta);
  trajectory_point.set_kappa(curvature);
  trajectory_point.set_dkappa(dkappa);
  trajectory_point.set_s(s);

  trajectory_point.set_absolute_time(t);
  trajectory_point.set_vel(v);
  trajectory_point.set_acc(a);
  trajectory_point.set_jerk(jerk);

  return trajectory_point;
}

}  // namespace trajectory
}  // namespace planning