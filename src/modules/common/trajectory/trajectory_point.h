#pragma once

#include "math/integral.h"
#include "src/modules/common/math/linear_interpolation.h"
#include "src/modules/common/math/math_utils.h"
#include "utils/path_point.h"

namespace planning {
namespace trajectory {

class TrajectoryPoint : public planning_math::PathPoint {
 public:
  TrajectoryPoint() = default;

  explicit TrajectoryPoint(const double x, const double y, const double heading,
                           const double vel, const double acc,
                           const double absolute_time,
                           const double heading_rate = 0.0,
                           const double jerk = 0.0, const double s = 0.0,
                           const double kappa = 0.0)
      : PathPoint(x, y, s, 0.0, heading, kappa, 0.0, 0.0) {
    vel_ = vel;
    acc_ = acc;
    jerk_ = jerk;
    heading_rate_ = heading_rate;
    absolute_time_ = absolute_time;
    // relative_time_ = relative_time;
  }

  virtual ~TrajectoryPoint() = default;

  double vel() const { return vel_; }

  void set_vel(const double vel) { vel_ = vel; }

  double acc() const { return acc_; }

  void set_acc(const double acc) { acc_ = acc; }

  void set_dkappa(const double dkappa) { dkappa_ = dkappa; }

  double heading_rate() const { return heading_rate_; }

  double jerk() const { return jerk_; }

  void set_jerk(const double jerk) {
    jerk_ = jerk;
    return;
  }

  double absolute_time() const { return absolute_time_; }

  void set_absolute_time(const double absolute_time) {
    absolute_time_ = absolute_time;
  }

  // double relative_time() const { return relative_time_; }

  // void set_relative_time(const double relative_time) {
  //   relative_time_ = relative_time;
  // }

  TrajectoryPoint InterpolateTrajectoryPoint(const TrajectoryPoint& point,
                                             const double absolute_time) const {
    double r = (absolute_time - absolute_time_) /
               (point.absolute_time() - absolute_time_);

    double x = planning_math::lerp(x_, point.x(), r);
    double y = planning_math::lerp(y_, point.y(), r);
    double s = planning_math::lerp(s_, point.s(), r);
    double heading = planning_math::slerp(theta_, absolute_time_, point.theta(),
                                          point.absolute_time(), absolute_time);
    double kappa = planning_math::lerp(kappa_, point.kappa(), r);

    double vel = planning_math::lerp(vel_, point.vel(), r);
    double acc = planning_math::lerp(acc_, point.acc(), r);
    double jerk = planning_math::lerp(jerk_, point.jerk(), r);

    double heading_rate =
        planning_math::lerp(heading_rate_, point.heading_rate(), r);

    return TrajectoryPoint(x, y, heading, vel, acc, absolute_time, heading_rate,
                           jerk, s, kappa);
  }

  TrajectoryPoint InterpolateTrajectoryPointByS(const TrajectoryPoint& point,
                                                const double s) const {
    double r = (s - s_) / (point.s() - s_);

    double x = planning_math::lerp(x_, point.x(), r);
    double y = planning_math::lerp(y_, point.y(), r);
    double absolute_time =
        planning_math::lerp(absolute_time_, point.absolute_time(), r);
    double heading =
        planning_math::slerp(theta_, s_, point.theta(), point.s(), s);
    double kappa = planning_math::lerp(kappa_, point.kappa(), r);

    double vel = planning_math::lerp(vel_, point.vel(), r);
    double acc = planning_math::lerp(acc_, point.acc(), r);
    double jerk = planning_math::lerp(jerk_, point.jerk(), r);

    double heading_rate =
        planning_math::lerp(heading_rate_, point.heading_rate(), r);

    return TrajectoryPoint(x, y, heading, vel, acc, absolute_time, heading_rate,
                           jerk, s, kappa);
  }

  static TrajectoryPoint ExtrapolateTrajectoryPoint(
      const TrajectoryPoint& input, const double delta_t) {
    const double x0 = input.x();
    const double y0 = input.y();
    const double theta0 = input.theta();
    const double theta_change_rate0 = input.heading_rate();  // dkappa/dt

    const double v0 = input.vel();
    const double a0 = input.acc();
    const double t0 = input.absolute_time();

    const double delta_s = v0 * delta_t + 0.5 * a0 * delta_t * delta_t;

    constexpr double kMinDeltaS = 0.1;
    if (std::fabs(delta_s) < kMinDeltaS) {
      TrajectoryPoint extrapolate_point = input;
      extrapolate_point.set_absolute_time(input.absolute_time() + delta_t);
      return extrapolate_point;
    }

    auto func_x = [v0, a0, theta0, theta_change_rate0](const double t) {
      const auto s = v0 * t + 0.5 * a0 * t * t;
      const auto theta = theta0 + s * theta_change_rate0 / (v0 + a0 * t);
      return std::cos(theta) * (v0 + a0 * t);
    };

    auto func_y = [v0, a0, theta0, theta_change_rate0](const double t) {
      const auto s = v0 * t + 0.5 * a0 * t * t;
      const auto theta = theta0 + s * theta_change_rate0 / (v0 + a0 * t);
      return std::sin(theta) * (v0 + a0 * t);
    };

    const double x1 =
        x0 + planning_math::IntegrateByGaussLegendre<10>(func_x, 0.0, delta_t);
    const double y1 =
        y0 + planning_math::IntegrateByGaussLegendre<10>(func_y, 0.0, delta_t);

    const double theta1 =
        planning_math::NormalizeAngle(theta0 + theta_change_rate0 * delta_t);
    const double theta_change_rate1 = theta_change_rate0;

    const double v1 = v0 + a0 * delta_t;
    const double a1 = a0;
    const double t1 = t0 + delta_t;

    return TrajectoryPoint(x1, y1, theta1, v1, a1, t1, theta_change_rate1, 0.0,
                           0.0, input.kappa());
  }

  std::string DebugString() const {
    std::stringstream ss;
    ss << "absolute time:" << absolute_time_ << ", x:" << x_ << ",y:" << y_
       << ",heading:" << theta_ << ",kappa:" << kappa_ << ",dkappa:" << dkappa_
       << ", vel:" << vel_ << ", acc" << acc_ << ",jerk:" << jerk_ << ",s "
       << s_;
    return ss.str();
  }

 private:
  double vel_ = 0.0;  // m/s
  double acc_ = 0.0;  // m/s^2
  double jerk_ = 0.0;
  double heading_rate_ = 0.0;   // rad/s
  double absolute_time_ = 0.0;  // s
  // double relative_time_ = 0.0;  // s
};

TrajectoryPoint InterpolateUsingLinearApproximation(const TrajectoryPoint &p0,
                                                    const TrajectoryPoint &p1,
                                                    const double s);

}  // namespace trajectory
}  // namespace planning