#include "uniform_jerk_curve.h"
#include "algorithm"
#include "cmath"

namespace planning {
UniformJerkCurve::UniformJerkCurve(const double s0, const double v0,
                                   const double a0, const double jerk,
                                   const SamplePolyCurveConfig& config) {
  cofficients_.clear();
  s0_ = s0;
  v0_ = v0;
  a0_ = a0;
  jerk_limit_upper_ = config.jerk_limit_upper_;
  jerk_limit_lower_ = config.jerk_limit_lower_;
  acc_limit_upper_ = config.acc_limit_upper_;
  acc_limit_lower_ = config.acc_limit_lower_;
  vel_limit_upper_ = config.vel_limit_upper_;
  vel_limit_lower_ = config.vel_limit_lower_;
  bool is_over_speed_range = v0_ > vel_limit_upper_ || v0_ < vel_limit_lower_;
  if (std::fabs(jerk) < kZeroEpsilon || is_over_speed_range) {
    BuildZeroJerkTraj();
  } else {
    bool is_acc_enable =
        (a0_ < acc_limit_upper_ || jerk < -kZeroEpsilon / 2.0) &&
        (a0_ > acc_limit_lower_ || jerk > kZeroEpsilon / 2.0);
    if (is_acc_enable) {
      BuildUniformJerkTraj(jerk);
    } else {
      BuildZeroJerkTraj();
    }
  }
}

void UniformJerkCurve::BuildZeroJerkTraj() {
  if ((v0_ > vel_limit_upper_ && a0_ > kZeroEpsilon / 2.0) ||
      (v0_ < vel_limit_lower_ && a0_ < -kZeroEpsilon / 2.0)) {
    double current_jerk =
        v0_ > vel_limit_upper_ ? jerk_limit_lower_ : jerk_limit_upper_;
    double t = std::fabs(a0_ / current_jerk);
    cofficients_.time_points.push_back(0.0);
    cofficients_.cofficients.push_back(
        {s0_, v0_, a0_ / 2.0, current_jerk / 6.0});
    cofficients_.time_points.push_back(t);
    cofficients_.cofficients.push_back(
        {CalcS(t), CalcV(t), 0.0, 0.0});  // todo:consider the case that end_v0
                                          // is not vel_limit_upper_ or
                                          // vel_limit_lower_
  } else {
    if (std::fabs(a0_) > kZeroEpsilon) {
      double uniform_vel =
          a0_ > kZeroEpsilon ? vel_limit_upper_ : vel_limit_lower_;
      double uniform_vel_time =
          std::fabs((uniform_vel - v0_) /
                    a0_);  // todo:consider the case that end_a0 is zero
      cofficients_.time_points.push_back(0.0);
      cofficients_.cofficients.push_back({s0_, v0_, a0_ / 2.0, 0.0});
      cofficients_.time_points.push_back(uniform_vel_time);
      cofficients_.cofficients.push_back(
          {CalcS(uniform_vel_time), uniform_vel, 0.0, 0.0});
    } else {
      cofficients_.time_points.push_back(0.0);
      cofficients_.cofficients.push_back({s0_, v0_, 0.0, 0.0});
    }
  }
}

void UniformJerkCurve::BuildUniformJerkTraj(const double jerk) {
  double end_a0 = jerk > 0 ? acc_limit_upper_ : acc_limit_lower_;
  double end_v0 = jerk > 0 ? vel_limit_upper_ : vel_limit_lower_;
  double t_hit_acc = std::fabs((a0_ - end_a0) / jerk);
  double t_hit_vel = std::numeric_limits<double>::max();
  double disc = a0_ * a0_ - 2 * jerk * (v0_ - end_v0);
  if (jerk > 0 && disc >= 0) {
    t_hit_vel = (-a0_ + std::sqrt(disc)) / jerk;
  } else if (jerk < 0 && disc >= 0) {
    t_hit_vel = (-a0_ - std::sqrt(disc)) / jerk;
  }
  if (t_hit_acc < t_hit_vel) {
    cofficients_.time_points.push_back(0.0);
    cofficients_.cofficients.push_back({s0_, v0_, a0_ / 2.0, jerk / 6.0});
    cofficients_.time_points.push_back(t_hit_acc);
    cofficients_.cofficients.push_back(
        {CalcS(t_hit_acc), CalcV(t_hit_acc), end_a0 / 2.0, 0.0});
    t_hit_vel = t_hit_acc + std::fabs((end_v0 - CalcV(t_hit_acc)) / end_a0);
    cofficients_.time_points.push_back(t_hit_vel);
    cofficients_.cofficients.push_back({CalcS(t_hit_vel), end_v0, 0.0, 0.0});
  } else {
    cofficients_.time_points.push_back(0.0);
    cofficients_.cofficients.push_back({s0_, v0_, a0_ / 2.0, jerk / 6.0});
    cofficients_.time_points.push_back(t_hit_vel);
    cofficients_.cofficients.push_back({CalcS(t_hit_vel), end_v0, 0.0, 0.0});
  }
}

double UniformJerkCurve::CalcS(const double t) const {
  double s = 0.0;
  for (int i = 0; i < cofficients_.time_points.size(); ++i) {
    if (i == cofficients_.time_points.size() - 1) {
      s = cofficients_.cofficients[i][0] + cofficients_.cofficients[i][1] * t +
          cofficients_.cofficients[i][2] * t * t +
          cofficients_.cofficients[i][3] * t * t * t;
    } else {
      if (t < cofficients_.time_points[i] + kZeroEpsilon) {
        s = cofficients_.cofficients[i][0] +
            cofficients_.cofficients[i][1] * t +
            cofficients_.cofficients[i][2] * t * t +
            cofficients_.cofficients[i][3] * t * t * t;
        break;
      }
    }
  }
  return s;
}

double UniformJerkCurve::CalcV(const double t) const {
  double v = 0.0;
  for (int i = 0; i < cofficients_.time_points.size(); ++i) {
    if (i == cofficients_.time_points.size() - 1) {
      v = cofficients_.cofficients[i][1] +
          cofficients_.cofficients[i][2] * t * 2.0 +
          cofficients_.cofficients[i][3] * t * t * 3.0;
    } else {
      if (t < cofficients_.time_points[i] + kZeroEpsilon) {
        v = cofficients_.cofficients[i][1] +
            cofficients_.cofficients[i][2] * t * 2.0 +
            cofficients_.cofficients[i][3] * t * t * 3.0;
        break;
      }
    }
  }
  return v;
}

double UniformJerkCurve::CalcAcc(const double t) const {
  double a = 0.0;
  for (int i = 0; i < cofficients_.time_points.size(); ++i) {
    if (i == cofficients_.time_points.size() - 1) {
      a = cofficients_.cofficients[i][2] * 2.0 +
          cofficients_.cofficients[i][3] * t * 6.0;
    } else {
      if (t < cofficients_.time_points[i] + kZeroEpsilon) {
        a = cofficients_.cofficients[i][2] * 2.0 +
            cofficients_.cofficients[i][3] * t * 6.0;
        break;
      }
    }
  }
  return a;
}

double UniformJerkCurve::CalcJerk(const double t) const {
  double jerk = 0.0;
  for (int i = 0; i < cofficients_.time_points.size(); ++i) {
    if (i == cofficients_.time_points.size() - 1) {
      jerk = cofficients_.cofficients[i][3] * 6.0;
    } else {
      if (t < cofficients_.time_points[i] + kZeroEpsilon) {
        jerk = cofficients_.cofficients[i][3] * 6.0;
        break;
      }
    }
  }
  return jerk;
}
}  // namespace planning