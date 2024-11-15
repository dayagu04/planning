#include <algorithm>
#include <cassert>
#include <cmath>

#include "math/linear_interpolation.h"
#include "piecewise_jerk_trajectory1d.h"

namespace planning {

/**
 * This is constant acceleration longitudinal curve1d trajectory.
 * First a sample point = 0.0 m/s^2
 * First t sample point = 0.0 s
 * @param start_s first s sample point
 * @param start_v first v sample point
 **/
PiecewiseJerkAccelerationTrajectory1d::PiecewiseJerkAccelerationTrajectory1d(
    const double start_s, const double start_v) {
  s_.push_back(start_s);
  v_.push_back(start_v);
  a_.push_back(0.0);
  t_.push_back(0.0);
}

PiecewiseJerkAccelerationTrajectory1d::PiecewiseJerkAccelerationTrajectory1d(
    const double start_s, const double start_v, const double start_a) {
  s_.push_back(start_s);
  v_.push_back(start_v);
  a_.push_back(start_a);
  t_.push_back(0.0);
}

/**
 * @param a constant acceleration of this piecewise segment
 * @param t_duration time length of this piecewise segment
 */
void PiecewiseJerkAccelerationTrajectory1d::AppendSegment(
    const double a, const double t_duration) {
  double s0 = s_.back();
  double v0 = v_.back();
  double t0 = t_.back();

  double v1 = v0 + a * t_duration;

  double delta_s = (v0 + v1) * t_duration * 0.5;
  double s1 = s0 + delta_s;
  double t1 = t0 + t_duration;

  // TODO(all): this will lead to a bug; if t is between t0 and t1, s will be
  // less than s0 and s1.
  s1 = std::fmax(s1, s0);
  s_.push_back(s1);
  v_.push_back(v1);
  a_.push_back(a);
  t_.push_back(t1);
}

double PiecewiseJerkAccelerationTrajectory1d::ParamLength() const {
  return t_.back() - t_.front();
}

double PiecewiseJerkAccelerationTrajectory1d::Evaluate(
    const int32_t order, const double param) const {
  assert(t_.size() > 1);
  assert(t_.front() - 1.0e-3 <= param && param <= t_.back() + 1.0e-3);

  switch (order) {
    case 0:
      return Evaluate_s(param);
    case 1:
      return Evaluate_v(param);
    case 2:
      return Evaluate_a(param);
    case 3:
      return Evaluate_j(param);
  }
  return 0.0;
}

double PiecewiseJerkAccelerationTrajectory1d::Evaluate_s(const double t) const {
  auto it_lower = std::lower_bound(t_.begin(), t_.end(), t);
  if (it_lower == t_.begin()) {
    return s_.front();
  }
  if (it_lower == t_.end()) {
    it_lower -= 1;
  }
  auto index = std::distance(t_.begin(), it_lower);

  double s0 = s_[index - 1];
  double v0 = v_[index - 1];
  double t0 = t_[index - 1];

  double v1 = v_[index];
  double t1 = t_[index];

  double ratio = planning_math::GetInterpolationRatio(t0, t1, t);
  double v = planning_math::LinearInterpolate(v0, v1, ratio);
  double s = (v0 + v) * (t - t0) * 0.5 + s0;
  return s;
}

double PiecewiseJerkAccelerationTrajectory1d::Evaluate_v(const double t) const {
  auto it_lower = std::lower_bound(t_.begin(), t_.end(), t);
  if (it_lower == t_.begin()) {
    return v_.front();
  }
  if (it_lower == t_.end()) {
    it_lower -= 1;
  }

  auto index = std::distance(t_.begin(), it_lower);

  double v0 = v_[index - 1];
  double t0 = t_[index - 1];

  double v1 = v_[index];
  double t1 = t_[index];

  double ratio = planning_math::GetInterpolationRatio(t0, t1, t);
  double v = planning_math::LinearInterpolate(v0, v1, ratio);
  return v;
}

double PiecewiseJerkAccelerationTrajectory1d::Evaluate_a(const double t) const {
  auto it_lower = std::lower_bound(t_.begin(), t_.end(), t);
  auto index = std::distance(t_.begin(), it_lower);
  if (index == static_cast<int>(t_.size())) {
    index -= 1;
  }
  return a_[index];
}

double PiecewiseJerkAccelerationTrajectory1d::Evaluate_j(const double t) const {
  return 0.0;
}

std::array<double, 4> PiecewiseJerkAccelerationTrajectory1d::Evaluate(
    const double t) const {
  assert(t_.size() > 1);

  auto it_lower = std::lower_bound(t_.begin(), t_.end(), t);
  if (it_lower == t_.begin()) {
    return {s_.front(), v_.front(), 0.0, 0.0};
  }
  if (it_lower == t_.end()) {
    it_lower -= 1;
  }
  auto index = std::distance(t_.begin(), it_lower);

  double s0 = s_[index - 1];
  double v0 = v_[index - 1];
  double t0 = t_[index - 1];

  double v1 = v_[index];
  double t1 = t_[index];

  double ratio = planning_math::GetInterpolationRatio(t0, t1, t);
  double v = planning_math::LinearInterpolate(v0, v1, ratio);
  double s = (v0 + v) * (t - t0) * 0.5 + s0;

  double a = a_[index];
  double j = 0.0;

  return {{s, v, a, j}};
}

}  // namespace planning