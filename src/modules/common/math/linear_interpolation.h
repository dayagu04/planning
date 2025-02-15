#ifndef MODULES_PLANNING_COMMON_LINEAR_INTERPOLATION_H_
#define MODULES_PLANNING_COMMON_LINEAR_INTERPOLATION_H_

#include <cmath>
#include <iostream>

#include "config/basic_type.h"
#include "config/message_type.h"
#include "planning_plan_c.h"
#include "prediction_object.h"
#include "src/modules//common/utils/path_point.h"
/**
 * @namespace apollo::common::math
 * @brief apollo::common::math
 */
namespace planning {
namespace planning_math {

/**
 * @brief Linear interpolation between two points of type T.
 * @param x0 The coordinate of the first point.
 * @param t0 The interpolation parameter of the first point.
 * @param x1 The coordinate of the second point.
 * @param t1 The interpolation parameter of the second point.
 * @param t The interpolation parameter for interpolation.
 * @param x The coordinate of the interpolated point.
 * @return Interpolated point.
 */
template <typename T>
T lerp(const T &x0, const double t0, const T &x1, const double t1,
       const double t) {
  if (std::abs(t1 - t0) <= 1.0e-6) {
    std::cerr << "input time difference is too small";
    return x0;
  }
  const double r = (t - t0) / (t1 - t0);
  const T x = x0 + r * (x1 - x0);
  return x;
}
template <typename T>
T lerp(const T x0, const T x1, const T r) {
  return x0 + r * (x1 - x0);
}

inline double LerpWithLimit(const double x0, const double t0, const double x1,
                            const double t1, const double t) {
  // t0 <t1
  if (fabs(t1 - t0) <= 1.0e-6) {
    return x0;
  }
  if (t < t0) {
    return x0;
  }
  if (t > t1) {
    return x1;
  }
  double r = (t - t0) / (t1 - t0);
  return x0 + r * (x1 - x0);
}
/**
 * @brief Spherical linear interpolation between two angles.
 *        The two angles are within range [-M_PI, M_PI).
 * @param a0 The value of the first angle.
 * @param t0 The interpolation parameter of the first angle.
 * @param a1 The value of the second angle.
 * @param t1 The interpolation parameter of the second angle.
 * @param t The interpolation parameter for interpolation.
 * @param a The value of the spherically interpolated angle.
 * @return Interpolated angle.
 */
double slerp(const double a0, const double t0, const double a1, const double t1,
             const double t);

template <class T>
typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type
floating_equal(T x, T y) {
  // the machine epsilon has to be scaled to the magnitude of the values used
  // and multiplied by the desired precision in ULPs (units in the last place)
  T ulp = 2;
  return std::fabs(x - y) <=
             std::numeric_limits<T>::epsilon() * std::fabs(x + y) * ulp
         // unless the result is subnormal
         || std::fabs(x - y) < std::numeric_limits<T>::min();
}

inline double GetInterpolationRatio(double a, double b, double t) {
  return floating_equal(a, b) ? 0 : (t - a) / (b - a);
}

// Calculates linear interpolation result. Note that ratio may or may not be
// in the interval of [0, 1].
inline double LinearInterpolate(double a, double b, double ratio) {
  return (1 - ratio) * a + ratio * b;
}

// Calculate the corresponding f(x) in a lookup table
inline double TableInterpolate(
    const std::vector<std::pair<double, double>> &table, double x) {
  if (table.size() == 0) {
    // std::cout << "The table is empty!";
    return x;
  }

  // Assumes that "table" is sorted by .first. assert if x is out of bound
  if (x >= table.back().first) return table.back().second;
  if (x <= table.front().first) return table.front().second;
  auto upper_pair =
      std::lower_bound(table.begin(), table.end(), std::make_pair(x, 0.0));
  // Corner case
  if (upper_pair == table.begin()) return upper_pair->second;
  auto lower_pair = upper_pair;
  --lower_pair;
  double ratio = GetInterpolationRatio(lower_pair->first, upper_pair->first, x);
  return LinearInterpolate(lower_pair->second, upper_pair->second, ratio);
}

planning::SLPoint InterpolateUsingLinearApproximation(const SLPoint &p0,
                                                      const SLPoint &p1,
                                                      const double w);

PathPoint InterpolateUsingLinearApproximation(const PathPoint &p0,
                                              const PathPoint &p1,
                                              const double s);

planning::PncTrajectoryPoint InterpolateUsingLinearApproximation(
    const PncTrajectoryPoint &tp0, const PncTrajectoryPoint &tp1,
    const double t);
planning::PredictionTrajectoryPoint InterpolateUsingLinearApproximation(
    const PredictionTrajectoryPoint &tp0, const PredictionTrajectoryPoint &tp1,
    const double t);

iflyauto::TrajectoryPoint InterpolateUsingLinearApproximation(
    const iflyauto::TrajectoryPoint &p0, const iflyauto::TrajectoryPoint &p1,
    const double s);

TrajectoryPoint InterpolateUsingLinearApproximation(const TrajectoryPoint &p0,
                                                    const TrajectoryPoint &p1,
                                                    const double t);
}  // namespace planning_math
}  // namespace planning

#endif /* MODULES_PLANNING_COMMON_LINEAR_INTERPOLATION_H_ */
