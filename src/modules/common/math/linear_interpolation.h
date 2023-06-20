#ifndef MODULES_PLANNING_COMMON_LINEAR_INTERPOLATION_H_
#define MODULES_PLANNING_COMMON_LINEAR_INTERPOLATION_H_

#include <cmath>
#include <iostream>

#include "planning_plan.pb.h"

#include "config/basic_type.h"
#include "config/message_type.h"
#include "path_point.h"
#include "prediction_object.h"
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
T lerp(const T &x0, const double t0, const T &x1, const double t1, const double t) {
  if (std::abs(t1 - t0) <= 1.0e-6) {
    std::cerr << "input time difference is too small";
    return x0;
  }
  const double r = (t - t0) / (t1 - t0);
  const T x = x0 + r * (x1 - x0);
  return x;
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
double slerp(const double a0, const double t0, const double a1, const double t1, const double t);

planning::SLPoint InterpolateUsingLinearApproximation(const SLPoint &p0, const SLPoint &p1, const double w);

planning::PathPoint InterpolateUsingLinearApproximation(const PathPoint &p0, const PathPoint &p1, const double s);

planning::PncTrajectoryPoint InterpolateUsingLinearApproximation(const PncTrajectoryPoint &tp0,
                                                                 const PncTrajectoryPoint &tp1, const double t);
planning::PredictionTrajectoryPoint InterpolateUsingLinearApproximation(const PredictionTrajectoryPoint &tp0,
                                                                        const PredictionTrajectoryPoint &tp1,
                                                                        const double t);

::PlanningOutput::TrajectoryPoint InterpolateUsingLinearApproximation(
    const ::PlanningOutput::TrajectoryPoint &p0,
    const ::PlanningOutput::TrajectoryPoint &p1, const double s);
}  // namespace planning_math
}  // namespace planning

#endif /* MODULES_PLANNING_COMMON_LINEAR_INTERPOLATION_H_ */
