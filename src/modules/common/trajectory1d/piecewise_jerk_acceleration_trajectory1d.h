#pragma once

#include <array>
#include <string>
#include <vector>

#include "trajectory1d.h"

namespace planning {

/**
 * This is constant acceleration longitudinal Trajectory1d trajectory.
 **/
class PiecewiseJerkAccelerationTrajectory1d : public Trajectory1d {
 public:
  PiecewiseJerkAccelerationTrajectory1d(const double start_s,
                                        const double start_v);

  PiecewiseJerkAccelerationTrajectory1d(const double start_s,
                                        const double start_v,
                                        const double start_a);

  virtual ~PiecewiseJerkAccelerationTrajectory1d() = default;

  void AppendSegment(const double a, const double t_duration);

  double ParamLength() const override;

  double Evaluate(const int32_t order, const double param) const override;

  std::array<double, 4> Evaluate(const double t) const;

 private:
  double Evaluate_s(const double t) const;

  double Evaluate_v(const double t) const;

  double Evaluate_a(const double t) const;

  double Evaluate_j(const double t) const;

 private:
  // accumulated s
  std::vector<double> s_;

  std::vector<double> v_;

  // accumulated t
  std::vector<double> t_;

  std::vector<double> a_;
};

}  // namespace planning