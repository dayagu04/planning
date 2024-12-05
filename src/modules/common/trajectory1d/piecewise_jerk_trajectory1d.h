#pragma once

#include <array>
#include <string>
#include <vector>

#include "trajectory1d.h"
#include "constant_jerk_trajectory1d.h"

namespace planning {

class PiecewiseJerkTrajectory1d : public Trajectory1d {
public:
  // TODO(all): remove the parameter init_param after the reference_curve_generator related code is
  // removed.
  PiecewiseJerkTrajectory1d(const double p, const double v, const double a,
                            const double init_param = 0);

  PiecewiseJerkTrajectory1d(const PiecewiseJerkTrajectory1d& other);

  virtual ~PiecewiseJerkTrajectory1d() = default;

  double Evaluate(const int32_t order, const double param) const override;

  double ParamLength() const override;

  void AppendSegment(const double jerk, const double param);

  // TODO(all): this is a temp. solution to reduce accumulated numerical error.
  void AppendSegment(const double p, const double v, const double a, const double param);

private:
  std::vector<ConstantJerkTrajectory1d> segments_;

  double last_p_ = 0.0;

  double last_v_ = 0.0;

  double last_a_ = 0.0;

  std::vector<double> param_;

  double init_param_ = 0.0;
};

} // namespace planning