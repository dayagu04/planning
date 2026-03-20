#include <assert.h>

#include <algorithm>

#include "piecewise_jerk_trajectory1d.h"

namespace planning {

PiecewiseJerkTrajectory1d::PiecewiseJerkTrajectory1d(const double p,
                                                     const double v,
                                                     const double a,
                                                     const double init_param)
    : last_p_(p), last_v_(v), last_a_(a), init_param_(init_param) {
  param_.push_back(0.0);
}

void PiecewiseJerkTrajectory1d::AppendSegment(const double jerk,
                                              const double param) {
  param_.push_back(param_.back() + param);

  segments_.emplace_back(last_p_, last_v_, last_a_, jerk, param);

  last_p_ = segments_.back().end_position();

  last_v_ = segments_.back().end_velocity();

  last_a_ = segments_.back().end_acceleration();
}

double PiecewiseJerkTrajectory1d::Evaluate(const int32_t order,
                                           const double param) const {
  const double relative_param = param - init_param_;

  auto it_lower =
      std::lower_bound(param_.begin(), param_.end(), relative_param);

  if (it_lower == param_.begin()) {
    return segments_[0].Evaluate(order, relative_param);
  }

  if (it_lower == param_.end()) {
    auto index = std::max(0, static_cast<int>(param_.size() - 2));
    return segments_.back().Evaluate(order, relative_param - param_[index]);
  }

  auto index = std::distance(param_.begin(), it_lower);
  return segments_[index - 1].Evaluate(order,
                                       relative_param - param_[index - 1]);
}

double PiecewiseJerkTrajectory1d::ParamLength() const { return param_.back(); }

PiecewiseJerkTrajectory1d::PiecewiseJerkTrajectory1d(
    const PiecewiseJerkTrajectory1d& other) {
  param_ = other.param_;
  segments_ = other.segments_;
  last_p_ = other.last_p_;
  last_v_ = other.last_v_;
  last_a_ = other.last_a_;
  init_param_ = other.init_param_;
}

void PiecewiseJerkTrajectory1d::AppendSegment(const double p, const double v,
                                              const double a,
                                              const double param) {
  assert(param > 0);
  const double jerk = (a - last_a_) / param;

  param_.push_back(param_.back() + param);

  segments_.emplace_back(last_p_, last_v_, last_a_, jerk, param);

  last_p_ = p;

  last_v_ = v;

  last_a_ = a;
}

}  // namespace planning