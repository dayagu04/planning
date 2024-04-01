#pragma once

#include <vector>

#include "trajectory1d.h"

namespace planning {

class SecondOrderTimeOptimalTrajectory : public Trajectory1d {
 public:
  SecondOrderTimeOptimalTrajectory() = default;
  SecondOrderTimeOptimalTrajectory(const double p0, const double v0,
                                   const double a0, const double v_end,
                                   const double acc_min, const double acc_max,
                                   const double jerk_min,
                                   const double jerk_max);

  SecondOrderTimeOptimalTrajectory(const LonState& init_state,
                                   const StateLimit& state_limit);
  virtual ~SecondOrderTimeOptimalTrajectory() = default;

  virtual double Evaluate(const int32_t order,
                          const double param) const override;
  virtual double ParamLength() const override;

  static SecondOrderParam VelocityTargetSolver(const LonState& init_state,
                                               const StateLimit& state_limit);
  static LonState GetSecondOrderTrajectoryState(
      const LonState& init_state, const SecondOrderParam& second_order_param,
      const double t);

  static inline LonState EvaluatePolyState(const LonState& input_state);

  const LonState& init_state() const { return init_state_; };
  const StateLimit& state_limit() const { return state_limit_; };
  const SecondOrderParam& second_order_param() const {
    return second_order_param_;
  };

  const bool empty() const { return !valid_; };
  bool valid_{false};

 private:
  LonState init_state_;
  SecondOrderParam second_order_param_;
  StateLimit state_limit_;
};
}  // namespace planning
