#pragma once

#include "second_order_time_optimal_trajectory.h"

namespace planning {

// compute a time optimal trajectory from a given state to end state
// assume: end_vel==0.0, end_acc==0.0

class ThirdOrderTimeOptimalTrajectory : Trajectory1d {
 public:
  ThirdOrderTimeOptimalTrajectory() = default;

  ThirdOrderTimeOptimalTrajectory(const double p0, const double v0,
                                  const double a0, const double p_end,
                                  const double v_min, const double v_max,
                                  const double acc_min, const double acc_max,
                                  const double jerk_min, const double jerk_max,
                                  const double p_precision);

  ThirdOrderTimeOptimalTrajectory(const LonState& init_state,
                                  const StateLimit& state_limit,
                                  const double p_precision);
  ThirdOrderTimeOptimalTrajectory(const LonState& init_state,
                                  const StateLimit& state_limit_pa,
                                  const StateLimit& state_limit_pb,
                                  const double p_precision);
  virtual ~ThirdOrderTimeOptimalTrajectory() = default;

  virtual double Evaluate(const int32_t order,
                          const double param) const override;
  virtual double ParamLength() const override;

  static ThirdOrderParam PositionTargetSolver(const LonState& init_state,
                                              const StateLimit& state_limit_pa,
                                              const StateLimit& state_limit_pb,
                                              const double p_precision);

  static LonState GetThirdOrderTrajectoryState(
      const LonState& init_state, const ThirdOrderParam& third_order_param,
      const double t);

 protected:
  double p_precision_ = 0.0;
  LonState init_state_;
  ThirdOrderParam third_order_param_;
  StateLimit state_limit_;
  StateLimit state_limit_pa_;
  StateLimit state_limit_pb_;
};

}  // namespace planning
