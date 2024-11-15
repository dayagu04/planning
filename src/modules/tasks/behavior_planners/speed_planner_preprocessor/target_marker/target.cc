#include "target.h"

#include "ego_planning_config.h"
#include "environmental_model.h"
#include "trajectory1d/piecewise_jerk_acceleration_trajectory1d.h"
#include "trajectory1d/second_order_time_optimal_trajectory.h"

namespace planning {

Target::Target(const SpeedPlannerConfig config, framework::Session* session)
    : config_(config),
      session_(session),
      planning_time_(config.planning_time),
      dt_(config.dt) {
  const auto& ego_state_manager =
      session->environmental_model().get_ego_state_manager();
  const auto& init_point = ego_state_manager->planning_init_point();
  init_lon_state_ = {0.0, init_point.v, init_point.a};
  plan_points_num_ = static_cast<size_t>(planning_time_ / dt_) + 1;
  virtual_zero_acc_curve_ = MakeVirtualZeroAccCurve();
  max_speed_limit_curve_ = MakeMaxSpeedLimitCurve();
}

std::unique_ptr<Trajectory1d> Target::MakeVirtualZeroAccCurve() {
  auto virtual_zero_acc_curve =
      std::make_unique<PiecewiseJerkAccelerationTrajectory1d>(
          init_lon_state_[0], init_lon_state_[1]);
  virtual_zero_acc_curve->AppendSegment(init_lon_state_[2], dt_);

  const double zero_acc_jerk_max = config_.zero_acc_jerk_max;
  const double zero_acc_jerk_min = config_.zero_acc_jerk_min;
  for (double t = dt_; t <= planning_time_; t += dt_) {
    const double acc = virtual_zero_acc_curve->Evaluate(2, t);
    const double vel = virtual_zero_acc_curve->Evaluate(1, t);

    double a_next = 0.0;
    // if init acc < 0.0, move a to zero with jerk max
    // if init acc > 0.0, move a to zero with jerk min
    if (init_lon_state_[2] < 0.0) {
      a_next = acc + dt_ * zero_acc_jerk_max;
    } else {
      a_next = acc + dt_ * zero_acc_jerk_min;
    }

    if (init_lon_state_[2] * acc <= 0.0) {
      a_next = 0.0;
    }

    if (vel <= 0.0) {
      a_next = 0.0;
    }
    virtual_zero_acc_curve->AppendSegment(a_next, dt_);
  }
  return virtual_zero_acc_curve;
}

std::unique_ptr<Trajectory1d> Target::MakeMaxSpeedLimitCurve() {
  const double speed_upper_bound =
      config_.max_speed_limit_curve_velocity_upper_bound;  // 27 m/s
  const double acc_min = config_.max_speed_limit_curve_acc_lower_bound;  // -1.0
  const double acc_max =
      config_.max_speed_limit_curve_acc_upper_bound;  // 2.4,a larger acc,if not
                                                      // lane change ,we can
                                                      // use 1.8 for upper
  const double jerk_min =
      config_.max_speed_limit_curve_jerk_lower_bound;  //-1.0
  const double jerk_max =
      config_.max_speed_limit_curve_jerk_upper_bound;  // 10.0

  StateLimit state_limit;
  state_limit.v_end = speed_upper_bound;
  state_limit.v_max = std::fmax(init_lon_state_[1], speed_upper_bound);
  state_limit.v_min = -1e-4;
  state_limit.a_min = acc_min;
  state_limit.a_max = acc_max;
  state_limit.j_min = jerk_min;
  state_limit.j_max = jerk_max;

  // for gap decider
  constexpr double kTargetGapMaxSpeed = 21.0;
  constexpr double kTargetGapAccMin = -1.0;
  constexpr double kJerkMin = -1.0;
  state_limit.v_end = std::fmax(speed_upper_bound, kTargetGapMaxSpeed);
  state_limit.a_min = kTargetGapAccMin;
  state_limit.j_min = kJerkMin;

  LonState init_state;
  init_state.p = init_lon_state_[0];
  init_state.v = init_lon_state_[1];
  init_state.a = init_lon_state_[2];
  return std::make_unique<SecondOrderTimeOptimalTrajectory>(init_state,
                                                            state_limit);
}

bool Target::has_target(const double t) const {
  size_t index = static_cast<size_t>(std::round(t / dt_));
  if (index > target_values_.size() - 1) {
    return false;
  }
  return target_values_[index].has_target();
}

TargetValue Target::target_value(const double t) const {
  if (!has_target(t)) {
    return TargetValue(0.0, false, 0.0, 0.0, TargetType::kNotSet);
  }
  size_t index = static_cast<size_t>(std::round(t / dt_));
  return target_values_[index];
}

}  // namespace planning