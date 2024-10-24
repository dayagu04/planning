#include "st_search_input.h"

#include "utils/kd_path.h"

namespace planning {

StSearchInput::StSearchInput(
    const trajectory::TrajectoryPoint& ego_init_state,
    const double planning_distance, const double planning_time_horizon,
    const double cruise_speed, const double max_accel_limit,
    const double min_accel_limit, const double max_jerk_limit,
    const double min_jerk_limit, const int64_t accel_sample_num,
    const double s_step, const double t_step, const double vel_step)
    : ego_init_state_(ego_init_state),
      init_vel_(ego_init_state.vel()),
      planning_distance_(planning_distance),
      planning_time_horizon_(planning_time_horizon),
      cruise_speed_(cruise_speed),
      max_accel_limit_(max_accel_limit),
      min_accel_limit_(min_accel_limit),
      max_jerk_limit_(max_jerk_limit),
      min_jerk_limit_(min_jerk_limit),
      accel_sample_num_(accel_sample_num),
      s_step_(s_step),
      t_step_(t_step),
      vel_step_(vel_step) {
  init_s_ = 0.0;
  init_t_ = 0.0;
  init_vel_ = ego_init_state.vel();
  speed_limit_ = std::fmax(init_vel_, cruise_speed_);
  speed_limit_inverse_ = 1.0 / std::max(speed_limit_, 1.0);

  planning_distance_inverse_ = 1.0 / std::max(1.0, planning_distance_);
  planning_time_horizon_inverse_ = 1.0 / std::max(1.0, planning_time_horizon_);

  t_step_inverse_ = 1.0 / std::max(t_step_, 1.0);
  t_step_square_ = t_step_ * t_step_;

  ComputeAccelStep();
}

void StSearchInput::ComputeAccelStep() {
  for (size_t i = 0; i < accel_sample_num_; ++i) {
    accel_step_.emplace_back(min_accel_limit_ +
                             i * (max_accel_limit_ - min_accel_limit_) /
                                 accel_sample_num_);
  }
}

double StSearchInput::init_s() const { return init_s_; }
double StSearchInput::init_t() const { return init_t_; }
double StSearchInput::init_vel() const { return init_vel_; }

double StSearchInput::planning_distance() const { return planning_distance_; }
double StSearchInput::planning_distance_inverse() const {
  return planning_distance_inverse_;
}
double StSearchInput::planning_time_horizon() const {
  return planning_time_horizon_;
}
double StSearchInput::planning_time_horizon_inverse() const {
  return planning_time_horizon_inverse_;
}

double StSearchInput::cruise_speed() const { return cruise_speed_; }
double StSearchInput::speed_limit() const { return speed_limit_; }
double StSearchInput::speed_limit_inverse() const {
  return speed_limit_inverse_;
}

double StSearchInput::max_accel_limit() const { return max_accel_limit_; }
double StSearchInput::min_accel_limit() const { return min_accel_limit_; }
double StSearchInput::max_jerk_limit() const { return max_jerk_limit_; }
double StSearchInput::min_jerk_limit() const { return min_jerk_limit_; }

int64_t StSearchInput::accel_sample_num() const { return accel_sample_num_; }
std::vector<double> StSearchInput::accel_step() const { return accel_step_; }

double StSearchInput::s_step() const { return s_step_; }
double StSearchInput::t_step() const { return t_step_; }
double StSearchInput::t_step_inverse() const { return t_step_inverse_; }
double StSearchInput::t_step_square() const { return t_step_square_; }
double StSearchInput::vel_step() const { return vel_step_; }

}  // namespace planning