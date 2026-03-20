#pragma once

#include "behavior_planners/st_graph_decider/st_search_node.h"
#include "trajectory/trajectory.h"
#include "trajectory/trajectory_point.h"
#include "utils/kd_path.h"

namespace planning {

class StSearchInput {
 public:
  StSearchInput(const trajectory::TrajectoryPoint& ego_init_state,
                const double planning_distance,
                const double planning_time_horizon, const double speed_limit,
                const double max_accel_limit, const double min_accel_limit,
                const double max_jerk_limit, const double min_jerk_limit,
                const int64_t accel_sample_num, const double s_step,
                const double t_step, const double vel_step);
  ~StSearchInput() = default;

  double init_s() const;
  double init_t() const;
  double init_vel() const;

  double planning_distance() const;
  double planning_distance_inverse() const;
  double planning_time_horizon() const;
  double planning_time_horizon_inverse() const;

  double cruise_speed() const;
  double speed_limit() const;
  double speed_limit_inverse() const;

  double max_accel_limit() const;
  double min_accel_limit() const;
  double max_jerk_limit() const;
  double min_jerk_limit() const;

  int64_t accel_sample_num() const;
  std::vector<double> accel_step() const;

  double s_step() const;
  double t_step() const;
  double t_step_inverse() const;
  double t_step_square() const;
  double vel_step() const;

 private:
  // make acc step by input min/max accel limit, and acc sample number
  void ComputeAccelStep();

 private:
  // init state
  const trajectory::TrajectoryPoint ego_init_state_;
  // const cp_common::trajectory::KDPath planned_path_;
  double init_s_ = 0.0;
  double init_t_ = 0.0;
  double init_vel_ = 0.0;

  // planning goal
  double planning_distance_ = 0.0;
  double planning_distance_inverse_ = 0.0;
  double planning_time_horizon_ = 0.0;
  double planning_time_horizon_inverse_ = 0.0;

  // limitations
  double cruise_speed_ = 0.0;
  double speed_limit_ = 0.0;
  double speed_limit_inverse_ = 0.0;

  double max_accel_limit_ = 0.0;
  double min_accel_limit_ = 0.0;
  double max_jerk_limit_ = 0.0;
  double min_jerk_limit_ = 0.0;

  double s_step_ = 1e-5;
  double t_step_ = 0.1;
  double t_step_inverse_ = 0.0;
  double t_step_square_ = 0.01;
  double vel_step_ = 1e-3;
  int64_t accel_sample_num_ = 100;
  std::vector<double> accel_step_;
};

}  // namespace planning