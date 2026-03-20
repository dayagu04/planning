#pragma once

#include <cstdint>
#include <string>

namespace planning {

class ParallelLongitudinalAvoidDeciderOutput {
 public:
  ParallelLongitudinalAvoidDeciderOutput() = default;
  ~ParallelLongitudinalAvoidDeciderOutput() = default;

  bool is_need_parallel_longitudinal_avoid() const;
  void set_is_need_parallel_longitudinal_avoid(const bool value);

  int32_t parallel_target_agent_id() const;
  void set_parallel_target_agent_id(const int32_t id);

  bool is_parallel_overtake() const;
  void set_is_parallel_overtake(const bool value);

  bool is_parallel_yield() const;
  void set_is_parallel_yield(const bool value);

  bool is_lead_and_target_is_truck() const;
  void set_is_lead_and_target_is_truck(const bool value);

  int32_t current_state() const;
  void set_current_state(const int32_t state);

  int32_t running_frame_count() const;
  void set_running_frame_count(const int32_t count);

  int32_t cooldown_frame_count() const;
  void set_cooldown_frame_count(const int32_t count);

  double trajectory_start_s() const;
  void set_trajectory_start_s(const double s);

  double trajectory_start_v() const;
  void set_trajectory_start_v(const double v);

  double lateral_distance() const;
  void set_lateral_distance(const double distance);

 private:
  bool is_need_parallel_longitudinal_avoid_ = false;
  int32_t parallel_target_agent_id_ = 0;
  bool is_parallel_overtake_ = false;
  bool is_parallel_yield_ = false;
  bool is_lead_and_target_is_truck_ = false;

  int32_t current_state_ = 0;
  int32_t running_frame_count_ = 0;
  int32_t cooldown_frame_count_ = 0;

  double trajectory_start_s_ = 0.0;
  double trajectory_start_v_ = 0.0;
  double lateral_distance_ = -1.0;
};

}  // namespace planning
