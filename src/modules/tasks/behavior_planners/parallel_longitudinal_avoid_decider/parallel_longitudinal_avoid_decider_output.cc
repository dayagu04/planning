#include "parallel_longitudinal_avoid_decider_output.h"

namespace planning {

bool ParallelLongitudinalAvoidDeciderOutput::
    is_need_parallel_longitudinal_avoid() const {
  return is_need_parallel_longitudinal_avoid_;
}

void ParallelLongitudinalAvoidDeciderOutput::
    set_is_need_parallel_longitudinal_avoid(const bool value) {
  is_need_parallel_longitudinal_avoid_ = value;
}

int32_t ParallelLongitudinalAvoidDeciderOutput::parallel_target_agent_id()
    const {
  return parallel_target_agent_id_;
}

void ParallelLongitudinalAvoidDeciderOutput::set_parallel_target_agent_id(
    const int32_t id) {
  parallel_target_agent_id_ = id;
}

bool ParallelLongitudinalAvoidDeciderOutput::is_parallel_overtake() const {
  return is_parallel_overtake_;
}

void ParallelLongitudinalAvoidDeciderOutput::set_is_parallel_overtake(
    const bool value) {
  is_parallel_overtake_ = value;
}

bool ParallelLongitudinalAvoidDeciderOutput::is_parallel_yield() const {
  return is_parallel_yield_;
}

void ParallelLongitudinalAvoidDeciderOutput::set_is_parallel_yield(
    const bool value) {
  is_parallel_yield_ = value;
}

bool ParallelLongitudinalAvoidDeciderOutput::is_lead_and_target_is_truck()
    const {
  return is_lead_and_target_is_truck_;
}

void ParallelLongitudinalAvoidDeciderOutput::set_is_lead_and_target_is_truck(
    const bool value) {
  is_lead_and_target_is_truck_ = value;
}

int32_t ParallelLongitudinalAvoidDeciderOutput::current_state() const {
  return current_state_;
}

void ParallelLongitudinalAvoidDeciderOutput::set_current_state(
    const int32_t state) {
  current_state_ = state;
}

int32_t ParallelLongitudinalAvoidDeciderOutput::running_frame_count() const {
  return running_frame_count_;
}

void ParallelLongitudinalAvoidDeciderOutput::set_running_frame_count(
    const int32_t count) {
  running_frame_count_ = count;
}

int32_t ParallelLongitudinalAvoidDeciderOutput::cooldown_frame_count() const {
  return cooldown_frame_count_;
}

void ParallelLongitudinalAvoidDeciderOutput::set_cooldown_frame_count(
    const int32_t count) {
  cooldown_frame_count_ = count;
}

double ParallelLongitudinalAvoidDeciderOutput::trajectory_start_s() const {
  return trajectory_start_s_;
}

void ParallelLongitudinalAvoidDeciderOutput::set_trajectory_start_s(
    const double s) {
  trajectory_start_s_ = s;
}

double ParallelLongitudinalAvoidDeciderOutput::trajectory_start_v() const {
  return trajectory_start_v_;
}

void ParallelLongitudinalAvoidDeciderOutput::set_trajectory_start_v(
    const double v) {
  trajectory_start_v_ = v;
}

double ParallelLongitudinalAvoidDeciderOutput::lateral_distance() const {
  return lateral_distance_;
}

void ParallelLongitudinalAvoidDeciderOutput::set_lateral_distance(
    const double distance) {
  lateral_distance_ = distance;
}

}  // namespace planning
