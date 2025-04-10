#pragma once
#include <cstdint>
#include <limits>

#include "basic_types.pb.h"
#include "behavior_planners/closest_in_path_vehicle_decider/closest_in_path_vehicle_decider_output.h"
#include "ego_planning_config.h"
#include "start_stop_decider_output.h"

namespace planning {
class StartStopStatusManager {
 public:
  StartStopStatusManager(const StartStopDeciderConfig& config);
  void Update();

  bool& mutable_current_traffic_light_can_pass() {
    return current_traffic_light_can_pass_;
  }
  const bool current_traffic_light_can_pass() const {
    return current_traffic_light_can_pass_;
  }

  double& mutable_current_distance_ego_to_stopline() {
    return current_distance_ego_to_stopline_;
  }

  const double current_distance_ego_to_stopline() const {
    return current_distance_ego_to_stopline_;
  }

  common::IntersectionState& mutable_current_intersection_state_ego() {
    return current_intersection_state_ego_;
  }

  const common::IntersectionState current_intersection_state_ego() const {
    return current_intersection_state_ego_;
  }

  bool& mutable_is_ego_reverse() { return is_ego_reverse_; }

  const bool is_ego_reverse() const { return is_ego_reverse_; }

  bool& mutable_dbw_status() { return dbw_status_; }

  const bool dbw_status() const { return dbw_status_; }

  double& mutable_cipv_vel_frenet() { return cipv_vel_frenet_; }

  const double cipv_vel_frenet() const { return cipv_vel_frenet_; }

  double& mutable_cipv_relative_s() { return cipv_relative_s_; }

  const double cipv_relative_s() const { return cipv_relative_s_; }

  int32_t& mutable_cipv_id() { return cipv_id_; }

  const int32_t cipv_id() const { return cipv_id_; }

  double& mutable_planning_init_state_velocity() {
    return planning_init_state_vel;
  }

  const double planning_init_state_velocity() const {
    return planning_init_state_vel;
  }

  common::StartStopInfo& mutable_ego_start_stop_info() {
    return ego_start_stop_info_;
  }

  const common::StartStopInfo& ego_start_stop_info() const {
    return ego_start_stop_info_;
  }

 private:
  const StartStopDeciderConfig& config_;
  // traffic light decision info
  bool current_traffic_light_can_pass_ = true;
  double current_distance_ego_to_stopline_ = std::numeric_limits<double>::max();
  common::IntersectionState current_intersection_state_ego_ =
      common::IntersectionState::UNKNOWN;
  // ego state info
  common::StartStopInfo ego_start_stop_info_;
  double planning_init_state_vel = 33.33;
  // dbw info
  bool dbw_status_ = true;
  // cipv info
  double cipv_vel_frenet_ = 0.0;
  double cipv_relative_s_ = 0.0;
  int32_t cipv_id_ = -1;
  bool is_ego_reverse_ = false;
};
}  // namespace planning