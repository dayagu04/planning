#include "start_stop_status_manager.h"

#include <cmath>
#include <cstddef>
#include <limits>

#include "agent/agent.h"
#include "basic_types.pb.h"
#include "behavior_planners/start_stop_decider/start_stop_decider_output.h"
#include "debug_info_log.h"

namespace planning {

StartStopStatusManager::StartStopStatusManager(
    const StartStopDeciderConfig& config)
    : config_(config) {}

void StartStopStatusManager::Update() {
  // start stop status of ego should be considered in different ODD:
  // 1.approaching intersection and in intersection
  // 2.except intersection
  const double ego_vel_start_mode_threshold =
      is_ego_reverse() ? config_.ego_vel_start_mode_threshold_rads
                       : config_.ego_vel_start_mode_threshold;
  if (dbw_status() == false) {
    const bool ego_start_condition = planning_init_state_velocity() > 0.2;
    const bool ego_stop_condition =
        planning_init_state_velocity() < config_.ego_vel_begin_stop_threshold;
    const bool ego_cruise_condition =
        planning_init_state_velocity() > ego_vel_start_mode_threshold;
    if (ego_start_stop_info_.state() == common::StartStopInfo::STOP &&
        ego_start_condition) {
      ego_start_stop_info_.set_state(common::StartStopInfo::START);
    } else if (ego_start_stop_info_.state() == common::StartStopInfo::START &&
               ego_cruise_condition) {
      ego_start_stop_info_.set_state(common::StartStopInfo::CRUISE);
    } else if (ego_start_stop_info_.state() == common::StartStopInfo::START &&
               ego_stop_condition) {
      ego_start_stop_info_.set_state(common::StartStopInfo::STOP);
    } else if (ego_start_stop_info_.state() == common::StartStopInfo::CRUISE &&
               ego_stop_condition) {
      ego_start_stop_info_.set_state(common::StartStopInfo::STOP);
    }
    JSON_DEBUG_VALUE("distance_to_go_condition", false)
    JSON_DEBUG_VALUE("v3_start_stop_status",
                     static_cast<int>(ego_start_stop_info_.state()))
    JSON_DEBUG_VALUE("cipv_relative_s", cipv_relative_s())
    JSON_DEBUG_VALUE("cipv_relative_s_ego_stop",
                     ego_start_stop_info_.cipv_relative_s_when_ego_stopped())
    JSON_DEBUG_VALUE("cipv_vel_frenet", cipv_vel_frenet())
    JSON_DEBUG_VALUE("traffic_light_can_pass", current_traffic_light_can_pass())
  } else {
    if (current_intersection_state_ego() ==
            common::IntersectionState::IN_INTERSECTION ||
        (current_intersection_state_ego() ==
             common::IntersectionState::APPROACH_INTERSECTION &&
         std::fabs(current_distance_ego_to_stopline()) <
             config_.distance_to_stop_line_ego_threshold)) {
      const bool cipv_nearby_intersection_exists =
          cipv_id() != agent::AgentDefaultInfo::kNoAgentId &&
          cipv_relative_s() < current_distance_ego_to_stopline();
      const bool cipv_nearby_intersection_is_static =
          cipv_nearby_intersection_exists &&
          std::fabs(cipv_vel_frenet()) < config_.cipv_vel_begin_start_threshold;
      const bool cipv_nearby_intersection_starts =
          cipv_nearby_intersection_exists &&
          (cipv_vel_frenet() > config_.cipv_vel_begin_start_threshold &&
           (cipv_relative_s() -
                ego_start_stop_info_.cipv_relative_s_when_ego_stopped() >
            config_.cipv_relative_s_begin_start_threshold));
      // intersection stop condition considers cipv and traffic light info
      const bool traffic_light_stop_condition =
          (!current_traffic_light_can_pass() &&
           planning_init_state_velocity() <
               config_.ego_vel_begin_stop_threshold) ||
          (planning_init_state_velocity() <
               config_.ego_vel_begin_stop_threshold &&
           cipv_nearby_intersection_is_static &&
           std::fabs(
               cipv_relative_s() -
               config_
                   .desired_stopped_distance_between_ego_and_cipv_threshold) <
               config_.distance_stop_buffer_between_ego_and_cipv_threshold);

      const bool distance_to_go_condition =
          cipv_nearby_intersection_is_static &&
          cipv_relative_s() > config_.distance_to_go_threshold;

      // intersection start condition considers both cipv and traffic light info
      bool traffic_light_start_condition = false;
      if (cipv_nearby_intersection_exists) {
        if (current_traffic_light_can_pass() &&
            cipv_nearby_intersection_is_static && !distance_to_go_condition) {
          traffic_light_start_condition = false;
        } else if (current_traffic_light_can_pass() &&
                   (cipv_nearby_intersection_starts ||
                    distance_to_go_condition)) {
          traffic_light_start_condition = true;
        }
      } else {
        traffic_light_start_condition = current_traffic_light_can_pass();
      }

      const bool ego_cruise_condition =
          planning_init_state_velocity() > ego_vel_start_mode_threshold;

      //  Update the state
      if (ego_start_stop_info_.state() == common::StartStopInfo::CRUISE &&
          traffic_light_stop_condition) {
        // CRUISE --> STOP
        ego_start_stop_info_.set_state(common::StartStopInfo::STOP);
        if (cipv_nearby_intersection_exists) {
          ego_start_stop_info_.set_cipv_relative_s_when_ego_stopped(
              cipv_relative_s());
        } else {
          ego_start_stop_info_.set_cipv_relative_s_when_ego_stopped(
              std::numeric_limits<double>::max());
        }
      } else if (ego_start_stop_info_.state() == common::StartStopInfo::STOP &&
                 traffic_light_start_condition) {
        // STOP --> START
        ego_start_stop_info_.set_state(common::StartStopInfo::START);
      } else if (ego_start_stop_info_.state() == common::StartStopInfo::START &&
                 ego_cruise_condition) {
        // START --> CRUISE
        ego_start_stop_info_.set_state(common::StartStopInfo::CRUISE);
      } else if (ego_start_stop_info_.state() == common::StartStopInfo::START &&
                 traffic_light_stop_condition) {
        // START --> STOP
        ego_start_stop_info_.set_state(common::StartStopInfo::STOP);
        if (cipv_nearby_intersection_exists) {
          ego_start_stop_info_.set_cipv_relative_s_when_ego_stopped(
              cipv_relative_s());
        } else {
          ego_start_stop_info_.set_cipv_relative_s_when_ego_stopped(
              std::numeric_limits<double>::max());
        }
      }
      JSON_DEBUG_VALUE("distance_to_go_condition", distance_to_go_condition)
    } else {
      const bool is_cipv_static =
          std::fabs(cipv_vel_frenet()) < config_.cipv_vel_begin_start_threshold;
      const bool ego_stop_condition =
          planning_init_state_velocity() <
              config_.ego_vel_begin_stop_threshold &&
          is_cipv_static &&
          std::fabs(
              cipv_relative_s() -
              config_.desired_stopped_distance_between_ego_and_cipv_threshold) <
              config_.distance_stop_buffer_between_ego_and_cipv_threshold;
      const bool ego_cruise_condition =
          planning_init_state_velocity() > ego_vel_start_mode_threshold;
      const bool cipv_start_condition =
          cipv_vel_frenet() > config_.cipv_vel_begin_start_threshold &&
          (cipv_relative_s() -
               ego_start_stop_info_.cipv_relative_s_when_ego_stopped() >
           config_.cipv_relative_s_begin_start_threshold);
      const bool distance_to_go_condition =
          is_cipv_static &&
          cipv_relative_s() > config_.distance_to_go_threshold;
      const bool ego_start_condition =
          cipv_start_condition || distance_to_go_condition;

      // 2. Update the state
      if (ego_start_stop_info_.state() == common::StartStopInfo::CRUISE &&
          ego_stop_condition) {
        // CRUISE --> STOP
        ego_start_stop_info_.set_state(common::StartStopInfo::STOP);
        // store the distance of cipv
        ego_start_stop_info_.set_cipv_relative_s_when_ego_stopped(
            cipv_relative_s());
      } else if (ego_start_stop_info_.state() == common::StartStopInfo::STOP &&
                 ego_start_condition) {
        // STOP --> START
        ego_start_stop_info_.set_state(common::StartStopInfo::START);
      } else if (ego_start_stop_info_.state() == common::StartStopInfo::START &&
                 ego_cruise_condition) {
        // START --> CRUISE
        ego_start_stop_info_.set_state(common::StartStopInfo::CRUISE);
      } else if (ego_start_stop_info_.state() == common::StartStopInfo::START &&
                 ego_stop_condition) {
        // START --> STOP
        ego_start_stop_info_.set_state(common::StartStopInfo::STOP);
        // store the distance of cipv
        ego_start_stop_info_.set_cipv_relative_s_when_ego_stopped(
            cipv_relative_s());
      }
      JSON_DEBUG_VALUE("distance_to_go_condition", distance_to_go_condition)
    }
    JSON_DEBUG_VALUE("v3_start_stop_status",
                     static_cast<int>(ego_start_stop_info_.state()))
    JSON_DEBUG_VALUE("cipv_relative_s", cipv_relative_s())
    JSON_DEBUG_VALUE("cipv_relative_s_ego_stop",
                     ego_start_stop_info_.cipv_relative_s_when_ego_stopped())
    JSON_DEBUG_VALUE("cipv_vel_frenet", cipv_vel_frenet())
    JSON_DEBUG_VALUE("traffic_light_can_pass", current_traffic_light_can_pass())
  }
}

}  // namespace planning