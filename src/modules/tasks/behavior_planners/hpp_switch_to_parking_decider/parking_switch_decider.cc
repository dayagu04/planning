#include "parking_switch_decider.h"
#include <cmath>
#include "environmental_model.h"
#include "planning_context.h"
#include "session.h"
#include "virtual_lane_manager.h"

namespace planning {
ParkingSwitchDecider::ParkingSwitchDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  config_ = config_builder->cast<HppParkingSwitchConfig>();
  name_ = "ParkingSwitchDecider";
  parking_switch_info_.Clear();
  timestamp_at_standstill_near_dest_ = 0.0;
}

bool ParkingSwitchDecider::Execute() {
  parking_switch_info_.Clear();
  // get distance_to_target_slot
  const EnvironmentalModel &env = session_->environmental_model();
  const auto &current_reference_path =
      env.get_reference_path_manager()->get_reference_path_by_current_lane();
  auto &parking_slot_manager = env.get_parking_slot_manager();
  parking_slot_manager->CalculateDistanceToTargetSlot(current_reference_path);
  const bool is_reached_target_slot =
      parking_slot_manager->IsReachedTargetSlot();
  double distance_to_target_slot =
      parking_slot_manager->GetDistanceToTargetSlot();
  const bool is_near_target_slot = (is_reached_target_slot ||
          distance_to_target_slot < config_.dist_to_parking_space_thr);

  ILOG_DEBUG << "distance_to_target_slot:" << distance_to_target_slot;
  JSON_DEBUG_VALUE("distance_to_target_slot", distance_to_target_slot);
  parking_switch_info_.dist_to_memory_slot = distance_to_target_slot;

  // hpp状态切park_in状态
  const auto &current_state =
      env.get_local_view().function_state_machine_info.current_state;
  const size_t successful_slot_info_list_size =
      session_->planning_context()
          .planning_output()
          .successful_slot_info_list_size;
  const bool is_target_slot_allowed_to_park = IsTargetSlotAllowedToPark();
  const double ego_v = env.get_ego_state_manager()->ego_v();
  const auto& parking_switch_decider_output =
      session_->planning_context().parking_switch_decider_output();
  if(current_state == iflyauto::FunctionalState_HPP_CRUISE_SEARCHING) {
    if(is_target_slot_allowed_to_park) {
      parking_switch_info_.is_selected_slot_allowed_to_park = true;
    }
    if(successful_slot_info_list_size > 0) {
      parking_switch_info_.has_parking_slot_in_hpp_searching = true;
    }
    // 单车位
    if (successful_slot_info_list_size < 2) {
      parking_switch_info_.is_selected_slot_allowed_to_park = false;
    }
  } else if(current_state == iflyauto::FunctionalState_HPP_CRUISE_ROUTING) {
    if (is_target_slot_allowed_to_park && is_near_target_slot) {
      parking_switch_info_.is_memory_slot_allowed_to_park = true;
    } else {
      if (is_near_target_slot && (ego_v <= 1e-2) ||
          (distance_to_target_slot <= (ego_v * 0.1 + 0.1))) {
        parking_switch_info_.is_memory_slot_occupied = true;
      }
      if ((parking_switch_decider_output.parking_switch_info
               .is_memory_slot_occupied)) {
        parking_switch_info_.is_memory_slot_occupied = true;
      }
    }
  } else {
    // do nothing
  }

  //for E541
  if(current_state == iflyauto::FunctionalState_HPP_CRUISE_ROUTING) {
    const auto last_is_standstill_near_routing_destination =
        parking_switch_info_.is_standstill_near_routing_destination;
    const double curr_timestamp = IflyTime::Now_ms();
    if (IsNearRoutingDestination() && (ego_v <= 1e-2)) {
      parking_switch_info_.is_standstill_near_routing_destination = true;
      if(last_is_standstill_near_routing_destination == false) {
        timestamp_at_standstill_near_dest_ = curr_timestamp;
      }
      const double duration_time_since_standstill_near_dest =
          (curr_timestamp - timestamp_at_standstill_near_dest_) / 1000.0;
      if (parking_switch_info_.is_memory_slot_allowed_to_park == false &&
          duration_time_since_standstill_near_dest >
              config_.memory_slot_allowed_to_park_time_thr) {
        parking_switch_info_.is_timeout_for_memory_slot_allowed_to_park = true;
      } else {
        parking_switch_info_.is_timeout_for_memory_slot_allowed_to_park = false;
      }
    } else {
      parking_switch_info_.is_standstill_near_routing_destination = false;
      timestamp_at_standstill_near_dest_ = 0.0;
    }
  }

  JSON_DEBUG_VALUE("is_memory_slot_allowed_to_park", parking_switch_info_.is_memory_slot_allowed_to_park);
  JSON_DEBUG_VALUE("is_standstill_near_routing_destination", parking_switch_info_.is_standstill_near_routing_destination);
  JSON_DEBUG_VALUE("is_timeout_for_memory_slot_allowed_to_park", parking_switch_info_.is_timeout_for_memory_slot_allowed_to_park);

  session_->mutable_planning_context()
      ->mutable_parking_switch_decider_output()
      .parking_switch_info = std::move(parking_switch_info_);
  return true;
}

bool ParkingSwitchDecider::IsNearRoutingDestination() {
  const EnvironmentalModel &env = session_->environmental_model();
  // TODO(taolu10): 改成距离目标位置中心线的距离,
  // 前提是修改 Cruise 阶段的终点在目标车位终点，
  // 否则无法触发 is_standstill_near_routing_destination = true
  double distance_to_destination =
      env.get_route_info()->get_route_info_output().distance_to_target_slot;
  const double dist_to_routing_destination_thr = config_.dist_to_routing_destination_thr;
  ILOG_DEBUG << "distance_to_destination:" << distance_to_destination;
  JSON_DEBUG_VALUE("distance_to_destination", distance_to_destination);
  return distance_to_destination <= dist_to_routing_destination_thr;
}

bool ParkingSwitchDecider::IsTargetSlotAllowedToPark() {
  const EnvironmentalModel &env = session_->environmental_model();
  const size_t successful_slot_info_list_size =
      session_->planning_context()
          .planning_output()
          .successful_slot_info_list_size;
  const auto& successful_slot_info_list =
      session_->planning_context().planning_output().successful_slot_info_list;
  const size_t target_slot_id =
      env.get_local_view().parking_fusion_info.select_slot_id;
  for (size_t i = 0; i < successful_slot_info_list_size; ++i) {
    if ((target_slot_id == successful_slot_info_list[i].id) &&
        (target_slot_id > 0)) {
      return true;
    }
  }
  return false;
}
}  // namespace planning
