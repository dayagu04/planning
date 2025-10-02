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
}

bool ParkingSwitchDecider::Execute() {
  parking_switch_info_.Clear();
  // get distance_to_target_slot
  const EnvironmentalModel &env = session_->environmental_model();
  const auto &current_reference_path =
      env.get_reference_path_manager()->get_reference_path_by_current_lane();
  auto &parking_slot_manager = env.get_parking_slot_manager();
  parking_slot_manager->CalculateDistanceToTargetSlot(current_reference_path);
  size_t target_slot_id =
      env.get_local_view().parking_fusion_info.select_slot_id;
  const bool is_reached_target_slot =
      parking_slot_manager->IsReachedTargetSlot();
  double distance_to_destination =
      env.get_route_info()->get_route_info_output().distance_to_target_slot;
  double distance_to_target_slot =
      parking_slot_manager->GetDistanceToTargetSlot();
  ILOG_DEBUG << "distance_to_target_slot:" << distance_to_target_slot;
  JSON_DEBUG_VALUE("distance_to_target_slot", distance_to_target_slot);
  parking_switch_info_.dist_to_memory_slot = distance_to_target_slot;

  // hpp状态切park_in状态
  const auto &current_state =
      env.get_local_view().function_state_machine_info.current_state;
  // const auto &apa_planning_status =
  // session_.planning_context().planning_output().planning_status.apa_planning_status;
  const size_t successful_slot_info_list_size =
      session_->planning_context()
          .planning_output()
          .successful_slot_info_list_size;
  const auto &successful_slot_info_list =
      session_->planning_context().planning_output().successful_slot_info_list;
  const double ego_v = env.get_ego_state_manager()->ego_v();
  if ((successful_slot_info_list_size > 0) &&
      (current_state == iflyauto::FunctionalState_HPP_CRUISE_SEARCHING)) {
    // 多车位
    for (size_t i = 0; i < successful_slot_info_list_size; ++i) {
      if ((target_slot_id == successful_slot_info_list[i].id) && (target_slot_id > 0)) {
        parking_switch_info_.is_selected_slot_allowed_to_park = true;
        break;
      }
    }
    // 单车位
    if (successful_slot_info_list_size < 2) {
      parking_switch_info_.is_selected_slot_allowed_to_park = false;
    }
    parking_switch_info_.has_parking_slot_in_hpp_searching = true;
  } else if ((is_reached_target_slot ||
             (distance_to_target_slot < config_.dist_to_parking_space_thr)) &&
             (current_state == iflyauto::FunctionalState_HPP_CRUISE_ROUTING)) {
    for (size_t i = 0; i < successful_slot_info_list_size; ++i) {
      if ((target_slot_id == successful_slot_info_list[i].id) && (target_slot_id > 0)) {
        parking_switch_info_.is_memory_slot_allowed_to_park = true;
        break;
      }
    }
    if ((ego_v <= 1e-2) ||
        (distance_to_target_slot <= (ego_v * 0.1 + 0.1))) {
      parking_switch_info_.is_memory_slot_occupied = true;
    }
  } else if (is_reached_target_slot &&
             (current_state == iflyauto::FunctionalState_HPP_CRUISE_ROUTING)) {
    parking_switch_info_.is_memory_slot_occupied = true;
  }

  ILOG_INFO << "parking_switch_info_.is_memory_slot_allowed_to_park"
            << parking_switch_info_.is_memory_slot_allowed_to_park;

  auto &parking_switch_decider_output =
      session_->mutable_planning_context()
          ->mutable_parking_switch_decider_output();
  if ((parking_switch_decider_output.parking_switch_info.is_memory_slot_occupied) &&
      (current_state == iflyauto::FunctionalState_HPP_CRUISE_ROUTING)) {
    parking_switch_info_.is_memory_slot_occupied = true;
  }
  if (parking_switch_info_.is_memory_slot_allowed_to_park) {
    parking_switch_info_.is_memory_slot_occupied = false;
  }
  parking_switch_decider_output.parking_switch_info =
      std::move(parking_switch_info_);
  return true;
}
}  // namespace planning
