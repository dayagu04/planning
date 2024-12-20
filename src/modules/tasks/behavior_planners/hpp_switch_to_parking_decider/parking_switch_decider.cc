#include "parking_switch_decider.h"
#include <cmath>
#include "planning_context.h"
#include "virtual_lane_manager.h"
#include "environmental_model.h"
#include "session.h"
#include "virtual_lane_manager.h"

namespace planning {
ParkingSwitchDecider::ParkingSwitchDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  config_ = config_builder->cast<HppParkingSwitchConfig>();
  name_ = "ParkingSwitchDecider";
}

bool ParkingSwitchDecider::Execute() {
  Clear();

  if (!session_->is_hpp_scene()) {
    return true;
  }
  // get distance_to_target_slot
  const EnvironmentalModel &env = session_->environmental_model();
  const auto& current_reference_path =
      env.get_reference_path_manager()->get_reference_path_by_current_lane();
  auto& parking_slot_manager = env.get_parking_slot_manager();
    parking_slot_manager->CalculateDistanceToTargetSlot(current_reference_path);
  size_t target_slot_id = env.get_local_view().parking_fusion_info.select_slot_id;
  double distance_to_destination =
      env.get_route_info()->get_route_info_output().distance_to_target_slot;
  double distance_to_target_slot =
        parking_slot_manager->GetDistanceToTargetSlot();
  LOG_DEBUG("distance_to_target_slot: %f\n", distance_to_target_slot);
  JSON_DEBUG_VALUE("distance_to_target_slot", distance_to_target_slot);
  parking_switch_info_.dist_to_memory_slot = distance_to_target_slot;

  // hpp状态切park_in状态
  const auto &current_state =
      env.get_local_view().function_state_machine_info.current_state;
  // const auto &apa_planning_status = session_.planning_context().planning_output().planning_status.apa_planning_status;
  const size_t successful_slot_info_list_size =
      session_->planning_context().planning_output().successful_slot_info_list_size;
  const auto &successful_slot_info_list =
      session_->planning_context().planning_output().successful_slot_info_list;
  const double ego_v = env.get_ego_state_manager()->ego_v();
  if ((successful_slot_info_list_size > 0) &&
      (current_state == iflyauto::FunctionalState_HPP_CRUISE_SEARCHING)) {
    parking_switch_info_.is_selected_slot_allowed_to_park = true;
  } else if ((distance_to_target_slot < config_.dist_to_parking_space_thr) &&
             (current_state == iflyauto::FunctionalState_HPP_CRUISE_ROUTING)) {
    for (const auto& slot_info : successful_slot_info_list) {
      if ((target_slot_id == slot_info.id) && (target_slot_id != 0)) {
        parking_switch_info_.is_memory_slot_allowed_to_park = true;
        break;
      }
    }
  }
  if ((distance_to_destination < config_.dist_to_parking_space_thr) &&
    (!parking_switch_info_.is_memory_slot_allowed_to_park) &&
    (current_state == iflyauto::FunctionalState_HPP_CRUISE_ROUTING)) {
    if ((ego_v <= 1e-2) || (distance_to_destination <= 1e-1)) {
      parking_switch_info_.is_memory_slot_occupied = true;
    }
  }

  auto &parking_switch_decider_output =
      session_->mutable_planning_context()
          ->mutable_parking_switch_decider_output();
  parking_switch_decider_output.parking_switch_info = std::move(parking_switch_info_);
  return true;
}

void ParkingSwitchDecider::Clear() {
  auto &parking_switch_decider_output =
      session_->mutable_planning_context()
          ->mutable_parking_switch_decider_output();
  parking_switch_decider_output.parking_switch_info.Clear();
  parking_switch_info_.Clear();
  return;
}

}  // namespace planning
