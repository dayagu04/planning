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

  // 基于 reference path 更新到终点和目标车位的信息
  UpdateTargetInfoBasedOnReferencePath(current_reference_path);

  // 更新自车和巡航终点/目标车位的关系
  const auto &route_info_output = env.get_route_info()->get_route_info_output();
  const auto &parking_slot_manager = env.get_parking_slot_manager();
  const bool is_exist_target_slot = parking_slot_manager->IsExistTargetSlot();
  const bool is_target_slot_allowed_to_park = IsTargetSlotAllowedToPark();

  const double dist_to_target_slot = route_info_output.distance_to_target_slot;
  const double dist_to_target_dest = route_info_output.distance_to_target_dest;
  const bool is_reached_target_slot =
      (is_exist_target_slot &&
       dist_to_target_slot < config_.dist_to_target_slot_thr);
  const bool is_reached_target_dest =
      dist_to_target_dest < config_.dist_to_target_dest_thr;
  const bool is_ego_still = env.get_ego_state_manager()->ego_v() <= 1e-2;
  session_->mutable_environmental_model()
      ->get_parking_slot_manager()
      ->SetIsReachedTarget(is_reached_target_slot, is_reached_target_dest);

  ILOG_DEBUG << "dist_to_target_slot:" << dist_to_target_slot;
  ILOG_DEBUG << "dist_to_target_dest:" << dist_to_target_dest;
  ILOG_DEBUG << "is_exist_target_slot:" << is_exist_target_slot;
  ILOG_DEBUG << "is_target_slot_allowed_to_park:" << is_target_slot_allowed_to_park;
  JSON_DEBUG_VALUE("dist_to_target_slot", dist_to_target_slot);
  JSON_DEBUG_VALUE("dist_to_target_dest", dist_to_target_dest);
  JSON_DEBUG_VALUE("is_exist_target_slot", is_exist_target_slot);
  JSON_DEBUG_VALUE("is_target_slot_allowed_to_park", is_target_slot_allowed_to_park);

  // hpp状态切park_in状态
  const auto &current_state =
      env.get_local_view().function_state_machine_info.current_state;
  const size_t successful_slot_info_list_size =
      session_->planning_context()
          .planning_output()
          .successful_slot_info_list_size;
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
    if(is_reached_target_slot) {
      if(is_target_slot_allowed_to_park) {
        parking_switch_info_.is_target_slot_allowed_to_park = true;
      } else if(is_ego_still) {
        parking_switch_info_.is_target_slot_occupied = true;
      } else {
        //do nothing
      }
    }

    //for E541
    const double curr_timestamp = IflyTime::Now_ms();
    if (is_reached_target_slot && is_ego_still) {
      parking_switch_info_.is_standstill_near_target_slot = true;
      if(last_is_standstill_near_target_slot_ == false) {
        timestamp_at_standstill_near_dest_ = curr_timestamp;
      }
      const double duration_time_since_standstill_near_dest =
          (curr_timestamp - timestamp_at_standstill_near_dest_) / 1000.0;
      if (parking_switch_info_.is_target_slot_allowed_to_park == false &&
          duration_time_since_standstill_near_dest >
              config_.memory_slot_allowed_to_park_time_thr) {
        parking_switch_info_.is_timeout_for_target_slot_allowed_to_park = true;
      } else {
        parking_switch_info_.is_timeout_for_target_slot_allowed_to_park = false;
      }
    } else {
      parking_switch_info_.is_standstill_near_target_slot = false;
      timestamp_at_standstill_near_dest_ = 0.0;
    }
    last_is_standstill_near_target_slot_ = parking_switch_info_.is_standstill_near_target_slot;
  } else {
    // do nothing
  }

  JSON_DEBUG_VALUE("is_target_slot_allowed_to_park", parking_switch_info_.is_target_slot_allowed_to_park);
  JSON_DEBUG_VALUE("is_standstill_near_target_slot", parking_switch_info_.is_standstill_near_target_slot);
  JSON_DEBUG_VALUE("is_timeout_for_target_slot_allowed_to_park", parking_switch_info_.is_timeout_for_target_slot_allowed_to_park);

  session_->mutable_planning_context()
      ->mutable_parking_switch_decider_output()
      .parking_switch_info = std::move(parking_switch_info_);
  return true;
}

bool ParkingSwitchDecider::UpdateTargetInfoBasedOnReferencePath(
    const std::shared_ptr<ReferencePath>& reference_path) {
  const auto &route_info_output =
      session_->environmental_model().get_route_info()->get_route_info_output();
  const auto &parking_slot_manager =
      session_->environmental_model().get_parking_slot_manager();
  double dist_to_target_slot = route_info_output.distance_to_target_slot;
  double dist_to_target_dest = route_info_output.distance_to_target_dest;

  if (reference_path != nullptr && parking_slot_manager->IsExistTargetSlot()) {
    const double ego_s = reference_path->get_frenet_ego_state().s();
    const auto& frenet_coord = reference_path->get_frenet_coord();

    const auto& target_slot_center = parking_slot_manager->GetTargetSlotCenter();
    Point2D target_slot_cart_pt(target_slot_center.x(), target_slot_center.y());
    Point2D target_slot_frenet_pt{0.0, 0.0};
    if (frenet_coord->XYToSL(target_slot_cart_pt, target_slot_frenet_pt)) {
      dist_to_target_slot = std::fabs(target_slot_frenet_pt.x - ego_s);
    }

    const auto& target_dest_point = route_info_output.target_dest_point;
    Point2D target_dest_cart_pt(target_dest_point.x(), target_dest_point.y());
    Point2D target_dest_frenet_pt{0.0, 0.0};
    if (frenet_coord->XYToSL(target_dest_cart_pt, target_dest_frenet_pt)) {
      dist_to_target_dest = std::fabs(target_dest_frenet_pt.x - ego_s);
    }
  }
  session_->mutable_environmental_model()->get_route_info()->UpdateTargetInfo(
      dist_to_target_slot, dist_to_target_dest);
  return true;
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
