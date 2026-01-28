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
  // 从 hpp_stop_decider 获取输出（距离信息已在 hpp_stop_decider 中更新）
  const auto &hpp_stop_decider_output = 
      session_->planning_context().hpp_stop_decider_output();
  
  // 使用 hpp_stop_decider 的输出
  const bool is_reached_target_slot =
      hpp_stop_decider_output.is_reached_target_slot;
  const bool is_reached_target_dest =
      hpp_stop_decider_output.is_reached_target_dest;
  
  // 判断车辆停车满足3帧后，使用 is_stopped_at_destination 替代 is_reached_target_slot
  const bool is_stopped_at_destination = 
      (hpp_stop_decider_output.is_stopped_at_destination && 
       hpp_stop_decider_output.stop_frame_count >= 3);
  
  const auto &parking_slot_manager = env.get_parking_slot_manager();
  const bool is_exist_target_slot = parking_slot_manager->IsExistTargetSlot();
  const bool is_target_slot_allowed_to_park = IsTargetSlotAllowedToPark();
  const bool is_ego_still = env.get_ego_state_manager()->ego_v() <= 2e-1;
  
  // 获取距离信息用于日志和E541逻辑
  const auto &route_info_output = env.get_route_info()->get_route_info_output();
  const double dist_to_target_slot = route_info_output.distance_to_target_slot;
  const double dist_to_target_dest = route_info_output.distance_to_target_dest;

  ILOG_DEBUG << "is_reached_target_slot:" << is_reached_target_slot;
  ILOG_DEBUG << "is_reached_target_dest:" << is_reached_target_dest;
  ILOG_DEBUG << "is_stopped_at_destination:" << is_stopped_at_destination;
  ILOG_DEBUG << "stop_frame_count:" << hpp_stop_decider_output.stop_frame_count;
  ILOG_DEBUG << "dist_to_target_slot:" << dist_to_target_slot;
  ILOG_DEBUG << "dist_to_target_dest:" << dist_to_target_dest;
  ILOG_DEBUG << "is_exist_target_slot:" << is_exist_target_slot;
  ILOG_DEBUG << "is_target_slot_allowed_to_park:" << is_target_slot_allowed_to_park;
  JSON_DEBUG_VALUE("is_reached_target_slot", is_reached_target_slot);
  JSON_DEBUG_VALUE("is_reached_target_dest", is_reached_target_dest);
  JSON_DEBUG_VALUE("is_stopped_at_destination", is_stopped_at_destination);
  JSON_DEBUG_VALUE("stop_frame_count", hpp_stop_decider_output.stop_frame_count);
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
    // 使用 is_stopped_at_destination（满足3帧停车条件）替代 is_reached_target_slot
    if(is_stopped_at_destination) {
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
    // 使用 is_stopped_at_destination（满足3帧停车条件）替代 is_reached_target_slot
    if (is_stopped_at_destination && is_ego_still) {
      if(last_is_standstill_near_target_slot_ == false) {
        timestamp_at_standstill_near_dest_ = curr_timestamp;
      }
      const double duration_time_since_standstill_near_dest =
          (curr_timestamp - timestamp_at_standstill_near_dest_) / 1000.0;

      if (dist_to_target_slot < 1.0 ||
          duration_time_since_standstill_near_dest >
              config_.keeping_still_time_thr_for_switch_parking) {
        parking_switch_info_.is_standstill_near_target_slot = true;
      } else {
        parking_switch_info_.is_standstill_near_target_slot = false;
      }

      if (parking_switch_info_.is_target_slot_allowed_to_park == false &&
          duration_time_since_standstill_near_dest >
              config_.timeout_still_time_thr_for_giving_up_parking) {
        parking_switch_info_.is_timeout_for_target_slot_allowed_to_park = true;
      } else {
        parking_switch_info_.is_timeout_for_target_slot_allowed_to_park = false;
      }
      last_is_standstill_near_target_slot_ = true;
    } else {
      parking_switch_info_.is_standstill_near_target_slot = false;
      timestamp_at_standstill_near_dest_ = 0.0;
      last_is_standstill_near_target_slot_ = false;
    }
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
