#include "apa_state_machine_manager.h"

#include <cmath>

#include "debug_info_log.h"
#include "func_state_machine_c.h"
#include "fusion_parking_slot_c.h"
#include "hmi_inner_c.h"
#include "log_glog.h"

namespace planning {
namespace apa_planner {

void ApaStateMachineManager::Update(const LocalView* local_view_ptr) {
  Reset();
  if (local_view_ptr == nullptr) {
    ILOG_ERROR << "Update ApaStateMachineManager, local_view_ptr is nullptr";
    return;
  }

  ILOG_INFO << "Update ApaStateMachineManager";

  const iflyauto::FuncStateMachine& fun_state_machine_info =
      local_view_ptr->function_state_machine_info;

  const iflyauto::ParkingFusionInfo& parking_fusion_info =
      local_view_ptr->parking_fusion_info;

  ILOG_INFO << "fun_state_machine_info.current_state = "
            << static_cast<int>(fun_state_machine_info.current_state);

  switch (fun_state_machine_info.current_state) {
    case iflyauto::FunctionalState_PARK_STANDBY:
      state_machine_ = ApaStateMachine::STANDBY;
      break;
    case iflyauto::FunctionalState_PARK_ERROR:
      state_machine_ = ApaStateMachine::ERROR;
      break;
    case iflyauto::FunctionalState_PARK_IN_SEARCHING:
    case iflyauto::FunctionalState_HPP_CRUISE_ROUTING:
    case iflyauto::FunctionalState_HPP_CRUISE_SEARCHING:
      if (parking_fusion_info.select_slot_id == 0) {
        state_machine_ = ApaStateMachine::SEARCH_IN_NO_SELECTED;
      } else if (fun_state_machine_info.parking_req.apa_free_slot_info.is_free_slot_selected
        == iflyauto::FreeSlotSelectedStatus::FREE_SLOT_SELECTED_STATUS_FINISHED) {
        if (!parking_fusion_info.parking_fusion_slot_lists[parking_fusion_info.select_slot_id].is_turn_corner) {
          state_machine_ = ApaStateMachine::SEARCH_IN_SELECTED_CAR_REAR;
        } else {
          state_machine_ = ApaStateMachine::SEARCH_IN_SELECTED_CAR_FRONT;
        }
      } else if (fun_state_machine_info.parking_req.apa_free_slot_info.is_free_slot_selected
        == iflyauto::FreeSlotSelectedStatus::FREE_SLOT_SELECTED_STATUS_DEFAULT) {
        if (fun_state_machine_info.parking_req.apa_parking_direction ==
            iflyauto::BACK_END_PARKING_DIRECTION) {
          state_machine_ = ApaStateMachine::SEARCH_IN_SELECTED_CAR_REAR;
        } else if (fun_state_machine_info.parking_req.apa_parking_direction ==
                   iflyauto::FRONT_END_PARKING_DIRECTION) {
          state_machine_ = ApaStateMachine::SEARCH_IN_SELECTED_CAR_FRONT;
        }
      }
      break;
    case iflyauto::FunctionalState_PARK_OUT_SEARCHING:
      if (fun_state_machine_info.parking_req.apa_park_out_direction ==
          iflyauto::PRK_OUT_DIRECTION_INVALID) {
        state_machine_ = ApaStateMachine::SEARCH_OUT_NO_SELECTED;
      } else {
        switch (fun_state_machine_info.parking_req.apa_park_out_direction) {
          case iflyauto::PRK_OUT_TO_FRONT_LEFT_CROSS:
          case iflyauto::PRK_OUT_TO_FRONT_LEFT_PARALLEL:
          case iflyauto::PRK_OUT_TO_FRONT_OUT:
          case iflyauto::PRK_OUT_TO_FRONT_RIGHT_CROSS:
          case iflyauto::PRK_OUT_TO_FRONT_RIGHT_PARALLEL:
            state_machine_ = ApaStateMachine::SEARCH_OUT_SELECTED_CAR_FRONT;
            break;
          case iflyauto::PRK_OUT_TO_BACK_OUT:
          case iflyauto::PRK_OUT_TO_BACK_LEFT_CROSS:
          case iflyauto::PRK_OUT_TO_BACK_RIGHT_CROSS:
            state_machine_ = ApaStateMachine::SEARCH_OUT_SELECTED_CAR_REAR;
            break;
          default:
            break;
        }
      }
      break;
    case iflyauto::FunctionalState_PARK_GUIDANCE:
    case iflyauto::FunctionalState_PARK_PRE_ACTIVE:
    case iflyauto::FunctionalState_HPP_PARKING_IN:
      if (fun_state_machine_info.parking_req.apa_work_mode ==
          iflyauto::APA_WORK_MODE_PARKING_IN) {
        if (fun_state_machine_info.parking_req.apa_free_slot_info.is_free_slot_selected ==
        iflyauto::FreeSlotSelectedStatus::FREE_SLOT_SELECTED_STATUS_FINISHED) {
          if (!parking_fusion_info.parking_fusion_slot_lists[parking_fusion_info.select_slot_id].is_turn_corner) {
            state_machine_ = ApaStateMachine::ACTIVE_IN_CAR_REAR;
          }
          else {
            state_machine_ = ApaStateMachine::ACTIVE_IN_CAR_FRONT;
          }
        } else if (fun_state_machine_info.parking_req.apa_free_slot_info.is_free_slot_selected ==
        iflyauto::FreeSlotSelectedStatus::FREE_SLOT_SELECTED_STATUS_DEFAULT) {
          if (fun_state_machine_info.parking_req.apa_parking_direction ==
              iflyauto::BACK_END_PARKING_DIRECTION) {
            state_machine_ = ApaStateMachine::ACTIVE_IN_CAR_REAR;
          } else if (fun_state_machine_info.parking_req.apa_parking_direction ==
                    iflyauto::FRONT_END_PARKING_DIRECTION) {
            state_machine_ = ApaStateMachine::ACTIVE_IN_CAR_FRONT;
          }
        }
      } else if (fun_state_machine_info.parking_req.apa_work_mode ==
                 iflyauto::APA_WORK_MODE_PARKING_OUT) {
        switch (fun_state_machine_info.parking_req.apa_park_out_direction) {
          case iflyauto::PRK_OUT_TO_FRONT_LEFT_CROSS:
          case iflyauto::PRK_OUT_TO_FRONT_LEFT_PARALLEL:
          case iflyauto::PRK_OUT_TO_FRONT_OUT:
          case iflyauto::PRK_OUT_TO_FRONT_RIGHT_CROSS:
          case iflyauto::PRK_OUT_TO_FRONT_RIGHT_PARALLEL:
            state_machine_ = ApaStateMachine::ACTIVE_OUT_CAR_FRONT;
            break;
          case iflyauto::PRK_OUT_TO_BACK_OUT:
          case iflyauto::PRK_OUT_TO_BACK_LEFT_CROSS:
          case iflyauto::PRK_OUT_TO_BACK_RIGHT_CROSS:
            state_machine_ = ApaStateMachine::ACTIVE_OUT_CAR_REAR;
            break;
          default:
            break;
        }
      }
      break;
    case iflyauto::FunctionalState_PARK_SUSPEND:
    case iflyauto::FunctionalState_HPP_SUSPEND:
      state_machine_ = ApaStateMachine::SUSPEND;
      break;
    case iflyauto::FunctionalState_PARK_COMPLETED:
    case iflyauto::FunctionalState_HPP_COMPLETE:
      state_machine_ = ApaStateMachine::COMPLETE;
      break;
    default:
      break;
  }

  switch (fun_state_machine_info.parking_req.apa_park_out_direction) {
    case iflyauto::PRK_OUT_TO_FRONT_LEFT_CROSS:
    case iflyauto::PRK_OUT_TO_FRONT_LEFT_PARALLEL:
      out_direction_ = ApaParkOutDirection::LEFT_FRONT;
      break;
    case iflyauto::PRK_OUT_TO_FRONT_RIGHT_CROSS:
    case iflyauto::PRK_OUT_TO_FRONT_RIGHT_PARALLEL:
      out_direction_ = ApaParkOutDirection::RIGHT_FRONT;
      break;
    case iflyauto::PRK_OUT_TO_BACK_LEFT_CROSS:
      out_direction_ = ApaParkOutDirection::LEFT_REAR;
      break;
    case iflyauto::PRK_OUT_TO_BACK_RIGHT_CROSS:
      out_direction_ = ApaParkOutDirection::RIGHT_REAR;
      break;
    case iflyauto::PRK_OUT_TO_FRONT_OUT:
      out_direction_ = ApaParkOutDirection::FRONT;
      break;
    case iflyauto::PRK_OUT_TO_BACK_OUT:
      out_direction_ = ApaParkOutDirection::REAR;
      break;
    default:
      break;
  }

  switch (fun_state_machine_info.parking_req.apa_user_preference
              .horizontal_preference_slot) {
    case iflyauto::HORIZONTAL_PREFERENCE_SLOT_LEFT:
      slot_lat_pos_preference_ = ApaSlotLatPosPreference::LEFT;
      break;
    case iflyauto::HORIZONTAL_PREFERENCE_SLOT_RIGHT:
      slot_lat_pos_preference_ = ApaSlotLatPosPreference::RIGHT;
      break;
    default:
      slot_lat_pos_preference_ = ApaSlotLatPosPreference::MID;
      break;
  }
  switch (fun_state_machine_info.parking_req.parking_speed_set) {
    case iflyauto::PARKING_SPEED_SET_NONE:
      parking_speed_mode_ = ParkingSpeedMode::INVALID;
      break;
    case iflyauto::PARKING_SPEED_SET_SLOW:
      parking_speed_mode_ = ParkingSpeedMode::SLOW;
      break;
    case iflyauto::PARKING_SPEED_SET_NORMAL:
      parking_speed_mode_ = ParkingSpeedMode::NORMAL;
      break;
    case iflyauto::PARKING_SPEED_SET_FAST:
      parking_speed_mode_ = ParkingSpeedMode::FAST;
      break;
    default:
      parking_speed_mode_ = ParkingSpeedMode::FAST;
      break;
  }

  PrintApaStateMachine(state_machine_);
  PrintApaParkOutDirection(out_direction_);
  PrintApaSlotLatPosPreference(slot_lat_pos_preference_);
  PrintParkingSpeedMode(parking_speed_mode_);

  JSON_DEBUG_VALUE("apa_state_machine", static_cast<int>(state_machine_))
  JSON_DEBUG_VALUE("apa_out_direction", static_cast<int>(out_direction_))
  JSON_DEBUG_VALUE("apa_slot_lat_pos_preference",
                   static_cast<int>(slot_lat_pos_preference_))

  return;
}
const bool ApaStateMachineManager::IsSeachingStatus() const {
  if (state_machine_ == ApaStateMachine::SEARCH_IN_NO_SELECTED ||
      state_machine_ == ApaStateMachine::SEARCH_IN_SELECTED_CAR_REAR ||
      state_machine_ == ApaStateMachine::SEARCH_IN_SELECTED_CAR_FRONT ||
      state_machine_ == ApaStateMachine::SEARCH_OUT_NO_SELECTED ||
      state_machine_ == ApaStateMachine::SEARCH_OUT_SELECTED_CAR_REAR ||
      state_machine_ == ApaStateMachine::SEARCH_OUT_SELECTED_CAR_FRONT) {
    return true;
  }

  return false;
}

const bool ApaStateMachineManager::IsSeachingOutStatus() const {
  if (state_machine_ == ApaStateMachine::SEARCH_OUT_NO_SELECTED ||
      state_machine_ == ApaStateMachine::SEARCH_OUT_SELECTED_CAR_REAR ||
      state_machine_ == ApaStateMachine::SEARCH_OUT_SELECTED_CAR_FRONT) {
    return true;
  }

  return false;
}

const bool ApaStateMachineManager::IsParkingStatus() const {
  if (state_machine_ == ApaStateMachine::ACTIVE_IN_CAR_FRONT ||
      state_machine_ == ApaStateMachine::ACTIVE_IN_CAR_REAR ||
      state_machine_ == ApaStateMachine::ACTIVE_OUT_CAR_FRONT ||
      state_machine_ == ApaStateMachine::ACTIVE_OUT_CAR_REAR) {
    return true;
  }

  return false;
}

const bool ApaStateMachineManager::IsParkOutStatus() const {
  if (state_machine_ == ApaStateMachine::SEARCH_OUT_NO_SELECTED ||
      state_machine_ == ApaStateMachine::SEARCH_OUT_SELECTED_CAR_FRONT ||
      state_machine_ == ApaStateMachine::SEARCH_OUT_SELECTED_CAR_REAR ||
      state_machine_ == ApaStateMachine::ACTIVE_OUT_CAR_FRONT ||
      state_machine_ == ApaStateMachine::ACTIVE_OUT_CAR_REAR) {
    return true;
  }

  return false;
}

const bool ApaStateMachineManager::IsHeadOutStatus() const {
  if (state_machine_ == ApaStateMachine::SEARCH_OUT_SELECTED_CAR_FRONT ||
      state_machine_ == ApaStateMachine::ACTIVE_OUT_CAR_FRONT) {
    return true;
  }

  return false;
}

const bool ApaStateMachineManager::IsTailOutStatus() const {
  if (state_machine_ == ApaStateMachine::SEARCH_OUT_SELECTED_CAR_REAR ||
      state_machine_ == ApaStateMachine::ACTIVE_OUT_CAR_REAR) {
    return true;
  }

  return false;
}

const bool ApaStateMachineManager::IsParkInStatus() const {
  if (state_machine_ == ApaStateMachine::SEARCH_IN_NO_SELECTED ||
      state_machine_ == ApaStateMachine::SEARCH_IN_SELECTED_CAR_FRONT ||
      state_machine_ == ApaStateMachine::SEARCH_IN_SELECTED_CAR_REAR ||
      state_machine_ == ApaStateMachine::ACTIVE_IN_CAR_FRONT ||
      state_machine_ == ApaStateMachine::ACTIVE_IN_CAR_REAR) {
    return true;
  }

  return false;
}

std::string ApaStateMachineManager::GetApaStateMachineString(
    const ApaStateMachine state_machine) {
  std::string state = "INVALID";
  switch (state_machine) {
    case ApaStateMachine::SEARCH_IN_NO_SELECTED:
      state = "SEARCH_IN_NO_SELECTED";
      break;
    case ApaStateMachine::SEARCH_IN_SELECTED_CAR_REAR:
      state = "SEARCH_IN_SELECTED_CAR_REAR";
      break;
    case ApaStateMachine::SEARCH_IN_SELECTED_CAR_FRONT:
      state = "SEARCH_IN_SELECTED_CAR_FRONT";
      break;
    case ApaStateMachine::SEARCH_OUT_NO_SELECTED:
      state = "SEARCH_OUT_NO_SELECTED";
      break;
    case ApaStateMachine::SEARCH_OUT_SELECTED_CAR_REAR:
      state = "SEARCH_OUT_SELECTED_CAR_REAR";
      break;
    case ApaStateMachine::SEARCH_OUT_SELECTED_CAR_FRONT:
      state = "SEARCH_OUT_SELECTED_CAR_FRONT";
      break;
    case ApaStateMachine::ACTIVE_IN_CAR_FRONT:
      state = "ACTIVE_IN_CAR_FRONT";
      break;
    case ApaStateMachine::ACTIVE_IN_CAR_REAR:
      state = "ACTIVE_IN_CAR_REAR";
      break;
    case ApaStateMachine::ACTIVE_OUT_CAR_FRONT:
      state = "ACTIVE_OUT_CAR_FRONT";
      break;
    case ApaStateMachine::ACTIVE_OUT_CAR_REAR:
      state = "ACTIVE_OUT_CAR_REAR";
      break;
    case ApaStateMachine::SECURE:
      state = "SECURE";
      break;
    case ApaStateMachine::SUSPEND:
      state = "SUSPEND";
      break;
    case ApaStateMachine::COMPLETE:
      state = "COMPLETE";
      break;
    case ApaStateMachine::STANDBY:
      state = "STANDBY";
      break;
    case ApaStateMachine::ERROR:
      state = "ERROR";
      break;
    default:
      state = "INVALID";
      break;
  }
  return state;
}

void ApaStateMachineManager::PrintApaStateMachine(
    const ApaStateMachine state_machine) {
  ILOG_INFO << "apa_state = " << GetApaStateMachineString(state_machine);
}

std::string ApaStateMachineManager::GetApaParkOutDirectionString(
    const ApaParkOutDirection out_direction) {
  std::string out_dir = "INVALID";
  switch (out_direction) {
    case ApaParkOutDirection::LEFT_FRONT:
      out_dir = "LEFT_FRONT";
      break;
    case ApaParkOutDirection::RIGHT_FRONT:
      out_dir = "RIGHT_FRONT";
      break;
    case ApaParkOutDirection::LEFT_REAR:
      out_dir = "LEFT_REAR";
      break;
    case ApaParkOutDirection::RIGHT_REAR:
      out_dir = "RIGHT_REAR";
      break;
    case ApaParkOutDirection::REAR:
      out_dir = "REAR";
      break;
    case ApaParkOutDirection::FRONT:
      out_dir = "FRONT";
      break;
    default:
      out_dir = "INVALID";
      break;
  }
  return out_dir;
}

void ApaStateMachineManager::PrintApaParkOutDirection(
    const ApaParkOutDirection out_direction) {
  ILOG_INFO << "out_direction = "
            << GetApaParkOutDirectionString(out_direction);
}

std::string ApaStateMachineManager::GetApaSlotLatPosPreferenceString(
    const ApaSlotLatPosPreference slot_lat_pos_preference) {
  std::string slot_lat_pos = "MID";
  switch (slot_lat_pos_preference) {
    case ApaSlotLatPosPreference::LEFT:
      slot_lat_pos = "LEFT";
      break;
    case ApaSlotLatPosPreference::RIGHT:
      slot_lat_pos = "RIGHT";
      break;
    default:
      break;
  }
  return slot_lat_pos;
}

void ApaStateMachineManager::PrintApaSlotLatPosPreference(
    const ApaSlotLatPosPreference slot_lat_pos_preference) {
  ILOG_INFO << "slot_lat_pos_preference = "
            << GetApaSlotLatPosPreferenceString(slot_lat_pos_preference);
}
std::string ApaStateMachineManager::GetParkingSpeedModeString(
    const ParkingSpeedMode parking_speed_mode) {
  std::string speed_mode = "INVALID";
  switch (parking_speed_mode) {
    case ParkingSpeedMode::SLOW:
      speed_mode = "SLOW";
      break;
    case ParkingSpeedMode::NORMAL:
      speed_mode = "NORMAL";
      break;
    case ParkingSpeedMode::FAST:
      speed_mode = "FAST";
      break;
    case ParkingSpeedMode::INVALID:
    default:
      speed_mode = "INVALID";
      break;
  }
  return speed_mode;
};

void ApaStateMachineManager::PrintParkingSpeedMode(
    const ParkingSpeedMode parking_speed_mode) {
  ILOG_INFO << "speed_mode = " << GetParkingSpeedModeString(parking_speed_mode);
};

const bool ApaStateMachineManager::IsParkSuspendStatus() const {
  if (state_machine_ == ApaStateMachine::SUSPEND) {
    return true;
  }

  return false;
}

const bool ApaStateMachineManager::IsParkInvalidStatus() const {
  if (state_machine_ == ApaStateMachine::STANDBY ||
      state_machine_ == ApaStateMachine::ERROR) {
    return true;
  }

  return false;
}

}  // namespace apa_planner
}  // namespace planning