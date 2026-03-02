#include "apa_state_machine_manager.h"

#include <cmath>

#include "debug_info_log.h"
#include "func_state_machine_c.h"
#include "fusion_parking_slot_c.h"
#include "hmi_inner_c.h"
#include "log_glog.h"

namespace planning {
namespace apa_planner {

const int kSlotFreeCorner1 = 0;
const int kSlotFreeCorner2 = 3;

void ApaStateMachineManager::Update(const LocalView* local_view_ptr) {
  state_machine_last_ = state_machine_;
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

  const iflyauto::ApaFreeSlotInfo& free_slot_info =
      fun_state_machine_info.parking_req.apa_free_slot_info;

  const iflyauto::ApaParkingDirection parking_in_direction =
      fun_state_machine_info.parking_req.apa_parking_direction;

  const iflyauto::ApaParkOutDirection park_out_direction =
      fun_state_machine_info.parking_req.apa_park_out_direction;

  free_slot_pos_dir_ = free_slot_info.corner_points[kSlotFreeCorner1].x >
                       free_slot_info.corner_points[kSlotFreeCorner2].x;
  bool is_turn_corner = false;
  if (fun_state_machine_info.running_mode ==
      iflyauto::RunningMode::RUNNING_MODE_PA) {
    running_mode_ = ApaRunningMode::RUNNING_PA;
    if (fun_state_machine_info.parking_req.pa_direction ==
        iflyauto::PA_DIRECTION_RIGHT) {
      pa_direction_ = ApaPADirection::PA_RIGHT;
    } else if (fun_state_machine_info.parking_req.pa_direction ==
               iflyauto::PA_DIRECTION_LEFT) {
      pa_direction_ = ApaPADirection::PA_LEFT;
    } else {
      pa_direction_ = ApaPADirection::PA_INVALID;
    }
  } else if (fun_state_machine_info.running_mode ==
             iflyauto::RunningMode::RUNNING_MODE_MEMORY_PARKING) {
    running_mode_ = ApaRunningMode::RUNNING_HPP;
  } else if (fun_state_machine_info.parking_req.apa_free_slot_info
                 .free_slot_activate) {
    running_mode_ = ApaRunningMode::RUNNING_SAPA;
    for (const auto& slot : parking_fusion_info.parking_fusion_slot_lists) {
      if (slot.id == kSlotFreeIdx_) {
        is_turn_corner = slot.is_turn_corner;
      }
    }
    const auto is_free_slot_selected = free_slot_info.is_free_slot_selected;
    if (is_free_slot_selected ==
        iflyauto::FreeSlotSelectedStatus::FREE_SLOT_SELECTED_STATUS_DRAGING) {
      sapa_status_ = ApaSAPAStatus::SAPA_STATUS_DRAGING;
    } else if (is_free_slot_selected ==
               iflyauto::FreeSlotSelectedStatus::
                   FREE_SLOT_SELECTED_STATUS_FINISHED) {
      sapa_status_ = ApaSAPAStatus::SAPA_STATUS_FINISHED;
    } else {
      sapa_status_ = ApaSAPAStatus::SAPA_STATUS_DEFAULT;
    }
  } else {
    running_mode_ = ApaRunningMode::RUNNING_NORMAL;
  }

  switch (fun_state_machine_info.current_state) {
    case iflyauto::FunctionalState_MANUAL_PARKING:
      state_machine_ = ApaStateMachine::MANUAL_PARKING;
      break;
    case iflyauto::FunctionalState_PARK_STANDBY:
      state_machine_ = ApaStateMachine::STANDBY;
      break;
    case iflyauto::FunctionalState_PARK_ERROR:
      state_machine_ = ApaStateMachine::ERROR;
      break;
    case iflyauto::FunctionalState_PARK_IN_SEARCHING:
    case iflyauto::FunctionalState_HPP_CRUISE_ROUTING:
    case iflyauto::FunctionalState_HPP_CRUISE_SEARCHING:
      task_direction_ = ApaTaskDirection::APA_TASK_IN;
      if (running_mode_ == ApaRunningMode::RUNNING_PA) {
        state_machine_ = ApaStateMachine::SEARCH_IN_SELECTED_CAR_REAR;
      } else if (running_mode_ == ApaRunningMode::RUNNING_SAPA) {
        if (sapa_status_ == ApaSAPAStatus::SAPA_STATUS_FINISHED) {
          if (!is_turn_corner) {
            state_machine_ = ApaStateMachine::SEARCH_IN_SELECTED_CAR_REAR;
          } else {
            state_machine_ = ApaStateMachine::SEARCH_IN_SELECTED_CAR_FRONT;
          }
        } else {
          state_machine_ = ApaStateMachine::SEARCH_IN_NO_SELECTED;
        }
      } else {
        if (parking_fusion_info.select_slot_id == 0) {
          state_machine_ = ApaStateMachine::SEARCH_IN_NO_SELECTED;
        } else {
          if (parking_in_direction == iflyauto::BACK_END_PARKING_DIRECTION) {
            state_machine_ = ApaStateMachine::SEARCH_IN_SELECTED_CAR_REAR;
          } else if (parking_in_direction ==
                     iflyauto::FRONT_END_PARKING_DIRECTION) {
            state_machine_ = ApaStateMachine::SEARCH_IN_SELECTED_CAR_FRONT;
          }
        }
      }
      break;
    case iflyauto::FunctionalState_PARK_OUT_SEARCHING:
      task_direction_ = ApaTaskDirection::APA_TASK_OUT;
      switch (park_out_direction) {
        case iflyauto::PRK_OUT_DIRECTION_INVALID:
          state_machine_ = ApaStateMachine::SEARCH_OUT_NO_SELECTED;
          break;
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
      break;
    case iflyauto::FunctionalState_PARK_GUIDANCE:
    case iflyauto::FunctionalState_PARK_PRE_ACTIVE:
    case iflyauto::FunctionalState_HPP_PARKING_IN:
    case iflyauto::FunctionalState_HPP_PRE_ACTIVE_PARKING:
      if (running_mode_ == ApaRunningMode::RUNNING_PA) {
        state_machine_ = ApaStateMachine::ACTIVE_IN_CAR_REAR;
      } else {
        if (task_direction_ == ApaTaskDirection::APA_TASK_IN) {
          if (running_mode_ == ApaRunningMode::RUNNING_SAPA) {
            if (!is_turn_corner) {
              state_machine_ = ApaStateMachine::ACTIVE_IN_CAR_REAR;
            } else {
              state_machine_ = ApaStateMachine::ACTIVE_IN_CAR_FRONT;
            }
          } else {
            if (parking_in_direction == iflyauto::BACK_END_PARKING_DIRECTION) {
              state_machine_ = ApaStateMachine::ACTIVE_IN_CAR_REAR;
            } else if (parking_in_direction ==
                       iflyauto::FRONT_END_PARKING_DIRECTION) {
              state_machine_ = ApaStateMachine::ACTIVE_IN_CAR_FRONT;
            }
          }
        } else {
          switch (park_out_direction) {
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
      parking_speed_mode_ = ParkingSpeedMode::NORMAL;
      break;
  }
  is_switch_to_search_ = IsSwitchToSearch();

  PrintApaStateMachine(state_machine_);
  PrintApaParkOutDirection(out_direction_);
  PrintApaSlotLatPosPreference(slot_lat_pos_preference_);
  PrintParkingSpeedMode(parking_speed_mode_);
  PrintParkingRunningMode(running_mode_);
  PrintParkingPADirection(pa_direction_);
  PrintParkingSAPAStatus(sapa_status_);

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

const bool ApaStateMachineManager::IsSearchingInStatus() const {
  if (state_machine_ == ApaStateMachine::SEARCH_IN_NO_SELECTED ||
      state_machine_ == ApaStateMachine::SEARCH_IN_SELECTED_CAR_REAR ||
      state_machine_ == ApaStateMachine::SEARCH_IN_SELECTED_CAR_FRONT) {
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

const bool ApaStateMachineManager::IsParkingInStatus() const {
  if (state_machine_ == ApaStateMachine::ACTIVE_IN_CAR_FRONT ||
      state_machine_ == ApaStateMachine::ACTIVE_IN_CAR_REAR) {
    return true;
  }

  return false;
}

const bool ApaStateMachineManager::IsParkingOutStatus() const {
  if (state_machine_ == ApaStateMachine::ACTIVE_OUT_CAR_FRONT ||
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
    case ApaStateMachine::MANUAL_PARKING:
      state = "MANUAL_PARKING";
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

std::string ApaStateMachineManager::GetParkingRunningModelString(
    const ApaRunningMode running_mode) {
  std::string mode = "RUNNING_NORMAL";
  switch (running_mode) {
    case ApaRunningMode::RUNNING_PA:
      mode = "RUNNING_PA";
      break;
    case ApaRunningMode::RUNNING_SAPA:
      mode = "SELF_DEFINE";
      break;
    default:
      mode = "RUNNING_NORMAL";
      break;
  }
  return mode;
};
void ApaStateMachineManager::PrintParkingRunningMode(
    const ApaRunningMode running_mode) {
  ILOG_INFO << "running_mode = " << GetParkingRunningModelString(running_mode);
};

std::string ApaStateMachineManager::GetParkingPADirectionString(
    const ApaPADirection pa_direction) {
  std::string direction = "PA_INVALID";
  switch (pa_direction) {
    case ApaPADirection::PA_LEFT:
      direction = "PA_LEFT";
      break;
    case ApaPADirection::PA_RIGHT:
      direction = "PA_RIGHT";
      break;
    default:
      direction = "PA_INVALID";
      break;
  }
  return direction;
};
void ApaStateMachineManager::PrintParkingPADirection(
    const ApaPADirection pa_direction) {
  ILOG_INFO << "PA direction = " << GetParkingPADirectionString(pa_direction);
};

std::string ApaStateMachineManager::GetParkingSAPAStatusString(
    const ApaSAPAStatus sapa_status) {
  std::string res_string = "SAPA_STATUS_DEFAULT";
  switch (sapa_status) {
    case ApaSAPAStatus::SAPA_STATUS_DEFAULT:
      res_string = "SAPA_STATUS_DEFAULT";
      break;
    case ApaSAPAStatus::SAPA_STATUS_DRAGING:
      res_string = "SAPA_STATUS_DRAGING";
      break;
    case ApaSAPAStatus::SAPA_STATUS_FINISHED:
      res_string = "SAPA_STATUS_FINISHED";
      break;
    default:
      res_string = "SAPA_STATUS_DEFAULT";
  }
  return res_string;
}
void ApaStateMachineManager::PrintParkingSAPAStatus(
    const ApaSAPAStatus sapa_status) {
  ILOG_INFO << "SAPA status = " << GetParkingSAPAStatusString(sapa_status);
}

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

const bool ApaStateMachineManager::IsSwitchToSearch() const {
  if (state_machine_ == ApaStateMachine::SEARCH_IN_NO_SELECTED ||
      state_machine_ == ApaStateMachine::SEARCH_IN_SELECTED_CAR_REAR ||
      state_machine_ == ApaStateMachine::SEARCH_IN_SELECTED_CAR_FRONT ||
      state_machine_ == ApaStateMachine::SEARCH_OUT_NO_SELECTED ||
      state_machine_ == ApaStateMachine::SEARCH_OUT_SELECTED_CAR_REAR ||
      state_machine_ == ApaStateMachine::SEARCH_OUT_SELECTED_CAR_FRONT) {
    if (state_machine_last_ != ApaStateMachine::SEARCH_IN_NO_SELECTED &&
        state_machine_last_ != ApaStateMachine::SEARCH_IN_SELECTED_CAR_REAR &&
        state_machine_last_ != ApaStateMachine::SEARCH_IN_SELECTED_CAR_FRONT &&
        state_machine_last_ != ApaStateMachine::SEARCH_OUT_NO_SELECTED &&
        state_machine_last_ != ApaStateMachine::SEARCH_OUT_SELECTED_CAR_REAR &&
        state_machine_last_ != ApaStateMachine::SEARCH_OUT_SELECTED_CAR_FRONT) {
      return true;
    }
  }

  return false;
}

}  // namespace apa_planner
}  // namespace planning
