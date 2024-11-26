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
  if (local_view_ptr == nullptr) {
    ILOG_ERROR << "Update ApaStateMachineTManager, local_view_ptr is nullptr";
    return;
  }

  const iflyauto::FuncStateMachine& fun_state_machine_info =
      local_view_ptr->function_state_machine_info;

  const iflyauto::ParkingFusionInfo& parking_fusion_info =
      local_view_ptr->parking_fusion_info;

  switch (fun_state_machine_info.current_state) {
    case iflyauto::FunctionalState_PARK_IN_SEARCHING:
      if (parking_fusion_info.select_slot_id == 0) {
        state_machine_ = ApaStateMachineT::SEARCH_IN_NO_SELECTED;
      } else {
        if (fun_state_machine_info.parking_req.apa_parking_direction ==
            iflyauto::BACK_END_PARKING_DIRECTION) {
          state_machine_ = ApaStateMachineT::SEARCH_IN_SELECTED_CAR_REAR;
        } else if (fun_state_machine_info.parking_req.apa_parking_direction ==
                   iflyauto::FRONT_END_PARKING_DIRECTION) {
          state_machine_ = ApaStateMachineT::SEARCH_IN_SELECTED_CAR_FRONT;
        }
      }
      break;
    case iflyauto::FunctionalState_PARK_OUT_SEARCHING:
      if (fun_state_machine_info.parking_req.apa_park_out_direction ==
          iflyauto::PRK_OUT_DIRECTION_INVALID) {
        state_machine_ = ApaStateMachineT::SEARCH_OUT_NO_SELECTED;
      } else {
        switch (fun_state_machine_info.parking_req.apa_park_out_direction) {
          case iflyauto::PRK_OUT_TO_FRONT_LEFT_CROSS:
          case iflyauto::PRK_OUT_TO_FRONT_LEFT_PARALLEL:
          case iflyauto::PRK_OUT_TO_FRONT_OUT:
          case iflyauto::PRK_OUT_TO_FRONT_RIGHT_CROSS:
          case iflyauto::PRK_OUT_TO_FRONT_RIGHT_PARALLEL:
            state_machine_ = ApaStateMachineT::SEARCH_OUT_SELECTED_CAR_FRONT;
            break;
          case iflyauto::PRK_OUT_TO_BACK_OUT:
          case iflyauto::PRK_OUT_TO_BACK_LEFT_CROSS:
          case iflyauto::PRK_OUT_TO_BACK_RIGHT_CROSS:
            state_machine_ = ApaStateMachineT::SEARCH_OUT_SELECTED_CAR_REAR;
            break;
          default:
            break;
        }
      }
      break;
    case iflyauto::FunctionalState_PARK_GUIDANCE:
    case iflyauto::FunctionalState_PARK_HANDSHAKE:
      if (fun_state_machine_info.parking_req.apa_work_mode ==
          iflyauto::APA_WORK_MODE_PARKING_IN) {
        if (fun_state_machine_info.parking_req.apa_parking_direction ==
            iflyauto::BACK_END_PARKING_DIRECTION) {
          state_machine_ = ApaStateMachineT::ACTIVE_IN_CAR_REAR;
        } else if (fun_state_machine_info.parking_req.apa_parking_direction ==
                   iflyauto::FRONT_END_PARKING_DIRECTION) {
          state_machine_ = ApaStateMachineT::ACTIVE_IN_CAR_FRONT;
        }
      } else if (fun_state_machine_info.parking_req.apa_work_mode ==
                 iflyauto::APA_WORK_MODE_PARKING_OUT) {
        switch (fun_state_machine_info.parking_req.apa_park_out_direction) {
          case iflyauto::PRK_OUT_TO_FRONT_LEFT_CROSS:
          case iflyauto::PRK_OUT_TO_FRONT_LEFT_PARALLEL:
          case iflyauto::PRK_OUT_TO_FRONT_OUT:
          case iflyauto::PRK_OUT_TO_FRONT_RIGHT_CROSS:
          case iflyauto::PRK_OUT_TO_FRONT_RIGHT_PARALLEL:
            state_machine_ = ApaStateMachineT::ACTIVE_OUT_CAR_FRONT;
            break;
          case iflyauto::PRK_OUT_TO_BACK_OUT:
          case iflyauto::PRK_OUT_TO_BACK_LEFT_CROSS:
          case iflyauto::PRK_OUT_TO_BACK_RIGHT_CROSS:
            state_machine_ = ApaStateMachineT::ACTIVE_OUT_CAR_REAR;
            break;
          default:
            break;
        }
      }
      break;
    case iflyauto::FunctionalState_PARK_SUSPEND:
      state_machine_ = ApaStateMachineT::SUSPEND;
      break;
    case iflyauto::FunctionalState_PARK_COMPLETED:
      state_machine_ = ApaStateMachineT::COMPLETE;
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

  PrintApaStateMachineT(state_machine_);
  PrintApaParkOutDirection(out_direction_);

  JSON_DEBUG_VALUE("apa_state_machine", static_cast<int>(state_machine_))
  JSON_DEBUG_VALUE("apa_out_direction", static_cast<int>(out_direction_))

  return;
}
const bool ApaStateMachineManager::IsSeachingStatus() const {
  if (state_machine_ == ApaStateMachineT::SEARCH_IN_NO_SELECTED ||
      state_machine_ == ApaStateMachineT::SEARCH_IN_SELECTED_CAR_REAR ||
      state_machine_ == ApaStateMachineT::SEARCH_IN_SELECTED_CAR_FRONT ||
      state_machine_ == ApaStateMachineT::SEARCH_OUT_NO_SELECTED ||
      state_machine_ == ApaStateMachineT::SEARCH_OUT_SELECTED_CAR_REAR ||
      state_machine_ == ApaStateMachineT::SEARCH_OUT_SELECTED_CAR_FRONT) {
    return true;
  }

  return false;
}

const bool ApaStateMachineManager::IsParkingStatus() const {
  if (state_machine_ == ApaStateMachineT::ACTIVE_IN_CAR_FRONT ||
      state_machine_ == ApaStateMachineT::ACTIVE_IN_CAR_REAR ||
      state_machine_ == ApaStateMachineT::ACTIVE_OUT_CAR_FRONT ||
      state_machine_ == ApaStateMachineT::ACTIVE_OUT_CAR_FRONT) {
    return true;
  }

  return false;
}

void ApaStateMachineManager::PrintApaStateMachineT(
    const ApaStateMachineT state_machine) {
  switch (state_machine) {
    case ApaStateMachineT::SEARCH_IN_NO_SELECTED:
      ILOG_INFO << "apa_state = SEARCH_IN_NO_SELECTED";
      break;
    case ApaStateMachineT::SEARCH_IN_SELECTED_CAR_REAR:
      ILOG_INFO << "apa_state = SEARCH_IN_SELECTED_CAR_REAR";
      break;
    case ApaStateMachineT::SEARCH_IN_SELECTED_CAR_FRONT:
      ILOG_INFO << "apa_state = SEARCH_IN_SELECTED_CAR_FRONT";
      break;
    case ApaStateMachineT::SEARCH_OUT_NO_SELECTED:
      ILOG_INFO << "apa_state = SEARCH_OUT_NO_SELECTED";
      break;
    case ApaStateMachineT::SEARCH_OUT_SELECTED_CAR_REAR:
      ILOG_INFO << "apa_state = SEARCH_OUT_SELECTED_CAR_REAR";
      break;
    case ApaStateMachineT::SEARCH_OUT_SELECTED_CAR_FRONT:
      ILOG_INFO << "apa_state = SEARCH_OUT_SELECTED_CAR_FRONT";
      break;
    case ApaStateMachineT::ACTIVE_IN_CAR_FRONT:
      ILOG_INFO << "apa_state = ACTIVE_IN_CAR_FRONT";
      break;
    case ApaStateMachineT::ACTIVE_IN_CAR_REAR:
      ILOG_INFO << "apa_state = ACTIVE_IN_CAR_REAR";
      break;
    case ApaStateMachineT::ACTIVE_OUT_CAR_FRONT:
      ILOG_INFO << "apa_state = ACTIVE_OUT_CAR_FRONT";
      break;
    case ApaStateMachineT::ACTIVE_OUT_CAR_REAR:
      ILOG_INFO << "apa_state = ACTIVE_OUT_CAR_REAR";
      break;
    case ApaStateMachineT::SECURE:
      ILOG_INFO << "apa_state = SECURE";
      break;
    case ApaStateMachineT::SUSPEND:
      ILOG_INFO << "apa_state = SUSPEND";
      break;
    case ApaStateMachineT::COMPLETE:
      ILOG_INFO << "apa_state = COMPLETE";
      break;
    default:
      ILOG_INFO << "apa_state = INVALID";
      break;
  }
}

void ApaStateMachineManager::PrintApaParkOutDirection(
    const ApaParkOutDirection out_direction) {
  switch (out_direction) {
    case ApaParkOutDirection::LEFT_FRONT:
      ILOG_INFO << "out_direction = LEFT_FRONT";
      break;
    case ApaParkOutDirection::RIGHT_FRONT:
      ILOG_INFO << "out_direction = RIGHT_FRONT";
      break;
    case ApaParkOutDirection::LEFT_REAR:
      ILOG_INFO << "out_direction = LEFT_REAR";
      break;
    case ApaParkOutDirection::RIGHT_REAR:
      ILOG_INFO << "out_direction = RIGHT_REAR";
      break;
    case ApaParkOutDirection::REAR:
      ILOG_INFO << "out_direction = REAR";
      break;
    case ApaParkOutDirection::FRONT:
      ILOG_INFO << "out_direction = FRONT";
      break;
    default:
      ILOG_INFO << "out_direction = INVALID";
      break;
  }
}

}  // namespace apa_planner
}  // namespace planning