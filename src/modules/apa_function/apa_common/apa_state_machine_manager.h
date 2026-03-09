#pragma once

#include <string>

#include "local_view.h"
#include "src/modules/common/speed/apa_speed_decision.h"

namespace planning {
namespace apa_planner {

enum class ApaSlotLatPosPreference : uint8_t {
  MID,
  LEFT,
  RIGHT,
};

enum class ApaStateMachine : uint8_t {
  SEARCH_IN_NO_SELECTED,
  SEARCH_IN_SELECTED_CAR_REAR,
  SEARCH_IN_SELECTED_CAR_FRONT,
  ACTIVE_IN_CAR_REAR,
  ACTIVE_IN_CAR_FRONT,
  SEARCH_OUT_NO_SELECTED,
  SEARCH_OUT_SELECTED_CAR_REAR,
  SEARCH_OUT_SELECTED_CAR_FRONT,
  ACTIVE_OUT_CAR_REAR,
  ACTIVE_OUT_CAR_FRONT,
  SUSPEND,
  SECURE,
  COMPLETE,
  MANUAL_PARKING,
  STANDBY,
  ERROR,
  COUNT,
  INVALID,
};

enum class ApaParkOutDirection : uint8_t {
  LEFT_FRONT,
  RIGHT_FRONT,
  FRONT,
  LEFT_REAR,
  RIGHT_REAR,
  REAR,
  INVALID,
};

enum class ApaRunningMode :uint8_t {
  RUNNING_NORMAL = 0,
  RUNNING_PA = 1,       //一键贴边
  RUNNING_SAPA = 2,     //自选车位
  RUNNING_HPP = 3,      //记忆泊车
};

enum class ApaPADirection : uint8_t {
  PA_INVALID = 0,
  PA_LEFT = 1,
  PA_RIGHT = 2,
};

enum class ApaSAPAStatus : uint8_t {
  SAPA_STATUS_DEFAULT = 0,
  SAPA_STATUS_DRAGING = 1,
  SAPA_STATUS_FINISHED = 2,
};

enum class ApaTaskDirection: uint8_t {
  APA_TASK_IN = 0,
  APA_TASK_OUT = 1
};

class ApaStateMachineManager final {
 public:
  ApaStateMachineManager(){};
  ~ApaStateMachineManager(){};

  void Update(const LocalView* local_view_ptr);

  const ApaStateMachine GetStateMachine() const { return state_machine_; }

  const ApaParkOutDirection GetParkOutDirection() const {
    return out_direction_;
  }

  const ApaSlotLatPosPreference GetSlotLatPosPreference() const {
    return slot_lat_pos_preference_;
  }
  const ParkingSpeedMode GetParkingSpeedMode() const {
    return parking_speed_mode_;
  }

  //TODO(taolu10): 移到 ApaSlotManager 中
  const bool GetFreeSlotPosDir() const {
    return free_slot_pos_dir_;
  }

  //TODO(taolu10): 确认一下这个函数存在的必要性
  void SetParkOutDirection(const ApaParkOutDirection& park_out_direction) {
    out_direction_ = park_out_direction;
  }

  const ApaRunningMode GetParkRunningMode() const {
    return running_mode_;
  }

  const ApaPADirection GetPADirection() const {
    return pa_direction_;
  }

  const ApaSAPAStatus GetSAPAStatus() const {
    return sapa_status_;
  }

  const bool IsPAMode() const {
    return running_mode_ == ApaRunningMode::RUNNING_PA;
  }

  const bool IsSAPAMode() const {
    return running_mode_ == ApaRunningMode::RUNNING_SAPA;
  }

  const bool IsManualStatus() const {
    return state_machine_ == ApaStateMachine::MANUAL_PARKING;
  }
  const bool IsParkingStatus() const;
  const bool IsParkingInStatus() const;
  const bool IsParkingOutStatus() const;

  const bool IsSeachingStatus() const;

  const bool IsSearchingInStatus() const;

  const bool IsSeachingOutStatus() const;

  const bool IsParkOutStatus() const;
  const bool IsHeadOutStatus() const;
  const bool IsTailOutStatus() const;

  const bool IsParkInStatus() const;

  const bool IsParkSuspendStatus() const;

  const bool IsParkInvalidStatus() const;
  const bool IsSwitchToSearch() const;
  const bool IsSwitchToSearchOutput() const{
    return is_switch_to_search_;
  };
  const bool IsHppCruise() const {
    return running_mode_ == ApaRunningMode::RUNNING_HPP && is_hpp_cruise_;
  }

  void Reset() {
    state_machine_ = ApaStateMachine::INVALID;
    out_direction_ = ApaParkOutDirection::INVALID;
    slot_lat_pos_preference_ = ApaSlotLatPosPreference::MID;
    parking_speed_mode_ = ParkingSpeedMode::INVALID;
    free_slot_pos_dir_ = false;
    running_mode_ = ApaRunningMode::RUNNING_NORMAL;
    pa_direction_ = ApaPADirection::PA_INVALID;
    is_switch_to_search_ = false;
    is_hpp_cruise_ = false;
  }

  static std::string GetApaStateMachineString(
      const ApaStateMachine state_machine);
  static void PrintApaStateMachine(const ApaStateMachine state_machine);

  static std::string GetApaParkOutDirectionString(
      const ApaParkOutDirection out_direction);
  static void PrintApaParkOutDirection(const ApaParkOutDirection out_direction);

  static std::string GetApaSlotLatPosPreferenceString(
      const ApaSlotLatPosPreference slot_lat_pos_preference);
  static void PrintApaSlotLatPosPreference(
      const ApaSlotLatPosPreference slot_lat_pos_preference);

  static std::string GetParkingSpeedModeString(
      const ParkingSpeedMode parking_speed_mode);
  static void PrintParkingSpeedMode(const ParkingSpeedMode parking_speed_mode);

  static std::string GetParkingRunningModelString(const ApaRunningMode running_mode);
  static void PrintParkingRunningMode(const ApaRunningMode running_mode);

  static std::string GetParkingPADirectionString(const ApaPADirection pa_direction);
  static void PrintParkingPADirection(const ApaPADirection pa_direction);

  static std::string GetParkingSAPAStatusString(const ApaSAPAStatus sapa_status);
  static void PrintParkingSAPAStatus(const ApaSAPAStatus sapa_status);
 public:
  static constexpr int kSlotFreeIdx_ = 1;
 private:
  ApaStateMachine state_machine_ = ApaStateMachine::INVALID;
  ApaStateMachine state_machine_last_ = ApaStateMachine::INVALID;
  ApaParkOutDirection out_direction_ = ApaParkOutDirection::INVALID;
  ApaSlotLatPosPreference slot_lat_pos_preference_ =
      ApaSlotLatPosPreference::MID;
  ParkingSpeedMode parking_speed_mode_ = ParkingSpeedMode::INVALID;
  ApaRunningMode running_mode_ = ApaRunningMode::RUNNING_NORMAL;
  ApaPADirection pa_direction_ = ApaPADirection::PA_INVALID;
  ApaSAPAStatus sapa_status_ = ApaSAPAStatus::SAPA_STATUS_DEFAULT;

  bool free_slot_pos_dir_ = false;
  bool is_switch_to_search_ = false;

  bool is_hpp_cruise_ = false;

  /*inernal use*/
  ApaTaskDirection task_direction_ = ApaTaskDirection::APA_TASK_IN;
};

}  // namespace apa_planner
}  // namespace planning
