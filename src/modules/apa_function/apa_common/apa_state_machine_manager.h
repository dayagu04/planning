#pragma once

#include <string>

#include "local_view.h"

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

  void SetParkOutDirection(const ApaParkOutDirection& park_out_direction) {
    out_direction_ = park_out_direction;
  }

  const bool IsParkingStatus() const;

  const bool IsSeachingStatus() const;

  const bool IsSeachingOutStatus() const;

  const bool IsParkOutStatus() const;

  const bool IsHeadOutStatus() const;

  const bool IsTailOutStatus() const;

  const bool IsParkInStatus() const;

  const bool IsParkSuspendStatus() const;

  void Reset() {
    state_machine_ = ApaStateMachine::INVALID;
    out_direction_ = ApaParkOutDirection::INVALID;
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

 private:
  ApaStateMachine state_machine_ = ApaStateMachine::INVALID;

  ApaParkOutDirection out_direction_ = ApaParkOutDirection::INVALID;

  ApaSlotLatPosPreference slot_lat_pos_preference_ =
      ApaSlotLatPosPreference::MID;
};

}  // namespace apa_planner
}  // namespace planning
