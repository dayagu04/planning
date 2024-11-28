#pragma once

#include <string>

#include "local_view.h"

namespace planning {
namespace apa_planner {

enum class ApaStateMachineT : uint8_t {
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

  const ApaStateMachineT GetStateMachine() const { return state_machine_; }

  const ApaParkOutDirection GetParkOutDirection() const {
    return out_direction_;
  }

  const bool IsParkingStatus() const;

  const bool IsSeachingStatus() const;

  void Reset() {
    state_machine_ = ApaStateMachineT::INVALID;
    out_direction_ = ApaParkOutDirection::INVALID;
  }

  static std::string GetApaStateMachineTString(
      const ApaStateMachineT state_machine);

  static void PrintApaStateMachineT(const ApaStateMachineT state_machine);

  static std::string GetApaParkOutDirectionString(
      const ApaParkOutDirection out_direction);

  static void PrintApaParkOutDirection(const ApaParkOutDirection out_direction);

 private:
  ApaStateMachineT state_machine_ = ApaStateMachineT::INVALID;

  ApaParkOutDirection out_direction_ = ApaParkOutDirection::INVALID;
};

}  // namespace apa_planner
}  // namespace planning
