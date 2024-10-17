
#include "hybrid_astar_common.h"

namespace planning {

std::string PathGearDebugString(const AstarPathGear gear) {
  switch (gear) {
    case AstarPathGear::drive:
      return "drive";
    case AstarPathGear::reverse:
      return "reverse";
    case AstarPathGear::normal:
      return "normal";
    default:
      return "parking";
      break;
  }

  return "none";
}

std::string GetPathSteerDebugString(const AstarPathSteer type) {
  switch (type) {
    case AstarPathSteer::left:
      return "left";
    case AstarPathSteer::right:
      return "right";
    case AstarPathSteer::straight:
      return "straight";
    default:
      break;
  }

  return "none";
}

bool IsGearDifferent(const AstarPathGear left, const AstarPathGear right) {
  if (left == AstarPathGear::drive && right == AstarPathGear::reverse) {
    return true;
  }

  if (left == AstarPathGear::reverse && right == AstarPathGear::drive) {
    return true;
  }

  return false;
}

std::string PlanReasonDebugString(const PlanningReason reason) {
  switch (reason) {
    case PlanningReason::FIRST_PLAN:
      return "FIRST_PLAN";
    case PlanningReason::ADJUST_SELF_CAR_POSE:
      return "ADJUST_SELF_CAR_POSE";
    case PlanningReason::GEOMETRY_CURVE_FAIL:
      return "GEOMETRY_CURVE_FAIL";
    case PlanningReason::PATH_COMPLETED:
      return "PATH_COMPLETED";
    case PlanningReason::PATH_STUCKED:
      return "PATH_STUCKED";
    case PlanningReason::SLOT_CHANGED:
      return "SLOT_CHANGED";
    case PlanningReason::SIMULATION_TRIGGER:
      return "SIMULATION_TRIGGER";
    default:
      return "none";
      break;
  }

  return "none";
}

}  // namespace planning