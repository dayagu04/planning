
#include "hybrid_astar_common.h"
#include <cmath>

namespace planning {

std::string PathGearDebugString(const AstarPathGear gear) {
  switch (gear) {
    case AstarPathGear::DRIVE:
      return "drive";
    case AstarPathGear::REVERSE:
      return "reverse";
    case AstarPathGear::NORMAL:
      return "normal";
    default:
      return "parking";
      break;
  }

  return "none";
}

std::string GetPathSteerDebugString(const AstarPathSteer type) {
  switch (type) {
    case AstarPathSteer::LEFT:
      return "left";
    case AstarPathSteer::RIGHT:
      return "right";
    case AstarPathSteer::STRAIGHT:
      return "straight";
    default:
      break;
  }

  return "none";
}

bool IsGearDifferent(const AstarPathGear left, const AstarPathGear right) {
  if (left == AstarPathGear::DRIVE && right == AstarPathGear::REVERSE) {
    return true;
  }

  if (left == AstarPathGear::REVERSE && right == AstarPathGear::DRIVE) {
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

const bool PolynomialPathBetter(const PolynomialPathCost& path,
                                const PolynomialPathCost& base) {
  if (std::fabs(path.tail_heading) < std::fabs(base.tail_heading)) {
    if (std::fabs(path.offset_to_center) <
        std::fabs(base.offset_to_center) + 0.01) {
      return true;
    }
  } else if (std::fabs(path.tail_heading) == std::fabs(base.tail_heading)) {
    if (std::fabs(path.offset_to_center) < std::fabs(base.offset_to_center)) {
      return true;
    } else if (std::fabs(path.offset_to_center) ==
               std::fabs(base.offset_to_center)) {
      if (path.accumulated_s < base.accumulated_s) {
        return true;
      }
    }
  }

  return false;
}

}  // namespace planning