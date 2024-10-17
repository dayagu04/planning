
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

}  // namespace planning