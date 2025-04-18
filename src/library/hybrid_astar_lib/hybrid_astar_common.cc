
#include "hybrid_astar_common.h"
#include <cmath>
#include "point_cloud_obstacle.h"

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

std::string GetNodeCurveDebugString(const AstarPathType type) {
  switch (type) {
    case AstarPathType::REEDS_SHEPP:
      return "REEDS_SHEPP";
    case AstarPathType::DUBINS:
      return "DUBINS";
    case AstarPathType::NODE_SEARCHING:
      return "NODE_SEARCHING";
    case AstarPathType::LINE_SEGMENT:
      return "LINE_SEGMENT";
    case AstarPathType::CUBIC_POLYNOMIAL:
      return "CUBIC_POLYNOMIAL";
    case AstarPathType::QUNTIC_POLYNOMIAL:
      return "QUNTIC_POLYNOMIAL";
    case AstarPathType::SPIRAL:
      return "SPIRAL";
    default:
      break;
  }

  return "NONE";
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
    case PlanningReason::SLOT_REFRESHED:
      return "SLOT_REFRESHED";
    case PlanningReason::SIMULATION_TRIGGER:
      return "SIMULATION_TRIGGER";
    default:
      return "none";
      break;
  }

  return "none";
}

void DebugPolynomialPath(const std::vector<AStarPathPoint>& poly_path) {
  for (size_t i = 0; i < poly_path.size(); i++) {
    ILOG_INFO << "x = " << poly_path[i].x << ",y=" << poly_path[i].y
              << ",theta=" << poly_path[i].phi
              << ",kappa=" << poly_path[i].kappa
              << ",s = " << poly_path[i].accumulated_s
              << ", gear = " << static_cast<int>(poly_path[i].gear);
  }

  return;
}

void DebugPathString(const HybridAStarResult* result) {
  ILOG_INFO << "path x point size " << result->x.size() << " gear size "
            << result->gear.size() << "y size " << result->y.size()
            << "phi size " << result->phi.size() << "type size "
            << result->type.size() << "s size " << result->accumulated_s.size();

  for (size_t i = 0; i < result->x.size(); i++) {
    ILOG_INFO << "i = " << i << " x, y, theta, gear:  " << result->x[i] << ", "
              << result->y[i] << ", " << result->phi[i] * 57.4 << ", "
              << PathGearDebugString(result->gear[i])
              << ",path type = " << GetNodeCurveDebugString(result->type[i])
              << ", s = " << result->accumulated_s[i];
  }

  return;
}

}  // namespace planning