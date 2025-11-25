#include "hybrid_astar_request.h"

#include "ad_common/math/math_utils.h"
#include "hybrid_astar_common.h"
#include "log_glog.h"
#include "node3d.h"

namespace planning {

void DebugAstarRequestString(const AstarRequest &request) {
  ILOG_INFO << "has request = " << request.first_action_request.has_request
            << ", gear = "
            << PathGearDebugString(request.first_action_request.gear_request)
            << ", dist = " << request.first_action_request.dist_request
            << ", path method = "
            << static_cast<int>(request.path_generate_method)
            << ", history gear = " << PathGearDebugString(request.history_gear)
            << ", slot type = " << static_cast<int>(request.space_type);

  ILOG_INFO << " rs request: " << GetRSRequestType(request.rs_request)
            << ", plan reason = " << PlanReasonDebugString(request.plan_reason)
            << ", swap goal = " << request.swap_start_goal
            << ", dir = " << static_cast<int>(request.direction_request)
            << ", gear num = " << request.gear_switch_num;

  // ILOG_INFO << "start pose";
  // request.start_pose.DebugString();
  // ILOG_INFO << "goal pose";
  // request.goal.DebugString();
  // ILOG_INFO << "goal pose";
  // request.real_goal.DebugString();

  return;
}

void ClearFirstActionReqeust(AstarRequest *request) {
  request->first_action_request.has_request = false;
  request->first_action_request.dist_request = 0.0;
  request->first_action_request.gear_request = AstarPathGear::NONE;
  return;
}

const bool IsSamplingBasedPlanning(const AstarPathGenerateType type) {
  if (type == AstarPathGenerateType::REEDS_SHEPP_SAMPLING ||
      type == AstarPathGenerateType::CUBIC_POLYNOMIAL_SAMPLING ||
      type == AstarPathGenerateType::SPIRAL_SAMPLING ||
      type == AstarPathGenerateType::QUNTIC_POLYNOMIAL_SAMPLING) {
    return true;
  }

  return false;
}

const bool IsSearchBasedPlanning(const AstarPathGenerateType type) {
  if (type == AstarPathGenerateType::ASTAR_SEARCHING ||
      type == AstarPathGenerateType::GEAR_DRIVE_SEARCHING ||
      type == AstarPathGenerateType::GEAR_REVERSE_SEARCHING) {
    return true;
  }

  return false;
}

const bool IsHeadOutRequest(const ParkingVehDirection &direction) {
  if (direction == ParkingVehDirection::HEAD_OUT_TO_LEFT ||
      direction == ParkingVehDirection::HEAD_OUT_TO_MIDDLE ||
      direction == ParkingVehDirection::HEAD_OUT_TO_RIGHT) {
    return true;
  }

  return false;
}

const bool IsTailOutRequest(const ParkingVehDirection &direction) {
  if (direction == ParkingVehDirection::TAIL_OUT_TO_LEFT ||
      direction == ParkingVehDirection::TAIL_OUT_TO_MIDDLE ||
      direction == ParkingVehDirection::TAIL_OUT_TO_RIGHT) {
    return true;
  }

  return false;
}

const bool IsParkingOutRequest(const ParkingVehDirection &direction) {
  if (direction == ParkingVehDirection::HEAD_OUT_TO_LEFT ||
      direction == ParkingVehDirection::HEAD_OUT_TO_MIDDLE ||
      direction == ParkingVehDirection::HEAD_OUT_TO_RIGHT ||
      direction == ParkingVehDirection::TAIL_OUT_TO_LEFT ||
      direction == ParkingVehDirection::TAIL_OUT_TO_MIDDLE ||
      direction == ParkingVehDirection::TAIL_OUT_TO_RIGHT ||
      direction == ParkingVehDirection::NONE) {
    return true;
  }

  return false;
}
}  // namespace planning