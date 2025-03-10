#include "hybrid_astar_request.h"

#include "hybrid_astar_common.h"
#include "log_glog.h"
#include "node3d.h"

namespace planning {

void DebugAstarRequestString(const AstarRequest &request) {
  ILOG_INFO << "has request = " << request.first_action_request.has_request
            << ", gear = "
            << PathGearDebugString(request.first_action_request.gear_request)
            << ", dist = " << request.first_action_request.dist_request
            << ", path method = " << static_cast<int>(request.path_generate_method)
            << ", history gear = " << PathGearDebugString(request.history_gear)
            << ", slot type = " << static_cast<int>(request.space_type);

  ILOG_INFO << " rs request: " << GetRSRequestType(request.rs_request)
            << ", plan reason = " << PlanReasonDebugString(request.plan_reason)
            << ", swap goal = " << request.swap_start_goal << ", dir = "
            << static_cast<int>(request.direction_request);

  // ILOG_INFO << "start pose";
  // request.start_.DebugString();

  // ILOG_INFO << "goal pose";
  // request.goal_.DebugString();

  return;
}

void ClearFirstActionReqeust(AstarRequest *request) {
  request->first_action_request.has_request = false;
  request->first_action_request.dist_request = 0.0;
  request->first_action_request.gear_request = AstarPathGear::NONE;
  return;
}

const bool IsEgoPoseAdjustPlanning(const AstarPathGenerateType type) {
  if (type == AstarPathGenerateType::REEDS_SHEPP_SAMPLING ||
      type == AstarPathGenerateType::CUBIC_POLYNOMIAL_SAMPLING ||
      type == AstarPathGenerateType::SPIRAL_SAMPLING ||
      type == AstarPathGenerateType::QUNTIC_POLYNOMIAL_SAMPLING) {
    return true;
  }

  return false;
}

}  // namespace planning