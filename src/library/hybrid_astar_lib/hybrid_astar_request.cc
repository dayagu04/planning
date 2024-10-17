#include "hybrid_astar_request.h"

#include "hybrid_astar_common.h"
#include "log_glog.h"
#include "node3d.h"

namespace planning {

void DebugAstarRequestString(const AstarRequest &request) {
  ILOG_INFO << "has request = " << request.first_action_request.has_request
            << ", gear "
            << PathGearDebugString(request.first_action_request.gear_request)
            << " dist " << request.first_action_request.dist_request
            << ",path method " << static_cast<int>(request.path_generate_method)
            << ",history gear=" << PathGearDebugString(request.history_gear);

  ILOG_INFO << " rs request: " << GetRSRequestType(request.rs_request)
            << ",plan reason = " << PlanReasonDebugString(request.plan_reason);

  return;
}
}  // namespace planning