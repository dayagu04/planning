#include "src/modules/scenario/lane_change_requests/active_lane_change_request.h"

namespace planning {
// class: ActRequest
ActRequest::ActRequest(
    planning::framework::Session* session,
    std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
    std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr)
    : LaneChangeRequest(session, virtual_lane_mgr, lane_change_lane_mgr) {}

void ActRequest::update(int lc_status) {}

}  // namespace planning