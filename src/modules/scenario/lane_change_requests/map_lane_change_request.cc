#include "scenario/lane_change_requests/map_lane_change_request.h"

namespace planning {

// class: MapRequest
MapRequest::MapRequest(
    framework::Session* session, const EgoPlanningConfigBuilder* config_builder,
    std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
    std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr)
    : LaneChangeRequest(session, virtual_lane_mgr, lane_change_lane_mgr) {}

void MapRequest::update(int lc_status, int left_int_freeze_cnt,
                        int right_int_freeze_cnt) {
  ;
}

}  // namespace planning