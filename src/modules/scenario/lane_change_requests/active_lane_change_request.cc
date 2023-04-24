#include "src/modules/scenario/lane_change_requests/active_lane_change_request.h"

namespace planning {
// class: ActRequest
ActRequest::ActRequest(
    planning::framework::Session *session,
    std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
    std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr)
    : LaneChangeRequest(session, virtual_lane_mgr, lane_change_lane_mgr) {}

void ActRequest::Update(int lc_status, double lc_int_tfinish,
                        double lc_map_tfinish,
                        std::shared_ptr<ObjectSelector> &object_selector) {
  // 不允许右换道？
  double default_int_delay = 2;
  double default_ma_delay = 1.5;
  double diff_int = IflyTime::Now_s() - lc_int_tfinish;
  double diff_map = IflyTime::Now_s() - lc_map_tfinish;

  auto flane = lane_change_lane_mgr_->flane();
  auto olane = lane_change_lane_mgr_->olane();
  auto tlane = lane_change_lane_mgr_->tlane();
}

}  // namespace planning