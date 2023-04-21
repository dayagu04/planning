
#include "src/modules/scenario/lane_change_requests/interactive_lane_change_request.h"

namespace planning {
// class: IntRequest
IntRequest::IntRequest(
    planning::framework::Session* session,
    std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
    std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr)
    : LaneChangeRequest(session, virtual_lane_mgr, lane_change_lane_mgr) {
  auto config_builder = session_->mutable_environmental_model()->config_builder(
      planning::common::SceneType::HIGHWAY);
  auto int_request_config = config_builder->cast<ScenarioDisplayStateConfig>();
  enable_int_request_ = int_request_config.enable_int_request_function;
  count_trsh_ = int_request_config.int_rqt_cnt_trsh;
}

void IntRequest::Update(int lc_status) {}
}  // namespace planning