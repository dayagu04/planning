#include "modules/scenario/lane_change_lane_manager.h"

namespace planning {


LaneChangeLaneManager::LaneChangeLaneManager(
    std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
    planning::framework::Session* session) {
  session_ = session;
  virtual_lane_mgr_ = virtual_lane_mgr;
  if (virtual_lane_mgr == nullptr) {
    LOG_ERROR("[LaneChangeLaneManager::constructor] empty pointer");
  } else {
    flane_virtual_id_ = virtual_lane_mgr_->current_lane_virtual_id();
    olane_virtual_id_ = virtual_lane_mgr_->current_lane_virtual_id();
    tlane_virtual_id_ = virtual_lane_mgr_->current_lane_virtual_id();
  }
}

LaneChangeLaneManager::LaneChangeLaneManager(
    std::shared_ptr<LaneChangeLaneManager> source) {
  virtual_lane_mgr_ = source->virtual_lane_mgr_;
  session_ = source->session_;
  copy_lane_change_lanes(*source);
}

void LaneChangeLaneManager::assign_lc_lanes(int lane_virtual_id) {
  tlane_virtual_id_ = lane_virtual_id;
  olane_virtual_id_ = virtual_lane_mgr_->current_lane_virtual_id();
  flane_virtual_id_ = virtual_lane_mgr_->current_lane_virtual_id();
}

void LaneChangeLaneManager::reset_lc_lanes() {
  tlane_virtual_id_ = virtual_lane_mgr_->current_lane_virtual_id();
  olane_virtual_id_ = virtual_lane_mgr_->current_lane_virtual_id();
  flane_virtual_id_ = virtual_lane_mgr_->current_lane_virtual_id();
}

void LaneChangeLaneManager::copy_lane_change_lanes(
    LaneChangeLaneManager& source) {
  tlane_virtual_id_ = source.tlane_virtual_id_;
  olane_virtual_id_ = source.olane_virtual_id_;
  flane_virtual_id_ = source.flane_virtual_id_;
}


}  // namespace planning