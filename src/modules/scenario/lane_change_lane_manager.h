#pragma once

#include "modules/context/ego_state_manager.h"
#include "modules/context/virtual_lane_manager.h"

namespace planning {


// 这个类是用来管理一个变道过程中，原始车道origin lane，目标车道target
// lane，以及当前时刻想要去的车道fix lane。fix lane一定等于origin lane或者target
// lane
class LaneChangeLaneManager {
 public:
  LaneChangeLaneManager(std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
                        planning::framework::Session* session);
  LaneChangeLaneManager(std::shared_ptr<LaneChangeLaneManager> source);
  ~LaneChangeLaneManager() {}

  void assign_lc_lanes(int lane_virtual_id);
  void reset_lc_lanes();
  void reset_origin_lane() {
    olane_virtual_id_ = virtual_lane_mgr_->current_lane_virtual_id();
  }
  void set_fix_lane_to_target() { flane_virtual_id_ = tlane_virtual_id_; }
  void set_fix_lane_to_origin() { flane_virtual_id_ = olane_virtual_id_; }
  void set_target_lane(int target_lane_virtual_id) {
    tlane_virtual_id_ = target_lane_virtual_id;
  }

  bool has_target_lane() const {
    return virtual_lane_mgr_->has_lane(tlane_virtual_id_);
  }
  bool has_origin_lane() const {
    return virtual_lane_mgr_->has_lane(olane_virtual_id_);
  }
  bool has_fix_lane() const {
    return virtual_lane_mgr_->has_lane(flane_virtual_id_);
  }

  // lateral_offset, wait for ego_state_manager ready
  bool is_ego_on(const std::shared_ptr<VirtualLane>& lane) {
    if (lane == nullptr) {
      return false;
    }
    // hack
    return true;
  //   auto ego_state = session_->mutable_environmental_model()->ego_state_manager();
  //   double lateral_offset =
  //       lane->calc_fabs_lateral_offset(ego_state->x(), ego_state->y());
  //   return (lateral_offset < 1.6) ? true : false;
  }
  std::shared_ptr<VirtualLane> flane() {
    return virtual_lane_mgr_->get_lane_with_virtual_id(flane_virtual_id_);
  }
  std::shared_ptr<VirtualLane> tlane() {
    return virtual_lane_mgr_->get_lane_with_virtual_id(tlane_virtual_id_);
  }
  std::shared_ptr<VirtualLane> olane() {
    return virtual_lane_mgr_->get_lane_with_virtual_id(olane_virtual_id_);
  }
  int flane_virtual_id() { return flane_virtual_id_; }
  int olane_virtual_id() { return olane_virtual_id_; }
  int tlane_virtual_id() { return tlane_virtual_id_; }
  void copy_lane_change_lanes(LaneChangeLaneManager& source);
  void upload_fix_lane_virtual_id() {
    virtual_lane_mgr_->update_last_fix_lane_id(flane_virtual_id_);
  }

 private:
  int flane_virtual_id_;
  int olane_virtual_id_;
  int tlane_virtual_id_;

  std::shared_ptr<VirtualLaneManager> virtual_lane_mgr_;
  planning::framework::Session* session_;
};

}  // namespace planning