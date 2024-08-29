#pragma once

#include "config/basic_type.h"
#include "ego_planning_config.h"
#include "ifly_time.h"
#include "lane_change_requests/active_lane_change_request.h"
#include "lane_change_requests/cone_lane_change_request.h"
#include "lane_change_requests/emergence_avoid_lane_change_request.h"
#include "lane_change_requests/interactive_lane_change_request.h"
#include "lane_change_requests/lane_change_request.h"
#include "lane_change_requests/map_lane_change_request.h"
#include "lane_change_requests/merge_lane_change_request.h"
#include "lane_change_requests/overtake_lane_change_request.h"
#include "session.h"

namespace planning {

class VirtualLane;
class VirtualLaneManager;
class LaneChangeLaneManager;

/// @brief 管理所有的换道请求
class LaneChangeRequestManager {
 public:
  LaneChangeRequestManager(
      framework::Session* session,
      const EgoPlanningConfigBuilder* config_builder,
      std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
      std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr);
  virtual ~LaneChangeRequestManager() = default;

  void FinishRequest();

  bool Update(std::shared_ptr<ObjectSelector> object_selector, int lc_status,
              const bool hd_map_valid);

  /// @brief 获取换道请求开始和完成的时间
  double GetReqStartTime(int source) const;
  double GetReqFinishTime(int source) const;

  RequestType request() const { return request_; }
  RequestSource request_source() const { return request_source_; }
  IntCancelReasonType int_request_cancel_reason() const {
    return int_request_cancel_reason_;
  }
  std::string act_request_source() {
    return request_source_ == ACT_REQUEST ? act_request_.act_request_source()
                                          : "none";
  }
  int target_lane_virtual_id() { return target_lane_virtual_id_; }
  void set_target_lane_virtual_id(int target_lane_virtual_id) {
    target_lane_virtual_id_ = target_lane_virtual_id;
    map_request_.set_target_lane_virtual_id(target_lane_virtual_id);
    int_request_.set_target_lane_virtual_id(target_lane_virtual_id);
    act_request_.set_target_lane_virtual_id(target_lane_virtual_id);
  }
  RequestType turn_signal() const { return gen_turn_signal_; }
  bool AggressiveChange() const {
    if (request_source_ == NO_REQUEST) {
      return false;
    } else if (request_source_ == INT_REQUEST) {
      return true;
    } else if (request_source_ == MAP_REQUEST) {
      return map_request_.AggressiveChange();
    } else if (request_source_ == ACT_REQUEST) {
      return act_request_.AggressiveChange();
    } else if (request_source_ == OVERTAKE_REQUEST) {
      return overtake_request_.AggressiveChange();
    } else if (request_source_ == EMERGENCE_AVOID_REQUEST) {
      return emergence_avoid_request_.AggressiveChange();
    } else if (request_source_ == CONE_REQUEST) {
      return cone_change_request_.AggressiveChange();
    } else if (request_source_ == MERGE_REQUEST) {
      return merge_change_request_.AggressiveChange();
    }
    return false;
  }

  const RequestType get_ilc_virtual_request() const {
    return ilc_virtual_request_;
  }

  void GenerateHMIInfo();
  void GenerateHMIInfoForOvertake();

 private:
  EgoPlanningConfig config_;
  RequestType request_ = NO_CHANGE;
  RequestSource request_source_ = NO_REQUEST;
  RequestType gen_turn_signal_ = NO_CHANGE;
  IntCancelReasonType int_request_cancel_reason_ = NO_CANCEL;
  int target_lane_virtual_id_ = -1;
  IntRequest int_request_;
  MapRequest map_request_;
  ActRequest act_request_;
  OvertakeRequest overtake_request_;
  EmergenceAvoidRequest emergence_avoid_request_;
  ConeRequest cone_change_request_;
  MergeRequest merge_change_request_;

  std::shared_ptr<VirtualLaneManager> virtual_lane_mgr_;
  framework::Session* session_;
  RequestType ilc_virtual_request_ = NO_CHANGE;
};

}  // namespace planning
