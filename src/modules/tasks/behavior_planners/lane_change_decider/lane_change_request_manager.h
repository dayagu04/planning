#pragma once

#include "config/basic_type.h"
#include "ego_planning_config.h"
#include "ifly_time.h"
#include "lane_change_requests/active_lane_change_request.h"
#include "lane_change_requests/interactive_lane_change_request.h"
#include "lane_change_requests/lane_change_request.h"
#include "lane_change_requests/map_lane_change_request.h"
#include "lane_change_requests/overtake_lane_change_request.h"
#include "session.h"
#include "tasks/behavior_planners/lane_change_decider/lane_change_requests/overtake_lane_change_request.h"

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
    }
    return false;
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

  std::shared_ptr<VirtualLaneManager> virtual_lane_mgr_;
  framework::Session* session_;
};

}  // namespace planning
