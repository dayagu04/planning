#pragma once

#include "config/basic_type.h"
#include "ego_planning_config.h"
#include "lane_change_request.h"
#include "traffic_congestion_decider.h"
namespace planning {

/// @brief 换道请求子类：地图换道请求
class MapRequest : public LaneChangeRequest {
 public:
  MapRequest(framework::Session* session,
             const EgoPlanningConfigBuilder* config_builder,
             std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
             std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr);
  virtual ~MapRequest() = default;
  void Update(const int lc_status, const double lc_map_tfinish);
  virtual void SetLaneChangeCmd(std::uint8_t lane_change_cmd) {
    lane_change_cmd_ = lane_change_cmd;
  }
  virtual void SetLaneChangeCancelFromTrigger(bool trigger_lane_change_cancel) {
    trigger_lane_change_cancel_ = trigger_lane_change_cancel;
  }
  virtual IntCancelReasonType lc_request_cancel_reason() {
    return lc_request_cancel_reason_;
  }

 private:
  bool CheckMLCEnable(const int lc_status);
  bool IsTriggerMLCForRemainDistane();
  void GenerateMLCRequest();
  bool CheckTargetLaneLaneMarks(RequestType request_type);
  bool CheckTargetLaneMergeDirection(RequestType request_type);

  CongestionDetectionConfig congestion_detection_config;
  int avoidance_MLC_counter = 0;
  int suppression_counter = 0;
  bool is_in_avoidance_mlc = false;
};

}  // namespace planning