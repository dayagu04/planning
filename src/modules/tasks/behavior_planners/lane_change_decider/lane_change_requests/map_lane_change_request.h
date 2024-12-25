#pragma once

#include "config/basic_type.h"
#include "lane_change_request.h"

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

 private:
  bool CheckMLCEnable(const int lc_status);
  bool IsTriggerMLCForRemainDistane();
  void GenerateMLCRequest();
  bool CheckTargetLaneLaneMarks(RequestType request_type);
  bool CheckTargetLaneMergeDirection(RequestType request_type);
};

}  // namespace planning