#pragma once

#include "lane_change_requests/lane_change_request.h"

namespace planning {

/// @brief 换道请求子类：地图换道请求
class MapRequest : public LaneChangeRequest {
 public:
  MapRequest(framework::Session* session,
             const EgoPlanningConfigBuilder* config_builder,
             std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
             std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr);
  virtual ~MapRequest() = default;
  void update(int lc_status, double lc_map_tfinish);

 private:
  bool check_mlc_enable(double lc_map_tfinish);
  void print_forbid_generating_reason(
      const std::vector<std::string> forbid_generating_reason);
  void check_lc_forbid_reason(
      std::vector<std::string>& forbid_generating_left_reason,
      std::vector<std::string>& forbid_generating_right_reason,
      int left_int_freeze_cnt, int right_int_freeze_cnt);
  bool must_change_before_curr_intersection();
  bool IsDashEnoughForRepeatSegments(
      const int lc_map_decision,
      std::shared_ptr<VirtualLane> current_lane) const;
};

}  // namespace planning