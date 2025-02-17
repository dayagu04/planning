#pragma once

#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "adas_function/display_state_types.h"
#include "config/basic_type.h"
#include "ifly_time.h"
#include "lane_change_lane_manager.h"
#include "session.h"
#include "virtual_lane_manager.h"

namespace planning {
/// @brief 换道请求的基类，生成、结束换道请求等
class LaneChangeRequest {
 public:
  LaneChangeRequest(
      planning::framework::Session* session,
      std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
      std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr);
  virtual ~LaneChangeRequest() = default;

  void GenerateRequest(RequestType direction);
  void Finish();
  bool AggressiveChange() const;
  bool IsDashedLineEnough(RequestType direction, const double ego_vel,
                          std::shared_ptr<VirtualLaneManager> virtual_lane_mgr);

  RequestType request_type() const { return request_type_; }
  int target_lane_virtual_id() { return target_lane_virtual_id_; }
  void set_target_lane_virtual_id(int target_lane_virtual_id) {
    target_lane_virtual_id_ = target_lane_virtual_id;
  }
  RequestType turn_signal() const { return turn_signal_; }
  double tstart() const { return tstart_; }
  double tfinish() const { return tfinish_; }
  bool ComputeLcValid(RequestType direction);
  bool IsDashEnoughForRepeatSegments(const RequestType lc_request,
                                     const int origin_lane_id) const;
  iflyauto::LaneBoundaryType MakesureCurrentBoundaryType(
      const RequestType lc_request, const int origin_lane_id) const;
  bool IsRoadBorderSurpressLaneChange(const RequestType lc_request,
                                      const int origin_lane_id,
                                      const int target_lane_id);

 protected:
  TrackInfo lc_invalid_track_;
  RequestType request_type_ = NO_CHANGE;
  int target_lane_virtual_id_ = -1000;  // invalid
  int origin_lane_virtual_id_ = -1000;
  int origin_lane_order_id_ = -1000;
  RequestType turn_signal_ = NO_CHANGE;
  double tstart_ = 0.0;
  double tfinish_ = 0.0;
  framework::Session* session_;
  std::shared_ptr<VirtualLaneManager> virtual_lane_mgr_;
  std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr_;
  double aggressive_lane_change_distance_highway_{0.0};
  double aggressive_lane_change_distance_urban_{150.0};
};

}  // namespace planning
