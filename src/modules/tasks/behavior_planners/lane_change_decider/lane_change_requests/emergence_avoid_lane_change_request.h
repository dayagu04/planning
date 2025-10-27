#pragma once

#include "lane_change_request.h"
#include "tracked_object.h"

namespace planning {

/// @brief 紧急避障换道请求
class EmergenceAvoidRequest : public LaneChangeRequest {
 public:
  EmergenceAvoidRequest(
      planning::framework::Session* session,
      std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
      std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr);
  virtual ~EmergenceAvoidRequest() = default;

  void Update(int lc_status);

  void Reset();
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
  void UpdateEmergencyAvoidanceSituation(int lc_status);

  void LaneChangeDirection();

  bool is_emergency_avoidance_situation_ = false;
  double emergency_situation_timetstamp_ = std::numeric_limits<double>::max();
  int leading_vehicle_id_ = -1;
  std::shared_ptr<planning_math::KDPath> base_frenet_coord_;
  PlanningInitPoint planning_init_point_;
  std::shared_ptr<ReferencePath> left_reference_path_ = nullptr;
  std::shared_ptr<ReferencePath> right_reference_path_ = nullptr;
  std::shared_ptr<LateralObstacle> lateral_obstacle_ = nullptr;
  std::shared_ptr<LaneTracksManager> lane_tracks_manager_ = nullptr;
  RequestType lane_change_direction_ = NO_CHANGE;
  int right_lane_nums_ = 0;
  int left_lane_nums_ = 0;
};

}  // namespace planning