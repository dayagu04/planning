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

 private:
  void UpdateEmergencyAvoidanceSituation(int lc_status);

  bool is_emergency_avoidance_situation_ = false;
  double emergency_situation_timetstamp_ = std::numeric_limits<double>::max();
  int leading_vehicle_id_ = -1;
  std::shared_ptr<planning_math::KDPath> base_frenet_coord_;
  PlanningInitPoint planning_init_point_;
  std::shared_ptr<ReferencePath> left_reference_path_ = nullptr;
  std::shared_ptr<ReferencePath> right_reference_path_ = nullptr;
  std::shared_ptr<LateralObstacle> lateral_obstacle_ = nullptr;
  std::unordered_map<int, TrackedObject> tracks_map_;
  std::shared_ptr<LaneTracksManager> lane_tracks_manager_ = nullptr;
};

}  // namespace planning