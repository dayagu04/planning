#pragma once

#include "lane_change_request.h"

namespace planning {

/// @brief 交互式(Interactive)换道请求
class MergeRequest : public LaneChangeRequest {
 public:
  MergeRequest(planning::framework::Session* session,
               std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
               std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr);
  virtual ~MergeRequest() = default;

  void Update(int lc_status);

  void Reset();

 private:
  void UpdateLaneMergeSituation(int lc_status);

  void setLaneChangeRequestByMerge(int lc_status);

  void MakesureLaneMergeDirection(const int origin_lane_id);

 private:
  std::shared_ptr<KDPath> base_frenet_coord_;
  PlanningInitPoint planning_init_point_;
  std::shared_ptr<ReferencePath> left_reference_path_ = nullptr;
  std::shared_ptr<ReferencePath> right_reference_path_ = nullptr;
  bool enable_l_ = false;
  bool enable_r_ = false;
  bool is_merge_lane_change_situation_ = false;
  RequestType merge_lane_change_direction_ = NO_CHANGE;
  int merge_alc_trigger_counter_ = 0;
};

}  // namespace planning