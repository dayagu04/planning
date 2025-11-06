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
  virtual void SetLaneChangeCmd(std::uint8_t lane_change_cmd) {
    lane_change_cmd_ = lane_change_cmd;
  }
  virtual void SetLaneChangeCancelFromTrigger(bool trigger_lane_change_cancel) {
    trigger_lane_change_cancel_ = trigger_lane_change_cancel;
  }
  virtual IntCancelReasonType lc_request_cancel_reason() {
    return lc_request_cancel_reason_;
  }
  const bool is_merge_lane_change_situation() {
    return is_merge_lane_change_situation_;
  };

 private:
  void UpdateLaneMergeSituation(int lc_status);

  void setLaneChangeRequestByMerge(int lc_status);

  void MakesureLaneMergeDirection(const int origin_lane_id);

  void MakesureVirtualLaneSideIsVirtual(
    const std::shared_ptr<VirtualLane> base_lane,
    bool& virtual_lane_exist_virtual,
    const int lane_index);

 private:
  std::shared_ptr<planning_math::KDPath> base_frenet_coord_;
  PlanningInitPoint planning_init_point_;
  std::shared_ptr<ReferencePath> left_reference_path_ = nullptr;
  std::shared_ptr<ReferencePath> right_reference_path_ = nullptr;
  bool enable_l_ = false;
  bool enable_r_ = false;
  bool is_merge_lane_change_situation_ = false;
  bool both_lane_line_exist_virtual_or_not_ = false;
  RequestType merge_lane_change_direction_ = NO_CHANGE;
  bool use_map_is_merge_situation_ = false;
  int merge_alc_trigger_counter_ = 0;
  bool is_exist_left_merge_direction_ = false;
  bool is_exist_right_merge_direction_ = false;
  double distance_to_merge_point_ = NL_NMAX;
  MergeType lane_merge_direction_ = NO_MERGE;
};

}  // namespace planning