#pragma once

#include <type_traits>
#include "ego_planning_config.h"
#include "environmental_model.h"
#include "reference_path.h"
#include "reference_path_manager.h"
#include "session.h"
#include "tasks/task.h"

namespace planning {

enum RelativeDirection {
  LEFT_DIRECTION = 0,
  RIGHT_DIRECTION = 1,
};

class EgoLaneRoadRightDecider : public Task {
 public:
  EgoLaneRoadRightDecider(const EgoPlanningConfigBuilder* config_builder,
                          framework::Session* session);

  void Init();

  virtual ~EgoLaneRoadRightDecider() = default;

  bool Execute() override;

 private:
  void ComputeIsMergeRegion();

  void ComputeIsSplitRegion();

  void CheckIfMergeWithLeftLane();

  void CheckIfMergeWithRightLane();

  void ComputeRoadRightFromLaneMark();

  bool IsOverlapWithOtherLaneOnEndRegion(
      const std::shared_ptr<ReferencePath> reference_path,
      const RelativeDirection rel_dir);

  const double CalculateEgoFrontLineLength();

  void CalculateMergePoint(std::vector<Point2D>& merge_point_list,
                           int* calculate_nums);

  const double CalculateAverageKappa(
      const std::shared_ptr<planning_math::KDPath> cur_kd_path);

  void CalculateRoadRight(const int calculate_nums);

  bool IsVirtualLaneLine(const int lane_virtual_id);

 private:
  ScenarioStateMachineConfig config_;
  std::shared_ptr<VirtualLaneManager> virtual_lane_mgr_;
  std::shared_ptr<ReferencePathManager> ref_path_mgr_;
  framework::Session* session_;
  // int overlap_lane_virtual_id_ = 0;
  bool is_merge_region_ = false;
  bool is_split_region_ = false;
  MergeDirection merge_direction_ = NONE_LANE_MERGE;
  int merge_lane_virtual_id_;
  int split_lane_virtual_id_;
  Point2D merge_point_;
  Point2D boundary_merge_point_;
  bool cur_lane_is_continue_;
  bool boundary_merge_point_valid_ = false;
  bool ego_lane_boundary_exist_virtual_line_ = false;
  bool target_lane_boundary_exist_virtual_line_ = true;
  bool is_left_merge_direction_ = false;
  bool is_right_merge_direction_ = false;
};

}  // namespace planning
