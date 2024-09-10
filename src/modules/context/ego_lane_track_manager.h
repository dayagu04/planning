#pragma once

#include <climits>
#include <utility>
#include <vector>

#include "ego_planning_config.h"
#include "fusion_road_c.h"
#include "generated_refline.pb.h"
#include "intersection.h"
#include "local_view.h"
#include "log.h"
#include "session.h"
#include "virtual_lane.h"

namespace planning {

enum SplitRelativeDirection {
  None = 0,
  ON_LEFT = 1,
  ON_RIGHT = 2,
};

class EgoLaneTrackManger {
 public:
  explicit EgoLaneTrackManger(planning::framework::Session *session);
  // EgoLaneTrackManger() = default;
  ~EgoLaneTrackManger(){};

  void Update(
      const bool is_ego_on_expressway, const bool is_on_ramp,
      const double dis_to_ramp, const bool is_leaving_ramp,
      const std::pair<SplitRelativeDirection, double> first_split_dir_dis_info,
      const double distance_to_first_road_merge,
      const double distance_to_first_road_split,
      const double current_segment_passed_distance,
      const std::vector<std::pair<SplitRelativeDirection, double>> &split_dir_dis_info_list);

  void Reset();

  void CalculateVirtualLaneAttributes(
      std::vector<std::shared_ptr<VirtualLane>> &relative_id_lanes);

  void TrackEgoLane(
      std::vector<std::shared_ptr<VirtualLane>> &relative_id_lanes,
      std::vector<int> &order_ids_of_same_zero_relative_id,
      const std::unordered_map<int, std::shared_ptr<VirtualLane>>
          &virtual_id_mapped_lane);

  void UpdateLaneVirtualId(
      std::vector<std::shared_ptr<VirtualLane>> &relative_id_lanes,
      std::unordered_map<int, std::shared_ptr<VirtualLane>>
          &virtual_id_mapped_lane,
      int *last_fix_lane_virtual_id);

  void PreprocessRoadSplit(
      std::vector<std::shared_ptr<VirtualLane>> &relative_id_lanes,
      const std::vector<int> &order_ids);

  void PreprocessRampSplit(
      std::vector<std::shared_ptr<VirtualLane>> &relative_id_lanes,
      const std::vector<int> &order_ids);

  void PreprocessIntersectionSplit(
      std::vector<std::shared_ptr<VirtualLane>> &relative_id_lanes,
      const std::vector<int> &order_ids);

  void SelectEgoLaneWithoutPlan(
      std::vector<std::shared_ptr<VirtualLane>> &relative_id_lanes);

  void SelectEgoLaneWithPlan(
      std::vector<std::shared_ptr<VirtualLane>> &relative_id_lanes,
      int zero_relative_id_nums,
      const std::unordered_map<int, std::shared_ptr<VirtualLane>>
          &virtual_id_mapped_lane);

  bool CheckIfInRampSelectSplit(
      std::vector<std::shared_ptr<VirtualLane>> relative_id_lanes,
      const std::vector<int> &order_ids);

  bool CheckIfInRoadSelectRamp(
      std::vector<std::shared_ptr<VirtualLane>> relative_id_lanes,
      const std::vector<int> &order_ids);

  double ComputeTargetLaneSpecifiedRangeCurvature(
      const std::shared_ptr<VirtualLane> virtual_lane);

  bool CalcCrosslaneStatus(
      const std::shared_ptr<VirtualLane> lane,
      const std::vector<iflyauto::ReferencePoint> &center_line_pathpoints);

  std::shared_ptr<planning_math::KDPath> MakeBoundaryPath(
      const iflyauto::LaneBoundary &boundary);

  void CalcBoundaryCross(
      const planning_math::KDPath &lane_boundary_path,
      const std::vector<iflyauto::ReferencePoint> &center_line_pathpoints,
      bool *cross_lane);

  bool is_exist_split_on_ramp() const { return is_exist_split_on_ramp_; };

  bool is_exist_ramp_on_road() const { return is_exist_ramp_on_road_; };

  bool is_exist_intersection_split() const {
    return is_exist_split_on_intersection_;
  };

  bool is_in_ramp_select_split_situation() const {
    return is_in_ramp_select_split_situation_;
  };

  bool is_on_road_select_ramp_situation() const {
    return is_on_road_select_ramp_situation_;
  };

  bool is_select_ego_lane_without_plan() const {
    return is_select_ego_lane_without_plan_;
  };

  bool is_select_ego_lane_with_plan() const {
    return is_select_ego_lane_with_plan_;
  };

  bool is_virtual_lane_relative_id_switch() const {
    return virtual_lane_relative_id_switch_flag_;
  };

 private:
  double ComputeLanesMatchlaterakDisCost(
      int virtual_id,
      const std::shared_ptr<VirtualLane> current_relative_id_lane,
      const std::vector<std::shared_ptr<VirtualLane>> &relative_id_lanes,
      const std::unordered_map<int, std::shared_ptr<VirtualLane>>
          &virtual_id_mapped_lane);

  double ComputeAverageHeadingDiff(std::shared_ptr<VirtualLane> base_lane,
                                   const double ego_heading_angle);

 private:
  planning::framework::Session *session_ = nullptr;
  int last_fix_lane_virtual_id_ = 0;
  int current_lane_virtual_id_ = 0;
  std::shared_ptr<VirtualLane> current_lane_ = nullptr;
  std::shared_ptr<VirtualLane> left_lane_ = nullptr;
  std::shared_ptr<VirtualLane> right_lane_ = nullptr;
  uint lane_num_ = 0;

  bool is_ego_on_expressway_ = false;
  bool is_on_ramp_ = false;
  bool dis_to_ramp_ = NL_NMAX;
  bool is_leaving_ramp_ = false;
  std::pair<SplitRelativeDirection, double> first_split_dir_dis_info_;
  std::vector<std::pair<SplitRelativeDirection, double>>
      split_direction_dis_info_list_;
  double distance_to_first_road_merge_ = NL_NMAX;
  double distance_to_first_road_split_ = NL_NMAX;
  bool virtual_lane_relative_id_switch_flag_ = false;
  bool is_exist_split_on_ramp_ = false;
  bool is_exist_ramp_on_road_ = false;
  bool is_exist_split_on_intersection_ = false;
  bool is_in_ramp_select_split_situation_ = false;
  bool is_on_road_select_ramp_situation_ = false;
  bool is_select_ego_lane_without_plan_ = false;
  bool is_select_ego_lane_with_plan_ = false;
  double current_segment_passed_distance_ = 0.0;
};

}  // namespace planning