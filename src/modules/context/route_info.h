#pragma once
#include "ego_lane_track_manager.h"
#include "ego_planning_config.h"
#include "hdmap/hdmap.h"
#include "local_view.h"
#include "sdmap/sdmap.h"
#include "session.h"
#include "virtual_lane.h"

namespace planning {
struct SplitSegInfo {
  RampDirection split_direction;
  int split_seg_forward_lane_nums;
  int split_next_seg_forward_lane_nums;
};
class RouteInfo {
 public:
  RouteInfo(const EgoPlanningConfigBuilder* config_builder,
            planning::framework::Session* session);
  ~RouteInfo() = default;

  void SetConfig(const EgoPlanningConfigBuilder* config_builder);

  void Update();

  void UpdateMLCInfoDecider(
      std::vector<std::shared_ptr<VirtualLane>> relative_id_lanes);

  void UpdateVisionInfo() const;
  const RouteInfoOutput& get_route_info_output() { return route_info_output_; }
  bool get_hdmap_valid() const { return hdmap_valid_; }
  bool get_sdmap_valid() const { return sdmap_valid_; }
  const ad_common::sdmap::SDMap& get_sd_map() const { return sd_map_; }
  const ad_common::hdmap::HDMap& get_hd_map() const { return hd_map_; }

  const double get_virtual_extend_buff() const { return virtual_extend_buff_; }

 private:
  const planning::framework::Session* session_ = nullptr;
  EgoPlanningConfig config_;
  RouteInfoOutput route_info_output_;
  LocalView local_view_;

  // for NOA variables
  ad_common::sdmap::SDMap sd_map_;
  bool sdmap_valid_{false};
  uint64_t sd_map_info_updated_timestamp_ = 0;
  bool is_accumulate_dis_to_last_merge_point_more_than_threshold_ = false;
  const double dis_threshold_to_last_merge_point_ = 600.0;
  const double dis_threshold_to_is_merged_point_ = 800.0;

  // for HPP variables
  ad_common::hdmap::HDMap hd_map_;
  bool hdmap_valid_{false};
  uint64_t static_map_info_updated_timestamp_ = 0;
  uint64_t current_lane_id_ = 0;
  Pose2D current_pose_{0.0, 0.0, 0.0};
  ad_common::hdmap::LaneInfoConstPtr nearest_lane_hpp_;
  double nearest_s_hpp_{0.0};
  double sum_s_hpp_{0.0};
  ad_common::math::Box2d ego_box_hpp_;
  ad_common::math::Vec2d last_point_hpp_{NL_NMAX, NL_NMAX};
  double sum_distance_driving_ = -1;
  double distance_to_target_slot_ = NL_NMAX;
  double distance_to_next_speed_bump_ = NL_NMAX;
  double virtual_extend_buff_ = 0.0;

  // for NOA function
  void UpdateRouteInfoForNOA(const ad_common::sdmap::SDMap& sdmap);
  bool UpdateSdMap(const LocalView& local_view);
  const SdMapSwtx::Segment* UpdateEgoSegmentInfo(
      const ad_common::sdmap::SDMap& sd_map, double* nearest_s);
  void CaculateRampInfo(const ad_common::sdmap::SDMap& sdmap,
                        const SdMapSwtx::Segment& segment,
                        const double nearest_s, const double max_search_length);
  void CaculateMergeInfo(const ad_common::sdmap::SDMap& sdmap,
                         const SdMapSwtx::Segment& segment,
                         const double nearest_s,
                         const double max_search_length);
  void CaculateSplitInfo(const ad_common::sdmap::SDMap& sdmap,
                         const SdMapSwtx::Segment& segment,
                         const double nearest_s,
                         const double max_search_length);
  void CaculateDistanceToLastMergePointByInlink(const ad_common::sdmap::SDMap& sdmap,
                                        const SdMapSwtx::Segment& segment,
                                        const double nearest_s,
                                        const double max_search_length);
  void CaculateDistanceToLastMergePoint(const ad_common::sdmap::SDMap& sdmap,
                                        const SdMapSwtx::Segment& segment,
                                        const double nearest_s,
                                        const double max_search_length);
  void CaculateDistanceToLastSplitPoint(const ad_common::sdmap::SDMap& sdmap,
                                        const SdMapSwtx::Segment& segment,
                                        const double nearest_s,
                                        const double max_search_length);
  void CaculateDistanceToRoadEnd(const ad_common::sdmap::SDMap& sdmap,
                                 const SdMapSwtx::Segment& segment,
                                 const double nearest_s,
                                 const double max_search_length);
  void CaculateDistanceToTollStation(const ad_common::sdmap::SDMap& sdmap,
                                     const SdMapSwtx::Segment& segment,
                                     const double nearest_s,
                                     const double max_search_length);
  SplitSegInfo MakesureSplitDirectionByOutlink(const ::SdMapSwtx::Segment& split_segment,
                                      const ad_common::sdmap::SDMap& sd_map);
  SplitSegInfo MakesureSplitDirection(const ad_common::sdmap::SDMap& sd_map, const ::SdMapSwtx::Segment& split_segment);
  RampDirection MakesureMergeDirectionByInlink(
      const ::SdMapSwtx::Segment& merge_segment,
      const ad_common::sdmap::SDMap& sd_map);
  RampDirection MakesureMergeDirection(
      const ::SdMapSwtx::Segment& merge_segment,
      const ad_common::sdmap::SDMap& sd_map);

  double CrossProduct(const Point2D& first_pt_2d, const Point2D& second_pt_2d,
                      const Point2D& third_pt_2d) const;
  RampDirection JudgeABvsBC(const Point2D& A, const Point2D& B,
                            const Point2D& C) const;
  RampDirection JudgeBCvsAB(const Point2D& A, const Point2D& B,
                            const Point2D& C) const;
  // for HPP function
  void UpdateRouteInfoForHPP(const ad_common::hdmap::HDMap& hdmap);
  bool UpdateStaticMap(const LocalView& local_view);
  bool GetCurrentNearestLane();
  void CalculateHPPInfo();
  void ConstructBox();
  void ResetHpp();
  void CalculateDistanceToTraceEnd();
  void CalculateDistanceToNextSpeedBump();
  bool IsOnHPPLane();
  double CalculatePointAccumulateS(size_t lane_id);
};
}  // namespace planning