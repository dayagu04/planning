#pragma once
#include "config/basic_type.h"
#include "ego_lane_track_manager.h"
#include "ego_planning_config.h"
#include "hdmap/hdmap.h"
#include "local_view.h"
#include "sdmap/sdmap.h"
#include "sdpromap/sdpromap.h"
#include "session.h"
#include "virtual_lane.h"

namespace planning {
struct SplitSegInfo {
  RampDirection split_direction;
  int split_seg_forward_lane_nums;
  int split_next_seg_forward_lane_nums;
};

struct MLCDeciderRouteInfo {
  EgoStatusOnRoute ego_status_on_route = ON_MAIN;
  bool is_process_split = false;
  bool is_process_split_split = false;
  bool is_process_other_merge_split = false;
  bool is_process_other_merge = false;
  bool is_process_merge = false;
  double last_frame_dis_to_merge_start_point = NL_NMAX;
  double last_frame_dis_to_merge_point = NL_NMAX;
  double last_frame_dis_to_split = NL_NMAX;
  double end_fp_dis_to_split = 0;
  bool is_triggle_cal_dis_to_last_merge_point = false;
  NOASplitRegionInfo static_merge_region_info;
  NOASplitRegionInfo first_static_split_region_info;

  void reset () {
    ego_status_on_route = ON_MAIN;
    is_process_split = false;
    is_process_split_split = false;
    is_process_other_merge_split = false;
    is_process_other_merge = false;
    is_process_merge = false;
    last_frame_dis_to_merge_start_point = NL_NMAX;
    last_frame_dis_to_merge_point = NL_NMAX;
    last_frame_dis_to_split = NL_NMAX;
    end_fp_dis_to_split = 0;
    is_triggle_cal_dis_to_last_merge_point = false;
    static_merge_region_info.reset();
    first_static_split_region_info.reset();
  }
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

  void NewUpdateMLCInfoDecider(
      std::vector<std::shared_ptr<VirtualLane>> relative_id_lanes);

  void UpdateVisionInfo() const;
  const RouteInfoOutput& get_route_info_output() { return route_info_output_; }
  bool get_hdmap_valid() const { return hdmap_valid_; }
  bool get_sdmap_valid() const { return sdmap_valid_; }
  const ad_common::sdmap::SDMap& get_sd_map() const { return sd_map_; }
  const ad_common::hdmap::HDMap& get_hd_map() const { return hd_map_; }

  const double get_virtual_extend_buff() const { return virtual_extend_buff_; }

  void ResetMLCInfoDecider() {
    mlc_decider_route_info_.reset();
  }

 private:
  using MapLane = iflymapdata::sdpro::Lane;
  const planning::framework::Session* session_ = nullptr;
  EgoPlanningConfig config_;
  MLCDeciderConfig mlc_decider_config_;

  RouteInfoOutput route_info_output_;
  LocalView local_view_;

  // for NOA variables
  ad_common::sdmap::SDMap sd_map_;
  bool sdmap_valid_{false};
  uint64_t sd_map_info_updated_timestamp_ = 0;
  bool is_accumulate_dis_to_last_merge_point_more_than_threshold_ = false;
  const double dis_threshold_to_last_merge_point_ = 600.0;
  const double dis_threshold_to_is_merged_point_ = 800.0;
  // for sdpro
  ad_common::sdpromap::SDProMap sdpro_map_;
  bool sdpromap_valid_{false};
  uint64_t sdpro_map_info_updated_timestamp_ = 0;
  // EgoStatusOnRoute ego_status_on_route_ = EgoStatusOnRoute::ON_MAIN;
  // double last_frame_dis_to_split_ = NL_NMAX;
  // double end_fp_dis_to_split_ = 0;
  // NOASplitRegionInfo first_static_split_region_info_;
  // //for merge
  // double last_frame_dis_to_merge_start_point_ = NL_NMAX;
  // double last_frame_dis_to_merge_point_ = NL_NMAX;
  // bool is_triggle_cal_dis_to_last_merge_point_ = false;
  // NOASplitRegionInfo static_merge_region_info_;

  // bool is_process_split_ = false;
  // bool is_process_split_split_ = false;
  // bool is_process_other_merge_split_ = false;
  // bool is_process_other_merge_ = false;
  // bool is_process_merge_ = false;

  MLCDeciderRouteInfo mlc_decider_route_info_;

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

  struct RayInfo {
    char name;
    double angle;
    RayInfo(char n, double a) : name(n), angle(a) {}
  };

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
  SplitSegInfo MakesureSplitDirection(const ::SdMapSwtx::Segment& split_segment,
                                      const ad_common::sdmap::SDMap& sd_map);
  RampDirection MakesureMergeDirection(
      const ::SdMapSwtx::Segment& merge_segment,
      const ad_common::sdmap::SDMap& sd_map);

  // for sdpro
  void UpdateRouteInfoForNOA(const ad_common::sdpromap::SDProMap& sdpromap);
  bool UpdateSdProMap(const LocalView& local_view);
  const iflymapdata::sdpro::LinkInfo_Link* UpdateEgoLinkInfo(
      const ad_common::sdpromap::SDProMap& sdpro_map, double* nearest_s);
  void CaculateRampInfo(const ad_common::sdpromap::SDProMap& sdpromap,
                        const iflymapdata::sdpro::LinkInfo_Link& segment,
                        const double nearest_s, const double max_search_length);
  void CaculateMergeInfo(const ad_common::sdpromap::SDProMap& sdpromap,
                         const iflymapdata::sdpro::LinkInfo_Link& segment,
                         const double nearest_s,
                         const double max_search_length);
  void CaculateSplitInfo(const ad_common::sdpromap::SDProMap& sdpromap,
                         const iflymapdata::sdpro::LinkInfo_Link& segment,
                         const double nearest_s,
                         const double max_search_length);
  void CaculateDistanceToLastMergePoint(
      const ad_common::sdpromap::SDProMap& sdpromap,
      const iflymapdata::sdpro::LinkInfo_Link& segment, const double nearest_s,
      const double max_search_length);
  void CaculateDistanceToLastSplitPoint(
      const ad_common::sdpromap::SDProMap& sdpromap,
      const iflymapdata::sdpro::LinkInfo_Link& segment, const double nearest_s,
      const double max_search_length);
  void CaculateDistanceToRoadEnd(
      const ad_common::sdpromap::SDProMap& sdpromap,
      const iflymapdata::sdpro::LinkInfo_Link& segment, const double nearest_s,
      const double max_search_length);
  void CaculateDistanceToTollStation(
      const ad_common::sdpromap::SDProMap& sdpromap,
      const iflymapdata::sdpro::LinkInfo_Link& segment, const double nearest_s,
      const double max_search_length);

  SplitSegInfo MakesureSplitDirection(
      const iflymapdata::sdpro::LinkInfo_Link& split_link,
      const ad_common::sdpromap::SDProMap& sdpro_map);
  RampDirection MakesureMergeDirection(
      const iflymapdata::sdpro::LinkInfo_Link& merge_segment,
      const ad_common::sdpromap::SDProMap& sdpro_map);

  NOASplitRegionInfo CalculateSplitRegionLaneTupoInfo(
    const iflymapdata::sdpro::LinkInfo_Link& split_segment,
      const ad_common::sdpromap::SDProMap& sdpro_map);

  NOASplitRegionInfo CalculateMergeRegionLaneTupoInfo(
    const iflymapdata::sdpro::LinkInfo_Link& split_segment,
      const ad_common::sdpromap::SDProMap& sdpro_map);

  bool CalculateFeasibleLane(NOASplitRegionInfo* split_region_info, const ad_common::sdpromap::SDProMap& sdpro_map) const;
  bool CalculateFeasibleLane(NOASplitRegionInfo* split_region_info) const;
  bool CalculateMergeRegionFeasibleLane(NOASplitRegionInfo* split_region_info) const;

  std::vector<int> CommonElements(const std::vector<int>& A,
                                  const std::vector<int>& B) {
    std::vector<int> commonElements;
    // 查找A和B的共有元素
    for (int element : B) {
      if (std::find(A.begin(), A.end(), element) != A.end()) {
        commonElements.push_back(element);
      }
    }
    return commonElements;
  }

  bool IsEmergencyLane(const uint64 lane_id, const ad_common::sdpromap::SDProMap& sdpro_map) const;
  double CalculateAngle(const Point2D & o, const Point2D& p) {
      double dx = p.x - o.x;
      double dy = p.y - o.y;
      double angle = std::atan2(dy, dx);  // 相对于x轴正方向的夹角
      if (angle < 0) {
          angle += 2 * M_PI;  // 转换到0到2π
      }
      return angle;
  }

  // 正确排序跨越π的射线
  std::vector<char> SortRaysByDirection(const std::vector<RayInfo>& rays);

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