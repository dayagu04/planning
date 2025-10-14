#pragma once
#include <vector>

#include "config/basic_type.h"
#include "ego_lane_track_manager.h"
#include "ego_planning_config.h"
#include "hdmap/hdmap.h"
#include "local_view.h"
#include "sdmap/sdmap.h"
#include "sdpromap/sdpromap.h"
#include "session.h"
#include "vec2d.h"
#include "virtual_lane.h"
#include "modules/context/route_info_strategy/route_info_strategy.h"


namespace planning {
struct SplitSegInfo {
  RampDirection split_direction;
  int split_seg_forward_lane_nums;
  int split_next_seg_forward_lane_nums;
};

struct MLCRequestType {
  int lane_num;
  EgoMLCRequestType mlc_request_type;
  SplitDirection split_direction;

  void reset() {
    lane_num = -1;
    mlc_request_type = None_MLC;
    split_direction = SPLIT_NONE;
  }
};

struct LastExchangeRegionInfo {
  bool is_process_split = false;
  bool is_process_other_merge = false;
  bool is_process_merge = false;
  NOASplitRegionInfo last_exchange_info;
};

class RouteInfo {
 public:
  RouteInfo(const EgoPlanningConfigBuilder* config_builder,
            planning::framework::Session* session);
  ~RouteInfo() = default;

  void SetConfig(const EgoPlanningConfigBuilder* config_builder);

  void Update();

  void GetStrategy();

  void UpdateMLCInfoDecider(
      std::vector<std::shared_ptr<VirtualLane>> relative_id_lanes);
      
  void UpdateMLCInfoDeciderBaseBaidu(
      std::vector<std::shared_ptr<VirtualLane>> relative_id_lanes);

  void UpdateMLCInfoDeciderBaseTencent(
      std::vector<std::shared_ptr<VirtualLane>> relative_id_lanes);

  void UpdateVisionInfo() const;
  const RouteInfoOutput& get_route_info_output() { return route_info_output_; }
  bool get_hdmap_valid() const { return hdmap_valid_; }
  bool get_sdmap_valid() const { return sdmap_valid_; }
  const ad_common::sdmap::SDMap& get_sd_map() const { return sd_map_; }
  const ad_common::hdmap::HDMap& get_hd_map() const { return hd_map_; }
  bool get_sdpromap_valid() const { return sdpromap_valid_; }

  const ad_common::sdpromap::SDProMap& get_sdpro_map() const {
    return sdpro_map_;
  }

  const double get_virtual_extend_buff() const { return virtual_extend_buff_; }

  void ResetMLCInfoDecider() { mlc_decider_route_info_.reset(); }

 private:
  using MapLane = iflymapdata::sdpro::Lane;
  const planning::framework::Session* session_ = nullptr;
  EgoPlanningConfig config_;
  MLCDeciderConfig mlc_decider_config_;
  std::vector<MLCRequestType> mlc_request_info_;
  RouteInfoOutput route_info_output_;

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
  MLCDeciderRouteInfo mlc_decider_route_info_;
  const iflymapdata::sdpro::LinkInfo_Link* current_link_ = nullptr;
  std::string last_path_id_;
  bool last_path_id_is_set_ = false;
  bool last_frame_is_process_split_{false};
  bool is_need_judge_miss_split_ = false;
  ad_common::math::Vec2d split_point_{0, 0};
  uint64 split_link_id_ = 1;
  std::unique_ptr<RouteInfoStrategy> route_info_strategy_ = nullptr;

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
  LastExchangeRegionInfo last_exchange_region_info_;
  struct RayInfo {
    char name;
    double angle;
    RayInfo(char n, double a) : name(n), angle(a) {}
  };
  // int mismatch_counter = 0;
  // const int MISMATH_THRESHOLD = 3;

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
      const ad_common::sdpromap::SDProMap& sdpro_map, double* nearest_s,
      double* nearest_l);
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
      const ad_common::sdpromap::SDProMap& sdpro_map,
      const std::vector<std::pair<const iflymapdata::sdpro::LinkInfo_Link*,
                                  double>>& split_info_vec,
      const double ego_dis_to_split);

  NOASplitRegionInfo CalculateMergeRegionLaneTupoInfo(
      const iflymapdata::sdpro::LinkInfo_Link& split_segment,
      const ad_common::sdpromap::SDProMap& sdpro_map,
      const std::vector<std::pair<const iflymapdata::sdpro::LinkInfo_Link*,
                                  double>>& merge_info_vec,
      const std::vector<std::pair<const iflymapdata::sdpro::LinkInfo_Link*,
                                  double>>& split_info_vec,
      const double ego_dis_to_merge);

  bool CalculateMergeLaneInfo(
      std::map<int, SplitDirection>& merge_lane_sequence,
      double search_distance);
  bool CalculateLastFp(iflymapdata::sdpro::FeaturePoint* last_fp,
                       iflymapdata::sdpro::LinkInfo_Link* last_fp_link,
                       const uint64 fp_link_id,
                       const iflymapdata::sdpro::FeaturePoint& find_fp);

  bool CalculateMergeFP(MergeType* merge_type,
                        iflymapdata::sdpro::FeaturePoint* find_fp,
                        uint64* fp_link_id, double* dis_to_merge_fp,
                        double search_distance);
  bool CalculateFeasibleLane(
      NOASplitRegionInfo* split_region_info,
      const ad_common::sdpromap::SDProMap& sdpro_map) const;
  bool CalculateFeasibleLane(NOASplitRegionInfo* split_region_info);
  bool CalculateMergeRegionFeasibleLane(NOASplitRegionInfo* split_region_info);
  bool CalculateOtherMergeRoadFeasibleLane(
      NOASplitRegionInfo* split_region_info);
  bool IsMergeFP(iflymapdata::sdpro::LaneChangeType* merge_type,
                 const iflymapdata::sdpro::FeaturePoint& fp) const;
  bool IsExpandFP(iflymapdata::sdpro::LaneChangeType* expand_type,
                  const iflymapdata::sdpro::FeaturePoint& fp) const;
  const iflymapdata::sdpro::LinkInfo_Link* CalculateCurrentLink(double* s,
                                                                double* l);
  bool CalculateLastFPInCurrentLink(
      iflymapdata::sdpro::FeaturePoint* find_fp,
      const iflymapdata::sdpro::LinkInfo_Link* const cur_link, const double s);
  bool IsMissSplitPoint(const iflymapdata::sdpro::LinkInfo_Link& segment,
                        const double l, const double s);

  bool IsTriggerContinueLCInPerceptionSplitRegion(
      const int left_lane_num, const int right_lane_num) const;

  bool IsExistLengthSolidLine(
      std::vector<std::pair<const MarkingLineChangeType, double>>&
          mlc_fp_info_list,
      const uint64 fp_link_id, const iflymapdata::sdpro::FeaturePoint cur_fp,
      const double first_distance_to_split_point);

  bool CalculateLastMarkingLineChangeFp(
      iflymapdata::sdpro::FeaturePoint* mlc_fp, uint64* mlc_link_id,
      double* sum_dis, const uint64 fp_link_id,
      const iflymapdata::sdpro::FeaturePoint cur_fp,
      const double first_distance_to_split_point);

  bool IsExistMarkingLineChangeFP(iflymapdata::sdpro::FeaturePoint* last_fp,
                                  double* sum_dis, const uint64 fp_link_id);

  bool IsExistMarkingLineChangeFPCurLink(
      iflymapdata::sdpro::FeaturePoint* last_fp, double* sum_dis,
      const uint64 fp_link_id, const iflymapdata::sdpro::FeaturePoint cur_fp);

  bool IsDashSolidLineTypeChnage(
      MarkingLineChangeType* marking_line_change_type,
      const iflymapdata::sdpro::FeaturePoint& mlc_fp, const uint64 mlc_link_id);

  bool IsSolidBoundary(const uint64 lane_id);

  int EmergencyLaneNum(const iflymapdata::sdpro::FeaturePoint& mlc_fp);

  double LengthSolidLineJudge(const uint64 fp_link_id,
                              const iflymapdata::sdpro::FeaturePoint cur_fp,
                              const double first_distance_to_split_point);

  std::vector<int> CalculateMLCTaskNoLaneNum();

  bool CalculateDistanceToLastSplitPoint(double* dis, const double s) const;

  bool CalculateDistanceToLastMergePoint(double* dis, const double s) const;

  bool CalculateDistanceNextToLastSplitPoint(
      double* dis,
      const iflymapdata::sdpro::LinkInfo_Link* next_split_or_merge_link) const;

  bool CalculateDistanceNextToLastMergePoint(
      double* dis,
      const iflymapdata::sdpro::LinkInfo_Link* next_split_or_merge_link) const;

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
  std::vector<int> findMissingElements(const std::vector<int>& mainVec,
                                       const std::vector<int>& subVec) {
    std::vector<int> missingInSub;
    for (int elem : mainVec) {
      if (std::find(subVec.begin(), subVec.end(), elem) == subVec.end()) {
        missingInSub.push_back(elem);
      }
    }
    return missingInSub;
  }
  bool IsEmergencyLane(const uint64 lane_id,
                       const ad_common::sdpromap::SDProMap& sdpro_map) const;
  bool IsClosingIntersectionEntrance(
      const iflymapdata::sdpro::LinkInfo_Link* link,
      const ad_common::sdpromap::SDProMap& sdpro_map, double distance_on_link);
  double CalculateAngle(const Point2D& o, const Point2D& p) {
    double dx = p.x - o.x;
    double dy = p.y - o.y;
    double angle = std::atan2(dy, dx);  // 相对于x轴正方向的夹角
    if (angle < 0) {
      angle += 2 * M_PI;  // 转换到0到2π
    }
    return angle;
  }

  double DistanceToLine(const planning_math::Vec2d& point,
                        const planning_math::Vec2d& segment_start,
                        const planning_math::Vec2d& segment_end);

  // 正确排序跨越π的射线
  std::vector<char> SortRaysByDirection(const std::vector<RayInfo>& rays);

  void RemoveElement(std::vector<int>& vec, int target) {
    if (vec.size() <= 1) {
      return;
    }

    auto it = std::find(vec.begin(), vec.end(), target);

    if (it != vec.end()) {
      vec.erase(it);
    }
  }

  bool SortFPBaseProjection(
      std::vector<iflymapdata::sdpro::FeaturePoint>& sorted_fp,
      const iflymapdata::sdpro::LinkInfo_Link* link) const;

  void FindNextMergeExpandTypeFp(
      const FPPoint& fp_start,
      std::vector<iflymapdata::sdpro::FeaturePoint>& specific_fp,
      double max_search_distance);
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
  void EraseSplitSplitFeasibleLane(NOASplitRegionInfo& first_split_region_info,
                                   NOASplitRegionInfo& second_split_region_info,
                                   int erase_num);
  void OptimizeFeasibleLanesByDistance(
      NOASplitRegionInfo& exchange_region_info,
      std::map<int, double>& feasible_lane_distance, double max_distance);
  std::vector<int> GetIntersection(const std::vector<int>& vec1,
                                   const std::vector<int>& vec2);
  void ProcessLaneDistance(const std::shared_ptr<VirtualLane>& relative_id_lane,
                           const std::map<int, double>& feasible_lane_distance);

};
}  // namespace planning