#pragma once
#include "ego_lane_track_manager.h"
#include "ego_planning_config.h"
#include "hdmap/hdmap.h"
#include "local_view.h"
#include "sdmap/sdmap.h"
#include "session.h"
#include "virtual_lane.h"

namespace planning {

struct RouteInfoOutput {
  bool is_update_segment_success = false;
  bool is_on_ramp = false;
  double dis_to_ramp = NL_NMAX;
  RampDirection ramp_direction = RampDirection::RAMP_NONE;
  double distance_to_first_road_merge = NL_NMAX;
  double distance_to_first_road_split = NL_NMAX;
  double distance_to_second_road_merge = NL_NMAX;
  double distance_to_second_road_split = NL_NMAX;
  double distance_to_route_end = NL_NMAX;
  bool is_in_sdmaproad = false;
  bool is_ego_on_expressway = false;
  RampDirection first_split_direction = RampDirection::RAMP_NONE;
  RampDirection first_merge_direction = RampDirection::RAMP_NONE;
  RampDirection second_split_direction = RampDirection::RAMP_NONE;
  RampDirection second_merge_direction = RampDirection::RAMP_NONE;
  bool  is_leaving_ramp = false;
  double sum_dis_to_last_merge_point = NL_NMAX;
  double sum_dis_to_last_split_point = NL_NMAX;
  double accumulate_dis_ego_to_last_split_point = NL_NMAX;
  double sum_dis_to_last_split_point_on_ramp = NL_NMAX;
  double distance_to_toll_station = NL_NMAX;
  bool is_ego_on_city_expressway_hmi = false;
  bool is_ego_on_expressway_hmi = false;
  bool is_exist_toll_station = false;
  bool is_ramp_merge_to_road_on_expressway = false;
  bool is_road_merged_by_other_lane = false;
  bool is_ramp_merge_to_ramp_on_expressway = false;
  RampDirection other_lane_merge_dir = RampDirection::RAMP_NONE;
  bool is_nearing_other_lane_merge_to_road_point = false;
  bool is_on_highway = false;
  int split_seg_forward_lane_nums = 0;
  int split_next_seg_forward_lane_nums = 0;
  int lc_nums_for_split = 0;
  RampDirection last_split_seg_dir = RAMP_NONE;
  bool is_continuous_ramp = false;//for jwliu23
  std::pair<SplitRelativeDirection, double> first_split_dir_dis_info;//for xykuai
  std::vector<std::pair<SplitRelativeDirection, double>>
      split_dir_dis_info_list;//for xykuai
  double current_segment_passed_distance = 0.0;//for xykuai
  bool is_nearing_ramp = false;
  int cur_seg_forward_lane_num = 0;
  void reset() {
    is_update_segment_success = false;
    is_on_ramp = false;
    dis_to_ramp = NL_NMAX;
    ramp_direction = RampDirection::RAMP_NONE;
    distance_to_first_road_merge = NL_NMAX;
    distance_to_first_road_split = NL_NMAX;
    distance_to_second_road_merge = NL_NMAX;
    distance_to_second_road_split = NL_NMAX;
    distance_to_route_end = NL_NMAX;
    is_in_sdmaproad = false;
    is_ego_on_expressway = false;
    first_split_direction = RampDirection::RAMP_NONE;
    first_merge_direction = RampDirection::RAMP_NONE;
    second_split_direction = RampDirection::RAMP_NONE;
    second_merge_direction = RampDirection::RAMP_NONE;
    is_leaving_ramp = false;
    sum_dis_to_last_merge_point = NL_NMAX;
    sum_dis_to_last_split_point = NL_NMAX;
    accumulate_dis_ego_to_last_split_point = NL_NMAX;
    sum_dis_to_last_split_point_on_ramp = NL_NMAX;
    distance_to_toll_station = NL_NMAX;
    is_ego_on_city_expressway_hmi = false;
    is_ego_on_expressway_hmi = false;
    is_exist_toll_station = false;
    is_ramp_merge_to_road_on_expressway = false;
    is_road_merged_by_other_lane = false;
    is_ramp_merge_to_ramp_on_expressway = false;
    other_lane_merge_dir = RampDirection::RAMP_NONE;
    is_nearing_other_lane_merge_to_road_point = false;
    is_on_highway = false;
    split_seg_forward_lane_nums = 0;
    split_next_seg_forward_lane_nums = 0;
    lc_nums_for_split = 0;
    last_split_seg_dir = RAMP_NONE;
    is_continuous_ramp = false;
    first_split_dir_dis_info = std::make_pair(None, NL_NMAX);
    current_segment_passed_distance = 0.0;
    is_nearing_ramp = false;
    cur_seg_forward_lane_num = 0;
  }
};

struct SplitSegInfo {
  RampDirection split_direction;
  int split_seg_forward_lane_nums;
  int split_next_seg_forward_lane_nums;
};
class RouteInfo {
 public:
  RouteInfo(const EgoPlanningConfigBuilder *config_builder,
            planning::framework::Session *session);
  ~RouteInfo() = default;
  void Update();
  const RouteInfoOutput& get_route_info_output() { return route_info_output_; }

 private:
  const planning::framework::Session *session_ = nullptr;
  RouteInfoOutput route_info_output_;
  const LocalView *local_view_ = nullptr;
  uint64_t static_map_info_updated_timestamp_ = 0;
  uint64_t sd_map_info_updated_timestamp_ = 0;

  bool hdmap_valid_{false};
  bool sdmap_valid_{false};
  bool is_accumulate_dis_to_last_merge_point_more_than_threshold_ = false;
  const double dis_threshold_to_last_merge_point_ = 600.0;
  const double dis_threshold_to_is_merged_point_ = 800.0;

  void UpdateRouteInfoForNOA(const ad_common::sdmap::SDMap& sdmap);
  void UpdateRouteInfoForHPP(const ad_common::hdmap::HDMap& hdmap);

  bool UpdateSdMap(const LocalView &local_view, ad_common::sdmap::SDMap* sd_map);
  bool UpdateStaticMap(const LocalView &local_view, ad_common::hdmap::HDMap* hd_map);

  bool UpdateEgoSegmentInfo(const ad_common::sdmap::SDMap& sd_map, const SdMapSwtx::Segment** segment, double* nearest_s);

  void CaculateRampInfo(const ad_common::sdmap::SDMap& sdmap, const SdMapSwtx::Segment& segment,
                        const double nearest_s, const double max_search_length);
  void CaculateMergeInfo(const ad_common::sdmap::SDMap& sdmap, const SdMapSwtx::Segment& segment,
                        const double nearest_s, const double max_search_length);
  void CaculateSplitInfo(const ad_common::sdmap::SDMap& sdmap, const SdMapSwtx::Segment& segment,
                        const double nearest_s, const double max_search_length);
  void CaculateDistanceToLastMergePoint(const ad_common::sdmap::SDMap& sdmap, const SdMapSwtx::Segment& segment,
                        const double nearest_s, const double max_search_length);
  void CaculateDistanceToLastSplitPoint(const ad_common::sdmap::SDMap& sdmap, const SdMapSwtx::Segment& segment,
                        const double nearest_s, const double max_search_length);
  void CaculateDistanceToRoadEnd(const ad_common::sdmap::SDMap& sdmap, const SdMapSwtx::Segment& segment,
                        const double nearest_s, const double max_search_length);
  void CaculateDistanceToTollStation(const ad_common::sdmap::SDMap& sdmap, const SdMapSwtx::Segment& segment,
                        const double nearest_s, const double max_search_length);
  SplitSegInfo MakesureSplitDirection(
      const ::SdMapSwtx::Segment &split_segment,
      const ad_common::sdmap::SDMap &sd_map);
  RampDirection MakesureMergeDirection(
      const ::SdMapSwtx::Segment &merge_segment,
      const ad_common::sdmap::SDMap &sd_map);
  void UpdateMLCInfoDecider(std::vector<std::shared_ptr<VirtualLane>> relative_id_lanes);
};
}