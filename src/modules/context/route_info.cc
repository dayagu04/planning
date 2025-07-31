#include "route_info.h"
#include <linux/limits.h>
#include <cmath>
#include <cstddef>
#include <memory>
#include <vector>
#include "common_platform_type_soc.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "define/geometry.h"
#include "environmental_model.h"
#include "planning_context.h"

namespace planning {

namespace {
constexpr double kEpsilon = 1.0e-4;
}  // namespace

namespace {
constexpr uint64_t kStaticMapOvertimeThreshold = 20000000;  // 20s
}  // namespace
RouteInfo::RouteInfo(const EgoPlanningConfigBuilder* config_builder,
                     planning::framework::Session* session) {
  session_ = session;
  SetConfig(config_builder);
  mlc_decider_config_ = config_builder->cast<MLCDeciderConfig>();
}

void RouteInfo::SetConfig(const EgoPlanningConfigBuilder* config_builder) {
  config_ = config_builder->cast<EgoPlanningConfig>();
  virtual_extend_buff_ = config_.raw_ref_extend_buff;
}

void RouteInfo::Update() {
  route_info_output_.reset();

  local_view_ = session_->environmental_model().get_local_view();
  if (local_view_.localization.status.status_info.mode ==
      iflyauto::IFLYStatusInfoMode::IFLY_STATUS_INFO_MODE_ERROR) {
    std::cout << "localization invalid" << std::endl;
    return;
  }

  bool is_hpp_scene = session_->is_hpp_scene();
  if (!is_hpp_scene) {
    // if (UpdateSdMap(local_view_)) {
    //   UpdateRouteInfoForNOA(sd_map_);
    // } else {
    //   std::cout << "UpdateSdMap failed!!!" << std::endl;
    // }
    if (UpdateSdProMap(local_view_) && UpdateSdMap(local_view_)) {
      UpdateRouteInfoForNOA(sdpro_map_);
    } else {
      std::cout << "UpdateSdMap failed!!!" << std::endl;
    }
  } else {
    if (UpdateStaticMap(local_view_)) {
      UpdateRouteInfoForHPP(hd_map_);
    } else {
      std::cout << "UpdateHdMap failed!!!" << std::endl;
    }
  }
}

void RouteInfo::UpdateRouteInfoForNOA(const ad_common::sdmap::SDMap& sd_map) {
  double nearest_s;
  const double max_search_length = 7000.0;  // 搜索7km范围内得地图信息
  const SdMapSwtx::Segment* segment = UpdateEgoSegmentInfo(sd_map, &nearest_s);
  if (!segment) {
    std::cout << "update ego segment info failed!!!" << std::endl;
    return;
  }
  const SdMapSwtx::Segment& current_segment = *segment;

  //计算ramp信息
  CaculateRampInfo(sd_map, current_segment, nearest_s, max_search_length);

  // 计算merge信息
  CaculateMergeInfo(sd_map, current_segment, nearest_s, max_search_length);

  // 计算split信息
  CaculateSplitInfo(sd_map, current_segment, nearest_s, max_search_length);

  // 计算距离上一个merge点的信息
  CaculateDistanceToLastMergePoint(sd_map, current_segment, nearest_s,
                                   max_search_length);

  // 计算距离上一个split点的信息
  CaculateDistanceToLastSplitPoint(sd_map, current_segment, nearest_s,
                                   max_search_length);

  // 计算到路线终点的距离
  CaculateDistanceToRoadEnd(sd_map, current_segment, nearest_s,
                            max_search_length);

  // 计算到最近收费站的距离
  CaculateDistanceToTollStation(sd_map, current_segment, nearest_s,
                                max_search_length);
}

void RouteInfo::UpdateRouteInfoForNOA(
    const ad_common::sdpromap::SDProMap& sdpro_map) {
  double nearest_s;
  const double max_search_length = 7000.0;  // 搜索7km范围内得地图信息

  const SdMapSwtx::Segment* segment = UpdateEgoSegmentInfo(sd_map_, &nearest_s);
  if (segment == nullptr) {
    std::cout << "ego link not in expressway failed!!!" << std::endl;
    return;
  }

  const iflymapdata::sdpro::LinkInfo_Link* link =
      UpdateEgoLinkInfo(sdpro_map, &nearest_s);
  if (!link) {
    std::cout << "update ego link info failed!!!" << std::endl;
    return;
  }
  const iflymapdata::sdpro::LinkInfo_Link& current_link = *link;

  //计算ramp信息
  // CaculateRampInfo(sdpro_map, current_link, nearest_s, max_search_length);

  // 计算merge信息
  CaculateMergeInfo(sdpro_map, current_link, nearest_s, max_search_length);

  // 计算split信息
  CaculateSplitInfo(sdpro_map, current_link, nearest_s, max_search_length);

  // 计算距离上一个merge点的信息
  if (mlc_decider_route_info_.is_triggle_cal_dis_to_last_merge_point) {
    CaculateDistanceToLastMergePoint(sdpro_map, current_link, nearest_s,
                                    max_search_length);
  }
  // 计算距离上一个split点的信息
  if (mlc_decider_route_info_.ego_status_on_route == IN_EXCHANGE_AREAR_FRONT ||
        mlc_decider_route_info_.ego_status_on_route == IN_EXCHANGE_AREAR_REAR) {
    CaculateDistanceToLastSplitPoint(sdpro_map, current_link, nearest_s,
                                    max_search_length);
  }

  CaculateDistanceToLastSplitPoint(sdpro_map, current_link, nearest_s,
                                   max_search_length);

  // 计算到路线终点的距离
  CaculateDistanceToRoadEnd(sdpro_map, current_link, nearest_s,
                            max_search_length);

  // 计算到最近收费站的距离
  CaculateDistanceToTollStation(sdpro_map, current_link, nearest_s,
                                max_search_length);
}

void RouteInfo::UpdateRouteInfoForHPP(const ad_common::hdmap::HDMap& hd_map) {
  ILOG_DEBUG << "session_->is_hpp_scene():", session_->is_hpp_scene();
  if (!GetCurrentNearestLane()) {
    std::cout << "GetCurrentNearestLane failed!!!" << std::endl;
    return;
  }
  CalculateHPPInfo();
  CalculateDistanceToTraceEnd();
  CalculateDistanceToNextSpeedBump();
  return;
}

void RouteInfo::CaculateRampInfo(const ad_common::sdmap::SDMap& sd_map,
                                 const SdMapSwtx::Segment& segment,
                                 const double nearest_s,
                                 const double max_search_length) {
  // 计算ramp信息
  const auto& ramp_info =
      sd_map.GetRampInfo(segment.id(), nearest_s, max_search_length);
  if (ramp_info.second > 0) {
    route_info_output_.dis_to_ramp = ramp_info.second;
    const auto previous_seg =
        sd_map.GetPreviousRoadSegment(ramp_info.first->id());
    if (previous_seg) {
      SplitSegInfo split_seg_info;
      split_seg_info = MakesureSplitDirection(*previous_seg, sd_map);
      route_info_output_.ramp_direction = split_seg_info.split_direction;
    } else {
      std::cout << "previous_seg is nullprt!!!!!" << std::endl;
      route_info_output_.ramp_direction = RAMP_NONE;
    }
  }
}

// void RouteInfo::CaculateRampInfo(const ad_common::sdpromap::SDProMap& sdpro_map,
//                                  const iflymapdata::sdpro::LinkInfo_Link& link,
//                                  const double nearest_s,
//                                  const double max_search_length) {
//   // 计算ramp信息
//   const auto& ramp_info =
//       sdpro_map.GetRampInfo(link.id(), nearest_s, max_search_length);
//   if (ramp_info.second > 0) {
//     route_info_output_.dis_to_ramp = ramp_info.second;
//     auto previous_seg =
//         sdpro_map.GetPreviousLinkOnRoute(ramp_info.first->id());

//     if (!previous_seg) {
//       return;
//     }

//     SplitSegInfo split_seg_info;
//     split_seg_info = MakesureSplitDirection(*previous_seg, sdpro_map);
//     route_info_output_.ramp_direction = split_seg_info.split_direction;

//     const auto split_region_lane_tupo_info = CalculateSplitRegionLaneTupoInfo(
//         *previous_seg, sdpro_map);

//     route_info_output_.ramp_region_info = split_region_lane_tupo_info;

//     route_info_output_.ramp_region_info.distance_to_split_point = ramp_info.second;
//     route_info_output_.ramp_region_info.split_direction =
//         static_cast<SplitDirection>(split_seg_info.split_direction);
//   }
// }

SplitSegInfo RouteInfo::MakesureSplitDirection(
    const ::SdMapSwtx::Segment& split_segment,
    const ad_common::sdmap::SDMap& sd_map) {
  const auto out_link_size = split_segment.out_link_size();
  SplitSegInfo split_seg_info;
  split_seg_info.split_direction = RAMP_NONE;
  split_seg_info.split_next_seg_forward_lane_nums = 0;
  split_seg_info.split_seg_forward_lane_nums = 0;
  const auto& out_link = split_segment.out_link();
  // fengwang31(TODO):暂时假设在匝道上的分叉口只有两个方向
  if (out_link_size == 2) {
    const auto split_next_segment =
        sd_map.GetNextRoadSegment(split_segment.id());
    if (!split_next_segment) {
      std::cout << "out segment is nullptr!!!!!!!!" << std::endl;
      return split_seg_info;
    }
    ad_common::math::Vec2d segment_in_route_dir_vec;
    ad_common::math::Vec2d segment_not_in_route_dir_vec;
    Point2D anchor_point_of_cur_seg_to_next_seg = {
        split_segment.enu_points().rbegin()->x(),
        split_segment.enu_points().rbegin()->y()};

    auto other_segment = out_link[0].id() == split_next_segment->id()
                             ? out_link[1]
                             : out_link[0];
    split_seg_info.split_seg_forward_lane_nums =
        split_segment.forward_lane_num();
    split_seg_info.split_next_seg_forward_lane_nums =
        split_next_segment->forward_lane_num();
    // const auto other_segment = sd_map.GetRoadSegmentById(other_segment_id);
    const auto& split_next_segment_enu_point = split_next_segment->enu_points();
    const auto& other_segment_enu_point = other_segment.enu_points();
    if (split_next_segment_enu_point.size() > 1 &&
        other_segment_enu_point.size() > 1) {
      segment_in_route_dir_vec.set_x(split_next_segment_enu_point[1].x() -
                                     anchor_point_of_cur_seg_to_next_seg.x);
      segment_in_route_dir_vec.set_y(split_next_segment_enu_point[1].y() -
                                     anchor_point_of_cur_seg_to_next_seg.y);
      segment_not_in_route_dir_vec.set_x(other_segment_enu_point[1].x() -
                                         anchor_point_of_cur_seg_to_next_seg.x);
      segment_not_in_route_dir_vec.set_y(other_segment_enu_point[1].y() -
                                         anchor_point_of_cur_seg_to_next_seg.y);
      if (segment_in_route_dir_vec.CrossProd(segment_not_in_route_dir_vec) >
          0.0) {
        split_seg_info.split_direction = RampDirection::RAMP_ON_RIGHT;
      } else {
        split_seg_info.split_direction = RampDirection::RAMP_ON_LEFT;
      }
    } else {
      std::cout << "enu points error!!!!!!!!!!" << std::endl;
    }
  } else {
    std::cout << "out_link_size != 2!!!!!!!1" << std::endl;
  }
  return split_seg_info;
}

SplitSegInfo RouteInfo::MakesureSplitDirection(
    const iflymapdata::sdpro::LinkInfo_Link& split_link,
    const ad_common::sdpromap::SDProMap& sdpro_map) {
  const auto out_link_size = split_link.successor_link_ids().size();
  SplitSegInfo split_seg_info;
  split_seg_info.split_direction = RAMP_NONE;
  split_seg_info.split_next_seg_forward_lane_nums = 0;
  split_seg_info.split_seg_forward_lane_nums = 0;

  const auto& out_link = split_link.successor_link_ids();
  // fengwang31(TODO):暂时假设在匝道上的分叉口只有两个方向
  if (out_link_size == 2) {
    const auto split_next_link = sdpro_map.GetNextLinkOnRoute(split_link.id());
    if (!split_next_link) {
      std::cout << "out segment is nullptr!!!!!!!!" << std::endl;
      return split_seg_info;
    }
    ad_common::math::Vec2d segment_in_route_dir_vec;
    ad_common::math::Vec2d segment_not_in_route_dir_vec;
    Point2D anchor_point_of_cur_seg_to_next_seg = {
        split_link.points().boot().points().rbegin()->x(),
        split_link.points().boot().points().rbegin()->y()};

    auto other_link_id =
        out_link[0] == split_next_link->id() ? out_link[1] : out_link[0];
    const auto& other_link = sdpro_map.GetLinkOnRoute(other_link_id);
    if (other_link == nullptr) {
      return split_seg_info;
    }

    split_seg_info.split_seg_forward_lane_nums = split_link.lane_num();
    split_seg_info.split_next_seg_forward_lane_nums =
        split_next_link->lane_num();
    // const auto other_segment = sd_map.GetRoadSegmentById(other_segment_id);
    const auto& split_next_segment_enu_point =
        split_next_link->points().boot().points();
    const auto& other_segment_enu_point = other_link->points().boot().points();
    if (split_next_segment_enu_point.size() > 1 &&
        other_segment_enu_point.size() > 1) {
      segment_in_route_dir_vec.set_x(split_next_segment_enu_point[1].x() -
                                     anchor_point_of_cur_seg_to_next_seg.x);
      segment_in_route_dir_vec.set_y(split_next_segment_enu_point[1].y() -
                                     anchor_point_of_cur_seg_to_next_seg.y);
      segment_not_in_route_dir_vec.set_x(other_segment_enu_point[1].x() -
                                         anchor_point_of_cur_seg_to_next_seg.x);
      segment_not_in_route_dir_vec.set_y(other_segment_enu_point[1].y() -
                                         anchor_point_of_cur_seg_to_next_seg.y);
      if (segment_in_route_dir_vec.CrossProd(segment_not_in_route_dir_vec) >
          0.0) {
        split_seg_info.split_direction = RampDirection::RAMP_ON_RIGHT;
      } else {
        split_seg_info.split_direction = RampDirection::RAMP_ON_LEFT;
      }
    } else {
      std::cout << "enu points error!!!!!!!!!!" << std::endl;
    }
  } else if (out_link_size == 3) {
    const auto split_next_link = sdpro_map.GetNextLinkOnRoute(split_link.id());
    if (!split_next_link) {
      std::cout << "out segment is nullptr!!!!!!!!" << std::endl;
      return split_seg_info;
    }

    std::vector<uint64> other_link_ids;
    other_link_ids.reserve(2);

    for (int i = 0; i < out_link.size(); ++i) {
      if (out_link[i] != split_next_link->id()) {
        other_link_ids.emplace_back(out_link[i]);
      }
    }

    const auto& other_link1 = sdpro_map.GetLinkOnRoute(other_link_ids[0]);
    const auto& other_link2 = sdpro_map.GetLinkOnRoute(other_link_ids[1]);
    if (other_link1 == nullptr ||
        other_link2 == nullptr ||
        other_link1->points().boot().points().size() < 2 ||
        other_link2->points().boot().points().size() < 2) {
      return split_seg_info;
    }

    // 目标车道的point
    Point2D O{split_next_link->points().boot().points()[0].x(),
              split_next_link->points().boot().points()[0].y()};
    Point2D L{split_next_link->points().boot().points()[1].x(),
              split_next_link->points().boot().points()[1].y()};

    // 另外两条link的point；
    Point2D other1{other_link1->points().boot().points()[1].x(),
                   other_link1->points().boot().points()[1].y()};
    Point2D other2{other_link2->points().boot().points()[1].x(),
                   other_link2->points().boot().points()[1].y()};

    double OL = CalculateAngle(O, L);
    double Oother1 = CalculateAngle(O, other1);
    double Oother2 = CalculateAngle(O, other2);

    std::vector<RayInfo> rays = {{'A', OL}, {'B', Oother1}, {'C', Oother2}};

    const auto& result = SortRaysByDirection(rays);

    for (int i = 0; i < result.size(); i++) {
      if (rays[i].name == 'A') {
        if (i == 0) {
          split_seg_info.split_direction = RAMP_ON_RIGHT;
        } else if (i == 1) {
          split_seg_info.split_direction = RAMP_ON_MIDDLE;
        } else if (i == 2) {
          split_seg_info.split_direction = RAMP_ON_LEFT;
        }
      }
    }
  } else{
    std::cout << "out_link_size != 2!!!!!!!1" << std::endl;
  }
  return split_seg_info;
}

void RouteInfo::CaculateMergeInfo(const ad_common::sdmap::SDMap& sd_map,
                                  const SdMapSwtx::Segment& segment,
                                  const double nearest_s,
                                  const double max_search_length) {
  const auto& merge_info =
      sd_map.GetMergeInfoList(segment.id(), nearest_s, max_search_length);
  if (!merge_info.empty()) {
    const auto seg_of_first_road_merge = merge_info.begin()->first;
    const auto next_seg_of_first_road_merge =
        sd_map.GetNextRoadSegment(merge_info.begin()->first->id());
    int traverse_num = 0;
    bool is_find_first_merge_onfo = false;
    for (int i = 0; i < merge_info.size(); i++) {
      const auto& merge_info_temp = merge_info[i];
      if (merge_info_temp.second > kEpsilon) {
        const auto& merge_seg = merge_info_temp.first;
        if (!merge_seg) {
          break;
        }
        const auto& merge_seg_last_seg =
            sd_map.GetPreviousRoadSegment(merge_seg->id());
        if (!merge_seg_last_seg) {
          break;
        }

        if (!is_find_first_merge_onfo) {
          if (merge_seg_last_seg->usage() == SdMapSwtx::RoadUsage::RAMP &&
              merge_seg->usage() != SdMapSwtx::RoadUsage::RAMP &&
              route_info_output_.is_on_ramp) {
            route_info_output_.is_ramp_merge_to_road_on_expressway = true;
          }
          if (merge_seg_last_seg->usage() != SdMapSwtx::RoadUsage::RAMP &&
              merge_seg->usage() != SdMapSwtx::RoadUsage::RAMP &&
              !route_info_output_.is_on_ramp &&
              route_info_output_.is_ego_on_expressway) {
            route_info_output_.is_road_merged_by_other_lane = true;
          }
          if (merge_seg_last_seg->usage() == SdMapSwtx::RoadUsage::RAMP &&
              merge_seg->usage() == SdMapSwtx::RoadUsage::RAMP &&
              route_info_output_.is_on_ramp) {
            route_info_output_.is_ramp_merge_to_ramp_on_expressway = true;
          }
          route_info_output_.first_merge_direction =
              MakesureMergeDirection(*merge_seg, sd_map);
          route_info_output_.distance_to_first_road_merge =
              merge_info_temp.second;
          route_info_output_.merge_seg_forward_lane_nums =
              merge_seg->forward_lane_num();
          route_info_output_.merge_last_seg_forward_lane_nums =
              merge_seg_last_seg->forward_lane_num();
          is_find_first_merge_onfo = true;
          traverse_num++;
        } else if (is_find_first_merge_onfo) {
          route_info_output_.second_merge_direction =
              MakesureMergeDirection(*merge_seg, sd_map);
          route_info_output_.distance_to_second_road_merge =
              merge_info_temp.second;
          traverse_num++;
        }

        if (traverse_num >= 2) {
          break;
        }
      } else {
        continue;
      }
    }

    if (next_seg_of_first_road_merge != nullptr) {
      if (seg_of_first_road_merge->usage() == SdMapSwtx::RoadUsage::RAMP &&
          next_seg_of_first_road_merge->usage() == SdMapSwtx::RoadUsage::RAMP) {
        route_info_output_.is_continuous_ramp = true;
      }
    }
  } else {
    route_info_output_.distance_to_first_road_merge = NL_NMAX;
    std::cout << "merge_info.empty()!!!!!!!" << std::endl;
  }
}

void RouteInfo::CaculateMergeInfo(
    const ad_common::sdpromap::SDProMap& sdpro_map,
    const iflymapdata::sdpro::LinkInfo_Link& link, const double nearest_s,
    const double max_search_length) {
  const auto& merge_info =
      sdpro_map.GetMergeInfoList(link.id(), nearest_s, max_search_length);
  if (!merge_info.empty()) {
    const auto seg_of_first_road_merge = merge_info.begin()->first;
    const auto next_seg_of_first_road_merge =
        sdpro_map.GetNextLinkOnRoute(merge_info.begin()->first->id());
    int traverse_num = 0;
    bool is_find_first_merge_onfo = false;
    for (int i = 0; i < merge_info.size(); i++) {
      const auto& merge_info_temp = merge_info[i];
      if (merge_info_temp.second > kEpsilon) {
        const auto& merge_seg = merge_info_temp.first;
        if (!merge_seg) {
          break;
        }
        const auto& merge_seg_last_seg =
            sdpro_map.GetPreviousLinkOnRoute(merge_seg->id());
        if (!merge_seg_last_seg) {
          break;
        }

        if (!is_find_first_merge_onfo) {
          bool is_road_merged_by_other_lane = false;
          if (sdpro_map.isRamp(merge_seg_last_seg->link_type()) &&
              !sdpro_map.isRamp(merge_seg->link_type()) &&
              route_info_output_.is_on_ramp) {
            route_info_output_.is_ramp_merge_to_road_on_expressway = true;
          }
          if (!sdpro_map.isRamp(merge_seg_last_seg->link_type()) &&
              !sdpro_map.isRamp(merge_seg->link_type()) &&
              !route_info_output_.is_on_ramp &&
              route_info_output_.is_ego_on_expressway) {
            route_info_output_.is_road_merged_by_other_lane = true;
            is_road_merged_by_other_lane = true;
          }
          if (sdpro_map.isRamp(merge_seg_last_seg->link_type()) &&
              sdpro_map.isRamp(merge_seg->link_type()) &&
              route_info_output_.is_on_ramp) {
            route_info_output_.is_ramp_merge_to_ramp_on_expressway = true;
          }

          route_info_output_.first_merge_direction =
              MakesureMergeDirection(*merge_seg, sdpro_map);
          route_info_output_.distance_to_first_road_merge =
              merge_info_temp.second;
          route_info_output_.merge_seg_forward_lane_nums =
              merge_seg->lane_num();
          route_info_output_.merge_last_seg_forward_lane_nums =
              merge_seg_last_seg->lane_num();
          is_find_first_merge_onfo = true;
          traverse_num++;

          NOASplitRegionInfo first_merge_region_lane_tupo_info;
          const auto merge_region_lane_tupo_info =
              CalculateMergeRegionLaneTupoInfo(
                  *seg_of_first_road_merge, sdpro_map);
          if (merge_region_lane_tupo_info.is_valid) {
            first_merge_region_lane_tupo_info = merge_region_lane_tupo_info;

            first_merge_region_lane_tupo_info.distance_to_split_point =
                merge_info_temp.second;

            first_merge_region_lane_tupo_info.split_direction =
                static_cast<SplitDirection>(
                    route_info_output_.first_merge_direction);

            first_merge_region_lane_tupo_info.is_other_merge_to_road =
                is_road_merged_by_other_lane;

            route_info_output_.merge_region_info_list.emplace_back(
                first_merge_region_lane_tupo_info);
          }

        } else if (is_find_first_merge_onfo) {
          route_info_output_.second_merge_direction =
              MakesureMergeDirection(*merge_seg, sdpro_map);
          route_info_output_.distance_to_second_road_merge =
              merge_info_temp.second;
          traverse_num++;

          NOASplitRegionInfo second_merge_region_lane_tupo_info;
          const auto merge_region_lane_tupo_info =
              CalculateMergeRegionLaneTupoInfo(
                  *merge_seg, sdpro_map);
          if (merge_region_lane_tupo_info.is_valid) {
            second_merge_region_lane_tupo_info = merge_region_lane_tupo_info;
            second_merge_region_lane_tupo_info.distance_to_split_point =
                merge_info_temp.second;
            second_merge_region_lane_tupo_info.split_direction =
                static_cast<SplitDirection>(route_info_output_.second_merge_direction);
            route_info_output_.merge_region_info_list.emplace_back(
                second_merge_region_lane_tupo_info);
          }

        }

        if (traverse_num >= 2) {
          break;
        }
      } else {
        continue;
      }
    }

    if (next_seg_of_first_road_merge != nullptr) {
      if (sdpro_map.isRamp(seg_of_first_road_merge->link_type()) &&
          sdpro_map.isRamp(next_seg_of_first_road_merge->link_type())) {
        route_info_output_.is_continuous_ramp = true;
      }
    }
  } else {
    route_info_output_.distance_to_first_road_merge = NL_NMAX;
    std::cout << "merge_info.empty()!!!!!!!" << std::endl;
  }
}

void RouteInfo::CaculateSplitInfo(const ad_common::sdmap::SDMap& sd_map,
                                  const SdMapSwtx::Segment& segment,
                                  const double nearest_s,
                                  const double max_search_length) {
  route_info_output_.first_split_dir_dis_info = std::make_pair(None, NL_NMAX);
  const auto& split_info =
      sd_map.GetSplitInfoList(segment.id(), nearest_s, max_search_length);
  if (!split_info.empty()) {
    bool is_find_first_split_info = false;
    int traverse_num = 0;
    for (int i = 0; i < split_info.size(); i++) {
      const auto split_segment = split_info[i].first;
      if (split_segment && split_info[i].second > 0) {
        if (!is_find_first_split_info) {
          route_info_output_.distance_to_first_road_split =
              split_info[i].second;
          SplitSegInfo split_seg_info;
          split_seg_info = MakesureSplitDirection(*split_segment, sd_map);
          route_info_output_.first_split_direction =
              split_seg_info.split_direction;
          route_info_output_.split_seg_forward_lane_nums =
              split_seg_info.split_seg_forward_lane_nums;
          route_info_output_.split_next_seg_forward_lane_nums =
              split_seg_info.split_next_seg_forward_lane_nums;
          is_find_first_split_info = true;
          traverse_num++;
          route_info_output_.first_split_dir_dis_info =
              std::make_pair(static_cast<SplitRelativeDirection>(
                                 route_info_output_.first_split_direction),
                             route_info_output_.distance_to_first_road_split);
        } else if (is_find_first_split_info) {
          route_info_output_.distance_to_second_road_split =
              split_info[i].second;
          SplitSegInfo split_seg_info;
          split_seg_info = MakesureSplitDirection(*split_segment, sd_map);
          route_info_output_.second_split_direction =
              split_seg_info.split_direction;
          traverse_num++;
        }
        if (traverse_num >= 2) {
          break;
        }
      }
    }
  } else {
    route_info_output_.distance_to_first_road_split = NL_NMAX;
    route_info_output_.first_split_direction = RAMP_NONE;
    std::cout << "split_info.empty()!!!!!!!" << std::endl;
  }

  route_info_output_.split_dir_dis_info_list.clear();
  if (!split_info.empty()) {
    for (int i = 0; i < 2; i++) {
      if (i < split_info.size()) {
        const auto split_segment = split_info[i].first;
        double distance_to_road_split = NL_NMAX;
        RampDirection split_direction = RAMP_NONE;
        std::pair<SplitRelativeDirection, double> split_dir_dis_info;
        if (split_info[i].second > 0 && split_segment) {
          distance_to_road_split = split_info[i].second;
          SplitSegInfo split_seg_info;
          split_seg_info = MakesureSplitDirection(*split_segment, sd_map);
          split_direction = split_seg_info.split_direction;
        } else {
          distance_to_road_split = NL_NMAX;
          split_direction = RAMP_NONE;
        }
        split_dir_dis_info =
            std::make_pair(static_cast<SplitRelativeDirection>(split_direction),
                           distance_to_road_split);
        route_info_output_.split_dir_dis_info_list.emplace_back(
            split_dir_dis_info);
      }
    }
  } else {
    route_info_output_.distance_to_first_road_split = NL_NMAX;
    route_info_output_.first_split_direction = RAMP_NONE;
    std::cout << "split_info.empty()!!!!!!!" << std::endl;
  }
}

void RouteInfo::CaculateSplitInfo(
    const ad_common::sdpromap::SDProMap& sdpro_map,
    const iflymapdata::sdpro::LinkInfo_Link& link, const double nearest_s,
    const double max_search_length) {
  route_info_output_.first_split_dir_dis_info = std::make_pair(None, NL_NMAX);
  const auto& split_info =
      sdpro_map.GetSplitInfoList(link.id(), nearest_s, max_search_length);
  if (!split_info.empty()) {
    bool is_find_first_split_info = false;
    int traverse_num = 0;
    for (int i = 0; i < split_info.size(); i++) {
      const auto split_link = split_info[i].first;
      if (split_link && split_info[i].second > 0) {
        if (!is_find_first_split_info) {
          route_info_output_.distance_to_first_road_split =
              split_info[i].second;
          SplitSegInfo split_seg_info;
          split_seg_info = MakesureSplitDirection(*split_link, sdpro_map);
          route_info_output_.first_split_direction =
              split_seg_info.split_direction;
          route_info_output_.split_seg_forward_lane_nums =
              split_seg_info.split_seg_forward_lane_nums;
          route_info_output_.split_next_seg_forward_lane_nums =
              split_seg_info.split_next_seg_forward_lane_nums;

          NOASplitRegionInfo first_split_region_lane_tupo_info;
          const auto split_region_lane_tupo_info =
              CalculateSplitRegionLaneTupoInfo(
                  *split_link, sdpro_map);
          if (split_region_lane_tupo_info.is_valid) {
            first_split_region_lane_tupo_info = split_region_lane_tupo_info;
            first_split_region_lane_tupo_info.distance_to_split_point =
                split_info[i].second;
            first_split_region_lane_tupo_info.split_direction =
                static_cast<SplitDirection>(split_seg_info.split_direction);
            route_info_output_.split_region_info_list.emplace_back(
                first_split_region_lane_tupo_info);
          }

          is_find_first_split_info = true;
          traverse_num++;
        } else if (is_find_first_split_info) {
          route_info_output_.distance_to_second_road_split =
              split_info[i].second;
          SplitSegInfo split_seg_info;
          split_seg_info = MakesureSplitDirection(*split_link, sdpro_map);
          route_info_output_.second_split_direction =
              split_seg_info.split_direction;
          traverse_num++;

          NOASplitRegionInfo second_split_region_lane_tupo_info;
          const auto split_region_lane_tupo_info =
              CalculateSplitRegionLaneTupoInfo(
                  *split_link, sdpro_map);

          if (split_region_lane_tupo_info.is_valid) {
            second_split_region_lane_tupo_info = split_region_lane_tupo_info;
            second_split_region_lane_tupo_info.distance_to_split_point =
                split_info[i].second;
            second_split_region_lane_tupo_info.split_direction =
                static_cast<SplitDirection>(split_seg_info.split_direction);
            route_info_output_.split_region_info_list.emplace_back(
                second_split_region_lane_tupo_info);
          }
        }
        if (traverse_num >= 2) {
          break;
        }
      }
    }
  } else {
    route_info_output_.distance_to_first_road_split = NL_NMAX;
    route_info_output_.first_split_direction = RAMP_NONE;
    std::cout << "split_info.empty()!!!!!!!" << std::endl;
  }

  const auto& split_region_info_list = route_info_output_.split_region_info_list;
  for (const auto& split_region_info : split_region_info_list) {
    if (split_region_info.is_ramp_split) {
      route_info_output_.dis_to_ramp = split_region_info.distance_to_split_point;
      if (split_region_info.split_direction == SPLIT_LEFT) {
        route_info_output_.ramp_direction = RAMP_ON_LEFT;
      } else if (split_region_info.split_direction == SPLIT_RIGHT) {
        route_info_output_.ramp_direction = RAMP_ON_RIGHT;
      }
      break;
    }
  }
}

void RouteInfo::CaculateDistanceToLastMergePoint(
    const ad_common::sdmap::SDMap& sd_map, const SdMapSwtx::Segment& segment,
    const double nearest_s, const double max_search_length) {
  const SdMapSwtx::Segment* last_merge_seg = &segment;
  is_accumulate_dis_to_last_merge_point_more_than_threshold_ = false;
  double sum_dis_to_last_merge_point = nearest_s;
  route_info_output_.sum_dis_to_last_merge_point = NL_NMAX;
  if (!route_info_output_.is_on_ramp) {
    while (last_merge_seg->in_link().size() == 1) {
      last_merge_seg = sd_map.GetPreviousRoadSegment(last_merge_seg->id());
      // 判断是否为nullptr
      if (!last_merge_seg) {
        break;
      } else {
        sum_dis_to_last_merge_point =
            sum_dis_to_last_merge_point + last_merge_seg->dis();
      }
    }
    if (last_merge_seg && last_merge_seg->in_link().size() == 2) {
      // fengwang31:目前仅针对inlink是2的情况做处理
      const auto& merge_last_seg =
          sd_map.GetPreviousRoadSegment(last_merge_seg->id());
      if (merge_last_seg && merge_last_seg->usage() == SdMapSwtx::RAMP &&
          last_merge_seg->usage() != SdMapSwtx::RAMP) {
        route_info_output_.sum_dis_to_last_merge_point =
            sum_dis_to_last_merge_point;
      }
    }
    if (route_info_output_.sum_dis_to_last_merge_point >
        dis_threshold_to_last_merge_point_) {
      is_accumulate_dis_to_last_merge_point_more_than_threshold_ = true;
    }
  }
}

void RouteInfo::CaculateDistanceToLastMergePoint(
    const ad_common::sdpromap::SDProMap& sdpro_map,
    const iflymapdata::sdpro::LinkInfo_Link& link, const double nearest_s,
    const double max_search_length) {
  const iflymapdata::sdpro::LinkInfo_Link* last_merge_link = &link;
  is_accumulate_dis_to_last_merge_point_more_than_threshold_ = false;
  double sum_dis_to_last_merge_point = nearest_s;
  route_info_output_.sum_dis_to_last_merge_point = NL_NMAX;

  while (last_merge_link->predecessor_link_ids().size() == 1) {
    last_merge_link = sdpro_map.GetPreviousLinkOnRoute(last_merge_link->id());
    // 判断是否为nullptr
    if (!last_merge_link) {
      break;
    } else {
      sum_dis_to_last_merge_point =
          sum_dis_to_last_merge_point + last_merge_link->length() * 0.01;
    }
  }
  if (last_merge_link &&
      last_merge_link->predecessor_link_ids().size() == 2) {
    // fengwang31:目前仅针对inlink是2的情况做处理
    // const auto& merge_last_link =
    //     sdpro_map.GetPreviousLinkOnRoute(last_merge_link->id());
    // if (merge_last_link && sdpro_map.isRamp(merge_last_link->link_type()) &&
    //     !sdpro_map.isRamp(last_merge_link->link_type())) {
    //   route_info_output_.sum_dis_to_last_merge_point =
    //       sum_dis_to_last_merge_point;
    // }
    route_info_output_.sum_dis_to_last_merge_point =
        sum_dis_to_last_merge_point;
  }
}

void RouteInfo::CaculateDistanceToLastSplitPoint(
    const ad_common::sdmap::SDMap& sd_map, const SdMapSwtx::Segment& segment,
    const double nearest_s, const double max_search_length) {
  //计算在主路上距离上一个split点的信息
  const SdMapSwtx::Segment* last_split_seg =
      sd_map.GetPreviousRoadSegment(segment.id());
  double sum_dis_to_last_split_point = nearest_s;
  route_info_output_.sum_dis_to_last_split_point = NL_NMAX;
  if (segment.usage() != SdMapSwtx::RAMP) {
    if (last_split_seg != nullptr) {
      while (last_split_seg->out_link().size() == 1) {
        sum_dis_to_last_split_point =
            sum_dis_to_last_split_point + last_split_seg->dis();
        last_split_seg = sd_map.GetPreviousRoadSegment(last_split_seg->id());
        if (!last_split_seg) {
          break;
        }
      }
      if (last_split_seg && last_split_seg->out_link().size() == 2) {
        if (last_split_seg->usage() != SdMapSwtx::RAMP) {
          route_info_output_.sum_dis_to_last_split_point =
              sum_dis_to_last_split_point;
        }
      }
    }
  }

  // 计算在匝道上距离上一个split点的信息
  const SdMapSwtx::Segment* temp_last_split_seg =
      sd_map.GetPreviousRoadSegment(segment.id());
  double accumulate_dis_ego_to_last_split_point = nearest_s;
  route_info_output_.accumulate_dis_ego_to_last_split_point = NL_NMAX;
  if (temp_last_split_seg) {
    while (temp_last_split_seg->out_link().size() == 1) {
      accumulate_dis_ego_to_last_split_point =
          accumulate_dis_ego_to_last_split_point + temp_last_split_seg->dis();
      temp_last_split_seg =
          sd_map.GetPreviousRoadSegment(temp_last_split_seg->id());
      if (!temp_last_split_seg) {
        break;
      }
    }
    if (temp_last_split_seg && temp_last_split_seg->out_link().size() == 2) {
      route_info_output_.accumulate_dis_ego_to_last_split_point =
          accumulate_dis_ego_to_last_split_point;
      SplitSegInfo split_seg_info;
      split_seg_info = MakesureSplitDirection(*temp_last_split_seg, sd_map);
      route_info_output_.last_split_seg_dir = split_seg_info.split_direction;
    }
  }
}

void RouteInfo::CaculateDistanceToLastSplitPoint(
    const ad_common::sdpromap::SDProMap& sdpro_map,
    const iflymapdata::sdpro::LinkInfo_Link& link, const double nearest_s,
    const double max_search_length) {
  //TODO(fengwang31)：把这两个合并起来
  //计算在主路上距离上一个split点的信息
  // const iflymapdata::sdpro::LinkInfo_Link* last_split_link =
  //     sdpro_map.GetPreviousLinkOnRoute(link.id());
  // double sum_dis_to_last_split_point = nearest_s;
  // route_info_output_.sum_dis_to_last_split_point = NL_NMAX;
  // if (!sdpro_map.isRamp(link.link_type())) {
  //   if (last_split_link != nullptr) {
  //     while (last_split_link->successor_link_ids().size() == 1) {
  //       sum_dis_to_last_split_point =
  //           sum_dis_to_last_split_point + last_split_link->length() * 0.01;
  //       last_split_link =
  //           sdpro_map.GetPreviousLinkOnRoute(last_split_link->id());
  //       if (!last_split_link) {
  //         break;
  //       }
  //     }
  //     if (last_split_link &&
  //         last_split_link->successor_link_ids().size() >= 2) {
  //       if (!sdpro_map.isRamp(last_split_link->link_type())) {
  //         route_info_output_.sum_dis_to_last_split_point =
  //             sum_dis_to_last_split_point;
  //       }
  //     }
  //   }
  // }

  // 计算在匝道上距离上一个split点的信息
  const iflymapdata::sdpro::LinkInfo_Link* temp_last_split_seg =
      sdpro_map.GetPreviousLinkOnRoute(link.id());
  double accumulate_dis_ego_to_last_split_point = nearest_s;
  route_info_output_.accumulate_dis_ego_to_last_split_point = NL_NMAX;
  if (temp_last_split_seg) {
    while (temp_last_split_seg->successor_link_ids().size() == 1) {
      accumulate_dis_ego_to_last_split_point =
          accumulate_dis_ego_to_last_split_point +
          temp_last_split_seg->length() * 0.01;
      temp_last_split_seg =
          sdpro_map.GetPreviousLinkOnRoute(temp_last_split_seg->id());
      if (!temp_last_split_seg) {
        break;
      }
    }

    if (temp_last_split_seg &&
        temp_last_split_seg->successor_link_ids().size() >= 2) {
      route_info_output_.accumulate_dis_ego_to_last_split_point =
          accumulate_dis_ego_to_last_split_point;
      // SplitSegInfo split_seg_info;
      // split_seg_info = MakesureSplitDirection(*temp_last_split_seg, sdpro_map);
      // route_info_output_.last_split_seg_dir = split_seg_info.split_direction;
    }
  }
}

void RouteInfo::CaculateDistanceToRoadEnd(const ad_common::sdmap::SDMap& sd_map,
                                          const SdMapSwtx::Segment& segment,
                                          const double nearest_s,
                                          const double max_search_length) {
  double dis_to_end = NL_NMAX;
  int result =
      sd_map.GetDistanceToRouteEnd(segment.id(), nearest_s, dis_to_end);
  if (result == 0) {
    route_info_output_.distance_to_route_end = dis_to_end;
  } else {
    route_info_output_.distance_to_route_end = NL_NMAX;
  }
}

void RouteInfo::CaculateDistanceToRoadEnd(
    const ad_common::sdpromap::SDProMap& sdpro_map,
    const iflymapdata::sdpro::LinkInfo_Link& link, const double nearest_s,
    const double max_search_length) {
  double dis_to_end = NL_NMAX;
  int result =
      sdpro_map.GetDistanceToRouteEnd(link.id(), nearest_s, dis_to_end);
  if (result == 0) {
    route_info_output_.distance_to_route_end = dis_to_end;
  } else {
    route_info_output_.distance_to_route_end = NL_NMAX;
  }
}

void RouteInfo::CaculateDistanceToTollStation(
    const ad_common::sdmap::SDMap& sd_map, const SdMapSwtx::Segment& segment,
    const double nearest_s, const double max_search_length) {
  const auto& toll_station_info =
      sd_map.GetTollStationInfo(segment.id(), nearest_s, max_search_length);
  if (toll_station_info.first != nullptr) {
    route_info_output_.distance_to_toll_station = toll_station_info.second;
    route_info_output_.is_exist_toll_station = true;
  } else {
    std::cout << "not find toll station" << std::endl;
    route_info_output_.distance_to_toll_station = NL_NMAX;
    route_info_output_.is_exist_toll_station = false;
  }
}

void RouteInfo::CaculateDistanceToTollStation(
    const ad_common::sdpromap::SDProMap& sdpro_map,
    const iflymapdata::sdpro::LinkInfo_Link& link, const double nearest_s,
    const double max_search_length) {
  const auto& toll_station_info =
      sdpro_map.GetTollStationInfo(link.id(), nearest_s, max_search_length);
  if (toll_station_info.first != nullptr) {
    route_info_output_.distance_to_toll_station = toll_station_info.second;
    route_info_output_.is_exist_toll_station = true;
  } else {
    std::cout << "not find toll station" << std::endl;
    route_info_output_.distance_to_toll_station = NL_NMAX;
    route_info_output_.is_exist_toll_station = false;
  }
}

RampDirection RouteInfo::MakesureMergeDirection(
    const ::SdMapSwtx::Segment& merge_segment,
    const ad_common::sdmap::SDMap& sd_map) {
  const auto in_link_size = merge_segment.in_link_size();
  RampDirection merge_direction = RAMP_NONE;
  const auto& in_link = merge_segment.in_link();
  // fengwang31(TODO):暂时假设在merge处只有两个方向
  if (in_link_size == 2) {
    const auto merge_last_segment =
        sd_map.GetPreviousRoadSegment(merge_segment.id());
    if (!merge_last_segment) {
      std::cout << "in segment is nullptr!!!!!!!!" << std::endl;
      return merge_direction;
    }
    ad_common::math::Vec2d segment_in_route_dir_vec;
    ad_common::math::Vec2d segment_not_in_route_dir_vec;
    Point2D anchor_point_of_cur_seg_to_last_seg = {
        merge_segment.enu_points().begin()->x(),
        merge_segment.enu_points().begin()->y()};

    auto other_segment =
        in_link[0].id() == merge_last_segment->id() ? in_link[1] : in_link[0];
    const auto& merge_last_segment_enu_point = merge_last_segment->enu_points();
    const auto& other_segment_enu_point = other_segment.enu_points();
    const int point_num = merge_last_segment_enu_point.size();
    const int other_point_num = other_segment_enu_point.size();
    if (point_num > 1 && other_point_num > 1) {
      segment_in_route_dir_vec.set_x(
          merge_last_segment_enu_point[point_num - 2].x() -
          anchor_point_of_cur_seg_to_last_seg.x);
      segment_in_route_dir_vec.set_y(
          merge_last_segment_enu_point[point_num - 2].y() -
          anchor_point_of_cur_seg_to_last_seg.y);
      segment_not_in_route_dir_vec.set_x(
          other_segment_enu_point[other_point_num - 2].x() -
          anchor_point_of_cur_seg_to_last_seg.x);
      segment_not_in_route_dir_vec.set_y(
          other_segment_enu_point[other_point_num - 2].y() -
          anchor_point_of_cur_seg_to_last_seg.y);
      if (segment_in_route_dir_vec.CrossProd(segment_not_in_route_dir_vec) >
          0.0) {
        merge_direction = RampDirection::RAMP_ON_LEFT;
      } else {
        merge_direction = RampDirection::RAMP_ON_RIGHT;
      }
    } else {
      std::cout << "enu points error!!!!!!!!!!" << std::endl;
    }
  } else {
    std::cout << "out_link_size != 2!!!!!!!1" << std::endl;
  }
  return merge_direction;
}

RampDirection RouteInfo::MakesureMergeDirection(
    const iflymapdata::sdpro::LinkInfo_Link& merge_link,
    const ad_common::sdpromap::SDProMap& sdpro_map) {
  const auto in_link_size = merge_link.predecessor_link_ids().size();
  RampDirection merge_direction = RAMP_NONE;
  const auto& in_link = merge_link.predecessor_link_ids();
  // fengwang31(TODO):暂时假设在merge处只有两个方向
  if (in_link_size == 2) {
    const auto merge_last_segment =
        sdpro_map.GetPreviousLinkOnRoute(merge_link.id());
    if (!merge_last_segment) {
      std::cout << "in segment is nullptr!!!!!!!!" << std::endl;
      return merge_direction;
    }
    ad_common::math::Vec2d segment_in_route_dir_vec;
    ad_common::math::Vec2d segment_not_in_route_dir_vec;
    Point2D anchor_point_of_cur_seg_to_last_seg = {
        merge_link.points().boot().points().begin()->x(),
        merge_link.points().boot().points().begin()->y()};

    auto other_link_id =
        in_link[0] == merge_last_segment->id() ? in_link[1] : in_link[0];
    const auto& other_link = sdpro_map.GetLinkOnRoute(other_link_id);
    if (other_link == nullptr) {
      return merge_direction;
    }

    const auto& merge_last_segment_enu_point =
        merge_last_segment->points().boot().points();
    const auto& other_segment_enu_point = other_link->points().boot().points();
    const int point_num = merge_last_segment_enu_point.size();
    const int other_point_num = other_segment_enu_point.size();
    if (point_num > 1 && other_point_num > 1) {
      segment_in_route_dir_vec.set_x(
          merge_last_segment_enu_point[point_num - 2].x() -
          anchor_point_of_cur_seg_to_last_seg.x);
      segment_in_route_dir_vec.set_y(
          merge_last_segment_enu_point[point_num - 2].y() -
          anchor_point_of_cur_seg_to_last_seg.y);
      segment_not_in_route_dir_vec.set_x(
          other_segment_enu_point[other_point_num - 2].x() -
          anchor_point_of_cur_seg_to_last_seg.x);
      segment_not_in_route_dir_vec.set_y(
          other_segment_enu_point[other_point_num - 2].y() -
          anchor_point_of_cur_seg_to_last_seg.y);
      if (segment_in_route_dir_vec.CrossProd(segment_not_in_route_dir_vec) >
          0.0) {
        merge_direction = RampDirection::RAMP_ON_LEFT;
      } else {
        merge_direction = RampDirection::RAMP_ON_RIGHT;
      }
    } else {
      std::cout << "enu points error!!!!!!!!!!" << std::endl;
    }
  } else {
    std::cout << "out_link_size != 2!!!!!!!1" << std::endl;
  }
  return merge_direction;
}

const SdMapSwtx::Segment* RouteInfo::UpdateEgoSegmentInfo(
    const ad_common::sdmap::SDMap& sd_map, double* nearest_s) {
  const SdMapSwtx::Segment* segment = nullptr;
  if (!nearest_s) {
    return segment;
  }
  const double max_search_length = 7000.0;  // 搜索7km范围内得地图信息
  const double search_distance = 50.0;
  const double max_heading_diff = PI / 4;
  //获取当前的segment
  ad_common::math::Vec2d current_point;
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto& pose = ego_state->location_enu();
  current_point.set_x(pose.position.x);
  current_point.set_y(pose.position.y);
  double temp_nearest_s = 0;
  double nearest_l = 0;
  const double ego_heading_angle = ego_state->heading_angle();
  const SdMapSwtx::Segment* current_segment = sd_map.GetNearestRoadWithHeading(
      current_point, search_distance, ego_heading_angle, max_heading_diff,
      temp_nearest_s, nearest_l);
  route_info_output_.current_segment_passed_distance = temp_nearest_s;
  ILOG_DEBUG << "current_segment_passed_distance:" << route_info_output_.current_segment_passed_distance;
  if (!current_segment) {
    return segment;
  }

  // debug当前segment的经纬度信息
  if (current_segment->shape_points_size() > 0) {
    const auto& temp_point_LLH = current_segment->shape_points(0);
    std::cout << "lat:" << temp_point_LLH.lat()
              << ",lon:" << temp_point_LLH.lon()
              << ",height:" << temp_point_LLH.height() << std::endl;
    JSON_DEBUG_VALUE("point_LLH_lat", temp_point_LLH.lat());
    JSON_DEBUG_VALUE("point_LLH_lat", temp_point_LLH.lon());
    JSON_DEBUG_VALUE("point_LLH_lat", temp_point_LLH.height());
  }
  JSON_DEBUG_VALUE("current_segment_id", current_segment->id());
  JSON_DEBUG_VALUE("forward_lane_num", current_segment->forward_lane_num());

  //判断自车当前是否在高速或者高架上
  if (current_segment->has_priority()) {
    if (current_segment->priority() == SdMapSwtx::RoadPriority::EXPRESSWAY ||
        current_segment->priority() == SdMapSwtx::RoadPriority::CITY_EXPRESSWAY) {
      route_info_output_.is_ego_on_expressway = true;
      if (current_segment->priority() == SdMapSwtx::RoadPriority::EXPRESSWAY) {
        route_info_output_.is_ego_on_expressway_hmi = true;
      } else {
        route_info_output_.is_ego_on_city_expressway_hmi = true;
      }
    } else {
      std::cout << "current position not in EXPRESSWAY!!!" << std::endl;
      return segment;
    }
  } else {
    std::cout << "update ego link info failed!!!" << std::endl;
    return segment;
  }

  route_info_output_.is_in_sdmaproad = true;
  segment = current_segment;
  *nearest_s = temp_nearest_s;
  route_info_output_.is_on_highway =
      current_segment->priority() == SdMapSwtx::RoadPriority::EXPRESSWAY;
  // route_info_output_.is_on_ramp =
  //     current_segment->usage() == SdMapSwtx::RoadUsage::RAMP;
  // route_info_output_.cur_seg_forward_lane_num =
  //     current_segment->forward_lane_num();
  route_info_output_.is_update_segment_success = true;
  return segment;
}

const iflymapdata::sdpro::LinkInfo_Link* RouteInfo::UpdateEgoLinkInfo(
    const ad_common::sdpromap::SDProMap& sdpro_map, double* nearest_s) {
  const iflymapdata::sdpro::LinkInfo_Link* link = nullptr;
  if (!nearest_s) {
    return link;
  }
  const double max_search_length = 7000.0;  // 搜索7km范围内得地图信息
  const double search_distance = 50.0;
  const double max_heading_diff = PI / 4;
  //获取当前的segment
  ad_common::math::Vec2d current_point;
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto& pose = ego_state->location_enu();
  current_point.set_x(pose.position.x);
  current_point.set_y(pose.position.y);
  double temp_nearest_s = 0;
  double nearest_l = 0;
  const double ego_heading_angle = ego_state->heading_angle();

  const iflymapdata::sdpro::LinkInfo_Link* current_link =
      sdpro_map.GetNearestLinkWithHeading(current_point, search_distance,
                                          ego_heading_angle, max_heading_diff,
                                          temp_nearest_s, nearest_l);
  route_info_output_.current_segment_passed_distance = temp_nearest_s;
  LOG_DEBUG("current_segment_passed_distance:%f\n",
            route_info_output_.current_segment_passed_distance);
  if (!current_link) {
    return link;
  }

  // debug当前segment的经纬度信息
  if (current_link->llh_points().points().size() > 0) {
    const auto& temp_point_LLH = current_link->llh_points().points(0);
    std::cout << "lat:" << temp_point_LLH.lat()
              << ",lon:" << temp_point_LLH.lon()
              << ",height:" << temp_point_LLH.height() << std::endl;
    JSON_DEBUG_VALUE("point_LLH_lat", temp_point_LLH.lat());
    JSON_DEBUG_VALUE("point_LLH_lat", temp_point_LLH.lon());
    JSON_DEBUG_VALUE("point_LLH_lat", temp_point_LLH.height());
  }
  JSON_DEBUG_VALUE("current_segment_id", current_link->id());
  JSON_DEBUG_VALUE("forward_lane_num", current_link->lane_num());

  // //判断自车当前是否在高速或者高架上
  // if (current_link->link_class() ==
  //         iflymapdata::sdpro::LinkClass::LC_EXPRESSWAY ||
  //     current_link->link_class() ==
  //         iflymapdata::sdpro::LinkClass::LC_CITY_EXPRESSWAY) {
  //   route_info_output_.is_ego_on_expressway = true;
  //   if (current_link->link_class() ==
  //       iflymapdata::sdpro::LinkClass::LC_EXPRESSWAY) {
  //     route_info_output_.is_ego_on_expressway_hmi = true;
  //   } else {
  //     route_info_output_.is_ego_on_city_expressway_hmi = true;
  //   }
  // } else {
  //   std::cout << "current position not in EXPRESSWAY!!!" << std::endl;
  //   return link;
  // }

  // route_info_output_.is_in_sdmaproad = true;
  link = current_link;
  *nearest_s = temp_nearest_s;
  // route_info_output_.is_on_highway =
  //     current_link->link_class() ==
  //     iflymapdata::sdpro::LinkClass::LC_EXPRESSWAY;
  route_info_output_.is_on_ramp = sdpro_map.isRamp(current_link->link_type());
  route_info_output_.cur_seg_forward_lane_num = current_link->lane_num();
  route_info_output_.is_update_segment_success = true;
  return link;
}

bool RouteInfo::UpdateSdMap(const LocalView& local_view) {
  const auto sd_map_info_current_timestamp =
      local_view.sd_map_info.header().timestamp();
  if (sd_map_info_current_timestamp != sd_map_info_updated_timestamp_) {
    ad_common::sdmap::SDMap sd_map_temp;
    const int res = sd_map_temp.LoadMapFromProto(local_view.sd_map_info);
    if (res == 0) {
      sd_map_ = std::move(sd_map_temp);
      sdmap_valid_ = true;
      sd_map_info_updated_timestamp_ = sd_map_info_current_timestamp;
    }
  }
  if (sd_map_info_current_timestamp - sd_map_info_updated_timestamp_ >
      kStaticMapOvertimeThreshold) {
    //距离上一次更新时间超过阈值，则认为无效报错
    sdmap_valid_ = false;
    std::cout << "error!!! because more than 20s no update hdmap!!!"
              << std::endl;
  }
  JSON_DEBUG_VALUE("sdmap_valid_", sdmap_valid_)
  return sdmap_valid_;
}

bool RouteInfo::UpdateSdProMap(const LocalView& local_view) {
  const auto sdpro_map_info_current_timestamp =
      local_view.sdpro_map_info.header().timestamp();
  if (sdpro_map_info_current_timestamp != sdpro_map_info_updated_timestamp_) {
    ad_common::sdpromap::SDProMap sdpro_map_temp;
    int res = sdpro_map_temp.LoadMapFromProto(local_view.sdpro_map_info);
    if (res == 0) {
      sdpro_map_ = std::move(sdpro_map_temp);
      sdpromap_valid_ = true;
      sdpro_map_info_updated_timestamp_ = sdpro_map_info_current_timestamp;
    }
  }
  if (sdpro_map_info_current_timestamp - sdpro_map_info_updated_timestamp_ >
      kStaticMapOvertimeThreshold) {
    //距离上一次更新时间超过阈值，则认为无效报错
    sdpromap_valid_ = false;
    std::cout << "error!!! because more than 20s no update hdmap!!!"
              << std::endl;
  }
  JSON_DEBUG_VALUE("sdpromap_valid_", sdpromap_valid_)
  return sdpromap_valid_;
}

bool RouteInfo::UpdateStaticMap(const LocalView& local_view) {
  const auto static_map_info_current_timestamp =
      local_view.static_map_info.header().timestamp();
  if (static_map_info_current_timestamp != static_map_info_updated_timestamp_) {
    ad_common::hdmap::HDMap hd_map_tmp;
    const int res =
        hd_map_tmp.LoadMapFromProto(local_view.static_map_info.road_map());
    if (res == 0) {
      hd_map_ = std::move(hd_map_tmp);
      hdmap_valid_ = true;
      static_map_info_updated_timestamp_ = static_map_info_current_timestamp;
    }
  }
  if (static_map_info_current_timestamp - static_map_info_updated_timestamp_ >
      kStaticMapOvertimeThreshold) {
    //距离上一次更新时间超过阈值，则认为无效报错
    hdmap_valid_ = false;
    std::cout << "error!!! because more than 20s no update hdmap!!!"
              << std::endl;
  }
  JSON_DEBUG_VALUE("hdmap_valid", hdmap_valid_)
  return hdmap_valid_;
}

void RouteInfo::UpdateMLCInfoDecider(
    std::vector<std::shared_ptr<VirtualLane>> relative_id_lanes) {
  route_info_output_.lane_num_except_emergency = relative_id_lanes.size();
  if (!route_info_output_.is_update_segment_success ||
      route_info_output_.lane_num_except_emergency < 1) {
    return;
  }
  // 判断是否在匝道汇入主路场景：在匝道上接近汇入点100米以内；
  if (route_info_output_.is_ramp_merge_to_road_on_expressway &&
      route_info_output_.distance_to_first_road_merge <
          mlc_decider_config_
              .default_pre_triggle_merge_to_road_distance_threshold_value) {
    route_info_output_.is_leaving_ramp = true;
  }

  //判断是否是正在接近匝道
  const double dis_between_first_road_split_and_ramp =
      route_info_output_.distance_to_first_road_split -
      route_info_output_.dis_to_ramp;
  const double allow_error = 5.0;
  route_info_output_.is_nearing_ramp =
      fabs(dis_between_first_road_split_and_ramp) < allow_error &&
      route_info_output_.dis_to_ramp <
          mlc_decider_config_
              .default_pre_triggle_road_to_ramp_distance_threshold_value;

  //判断哪个场景在前
  if (route_info_output_.is_leaving_ramp &&
      route_info_output_.is_nearing_ramp) {
    if (route_info_output_.distance_to_first_road_merge <
        route_info_output_.dis_to_ramp) {
      // merge在ramp的前面
      route_info_output_.is_nearing_ramp = false;
    } else {
      // ramp在merge的前面
      route_info_output_.is_leaving_ramp = false;
    }
  }

  // 判断ego是否在最右边车道上
  bool is_ego_on_rightest_lane = true;
  bool is_ego_on_leftest_lane = true;
  for (const auto& relative_id_lane : relative_id_lanes) {
    if (relative_id_lane->get_relative_id() > 0) {
      is_ego_on_rightest_lane = false;
    }
    if (relative_id_lane->get_relative_id() < 0) {
      is_ego_on_leftest_lane = false;
    }
  }
  // 为了临时hack处理在匝道延长车道上的case，使得自车能在匝道延长车道上也能变道至lane上。
  // 同时满足以下4个条件则认为，还在匝道延长线上：
  //  1、自车当前不在匝道上；
  //  2、且距离下一个匝道距离在1300m以上,距离上一个merge点在700m以内；
  //  3、当前在最右边车道上；
  //  4、当前是在expressway上。
  if (!route_info_output_.is_on_ramp &&
      route_info_output_.dis_to_ramp > dis_threshold_to_last_merge_point_ &&
      !is_accumulate_dis_to_last_merge_point_more_than_threshold_ &&
      is_ego_on_rightest_lane && route_info_output_.is_on_highway) {
    route_info_output_.is_leaving_ramp = true;
  }
  //这里是hack匝道延长线800m范围内不在最右侧车道，如果也接近匝道了
  //根据到匝道的距离判断是匝道延长线汇入在前还是匝道在前
  if (route_info_output_.is_nearing_ramp &&
      !is_accumulate_dis_to_last_merge_point_more_than_threshold_ &&
      !route_info_output_.is_on_ramp &&
      route_info_output_.dis_to_ramp > dis_threshold_to_last_merge_point_ &&
      route_info_output_.is_on_highway) {
    route_info_output_.is_nearing_ramp = false;
  }
  // fengwang31:临时处理合肥测试路线上右边长匝道反复变道问题，后续需要删除
  if (route_info_output_.is_nearing_ramp &&
      route_info_output_.dis_to_ramp > 1180 &&
      route_info_output_.sum_dis_to_last_merge_point >
          dis_threshold_to_last_merge_point_ &&
      route_info_output_.sum_dis_to_last_merge_point < 810 &&
      route_info_output_.is_on_highway && !route_info_output_.is_on_ramp) {
    route_info_output_.is_nearing_ramp = false;
  }

  //(2)、判断当前在高速主路上是否在接近汇入点，如果接近汇入点800米以内，不让自车呆最右侧车道上，以避开汇流区域
  // fengwang31:增加优先级判断，如果前方汇入距离前方匝道的距离较近，那么避开汇入区域的逻辑被抑制
  bool is_ramp_and_merge_dis_error_more_than_threshold =
      (route_info_output_.dis_to_ramp -
       route_info_output_.distance_to_first_road_merge) > 500;
  bool is_driving_dir_conflict =
      (route_info_output_.first_merge_direction == RAMP_ON_LEFT &&
       route_info_output_.ramp_direction == RAMP_ON_RIGHT) ||
      (route_info_output_.first_merge_direction == RAMP_ON_RIGHT &&
       route_info_output_.ramp_direction == RAMP_ON_LEFT);
  bool is_car_flow_conflict =
      (is_ego_on_rightest_lane &&
       route_info_output_.first_merge_direction == RAMP_ON_LEFT) ||
      (is_ego_on_leftest_lane &&
       route_info_output_.first_merge_direction == RAMP_ON_RIGHT);
  if (route_info_output_.is_road_merged_by_other_lane &&
      (route_info_output_.distance_to_first_road_merge <
       dis_threshold_to_is_merged_point_) &&
      is_car_flow_conflict && is_ramp_and_merge_dis_error_more_than_threshold &&
      route_info_output_.is_on_highway) {
    route_info_output_.is_nearing_other_lane_merge_to_road_point = true;
  }

  //(3)、判断高速前方汇入点在前，还是匝道在前
  if (route_info_output_.is_nearing_ramp &&
      route_info_output_.is_road_merged_by_other_lane &&
      is_ramp_and_merge_dis_error_more_than_threshold &&
      !route_info_output_.is_on_ramp && is_driving_dir_conflict &&
      route_info_output_.is_ego_on_expressway) {
    route_info_output_.is_nearing_ramp = false;
  }

  //(4)、判断在主路上是否接近split区域
  //常规split场景是前方自车需向左行驶，有其他车道从右边向左边汇入自车道。这种场景，仅需要自车不呆在最右侧车道即可
  //特殊split场景：自车前方需向右行驶，且当前仅有一个车道(匝道上)，且在split之前有一个右边的汇入。
  // 此时如果在汇入之前触发split的逻辑，那么可能会错误地变道至merge region
  bool is_nearing_split = false;
  double err_buffer = 10;
  double nearing_split_dis_threshold = 2000;
  if (!route_info_output_.is_on_ramp && !route_info_output_.is_nearing_ramp &&
      route_info_output_.distance_to_first_road_split <
          nearing_split_dis_threshold) {
    is_nearing_split = true;
  }
  //(5)、判断是否需要生成split的变道任务
  if (is_nearing_split) {
    //后面的车道数比当前车道数少一条的sceneray，意味着可能有一条车道从当前分流出去
    if ((route_info_output_.split_seg_forward_lane_nums -
         route_info_output_.split_next_seg_forward_lane_nums) >= 0) {
      if (route_info_output_.first_split_direction == RAMP_ON_LEFT &&
          is_ego_on_rightest_lane) {
        route_info_output_.lc_nums_for_split = -1;
      } else if (route_info_output_.first_split_direction == RAMP_ON_RIGHT &&
                 is_ego_on_leftest_lane) {
        route_info_output_.lc_nums_for_split = 1;
      }
    }
  }
  //(6)、判断当前是否在split的起点后100m范围内；
  bool is_ego_on_split_region = false;
  const double dis_to_next_split_point =
      std::min(route_info_output_.distance_to_first_road_split,
               route_info_output_.dis_to_ramp);
  std::array<double, 2> xp{50, 150};
  std::array<double, 2> fp{0, 60};
  const double split_region_dis_threshold =
      interp(dis_to_next_split_point, xp, fp);
  if (route_info_output_.accumulate_dis_ego_to_last_split_point <
      split_region_dis_threshold) {
    is_ego_on_split_region = true;
  }
  route_info_output_.is_ego_on_split_region = is_ego_on_split_region;
  //(7)、判断当前是否需要在下匝道的分流区域继续生成下匝道的变道请求；
  // TODO:目前仅靠猜测车道数，后续需要根据感知给出的总车道数、当前所处的车道，做更加准确的地图导航变道决策
  // TODO:由于目前仅有左、中、右三条车道，所以暂时只考虑匝道上仅有1条或者2条车道的情况。
  bool is_exit_lane_on_last_ramp_dir =
      route_info_output_.last_split_seg_dir == RAMP_NONE
          ? false
          : (route_info_output_.last_split_seg_dir == RAMP_ON_LEFT
                 ? !is_ego_on_leftest_lane
                 : !is_ego_on_rightest_lane);
  const int cur_seg_forward_lane_num =
      route_info_output_.cur_seg_forward_lane_num;
  const auto& virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto& curret_lane = virtual_lane_manager->get_current_lane();
  const auto& right_lane = virtual_lane_manager->get_right_lane();
  bool is_need_continue_lc_on_off_ramp_region = false;
  //目前仅考虑了从右边下匝道的sceneray
  if (is_exit_lane_on_last_ramp_dir && is_ego_on_split_region &&
      route_info_output_.is_on_ramp) {
    if (cur_seg_forward_lane_num == 1) {
      is_need_continue_lc_on_off_ramp_region = true;
    } else if (cur_seg_forward_lane_num == 2) {
      // TODO:通过判断右边的车道边界线的虚、实，猜测在右边是否还有更多的车道
      if (route_info_output_.last_split_seg_dir == RAMP_ON_RIGHT &&
          right_lane) {
        const auto& right_lane_left_boundary =
            right_lane->get_left_lane_boundary();
        const auto& right_lane_left_boundary_path =
            virtual_lane_manager->MakeBoundaryPath(right_lane_left_boundary);
        const auto& right_lane_right_boundary =
            right_lane->get_right_lane_boundary();
        const auto& right_lane_right_boundary_path =
            virtual_lane_manager->MakeBoundaryPath(right_lane_right_boundary);
        bool right_lane_left_boundary_is_dash_line = right_lane->is_dash_line(
            *session_, LEFT_CHANGE, right_lane_left_boundary_path);
        bool right_lane_right_boundary_is_dash_line = right_lane->is_dash_line(
            *session_, RIGHT_CHANGE, right_lane_right_boundary_path);
        const auto& current_lane_right_boundary =
            curret_lane->get_right_lane_boundary();
        const auto& current_lane_right_boundary_path =
            virtual_lane_manager->MakeBoundaryPath(current_lane_right_boundary);
        bool current_lane_right_boundary_is_dash_line =
            curret_lane->is_dash_line(*session_, RIGHT_CHANGE,
                                      current_lane_right_boundary_path);
        if (current_lane_right_boundary_is_dash_line &&
            right_lane_left_boundary_is_dash_line &&
            right_lane_right_boundary_is_dash_line) {
          is_need_continue_lc_on_off_ramp_region = true;
        }
      }
    }
  }
  //计算需要变道的次数
  int need_continue_lc_num_on_off_ramp_region = 0;
  if (is_need_continue_lc_on_off_ramp_region) {
    need_continue_lc_num_on_off_ramp_region =
        route_info_output_.last_split_seg_dir == RAMP_ON_LEFT ? -1 : 1;
  }
  route_info_output_.need_continue_lc_num_on_off_ramp_region =
      need_continue_lc_num_on_off_ramp_region;
}

void RouteInfo::NewUpdateMLCInfoDecider(
    std::vector<std::shared_ptr<VirtualLane>> relative_id_lanes) {
  if (relative_id_lanes.empty()) {
    mlc_decider_route_info_.reset();
    return;
  }
  bool is_cal_feasible_lane_succeed = true;
  // 分场景处理：1、仅仅只是split；2、split + split；

  // 1、首先判断自车处在哪个状态上，比如接近ramp、split、merge
  // 1、优先判断split，如果触发了split的条件，需要检查在split之前是否需要处理merge
  // 2、如果split为false，则需要检查是否触发merge。
  auto& split_region_info_list = route_info_output_.split_region_info_list;
  auto& merge_region_info_list = route_info_output_.merge_region_info_list;

  if (split_region_info_list.empty() && merge_region_info_list.empty()) {
    mlc_decider_route_info_.reset();
    return;
  }

  const bool is_near_split =
      !split_region_info_list.empty() &&
      split_region_info_list[0].distance_to_split_point <
          mlc_decider_config_
              .default_pre_triggle_road_to_ramp_distance_threshold_value;

  bool is_near_merge = false;
  if (!merge_region_info_list.empty()) {
    if (merge_region_info_list[0].is_other_merge_to_road) {
      if (merge_region_info_list[0].distance_to_split_point < 500) {
        is_near_merge = true;
      }
    } else {
      if (merge_region_info_list[0].distance_to_split_point <
          mlc_decider_config_
              .default_pre_triggle_merge_to_road_distance_threshold_value) {
        is_near_merge = true;
      }
    }
  }

  const int split_num = 2;

  const bool is_two_splits_close =
      !split_region_info_list.empty() &&
      route_info_output_.split_region_info_list.size() >= split_num &&
      (route_info_output_.split_region_info_list[1].distance_to_split_point -
           route_info_output_.split_region_info_list[0]
               .distance_to_split_point <
       mlc_decider_config_.split_split_gap_threshold);

  std::vector<int> on_excr_feasible_lane;
  std::vector<int> before_excr_feasible_lane;

  // 2、判断自车状态
  double cur_dis_to_split =
      split_region_info_list.empty()
          ? NL_NMAX
          : split_region_info_list[0].distance_to_split_point;

  switch (mlc_decider_route_info_.ego_status_on_route) {
    case NEARING_SPLIT: {
      if (split_region_info_list.empty()) {
        mlc_decider_route_info_.reset();
        return;
      }

      bool is_entery_exchange_region =
          split_region_info_list[0].distance_to_split_point <
          std::abs(split_region_info_list[0]
                       .start_fp_point.fp_distance_to_split_point);
      
      //目前都是由右边下匝道的case大多数，由于版本还不稳定，当前优先处理右边split的场景
      bool is_is_entery_split_region = false;
      bool is_split_region = session_->planning_context()
                                        .ego_lane_road_right_decider_output()
                                        .is_split_region;
      if (is_split_region) {
        int split_lane_vitrual_id = session_->planning_context()
                                        .ego_lane_road_right_decider_output()
                                        .split_lane_virtual_id;
        const auto& split_direction = split_region_info_list[0].split_direction;

        const auto& virtual_lane_manager =
            session_->environmental_model().get_virtual_lane_manager();

        if (split_direction == SPLIT_RIGHT) {
          const auto& rlane = virtual_lane_manager->get_right_lane();
          if (rlane) {
            const int rlane_virtual_id = rlane->get_virtual_id();
            if (rlane_virtual_id == split_lane_vitrual_id) {
              is_is_entery_split_region = true;
            }
          }
        }
      }

      //当前先把这个值设为100m，后续可以调整一下
      bool is_triggle_split_region_mlc_threshold =
          split_region_info_list[0].distance_to_split_point <
          mlc_decider_config_.split_region_pre_mlc_threshold;

      bool is_triggle_pre_mlc_in_split_region =
          is_triggle_split_region_mlc_threshold && is_is_entery_split_region &&
          mlc_decider_route_info_.is_process_split;

      if (is_entery_exchange_region ||
          is_triggle_pre_mlc_in_split_region) {
        mlc_decider_route_info_.ego_status_on_route = IN_EXCHANGE_AREAR_FRONT;
        mlc_decider_route_info_.end_fp_dis_to_split =
            split_region_info_list[0].end_fp_point.fp_distance_to_split_point;
      }
      break;
    }
    case NEARING_MERGE: {
      if (merge_region_info_list.empty()) {
        mlc_decider_route_info_.ego_status_on_route = IN_EXCHANGE_AREAR_FRONT;
        mlc_decider_route_info_.last_frame_dis_to_merge_start_point = NL_NMAX;
        mlc_decider_route_info_.is_triggle_cal_dis_to_last_merge_point = true;
        break;
      }

      double dis_to_merge_start_point = 0;
      double dis_to_merge_point = 0;
      dis_to_merge_point = merge_region_info_list[0].distance_to_split_point;

      if (merge_region_info_list[0].start_fp_point.fp_distance_to_split_point >
          kEpsilon) {
        // 意味着fp_start在merge_point的前面，那么应该是自车经过merge点即认为已经进入到交换区了
        dis_to_merge_start_point =
            merge_region_info_list[0].distance_to_split_point;
      } else {
        dis_to_merge_start_point =
            merge_region_info_list[0].distance_to_split_point +
            merge_region_info_list[0].start_fp_point.fp_distance_to_split_point;
      }

      bool is_entery_exchange_region =
          dis_to_merge_start_point >
          mlc_decider_route_info_.last_frame_dis_to_merge_start_point;
      mlc_decider_route_info_.is_triggle_cal_dis_to_last_merge_point =
          dis_to_merge_point >
          mlc_decider_route_info_.last_frame_dis_to_merge_point;

      mlc_decider_route_info_.last_frame_dis_to_merge_start_point =
          dis_to_merge_start_point;
      mlc_decider_route_info_.last_frame_dis_to_merge_point =
          merge_region_info_list[0].distance_to_split_point;

      if (is_entery_exchange_region) {
        mlc_decider_route_info_.ego_status_on_route = IN_EXCHANGE_AREAR_FRONT;
        mlc_decider_route_info_.last_frame_dis_to_merge_start_point = NL_NMAX;
      }
      break;
    }
    case IN_EXCHANGE_AREAR_FRONT: {
      if (mlc_decider_route_info_.is_process_split ||
          mlc_decider_route_info_.is_process_split_split ||
          mlc_decider_route_info_.is_process_other_merge_split) {
        if (cur_dis_to_split >
            mlc_decider_route_info_.last_frame_dis_to_split) {
          mlc_decider_route_info_.ego_status_on_route = IN_EXCHANGE_AREAR_REAR;
        }
      } else if (mlc_decider_route_info_.is_process_merge) {
        if (mlc_decider_route_info_.is_triggle_cal_dis_to_last_merge_point) {
          bool is_triggle_to_ON_MAIN =
              route_info_output_.sum_dis_to_last_merge_point >
              mlc_decider_route_info_.static_merge_region_info.end_fp_point
                  .fp_distance_to_split_point;
          if (is_triggle_to_ON_MAIN) {
            mlc_decider_route_info_.reset();
          }
        } else {
          if (merge_region_info_list.empty()) {
            mlc_decider_route_info_.reset();
          } else {
            double dis_to_merge_point = 0;
            dis_to_merge_point =
                merge_region_info_list[0].distance_to_split_point;

            mlc_decider_route_info_.is_triggle_cal_dis_to_last_merge_point =
                dis_to_merge_point >
                mlc_decider_route_info_.last_frame_dis_to_merge_point;

            mlc_decider_route_info_.last_frame_dis_to_merge_point =
                dis_to_merge_point;
          }
        }
      }
      break;
    }
    case IN_EXCHANGE_AREAR_REAR: {
      if (route_info_output_.accumulate_dis_ego_to_last_split_point >
          mlc_decider_route_info_.end_fp_dis_to_split) {
        mlc_decider_route_info_.reset();
      }
      break;
    }
    case ON_MAIN: {
      if (mlc_decider_route_info_.ego_status_on_route == ON_MAIN) {
        if (is_near_split) {
          if (!merge_region_info_list.empty() &&
              merge_region_info_list[0].distance_to_split_point <
                  split_region_info_list[0].distance_to_split_point) {
            if (is_near_merge) {
              if (merge_region_info_list[0].is_other_merge_to_road) {
                double dis_err =
                    split_region_info_list[0].distance_to_split_point -
                    merge_region_info_list[0].distance_to_split_point;
                if (dis_err > mlc_decider_config_.other_merge_split_gap_threshold) {
                  mlc_decider_route_info_.is_process_other_merge = true;
                } else {
                  if (is_two_splits_close) {
                    mlc_decider_route_info_.is_process_split_split = true;
                  } else {
                    mlc_decider_route_info_.is_process_other_merge_split = true;
                  }
                }
              } else {
                mlc_decider_route_info_.is_process_merge = true;
              }
            }
          } else if (is_two_splits_close) {
            mlc_decider_route_info_.is_process_split_split = true;
          } else {
            mlc_decider_route_info_.is_process_split = true;
          }
        } else if (is_near_merge) {
          if (merge_region_info_list[0].is_other_merge_to_road) {
            mlc_decider_route_info_.is_process_other_merge = true;
          } else {
            mlc_decider_route_info_.is_process_merge = true;
          }
        }
      }

      if (mlc_decider_route_info_.ego_status_on_route == ON_MAIN) {
        if (mlc_decider_route_info_.is_process_split ||
            mlc_decider_route_info_.is_process_other_merge_split) {
          // 处理前方只有1个split的场景
          // exclnum = exchange_arear_lane_num;
          if (split_region_info_list.empty()) {
            mlc_decider_route_info_.reset();
            return;
          }
          auto& first_split_region_info = split_region_info_list[0];

          bool is_calculate_feasible_lane =
              CalculateFeasibleLane(&first_split_region_info);

          if (!is_calculate_feasible_lane) {
            mlc_decider_route_info_.reset();
            return;
          }

          route_info_output_.split_region_info_list[0] =
              first_split_region_info;
          mlc_decider_route_info_.first_static_split_region_info =
              route_info_output_.split_region_info_list[0];

        } else if (mlc_decider_route_info_.is_process_split_split) {
          // 处理前方有2个split的场景
          if (split_region_info_list.size() < 2) {
            mlc_decider_route_info_.reset();
            return;
          }

          auto& first_split_region_info = split_region_info_list[0];
          auto& second_split_region_info = split_region_info_list[1];

          bool is_calculate_first_feasible_lane =
              CalculateFeasibleLane(&first_split_region_info);

          if (!is_calculate_first_feasible_lane) {
            mlc_decider_route_info_.reset();
            return;
          }

          route_info_output_.split_region_info_list[0] =
              first_split_region_info;
          mlc_decider_route_info_.first_static_split_region_info =
              route_info_output_.split_region_info_list[0];
          
          //如果找连续的找不到，那么直接返回处理第一个split的场景

          bool is_calculate_second_feasible_lane =
              CalculateFeasibleLane(&second_split_region_info);

          if (is_calculate_first_feasible_lane &&
              !is_calculate_second_feasible_lane) {
            // 说明计算可行驶车道失败
            is_cal_feasible_lane_succeed = false;
            mlc_decider_route_info_.is_process_split_split = false;
            mlc_decider_route_info_.is_process_split = true;
            mlc_decider_route_info_.ego_status_on_route = NEARING_SPLIT;
            return;
          }

          JSON_DEBUG_VALUE("is_cal_feasible_lane_succeed",
                           is_cal_feasible_lane_succeed);

          const auto& sec_spl_before_feasi_lanes =
              second_split_region_info.recommend_lane_num[0];

          std::vector<int> temp1, temp2, temp3;

          temp1 =
              CommonElements(first_split_region_info.recommend_lane_num[2]
                                 .feasible_lane_sequence,
                             sec_spl_before_feasi_lanes.feasible_lane_sequence);
          if (temp1.empty()) {
            // mlc_decider_route_info_.reset();
            mlc_decider_route_info_.is_process_split_split = false;
            mlc_decider_route_info_.is_process_split = true;
            mlc_decider_route_info_.ego_status_on_route = NEARING_SPLIT;
            return;
          }

          temp2 = CommonElements(first_split_region_info.recommend_lane_num[1]
                                       .feasible_lane_sequence,
                                   temp1);
          if (temp2.empty()) {
            //在第一交换区中间和后面没有共同link时，就靠最右边车道行驶
            mlc_decider_route_info_.is_process_split_split = false;
            mlc_decider_route_info_.is_process_split = true;
            mlc_decider_route_info_.ego_status_on_route = NEARING_SPLIT;
            return;
          }

          temp3 = CommonElements(first_split_region_info.recommend_lane_num[0]
                                       .feasible_lane_sequence,
                                   temp2);
          if (temp3.empty()) {
            mlc_decider_route_info_.is_process_split_split = false;
            mlc_decider_route_info_.is_process_split = true;
            mlc_decider_route_info_.ego_status_on_route = NEARING_SPLIT;
            return;
          }

          route_info_output_.split_region_info_list[1]
              .recommend_lane_num[0]
              .feasible_lane_sequence = temp3;
          route_info_output_.split_region_info_list[0]
              .recommend_lane_num[2]
              .feasible_lane_sequence = temp3;
          route_info_output_.split_region_info_list[0]
              .recommend_lane_num[1]
              .feasible_lane_sequence = temp3;
          route_info_output_.split_region_info_list[0]
              .recommend_lane_num[0]
              .feasible_lane_sequence = temp3;

          mlc_decider_route_info_.first_static_split_region_info =
              route_info_output_.split_region_info_list[0];
        } else if (mlc_decider_route_info_.is_process_other_merge_split) {
          //
        } else if (mlc_decider_route_info_.is_process_other_merge) {
          //
        } else if (mlc_decider_route_info_.is_process_merge) {
          // exclnum = exchange_arear_lane_num;
          if (merge_region_info_list.empty()) {
            mlc_decider_route_info_.reset();
            return;
          }

          auto& first_merge_region_info = merge_region_info_list[0];

          bool is_process_merge_split = false;
          if (!split_region_info_list.empty()) {
            auto& first_split_region_info = split_region_info_list[0];
            const double err = first_split_region_info.distance_to_split_point -
                                first_merge_region_info.distance_to_split_point;

            bool is_satisfiy_dis_condition =
                err > kEpsilon &&
                err < mlc_decider_config_.merge_split_gap_threshold;

            bool is_exist_merge_split_scene =
                is_near_split && !split_region_info_list.empty();

            bool is_satisfy_dir_condition =
                first_merge_region_info.split_direction == SPLIT_RIGHT &&
                first_split_region_info.split_direction == SPLIT_LEFT;

            is_process_merge_split = is_satisfiy_dis_condition &&
                                      is_satisfy_dir_condition &&
                                      is_exist_merge_split_scene;
          }
          // TODO(fengwang31)：暂时先只考虑了右边汇入，左边分叉的case；

          if (is_process_merge_split) {
            auto& first_split_region_info = split_region_info_list[0];
            bool is_calculate_split_feasible_lane =
                CalculateFeasibleLane(&first_split_region_info);

            bool is_calculate_merge_feasible_lane =
                CalculateMergeRegionFeasibleLane(&first_merge_region_info);

            if (!is_calculate_split_feasible_lane ||
                !is_calculate_merge_feasible_lane) {
              mlc_decider_route_info_.reset();
              return;
            }

            route_info_output_.merge_region_info_list[0] =
                first_merge_region_info;

            //后面接着考虑两个区间的交集，继续更新feasible lane
            const auto& spl_before_feasi_lanes =
                first_split_region_info.recommend_lane_num[0];

            std::vector<int> temp1, temp2, temp3;

            temp1 =
            CommonElements(first_merge_region_info.recommend_lane_num[2]
                              .feasible_lane_sequence,
                          spl_before_feasi_lanes.feasible_lane_sequence);

            if (temp1.empty()) {
              mlc_decider_route_info_.reset();
              return;
            }

            temp2 = CommonElements(first_merge_region_info.recommend_lane_num[1]
                                      .feasible_lane_sequence,
                                  temp1);
            if (temp2.empty()) {
              mlc_decider_route_info_.reset();
              return;
            }

            route_info_output_.split_region_info_list[0]
                .recommend_lane_num[0]
                .feasible_lane_sequence = temp2;
            route_info_output_.merge_region_info_list[0]
                .recommend_lane_num[2]
                .feasible_lane_sequence = temp2;
            route_info_output_.merge_region_info_list[0]
                .recommend_lane_num[1]
                .feasible_lane_sequence = temp2;

            mlc_decider_route_info_.static_merge_region_info =
                route_info_output_.merge_region_info_list[0];

          } else {
            bool is_calculate_feasible_lane =
                CalculateMergeRegionFeasibleLane(&first_merge_region_info);

            if (!is_calculate_feasible_lane) {
              mlc_decider_route_info_.reset();
              return;
            }

            route_info_output_.merge_region_info_list[0] =
                first_merge_region_info;
            mlc_decider_route_info_.static_merge_region_info =
                route_info_output_.merge_region_info_list[0];
          }
        }
      }

      if (mlc_decider_route_info_.is_process_split ||
          mlc_decider_route_info_.is_process_split_split ||
          mlc_decider_route_info_.is_process_other_merge_split) {
        mlc_decider_route_info_.ego_status_on_route = NEARING_SPLIT;
      } else if (mlc_decider_route_info_.is_process_merge) {
        mlc_decider_route_info_.ego_status_on_route = NEARING_MERGE;
      }
      break;
    }
  }
  mlc_decider_route_info_.last_frame_dis_to_split = cur_dis_to_split;

  JSON_DEBUG_VALUE(
      "ego_status_on_route",
      static_cast<int>(mlc_decider_route_info_.ego_status_on_route));

  for (auto& relative_id_lane : relative_id_lanes) {
    const auto& lane_nums = relative_id_lane->get_lane_nums();
    // TODO(fengwang31):需要考虑这个车道数的准确性
    int left_lane_num = 0;
    int right_lane_num = 0;
    for (const auto& lane_num : lane_nums) {
      if (lane_num.end > kEpsilon) {
        left_lane_num = lane_num.left_lane_num;
        right_lane_num = lane_num.right_lane_num;
        break;
      }
    }

    std::vector<int> feasible_lane_sequence;

    if (mlc_decider_route_info_.ego_status_on_route ==
            IN_EXCHANGE_AREAR_FRONT ||
        mlc_decider_route_info_.ego_status_on_route == IN_EXCHANGE_AREAR_REAR) {
      if (mlc_decider_route_info_.is_process_merge) {
        feasible_lane_sequence =
            mlc_decider_route_info_.static_merge_region_info
                .recommend_lane_num[1]
                .feasible_lane_sequence;
      } else if (mlc_decider_route_info_.is_process_split ||
                 mlc_decider_route_info_.is_process_split_split ||
                 mlc_decider_route_info_.is_process_other_merge_split) {
        feasible_lane_sequence =
            mlc_decider_route_info_.first_static_split_region_info
                .recommend_lane_num[1]
                .feasible_lane_sequence;
      }
    } else if (mlc_decider_route_info_.ego_status_on_route == NEARING_SPLIT) {
      feasible_lane_sequence =
          mlc_decider_route_info_.first_static_split_region_info
              .recommend_lane_num[0]
              .feasible_lane_sequence;
    } else if (mlc_decider_route_info_.ego_status_on_route == NEARING_MERGE) {
      feasible_lane_sequence =
          mlc_decider_route_info_.static_merge_region_info.recommend_lane_num[0]
              .feasible_lane_sequence;

      std::vector<int> merge_lane;
      if (CalculateMergeLaneInfo(merge_lane) && !merge_lane.empty()) {
        for (int element : merge_lane) {
          auto it = std::find(feasible_lane_sequence.begin(),
                              feasible_lane_sequence.end(), element);

          if (it != feasible_lane_sequence.end()) {
            feasible_lane_sequence.erase(it);
          }
        }
      }

    } else if (mlc_decider_route_info_.ego_status_on_route == ON_MAIN) {
      // feasible_lane_sequence =
      //     split_region_info_list.recommend_lane_num[0].feasible_lane_sequence;
    }

    if (feasible_lane_sequence.empty()) {
      continue;
    }

    int minVal_seq = feasible_lane_sequence[0];
    int maxVal_seq = feasible_lane_sequence[0];
    for (int num : feasible_lane_sequence) {
      if (num < minVal_seq) {
        minVal_seq = num;
      }
      if (num > maxVal_seq) {
        maxVal_seq = num;
      }
    }

    int ego_seq = left_lane_num + 1;
    std::vector<int> lc_num_task;
    if (ego_seq >= minVal_seq && ego_seq <= maxVal_seq) {
      continue;
    } else if (ego_seq > maxVal_seq) {
      int err = ego_seq - maxVal_seq;
      for (int i = 0; i < err; i++) {
        lc_num_task.emplace_back(-1);
      }
    } else if (ego_seq < minVal_seq) {
      int err = minVal_seq - ego_seq;
      for (int i = 0; i < err; i++) {
        lc_num_task.emplace_back(1);
      }
    }

    relative_id_lane->set_current_tasks(lc_num_task);
  }
}

// for HPP function
bool RouteInfo::GetCurrentNearestLane() {
  ad_common::math::Vec2d point;
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  current_pose_ = ego_state->ego_pose_raw();
  point.set_x(current_pose_.x);
  point.set_y(current_pose_.y);
  // get nearest lane
  ad_common::hdmap::LaneInfoConstPtr nearest_lane;
  double nearest_s = 0.0;
  double nearest_l = 0.0;
  double sum_s = 0.0;
  // const double distance = 10.0;
  // const double central_heading = pose.heading();
  // const double max_heading_difference = PI / 4;
  // [hack](bsniu): for two floors, the first is current lane
  const auto& lines = local_view_.static_map_info.road_map().lanes();
  if (!lines.empty()) {
    size_t lane_id = lines[0].lane_id();
    for (const auto& line : lines) {
      if (line.predecessor_lane_id_size() == 0) {
        lane_id = line.lane_id();
        break;
      }
    }
    nearest_lane = hd_map_.GetLaneById(lane_id);
    if (nearest_lane == nullptr) {
      std::cout << "get nearest lane failed!!" << std::endl;
      return false;
    }
    current_lane_id_ = nearest_lane->id();
    if (!nearest_lane->lane().predecessor_lane_id().empty()) {
      sum_s += CalculatePointAccumulateS(
          nearest_lane->lane().predecessor_lane_id(0));
    }
    if (nearest_lane->GetProjection(point, &nearest_s, &nearest_l)) {
      sum_s += nearest_s;
    } else {
      std::cout << "current pose get projection fail!!" << std::endl;
      return false;
    }

  } else {
    std::cout << "no get nearest lane!!!" << std::endl;
    return false;
  }

  // if (hd_map_.GetNearestLaneAndSumDist(point,
  //     &nearest_lane, &nearest_s, &nearest_l, &sum_s) !=
  //     0) {
  //   std::cout << "no get nearest lane!!!" << std::endl;
  //   return false;
  // }
  // const int res = hd_map.GetNearestLaneWithHeading(
  //     point, distance, central_heading, max_heading_difference,
  //     &nearest_lane, &nearest_s, &nearest_l);
  // std::cout << "find current lane to current ego point dis:"
  //           << nearest_lane->DistanceTo(point) << std::endl;
  std::cout << "find the nearest lane!!!"
            << "nearest_s_:" << nearest_s
            << ",nearest lane group id:" << nearest_lane->lane_group_id()
            << std::endl;
  nearest_lane_hpp_ = nearest_lane;
  nearest_s_hpp_ = nearest_s;
  sum_s_hpp_ = sum_s;
  return true;
}

void RouteInfo::CalculateHPPInfo() {
  ConstructBox();
  if (IsOnHPPLane()) {
    std::cout << "is on hpp lane!" << std::endl;
    route_info_output_.is_on_hpp_lane = true;
    const auto trace_start =
        local_view_.static_map_info.parking_assist_info().trace_start();
    const ad_common::math::Vec2d trace_start_point_2d = {trace_start.x(),
                                                         trace_start.y()};
    // get trace_start point projection s
    double trace_start_point_accumulate_s;
    double trace_start_point_lateral;
    if (nearest_lane_hpp_->GetProjection(trace_start_point_2d,
                                         &trace_start_point_accumulate_s,
                                         &trace_start_point_lateral)) {
      std::cout << "trace_start point s:" << trace_start_point_accumulate_s
                << ",lateral:" << trace_start_point_lateral << std::endl;
    } else {
      std::cout << " trace_start point get projection fail!! " << std::endl;
      return;
    }
    // calculate sum distance
    bool is_reached_trace_start_point =
        sum_s_hpp_ >= trace_start_point_accumulate_s;
    const ad_common::math::Vec2d point(current_pose_.x, current_pose_.y);
    if (is_reached_trace_start_point) {
      std::cout << "reached trace start point!!" << std::endl;
      route_info_output_.is_reached_hpp_start_point = true;
      if (last_point_hpp_.x() != NL_NMAX && last_point_hpp_.y() != NL_NMAX) {
        sum_distance_driving_ += point.DistanceTo(last_point_hpp_);
      } else {
        sum_distance_driving_ = 0;
      }
      route_info_output_.sum_distance_driving = sum_distance_driving_;
      last_point_hpp_ = point;
    } else {
      std::cout << "cur point s less than trace start s" << std::endl;
    }
  } else {
    std::cout << "not in hpp lane!!!" << std::endl;
    ResetHpp();
  }
}

void RouteInfo::ConstructBox() {
  // ego box
  double ego_pose_x = current_pose_.x;
  double ego_pose_y = current_pose_.y;
  double yaw = current_pose_.theta;
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const auto center_x =
      ego_pose_x + std::cos(yaw) * vehicle_param.rear_axle_to_center;
  const auto center_y =
      ego_pose_y + std::sin(yaw) * vehicle_param.rear_axle_to_center;
  ad_common::math::Box2d ego_box({center_x, center_y}, yaw,
                                 vehicle_param.length, vehicle_param.width);
  ego_box_hpp_ = ego_box;
}

void RouteInfo::ResetHpp() {
  sum_distance_driving_ = -1;
  last_point_hpp_.set_x(NL_NMAX);
  last_point_hpp_.set_y(NL_NMAX);
}

void RouteInfo::CalculateDistanceToTraceEnd() {
  // get trace end projection point on line
  double sum_s = 0.0;
  const auto& lines = local_view_.static_map_info.road_map().lanes();
  if (!lines.empty()) {
    size_t lane_id = lines[lines.size() - 1].lane_id();
    for (const auto& line : lines) {
      if (line.successor_lane_id_size() == 0) {
        lane_id = line.lane_id();
        break;
      }
    }
    sum_s += CalculatePointAccumulateS(lane_id);
    route_info_output_.distance_to_target_slot = std::fabs(sum_s - sum_s_hpp_);
  } else {
    std::cout << "lines is empty from road_map!!!" << std::endl;
  }
}

void RouteInfo::CalculateDistanceToNextSpeedBump() {
  // ehr speed bump
  const auto& lane_groups =
      local_view_.static_map_info.road_map().lane_groups();
  for (auto& lane_group : lane_groups) {
    const auto& road_marks = lane_group.road_marks();
    double distance_to_speed_bump_tmp = 0;
    for (auto& road_mark : road_marks) {
      if (road_mark.type() == IFLYParkingMap::RoadMark::SPEED_BUMP &&
          road_mark.shape_size() == 4) {
        ad_common::hdmap::LaneInfoConstPtr speed_bump_nearest_lane;
        double speed_bump_nearest_s = 0.0;
        double speed_bump_nearest_l = 0.0;
        double speed_bump_sum_s = 0.0;

        ad_common::math::Vec2d speed_bump_center_point(
            (road_mark.shape(0).x() + road_mark.shape(3).x()) * 0.5,
            (road_mark.shape(0).y() + road_mark.shape(3).y()) * 0.5);
        const int speed_bump_res = hd_map_.GetNearestLane(
            speed_bump_center_point, &speed_bump_nearest_lane,
            &speed_bump_nearest_s, &speed_bump_nearest_l);
        if (speed_bump_res != 0) {
          std::cout << "not get speed_bump projection point on line!!!"
                    << std::endl;
          continue;
        } else {
          std::cout << "get s for speed_bump projection point on line:"
                    << speed_bump_nearest_s << std::endl;
        }
        speed_bump_sum_s += speed_bump_nearest_s;
        if (!speed_bump_nearest_lane->lane().predecessor_lane_id().empty()) {
          speed_bump_sum_s += CalculatePointAccumulateS(
              speed_bump_nearest_lane->lane().predecessor_lane_id(0));
        }
        distance_to_speed_bump_tmp = speed_bump_sum_s - sum_s_hpp_;
        if (distance_to_speed_bump_tmp > 0) {  // TODO: 假设挡位为前进档
          route_info_output_.distance_to_next_speed_bump =
              distance_to_speed_bump_tmp;
          break;
        }
      }
    }
    if (distance_to_speed_bump_tmp > 0) {  // TODO: 假设挡位为前进档
      break;
    }
  }
}

bool RouteInfo::IsOnHPPLane() {
  ad_common::math::Vec2d cur_point{current_pose_.x, current_pose_.y};
  if (nearest_lane_hpp_->IsOnLane(cur_point)) {
    double accumulate_s = 0.0;
    double lateral = 0.0;
    nearest_lane_hpp_->GetProjection(cur_point, &accumulate_s, &lateral);
    const double hpp_lane_heading = nearest_lane_hpp_->GetHeading(accumulate_s);
    const double ego_heading = current_pose_.theta;
    const double heading_diff_threshold = 60;
    const double angle_diff =
        planning_math::AngleDiff(hpp_lane_heading, ego_heading);
    if (std::abs(angle_diff) * 180 / M_PI < heading_diff_threshold) {
      return true;
    }
  }
  return false;
}

double RouteInfo::CalculatePointAccumulateS(size_t lane_id) {
  double accumulate_s = 0.0;
  ad_common::hdmap::LaneInfoConstPtr lane = hd_map_.GetLaneById(lane_id);
  if (lane != nullptr) {
    accumulate_s += lane->total_length();
    if (!lane->lane().predecessor_lane_id().empty()) {
      accumulate_s +=
          CalculatePointAccumulateS(lane->lane().predecessor_lane_id(0));
    }
  }
  return accumulate_s;
}
// bool RouteInfo::GetCurrentNearestLane(
//     const planning::framework::Session& session) {
//   if (session_->environmental_model().get_hdmap_valid()) {
//     const auto& local_view =
//     session_->environmental_model().get_local_view(); if
//     (local_view.localization.status.status_info.mode !=
//         iflyauto::IFLYStatusInfoMode::IFLY_STATUS_INFO_MODE_ERROR) {
//       std::cout << "hdmap_valid is true,current timestamp:"
//                 << session_->environmental_model()
//                        .get_local_view()
//                        .static_map_info.header()
//                        .timestamp()
//                 << std::endl;
//       const auto& hd_map = session.environmental_model().get_hd_map();
//       ad_common::math::Vec2d point;
//       // TODO(fengwang31):把noa和hpp的定位需要合在一起
//       const auto& ego_state =
//           session.environmental_model().get_ego_state_manager();
//       ego_pose_x_ = ego_state->ego_pose_raw().x;
//       ego_pose_y_ = ego_state->ego_pose_raw().y;
//       yaw_ = ego_state->ego_pose_raw().theta;
//       point.set_x(ego_pose_x_);
//       point.set_y(ego_pose_y_);
//       // get nearest lane
//       ad_common::hdmap::LaneInfoConstPtr nearest_lane;
//       double nearest_s = 0.0;
//       double nearest_l = 0.0;
//       printf("NearestLane point: %f, %f\n", ego_pose_x_, ego_pose_y_);
//       // const double distance = 10.0;
//       // const double central_heading = pose.heading();
//       // const double max_heading_difference = PI / 4;
//       if (hd_map.GetNearestLane(point, &nearest_lane, &nearest_s, &nearest_l)
//       !=
//           0) {
//         std::cout << "no get nearest lane!!!" << std::endl;
//         return false;
//       }
//       // const int res = hd_map.GetNearestLaneWithHeading(
//       //     point, distance, central_heading, max_heading_difference,
//       //     &nearest_lane, &nearest_s, &nearest_l);
//       // std::cout << "find current lane to current ego point dis:"
//       //           << nearest_lane->DistanceTo(point) << std::endl;
//       std::cout << "find the nearest lane!!!"
//                 << "nearest_s_:" << nearest_s_
//                 << ",nearest lane group id:" << nearest_lane->lane_group_id()
//                 << std::endl;
//       nearest_lane_ = nearest_lane;
//       nearest_s_ = nearest_s;
//       return true;
//     } else {
//       std::cout << "localization invalid" << std::endl;
//     }
//   } else {
//     std::cout << "hdmap  is invalid" << std::endl;
//   }
//   return false;
// }

// void RouteInfo::CalculateHPPInfo(
//     planning::framework::Session* session) {
//   const auto& local_view = session_->environmental_model().get_local_view();
//   // ego box
//   const auto& vehicle_param =
//       VehicleConfigurationContext::Instance()->get_vehicle_param();
//   const auto center_x =
//       ego_pose_x_ + std::cos(yaw_) * vehicle_param.rear_axle_to_center;
//   const auto center_y =
//       ego_pose_y_ + std::sin(yaw_) * vehicle_param.rear_axle_to_center;
//   const ad_common::math::Box2d ego_box(
//       {center_x, center_y}, yaw_, vehicle_param.length, vehicle_param.width);
//   // if on hpp lane
//   if (nearest_lane_->IsOnLane(ego_box)) {
//     std::cout << "is on hpp lane!" << std::endl;
//     is_on_hpp_lane_ = true;
//     const auto trace_start =
//     local_view.static_map_info.parking_assist_info().trace_start(); const
//     ad_common::math::Vec2d trace_start_point_2d = {trace_start.x(),
//                                                          trace_start.y()};
//     // get trace_start point projection s
//     double trace_start_point_accumulate_s;
//     double trace_start_point_lateral;
//     if (nearest_lane_->GetProjection(trace_start_point_2d,
//                                      &trace_start_point_accumulate_s,
//                                      &trace_start_point_lateral)) {
//       std::cout << "trace_start point s:" <<
//       trace_start_point_accumulate_s
//                 << ",lateral:" << trace_start_point_lateral << std::endl;
//     } else {
//       std::cout << " trace_start point get projection fail!! " << std::endl;
//       return;
//     }
//     // calculate sum distance
//     bool is_reached_trace_start_point =
//         nearest_s_ >= trace_start_point_accumulate_s;
//     const ad_common::math::Vec2d point(ego_pose_x_, ego_pose_y_);
//     if (is_reached_trace_start_point) {
//       std::cout << "reached trace start point!!" << std::endl;
//       is_reached_hpp_start_point_ = true;
//       if (last_point_hpp_.x() != NL_NMAX && last_point_hpp_.y() != NL_NMAX) {
//         sum_distance_driving_ += point.DistanceTo(last_point_hpp_);
//       } else {
//         sum_distance_driving_ = 0;
//       }
//       last_point_hpp_ = point;
//       // std::cout << "sum_distance_driving_:" << sum_distance_driving_
//       //           << std::endl;
//     } else {
//       std::cout << "cur point s less than trace start s" << std::endl;
//     }
//   } else {
//     std::cout << "not in hpp lane!!!" << std::endl;
//     ResetHpp();
//   }
// }

// void RouteInfo::ResetHpp() {
//   is_on_hpp_lane_ = false;
//   is_reached_hpp_start_point_ = false;
//   sum_distance_driving_ = -1;
//   last_point_hpp_.set_x(NL_NMAX);
//   last_point_hpp_.set_y(NL_NMAX);
// }

// void RouteInfo::CalculateDistanceToTargetSlot(
//     planning::framework::Session* session) {
//   const auto& local_view = session->environmental_model().get_local_view();
//   const auto& hd_map = session->environmental_model().get_hd_map();

//   // get target slot projection point on line
//   ad_common::hdmap::LaneInfoConstPtr tar_slot_nearest_lane;
//   double tar_slot_nearest_s = 0.0;
//   double tar_slot_nearest_l = 0.0;

//   const auto& lines = local_view.static_map_info.road_map().lanes();
//   if (!lines.empty()) {
//     const auto& last_point = lines[0].points_on_central_line().rbegin();
//     const double tar_slot_pose_x = last_point->x();
//     const double tar_slot_pose_y = last_point->y();
//     const int tar_slot_res = hd_map.GetNearestLane(
//         {tar_slot_pose_x, tar_slot_pose_y}, &tar_slot_nearest_lane,
//         &tar_slot_nearest_s, &tar_slot_nearest_l);
//     if (tar_slot_res != 0) {
//       std::cout << "not get target slot projection point on line!!!"
//                 << std::endl;
//       return;
//     }
//     distance_to_target_slot_ = tar_slot_nearest_s - nearest_s_;
//   } else {
//     std::cout << "lines is empty from road_map!!!" << std::endl;
//   }
// }

// void RouteInfo::CalculateDistanceToNextSpeedBump(
//     planning::framework::Session* session) {
//   distance_to_next_speed_bump_ = NL_NMAX;
//   auto& local_view = session_->environmental_model().get_local_view();
//   const auto& road_marks =
//   local_view.static_map_info.road_map().lane_groups(0).road_marks(); const
//   auto& hd_map = session->environmental_model().get_hd_map();

//   double distance_to_speed_bump_tmp = 0;
//   for (auto& road_mark : road_marks) {
//     if (road_mark.type() == IFLYParkingMap::RoadMark::SPEED_BUMP &&
//         road_mark.shape_size() == 4) {
//       ad_common::hdmap::LaneInfoConstPtr speed_bump_nearest_lane;
//       double speed_bump_nearest_s = 0.0;
//       double speed_bump_nearest_l = 0.0;

//       ad_common::math::Vec2d speed_bump_center_point(
//           (road_mark.shape(0).x() + road_mark.shape(3).x()) * 0.5,
//           (road_mark.shape(0).y() + road_mark.shape(3).y()) * 0.5);
//       const int speed_bump_res = hd_map.GetNearestLane(
//           speed_bump_center_point, &speed_bump_nearest_lane,
//           &speed_bump_nearest_s, &speed_bump_nearest_l);
//       if (speed_bump_res != 0) {
//         std::cout << "not get speed_bump projection point on line!!!"
//                   << std::endl;
//         continue;
//       } else {
//         std::cout << "get s for speed_bump projection point on line:"
//                   << speed_bump_nearest_s << std::endl;
//       }
//       distance_to_speed_bump_tmp = speed_bump_nearest_s - nearest_s_;
//       if (distance_to_speed_bump_tmp > 0) {  // TODO: 假设挡位为前进档
//         distance_to_next_speed_bump_ = distance_to_speed_bump_tmp;
//         break;
//       }
//     }
//   }
// }
void RouteInfo::UpdateVisionInfo() const {
  JSON_DEBUG_VALUE("is_leaving_ramp", route_info_output_.is_leaving_ramp);
  JSON_DEBUG_VALUE("is_nearing_ramp", route_info_output_.is_nearing_ramp);
  JSON_DEBUG_VALUE("distance_to_ramp", route_info_output_.dis_to_ramp);
  JSON_DEBUG_VALUE("distance_to_first_road_merge",
                   route_info_output_.distance_to_first_road_merge);
  JSON_DEBUG_VALUE("distance_to_first_road_split",
                   route_info_output_.distance_to_first_road_split);
  JSON_DEBUG_VALUE("is_ego_on_split_region",
                   route_info_output_.is_ego_on_split_region);
  JSON_DEBUG_VALUE("last_split_seg_dir",
                   int(route_info_output_.last_split_seg_dir));
  JSON_DEBUG_VALUE("need_continue_lc_num_on_off_ramp_region",
                   route_info_output_.need_continue_lc_num_on_off_ramp_region);
  JSON_DEBUG_VALUE(
      "is_nearing_other_lane_merge_to_road_point",
      route_info_output_.is_nearing_other_lane_merge_to_road_point);
  JSON_DEBUG_VALUE("is_ego_on_expressway",
                   route_info_output_.is_ego_on_expressway);
  JSON_DEBUG_VALUE("distance_to_route_end",
                   route_info_output_.distance_to_route_end);
  JSON_DEBUG_VALUE("sum_dis_to_last_split_point",
                   route_info_output_.sum_dis_to_last_split_point);
  JSON_DEBUG_VALUE("sum_dis_to_last_merge_point",
                   route_info_output_.sum_dis_to_last_merge_point);
  JSON_DEBUG_VALUE("ramp_direction",
                   static_cast<int>(route_info_output_.ramp_direction));
  JSON_DEBUG_VALUE("current_segment_passed_distance",
                   route_info_output_.current_segment_passed_distance);
  JSON_DEBUG_VALUE("first_split_direction",
                   (int)route_info_output_.first_split_direction);
  JSON_DEBUG_VALUE("first_merge_direction",
                   (int)route_info_output_.first_merge_direction);
}

NOASplitRegionInfo RouteInfo::CalculateSplitRegionLaneTupoInfo(
    const iflymapdata::sdpro::LinkInfo_Link& split_segment,
    const ad_common::sdpromap::SDProMap& sdpro_map) {
  NOASplitRegionInfo split_region_info;

  bool is_find_split_region_start = false;
  bool is_find_split_region_end = false;
  double fp_start_length = 0;

  const iflymapdata::sdpro::LinkInfo_Link* split_region_start_pre_link =
      nullptr;
  auto previous_seg = &split_segment;

  if (previous_seg == nullptr) {
    return split_region_info;
  }

  const auto split_seccessor_link =
      sdpro_map.GetNextLinkOnRoute(previous_seg->id());

  if (!split_seccessor_link) {
    return split_region_info;
  }

  if (previous_seg->successor_link_ids().size() < 2) {
    return split_region_info;
  }
  auto other_successor_link_ids =
      previous_seg->successor_link_ids()[0] == split_seccessor_link->id()
          ? previous_seg->successor_link_ids()[1]
          : previous_seg->successor_link_ids()[0];

  const auto other_successor_link =
      sdpro_map.GetLinkOnRoute(other_successor_link_ids);
  if (!other_successor_link) {
    return split_region_info;
  }

  iflymapdata::sdpro::FeaturePoint last_fp;
  bool is_find_last_fp = false;
  while (!is_find_split_region_start) {
    int fp_point_size = previous_seg->feature_points_size();
    for (int i = fp_point_size - 1; i >= 0; i--) {
      const auto& fp_point = previous_seg->feature_points(i);
      for (const auto fp_point_type : fp_point.type()) {
        if (fp_point_type ==
            iflymapdata::sdpro::FeaturePointType::EXCHANGE_AREA_START) {
          
          if (CalculateLastFp(&last_fp, previous_seg->id(), fp_point)){
            is_find_last_fp = true;
          }
          is_find_split_region_start = true;
          split_region_info.start_fp_point.lane_ids.clear();

          for (const auto& id : fp_point.lane_ids()) {
            if (!IsEmergencyLane(id, sdpro_map)) {
              split_region_info.start_fp_point.lane_ids.push_back(id);
            }
          }
          split_region_info.start_fp_point.link_id = previous_seg->id();
          break;
        }
      }
      if (is_find_split_region_start) {
        fp_start_length =
            fp_start_length +
            previous_seg->length() * 0.01 * (1 - fp_point.projection_percent());
        break;
      }
    }

    if (is_find_split_region_start) {
      split_region_start_pre_link =
          sdpro_map.GetPreviousLinkOnRoute(previous_seg->id());

      if (split_region_start_pre_link == nullptr) {
        return split_region_info;
      }

      break;
    }

    // 计算fp_length的累计长度
    fp_start_length = fp_start_length + previous_seg->length() * 0.01;

    previous_seg = sdpro_map.GetPreviousLinkOnRoute(previous_seg->id());
    if (previous_seg == nullptr) {
      return split_region_info;
    }
  }

  double fp_end_length = 0;
  const iflymapdata::sdpro::LinkInfo_Link* split_region_end_link =
      split_seccessor_link;
  while (!is_find_split_region_end) {
    int fp_point_size = split_region_end_link->feature_points_size();
    for (int i = fp_point_size - 1; i >= 0; i--) {
      const auto& fp_point = split_region_end_link->feature_points(i);
      for (const auto fp_point_type : fp_point.type()) {
        if (fp_point_type ==
            iflymapdata::sdpro::FeaturePointType::EXCHANGE_AREA_END) {
          is_find_split_region_end = true;
          split_region_info.end_fp_point.lane_ids.clear();

          for (const auto& id : fp_point.lane_ids()) {
            if (!IsEmergencyLane(id, sdpro_map)) {
              split_region_info.end_fp_point.lane_ids.push_back(id);
            }
          }
          split_region_info.end_fp_point.link_id = split_region_end_link->id();
          break;
        }
      }
      if (is_find_split_region_end) {
        fp_end_length = fp_end_length + split_region_end_link->length() * 0.01 *
                                            fp_point.projection_percent();
        break;
      }
    }

    if (is_find_split_region_end) {
      break;
    }

    // 计算fp_length的累计长度
    fp_end_length = fp_end_length + split_region_end_link->length() * 0.01;

    split_region_end_link =
        sdpro_map.GetNextLinkOnRoute(split_region_end_link->id());
    if (split_region_end_link == nullptr) {
      return split_region_info;
    }
  }

  if (!is_find_split_region_start || !is_find_split_region_end) {
    return split_region_info;
  }

  split_region_info.is_ramp_split =
      sdpro_map.isRamp(split_seccessor_link->link_type());
  split_region_info.split_link_id = split_segment.id();

  split_region_info.start_fp_point.fp_distance_to_split_point =
      -fp_start_length;
  split_region_info.end_fp_point.fp_distance_to_split_point = fp_end_length;

  split_region_info.recommend_lane_num.emplace_back(
      split_region_start_pre_link->lane_num(), std::vector<int>{});

  //在交换区内部，可能存在车道数变化，因此交换区的车道数以交换区终点的前一个fp的车道数为准
  int lane_num = 0;
  if (is_find_last_fp) {
    for (const auto& id : last_fp.lane_ids()) {
      if (!IsEmergencyLane(id, sdpro_map)) {
        lane_num++;
      }
    }
  } else {
    lane_num = previous_seg->lane_num();
  }

  split_region_info.recommend_lane_num.emplace_back(lane_num,
                                                    std::vector<int>{});

  split_region_info.recommend_lane_num.emplace_back(
      split_region_end_link->lane_num(), std::vector<int>{});

  split_region_info.recommend_lane_num.emplace_back(
      other_successor_link->lane_num(), std::vector<int>{});

  split_region_info.is_valid = true;
  return split_region_info;
}

NOASplitRegionInfo RouteInfo::CalculateMergeRegionLaneTupoInfo(
    const iflymapdata::sdpro::LinkInfo_Link& merge_segment,
    const ad_common::sdpromap::SDProMap& sdpro_map) {
  NOASplitRegionInfo merge_region_info;

  bool is_find_merge_region_start = false;
  bool is_find_merge_region_end = false;
  bool fp_start_point_pose_is_rear_merge_point = false;
  double fp_start_length = 0;

  const iflymapdata::sdpro::LinkInfo_Link* merge_region_start_pre_link =
      nullptr;
  auto temp_seg = &merge_segment;

  if (temp_seg == nullptr) {
    return merge_region_info;
  }

  while (!is_find_merge_region_start) {
    int fp_point_size = temp_seg->feature_points_size();
    for (int i = 0; i < fp_point_size; i++) {
      const auto& fp_point = temp_seg->feature_points(i);
      for (const auto fp_point_type : fp_point.type()) {
        if (fp_point_type ==
            iflymapdata::sdpro::FeaturePointType::EXCHANGE_AREA_START) {
          is_find_merge_region_start = true;
          // 清空原有数据（可选，根据业务需求）
          merge_region_info.start_fp_point.lane_ids.clear();

          // 遍历RepeatedField并逐个转换类型
          for (const auto& id : fp_point.lane_ids()) {
            // 显式转换类型（注意可能的溢出风险，见下方说明）
            if (!IsEmergencyLane(id, sdpro_map)) {
              merge_region_info.start_fp_point.lane_ids.push_back(id);
            }
          }
          merge_region_info.start_fp_point.link_id = temp_seg->id();
          break;
        }
      }
      if (is_find_merge_region_start) {
        double rate = 0;
        if (temp_seg->id() == merge_segment.id()) {
          rate = fp_point.projection_percent();
        } else {
          rate = 1 - fp_point.projection_percent();
        }
        fp_start_length =
            fp_start_length +
            temp_seg->length() * 0.01 * rate;
        break;
      }
    }

    if (is_find_merge_region_start) {
      merge_region_start_pre_link =
          sdpro_map.GetPreviousLinkOnRoute(temp_seg->id());
      if (merge_region_start_pre_link == nullptr) {
        return merge_region_info;
      }
      break;
    }

    // 计算fp_length的累计长度
    if (temp_seg->id() != merge_segment.id()) {
      fp_start_length = fp_start_length + temp_seg->length() * 0.01;
    }

    temp_seg = sdpro_map.GetPreviousLinkOnRoute(temp_seg->id());
    if (temp_seg == nullptr) {
      return merge_region_info;
    }
    fp_start_point_pose_is_rear_merge_point  = true;
  }

  double fp_end_length = 0;
  const iflymapdata::sdpro::LinkInfo_Link* merge_region_end_link =
      &merge_segment;

  if (merge_region_end_link == nullptr) {
    return merge_region_info;
  }

  while (!is_find_merge_region_end) {
    int fp_point_size = merge_region_end_link->feature_points_size();
    for (int i = 0; i < fp_point_size; i++) {
      const auto& fp_point = merge_region_end_link->feature_points(i);
      for (const auto fp_point_type : fp_point.type()) {
        if (fp_point_type ==
            iflymapdata::sdpro::FeaturePointType::EXCHANGE_AREA_END) {
          is_find_merge_region_end = true;
          merge_region_info.end_fp_point.lane_ids.clear();

          for (const auto& id : fp_point.lane_ids()) {
            if (!IsEmergencyLane(id, sdpro_map)) {
              merge_region_info.end_fp_point.lane_ids.push_back(
                  id);
            }
          }

          merge_region_info.end_fp_point.link_id = merge_region_end_link->id();
          break;
        }
      }
      if (is_find_merge_region_end) {
        fp_end_length = fp_end_length + merge_region_end_link->length() * 0.01 *
                                            fp_point.projection_percent();
        break;
      }
    }

    if (is_find_merge_region_end) {
      break;
    }

    // 计算fp_length的累计长度
    fp_end_length = fp_end_length + merge_region_end_link->length() * 0.01;

    merge_region_end_link =
        sdpro_map.GetNextLinkOnRoute(merge_region_end_link->id());
    if (merge_region_end_link == nullptr) {
      return merge_region_info;
    }
  }

  if (!is_find_merge_region_start || !is_find_merge_region_end) {
    return merge_region_info;
  }

  merge_region_info.is_ramp_split =
      sdpro_map.isRamp(merge_segment.link_type());
  merge_region_info.split_link_id = merge_segment.id();

  if (fp_start_point_pose_is_rear_merge_point) {
    fp_start_length = -fp_start_length;
  }
  merge_region_info.start_fp_point.fp_distance_to_split_point = fp_start_length;
  merge_region_info.end_fp_point.fp_distance_to_split_point = fp_end_length;

  merge_region_info.recommend_lane_num.emplace_back(
      merge_region_start_pre_link->lane_num(), std::vector<int>{});

  const auto& temp_link =
      sdpro_map.GetLinkOnRoute(merge_region_info.split_link_id);
  if (temp_link == nullptr) {
    return merge_region_info;
  }
  merge_region_info.recommend_lane_num.emplace_back(temp_link->lane_num(),
                                                    std::vector<int>{});

  merge_region_info.recommend_lane_num.emplace_back(
      sdpro_map.GetNextLinkOnRoute(merge_region_end_link->id())->lane_num(), std::vector<int>{});

  const auto& predecessor_link =
      sdpro_map.GetPreviousLinkOnRoute(merge_segment.id());
  if (predecessor_link == nullptr) {
    return merge_region_info;
  }

  int size = merge_segment.predecessor_link_ids().size();
  int other_lane_num = 0;
  for (int i = 0; i < size; i++) {
    const auto& temp_link =
        sdpro_map.GetLinkOnRoute(merge_segment.predecessor_link_ids(i));
    if (temp_link == nullptr) {
      return merge_region_info;
    }

    if (temp_link->id() != predecessor_link->id()) {
      other_lane_num = other_lane_num + temp_link->lane_num();
    }
  }

  merge_region_info.recommend_lane_num.emplace_back(other_lane_num,
                                                    std::vector<int>{});

  merge_region_info.is_valid = true;
  return merge_region_info;
}

bool RouteInfo::CalculateFeasibleLane(
    NOASplitRegionInfo* split_region_info,
    const ad_common::sdpromap::SDProMap& sdpro_map) const {
  const auto& recommand_lane_num = split_region_info->recommend_lane_num;

  const int before_exclnum = recommand_lane_num[0].total_lane_num;
  const int on_exclnum = recommand_lane_num[1].total_lane_num;
  const int successor_exclnum = recommand_lane_num[2].total_lane_num;
  const int successor_other_exclnum = recommand_lane_num[3].total_lane_num;

  // const auto& start_fp_point = split_region_info->start_fp_point;
  const auto& end_fp_point = split_region_info->end_fp_point;
  const auto& start_fp_point = split_region_info->start_fp_point;
  const int start_link_id = start_fp_point.link_id;
  const auto& before_link = sdpro_map.GetPreviousLinkOnRoute(start_link_id);


  const auto& end_fpp_lane_ids = end_fp_point.lane_ids;

  std::vector<std::vector<const MapLane*>> end_fp_lane_sequence_groups;
  std::vector<std::vector<const MapLane*>> on_fp_lane_sequence_groups;
  std::vector<std::vector<const MapLane*>> before_fp_lane_sequence_groups;

  //1、获取交换区终点后的lane信息；
  std::vector<const MapLane* > before_fpp_lanes;
  std::vector<const MapLane* > on_fpp_lanes;
  std::vector<const MapLane* > end_fpp_lanes;
  std::vector<std::vector<const MapLane* >> feasible_lane_groups;
  for (const auto lane_id: end_fpp_lane_ids) {
    const auto& lane = sdpro_map.GetLaneInfoByID(lane_id);
    if (lane == nullptr) {
      continue;
    }
    end_fpp_lanes.emplace_back(lane);
  }

  end_fp_lane_sequence_groups.emplace_back(end_fpp_lanes);

  //2、从交换区终点向前遍历交换区内的lane_groups；
  for (const auto& lane : end_fpp_lanes) {
    std::vector<const MapLane*> feasible_lane_group;
    feasible_lane_group.emplace_back(lane);

    bool is_find_before_lane = false;

    while (!is_find_before_lane) {
      double accumulative_distance = lane->length() * 0.01;
      // lane->predecessor_lane_ids();
      // 如果前继有多条，那么根据split的方向选择最靠近方向的那一条
      int target_sequence_index = 0;
      if (lane->predecessor_lane_ids().size() > 1) {
        if (split_region_info->split_direction == SplitDirection::SPLIT_RIGHT) {
          int min_sequence = 100;
          for (int i = 0; i < lane->predecessor_lane_ids().size(); i++) {
              int lane_id = lane->predecessor_lane_ids()[i];
              const auto& temp_lane = sdpro_map.GetLaneInfoByID(lane_id);
              if (!temp_lane) {
                return false;
              }
              if (temp_lane->sequence() < min_sequence) {
                min_sequence = temp_lane->sequence();
                target_sequence_index = i;
              }
          }
        } else if (split_region_info->split_direction ==
                   SplitDirection::SPLIT_LEFT) {
          int max_sequence = -1;
          for (int i = 0; i < lane->predecessor_lane_ids().size(); i++) {
              int lane_id = lane->predecessor_lane_ids()[i];
              const auto& temp_lane = sdpro_map.GetLaneInfoByID(lane_id);
              if (!temp_lane) {
                return false;
              }
              if (temp_lane->sequence() > max_sequence) {
                max_sequence = temp_lane->sequence();
                target_sequence_index = i;
              }
          }
        }
      }

      const int target_lane_id =
          lane->predecessor_lane_ids()[target_sequence_index];
      const auto& target_lane = sdpro_map.GetLaneInfoByID(target_lane_id);
      if (!target_lane) {
        return false;
      }

      accumulative_distance = accumulative_distance + target_lane->length() * 0.01;

      if (target_lane->link_id() == end_fp_point.link_id) {
        continue;
      } else if (target_lane->link_id() == before_link->id()) {
        is_find_before_lane = true;
        feasible_lane_group.emplace_back(target_lane);
        // before_fpp_lanes.emplace_back(target_lane);
      } else {
        // on_fpp_lanes.emplace_back(target_lane);
        feasible_lane_group.emplace_back(target_lane);
      }

      if (accumulative_distance > 1000.0) {
        return false;
      }
    }
    feasible_lane_groups.emplace_back(feasible_lane_group);
  }

  int flgs = feasible_lane_groups.size();

  return true;
}

bool RouteInfo::CalculateFeasibleLane(NOASplitRegionInfo* split_region_info) const {
  // const auto& first_split_region_info = split_region_info_list[0];
  if (split_region_info == nullptr) {
    return false;
  }

  if (split_region_info->recommend_lane_num.size() != 4) {
    return false;
  }
  const auto& recommand_lane_num = split_region_info->recommend_lane_num;

  const int before_exclnum = recommand_lane_num[0].total_lane_num;
  const int on_exclnum = recommand_lane_num[1].total_lane_num;
  const int successor_exclnum = recommand_lane_num[2].total_lane_num;
  const int successor_other_exclnum = recommand_lane_num[3].total_lane_num;

  bool is_split_right =
      split_region_info->split_direction == SplitDirection::SPLIT_RIGHT;
  bool is_split_left =
      split_region_info->split_direction == SplitDirection::SPLIT_LEFT;

  std::vector<int> on_excr_feasible_lane;
  std::vector<int> before_excr_feasible_lane;
  std::vector<int> succerssor_excr_feasible_lane;
  //TODO(fengwang31:需要考虑split的路是否未主路的情况)
  if (is_split_right) {
    //现在假设交换区终点后的lane都是由交换区分出来的，所以交换区的lane数为on_exclnum = successor_exclnum + successor_other_exclnum
    //通常认为右边是从主路分出去的路，因此增加的车道属于是右边增加了
    if (on_exclnum == successor_exclnum + successor_other_exclnum) {
      if (successor_other_exclnum == before_exclnum) {
        // int on_excr_feasible_lane_temp = on_exclnum - successor_other_exclnum;
        for (int i = 0; i < successor_exclnum; ++i) {
          on_excr_feasible_lane.emplace_back(successor_other_exclnum + i + 1);
        }
        before_excr_feasible_lane.emplace_back(before_exclnum);
      } else if (successor_other_exclnum < before_exclnum) {
        int before_excr_feasible_lane_temp = before_exclnum - successor_other_exclnum;
        for (int i = 0; i < before_excr_feasible_lane_temp; ++i) {
          before_excr_feasible_lane.emplace_back(successor_other_exclnum + i + 1);
        }

        int on_excr_feasible_lane_temp = on_exclnum - successor_other_exclnum;
        for (int i = 0; i < on_excr_feasible_lane_temp; ++i) {
          on_excr_feasible_lane.emplace_back(successor_other_exclnum + i + 1);
        }
      } else {
        on_excr_feasible_lane.emplace_back(on_exclnum);
        before_excr_feasible_lane.emplace_back(before_exclnum);
      }
    } else if (on_exclnum < successor_exclnum + successor_other_exclnum) {
      //认为是右边,交换区终点后新增加了车道
      if (successor_other_exclnum == before_exclnum &&
        successor_exclnum < on_exclnum &&
        successor_other_exclnum < on_exclnum) {
        int err = on_exclnum - successor_other_exclnum;
        for (int i = 0; i < err; i++) {
          on_excr_feasible_lane.emplace_back(successor_other_exclnum + i + 1);
        }

        before_excr_feasible_lane.emplace_back(before_exclnum);
      } else if (before_exclnum == on_exclnum &&
                 on_exclnum == successor_other_exclnum) {
        for (int i = 0; i < successor_exclnum; ++i) {
          on_excr_feasible_lane.emplace_back(on_exclnum + i + 1);
        }
        before_excr_feasible_lane.emplace_back(before_exclnum);

      } else {
        on_excr_feasible_lane.emplace_back(on_exclnum);
        before_excr_feasible_lane.emplace_back(before_exclnum);
      }
    }
  } else if (is_split_left) {
    //默认左边都是主路的，后续需要对是否是主路的属性做判断
    //1、后继车道数等于交换区的车道数；
    //2、交换区前、交换区、交换区后的车道数都相等；
    //3、交换区前、交换区车道数相等，交换区后车道数小于或者大于前面的车道数
    if (successor_exclnum <= on_exclnum) {
      if (successor_exclnum <= on_exclnum &&
          on_exclnum >= before_exclnum) {
        const auto start_link = sdpro_map_.GetLinkOnRoute(
            split_region_info->start_fp_point.link_id);
        if (start_link == nullptr) {
          return false;
        }
        const auto start_link_is_ramp =
            sdpro_map_.isRamp(start_link->link_type());

        const auto& end_fp_point = split_region_info->end_fp_point;
        const auto end_link = sdpro_map_.GetLinkOnRoute(end_fp_point.link_id);
        if (end_link == nullptr) {
          return false;
        }
        const auto end_link_is_ramp = sdpro_map_.isRamp(end_link->link_type());

        if (!end_link_is_ramp && !start_link_is_ramp) {
          //主路上，交换区内、前、后车道都一样
          for (int i = 0; i < successor_exclnum; ++i) {
              on_excr_feasible_lane.emplace_back(i + 1);
              before_excr_feasible_lane.emplace_back(i + 1);
          }
        } else {
          //交换区之后增加车道
          on_excr_feasible_lane.emplace_back(1);
          before_excr_feasible_lane.emplace_back(1);
        }

      } else if (successor_exclnum <= before_exclnum) {
        for (int i = 0; i < successor_exclnum; ++i) {
          on_excr_feasible_lane.emplace_back(i + 1);
          before_excr_feasible_lane.emplace_back(i + 1);
        }
      } else {
        for (int i = 0; i < before_exclnum; ++i) {
          on_excr_feasible_lane.emplace_back(i + 1);
          before_excr_feasible_lane.emplace_back(i + 1);
        }
      }
    }
  }

  for (int i = 0; i < successor_exclnum; ++i) {
    succerssor_excr_feasible_lane.emplace_back(i + 1);
  }

  if (before_excr_feasible_lane.empty() ||
      on_excr_feasible_lane.empty() ||
      succerssor_excr_feasible_lane.empty()) {
    return false;
  }

  split_region_info->recommend_lane_num[0].feasible_lane_sequence =
      before_excr_feasible_lane;
  split_region_info->recommend_lane_num[1].feasible_lane_sequence =
      on_excr_feasible_lane;
  split_region_info->recommend_lane_num[2].feasible_lane_sequence =
      succerssor_excr_feasible_lane;

  return true;
}

bool RouteInfo::CalculateMergeRegionFeasibleLane(NOASplitRegionInfo* split_region_info) const {
  if (split_region_info == nullptr) {
    return false;
  }

  if (split_region_info->recommend_lane_num.size() != 4) {
    return false;
  }
  const auto& recommand_lane_num = split_region_info->recommend_lane_num;

  const int before_exclnum = recommand_lane_num[0].total_lane_num;
  const int on_exclnum = recommand_lane_num[1].total_lane_num;
  const int successor_exclnum = recommand_lane_num[2].total_lane_num;
  const int predecessor_other_exclnum = recommand_lane_num[3].total_lane_num;

  bool is_merge_right =
      split_region_info->split_direction == SplitDirection::SPLIT_RIGHT;

  std::vector<int> on_excr_feasible_lane;
  std::vector<int> before_excr_feasible_lane;
  std::vector<int> succerssor_excr_feasible_lane;
  //TODO(fengwang31:需要考虑split的路是否未主路的情况)
  if (is_merge_right) {
    //现在假设交换区终点后的lane都是由交换区分出来的，所以交换区的lane数为on_exclnum = successor_exclnum + successor_other_exclnum
    //通常认为右边是从主路分出去的路，因此增加的车道属于是右边增加了
    if (on_exclnum == successor_exclnum) {
      for (int i = 0; i < successor_exclnum; ++i) {
        on_excr_feasible_lane.emplace_back(i + 1);
      }
      //目前假定都是从右边往左边汇入，所以都行驶到左边的车道上去
      //TODO(fengwang31):后续需要考虑左边的车道是否会收窄，如果会的话，则不能继续往左边汇
      before_excr_feasible_lane.emplace_back(1);
    } else if (on_exclnum > successor_exclnum) {
      for (int i = 0; i < successor_exclnum; ++i) {
        on_excr_feasible_lane.emplace_back(i + 1);
      }
      before_excr_feasible_lane.emplace_back(1);
    }
  }

  for (int i = 0; i < successor_exclnum; ++i) {
    succerssor_excr_feasible_lane.emplace_back(i + 1);
  }

  if (before_excr_feasible_lane.empty() ||
      on_excr_feasible_lane.empty() ||
      succerssor_excr_feasible_lane.empty()) {
    return false;
  }

  split_region_info->recommend_lane_num[0].feasible_lane_sequence =
      before_excr_feasible_lane;
  split_region_info->recommend_lane_num[1].feasible_lane_sequence =
      on_excr_feasible_lane;
  split_region_info->recommend_lane_num[2].feasible_lane_sequence =
      succerssor_excr_feasible_lane;

  return true;
}

bool RouteInfo::IsEmergencyLane(
    const uint64 lane_id,
    const ad_common::sdpromap::SDProMap& sdpro_map) const {

  const auto& lane_info = sdpro_map.GetLaneInfoByID(lane_id);

  if (lane_info == nullptr) {
    return true;
  }

  // 使用std::any_of检查是否存在紧急车道类型
  bool is_emergency_lane = std::any_of(
      lane_info->lane_type().begin(), lane_info->lane_type().end(),
      [](const auto& lane_type) {
        return lane_type == iflymapdata::sdpro::LaneType::LAT_EMERGENCY;
      });

  return is_emergency_lane;
}

std::vector<char> RouteInfo::SortRaysByDirection(const std::vector<RayInfo>& rays) {
    if (rays.empty()) return {};

    // 找到最小夹角
    double min_angle = rays[0].angle;
    for (const auto& ray : rays) {
        if (ray.angle < min_angle) {
            min_angle = ray.angle;
        }
    }

    // 转换到以min_angle为起点的坐标系
    std::vector<std::pair<double, char>> shifted_rays;
    for (const auto& ray : rays) {
        double shifted_angle = ray.angle - min_angle;
        // 如果shifted_angle超过π，转换到-π到π范围
        if (shifted_angle > M_PI) {
            shifted_angle -= 2 * M_PI;
        }
        shifted_rays.emplace_back(shifted_angle, ray.name);
    }

    // 按转换后的角度排序
    std::sort(shifted_rays.begin(), shifted_rays.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });

    // 提取排序后的射线名称
    std::vector<char> result;
    for (const auto& ray : shifted_rays) {
        result.push_back(ray.second);
    }
    return result;
}

bool RouteInfo::CalculateMergeLaneInfo(
  std::vector<int>& merge_lane_sequence) {
  iflymapdata::sdpro::FeaturePoint find_fp;
  iflymapdata::sdpro::FeaturePoint last_fp;
  uint64 fp_link_id;
  double ego_s_in_cur_link;

  if (!CalculateFP(&find_fp, &fp_link_id)) {
      return false;
  }

  if (!CalculateLastFp(&last_fp, fp_link_id, find_fp)) {
      merge_lane_sequence.emplace_back(1);
      return true;
  }

  for (const auto& lane_id : find_fp.lane_ids()) {
    const auto& lane_info = sdpro_map_.GetLaneInfoByID(lane_id);
    if (lane_info == nullptr) {
        continue;
    }
    // 暂时先处理左边的merge lane
    if (lane_info->change_type() ==
      iflymapdata::sdpro::LaneChangeType::LeftTurnMergingLane) {
      bool is_lane_num_satisfy =
          find_fp.lane_ids_size() + 1 == last_fp.lane_ids_size();
      // 默认是车道的前继车道左边为汇入车道
      if (is_lane_num_satisfy) {
        merge_lane_sequence.emplace_back(lane_info->sequence());
        return true;
      }
    }
  }
  return false;
}

bool RouteInfo::CalculateFP(iflymapdata::sdpro::FeaturePoint* find_fp, uint64* fp_link_id) {
  //后续需要把这部分代码封装一下#######################################
  const double max_search_length = 7000.0;  // 搜索7km范围内得地图信息
  const double search_distance = 50.0;
  const double max_heading_diff = PI / 4;
  //获取当前的segment
  ad_common::math::Vec2d current_point;
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto& pose = ego_state->location_enu();
  current_point.set_x(pose.position.x);
  current_point.set_y(pose.position.y);
  double temp_nearest_s = 0;
  double nearest_l = 0;
  const double ego_heading_angle = ego_state->heading_angle();

  const iflymapdata::sdpro::LinkInfo_Link* current_link =
      sdpro_map_.GetNearestLinkWithHeading(current_point, search_distance,
                                          ego_heading_angle, max_heading_diff,
                                          temp_nearest_s, nearest_l);
  route_info_output_.current_segment_passed_distance = temp_nearest_s;
  LOG_DEBUG("current_segment_passed_distance:%f\n",
            route_info_output_.current_segment_passed_distance);
  if (!current_link) {
    return false;
  }
  //############################################

  double itera_dis = 0;
  const auto& merge_region_info_list = route_info_output_.merge_region_info_list;
  if (merge_region_info_list.size() == 0) {
    return false;
  } 

  while (itera_dis < merge_region_info_list[0].distance_to_split_point) {
    for (const auto& fp: current_link->feature_points()) {
      for (const auto& fp_type: fp.type()) {
        if (fp_type == iflymapdata::sdpro::FeaturePointType::LANE_COUNT_CHANGE) {
          itera_dis = itera_dis +
                      fp.projection_percent() * current_link->length() * 0.01 -
                      temp_nearest_s;
          if (itera_dis < kEpsilon) {
              continue;
          }
          *find_fp = fp;
          *fp_link_id = current_link->id();
          return true;
        }
      }
    }
    itera_dis = itera_dis + current_link->length() * 0.01;
    current_link = sdpro_map_.GetNextLinkOnRoute(current_link->id());
  }
  return true;                 
}

bool RouteInfo::CalculateLastFp(
    iflymapdata::sdpro::FeaturePoint* last_fp,
    const uint64 fp_link_id, const iflymapdata::sdpro::FeaturePoint& find_fp) {
  const auto& fp_link = sdpro_map_.GetLinkOnRoute(fp_link_id);
  if (last_fp == nullptr || 
      fp_link == nullptr || 
      fp_link->feature_points().empty()) {
    return false;
  }

  int fp_index = -1;
  for (int i = 0; i < fp_link->feature_points_size(); i++) {
    const auto& fp = fp_link->feature_points()[i];
    if (fp.id() == find_fp.id()) {
      fp_index = i;
    }
  }

  if (fp_index < kEpsilon) {
    return false;
  }

  if (fp_index == 0) {
    const iflymapdata::sdpro::LinkInfo_Link* fp_pre_link =
        sdpro_map_.GetPreviousLinkOnRoute(fp_link_id);

    if (fp_pre_link == nullptr) {
      return false;
    }

    double itear_dis = 0.0;
    while (fp_pre_link->feature_points_size() == 0) {
      fp_pre_link = sdpro_map_.GetPreviousLinkOnRoute(fp_pre_link->id());
      itear_dis = itear_dis + fp_pre_link->length() * 0.01;
      if (fp_pre_link == nullptr || itear_dis > 500.0) {
        return false;
      }
    }

    //注：因为上面while计算，所以在这个地方feature_points_size不为0
    *last_fp = *fp_pre_link->feature_points().rbegin();
    return true;
  } else {
    if (fp_index - 1 >= fp_link->feature_points_size()) {
      return false;
    }

    *last_fp = fp_link->feature_points()[fp_index - 1];
    return true;
  }
  return true;
}

}  // namespace planning
