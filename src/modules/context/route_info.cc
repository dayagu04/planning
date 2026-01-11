#include "route_info.h"

#include <linux/limits.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <utility>
#include <vector>

#include "common_platform_type_soc.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "define/geometry.h"
#include "environmental_model.h"
#include "google/protobuf/arena.h"
#include "planning_context.h"
#include "route_info_strategy/LD_route_info_strategy.h"
#include "route_info_strategy/SDPro_route_info_strategy.h"
#include "sdmap/sdmap.h"
namespace planning {

namespace {
constexpr double kEpsilon = 1.0e-4;
}  // namespace

namespace {
constexpr uint64_t kStaticMapOvertimeThreshold = 20000000;  // 20s
constexpr double kStandardLaneWidth = 3.8;
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
  current_link_ = nullptr;

  const auto& local_view = session_->environmental_model().get_local_view();
  if (local_view.localization.status.status_info.mode ==
      iflyauto::IFLYStatusInfoMode::IFLY_STATUS_INFO_MODE_ERROR) {
    ILOG_INFO << "localization invalid";
    return;
  }

  bool is_hpp_scene = session_->is_hpp_scene();
  if (!is_hpp_scene) {
    if (local_view.sdpro_map_info.data_source() ==
        iflymapdata::sdpro::MAP_VENDOR_BAIDU_LD) {
      GetStrategy();
      route_info_strategy_->Update(route_info_output_);
      sdpromap_valid_ = route_info_strategy_->get_sdpromap_valid();
      sdpro_map_ = route_info_strategy_->get_sdpro_map();
      current_link_ = route_info_strategy_->get_current_link();
    } else if (local_view.sdpro_map_info.data_source() ==
               iflymapdata::sdpro::MAP_VENDOR_TENCENT_SD_PRO) {
      if (UpdateSdProMap(local_view)) {
        UpdateSdMap(local_view);
        UpdateRouteInfoForNOA(sdpro_map_);
      } else {
        ILOG_INFO << "UpdateSdMap failed!!!";
      }
    }

  } else {
    if (UpdateStaticMap(local_view)) {
      UpdateRouteInfoForHPP(hd_map_);
    } else {
      ILOG_INFO << "UpdateHdMap failed!!!";
    }
  }
}

void RouteInfo::UpdateRouteInfoForNOA(const ad_common::sdmap::SDMap& sd_map) {
  double nearest_s;
  const double max_search_length = 7000.0;  // 搜索7km范围内得地图信息
  const SdMapSwtx::Segment* segment = UpdateEgoSegmentInfo(sd_map, &nearest_s);
  if (!segment) {
    ILOG_DEBUG << "update ego segment info failed!!!";
    return;
  }
  const SdMapSwtx::Segment& current_segment = *segment;

  // 计算ramp信息
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
  if (!sdpro_map.isRouteValid()) {
    route_info_output_.reset();
    return;
  }

  double nearest_s;
  double nearest_l;
  double nearest_s_sd_map;
  const double max_search_length = 7000.0;  // 搜索7km范围内得地图信息

  const SdMapSwtx::Segment* segment =
      UpdateEgoSegmentInfo(sd_map_, &nearest_s_sd_map);
  // if (segment == nullptr) {
  //   std::cout << "ego link not in expressway failed!!!" << std::endl;
  //   return;
  // }
  const iflymapdata::sdpro::LinkInfo_Link* link =
      UpdateEgoLinkInfo(sdpro_map, &nearest_s, &nearest_l);
  if (!link) {
    ILOG_DEBUG << "update ego link info failed!!!";
    route_info_output_.reset();
    return;
  }
  is_in_tunnel_ = sdpro_map.isTunnel(link->link_type());

  if (IsMissSplitPoint(*link, nearest_l, nearest_s)) {
    route_info_output_.reset();
    route_info_output_.is_miss_split_point = true;
    return;
  }

  const auto& local_view = session_->environmental_model().get_local_view();
  auto& route_map_info = local_view.sdpro_map_info.route();
  // 根据地图数据path_id来判断是否更新导航路线
  std::string path_id = route_map_info.path_id();
  if (route_map_info.has_path_id() != last_path_id_is_set_ ||
      (last_path_id_ != path_id && route_map_info.has_path_id())) {
    mlc_decider_route_info_.reset();
    last_path_id_is_set_ = route_map_info.has_path_id();
    if (last_path_id_is_set_) {
      last_path_id_ = path_id;
    } else {
      last_path_id_.clear();
    }
  }

  const iflymapdata::sdpro::LinkInfo_Link& current_link = *link;
  const auto& sdpro_map_info = local_view.sdpro_map_info;
  route_info_output_.map_vendor = sdpro_map_info.data_source();

  // 计算ramp信息
  CaculateRampInfo(sdpro_map, current_link, nearest_s, max_search_length);

  // 计算merge信息
  CaculateMergeInfo(sdpro_map, current_link, nearest_s, max_search_length);

  // 计算split信息
  CaculateSplitInfo(sdpro_map, current_link, nearest_s, max_search_length);

  double distance_to_next_exchange_region = NL_NMAX;
  if (!route_info_output_.split_region_info_list.empty()) {
    distance_to_next_exchange_region = std::min(
        distance_to_next_exchange_region,
        route_info_output_.split_region_info_list[0].distance_to_split_point);
  }
  if (!route_info_output_.merge_region_info_list.empty()) {
    distance_to_next_exchange_region = std::min(
        distance_to_next_exchange_region,
        route_info_output_.merge_region_info_list[0].distance_to_split_point);
  }
  double intersection_search_distance =
      std::min(distance_to_next_exchange_region, 250.0);
  if (IsClosingIntersectionEntrance(
          link, sdpro_map, route_info_output_.current_segment_passed_distance,
          intersection_search_distance)) {
    ILOG_ERROR << "Trigger exit NOA for closing intersection!!!";
    route_info_output_.reset();
    return;
  }

  // 计算距离上一个merge点的信息
  if (mlc_decider_route_info_.is_triggle_cal_dis_to_last_merge_point) {
    CaculateDistanceToLastMergePoint(sdpro_map, current_link, nearest_s,
                                     max_search_length);
  }
  // 计算距离上一个split点的信息
  if (mlc_decider_route_info_.ego_status_on_route == IN_EXCHANGE_AREA_FRONT ||
      mlc_decider_route_info_.ego_status_on_route == IN_EXCHANGE_AREA_REAR) {
    CaculateDistanceToLastSplitPoint(sdpro_map, current_link, nearest_s,
                                     max_search_length);
  }

  CaculateDistanceToLastSplitPoint(sdpro_map, current_link, nearest_s,
                                   max_search_length);

  if (segment != nullptr) {
    const SdMapSwtx::Segment& current_segment = *segment;
    // 计算到路线终点的距离
    CaculateDistanceToRoadEnd(sd_map_, current_segment, nearest_s_sd_map,
                              max_search_length);
  }

  // 计算到最近收费站的距离
  CaculateDistanceToTollStation(sdpro_map, current_link, nearest_s,
                                max_search_length);
}

void RouteInfo::UpdateRouteInfoForHPP(const ad_common::hdmap::HDMap& hd_map) {
  ILOG_DEBUG << "session_->is_hpp_scene():", session_->is_hpp_scene();
  if (!GetCurrentNearestLane()) {
    ILOG_DEBUG << "GetCurrentNearestLane failed!!!";
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
      SplitSegInfo split_seg_info =
          MakesureSplitDirection(*previous_seg, sd_map);
      route_info_output_.ramp_direction = split_seg_info.split_direction;
    } else {
      ILOG_DEBUG << "previous_seg is nullprt!!!!!";
      route_info_output_.ramp_direction = RAMP_NONE;
    }
  }
}

void RouteInfo::CaculateRampInfo(const ad_common::sdpromap::SDProMap& sdpro_map,
                                 const iflymapdata::sdpro::LinkInfo_Link& link,
                                 const double nearest_s,
                                 const double max_search_length) {
  // 计算ramp信息
  const auto& ramp_info =
      sdpro_map.GetRampInfo(link.id(), nearest_s, max_search_length);
  const auto& sapa_info =
      sdpro_map.GetSaPaInfo(link.id(), nearest_s, max_search_length);
  if (ramp_info.second > 0) {
    route_info_output_.dis_to_ramp = ramp_info.second;
    auto previous_seg = sdpro_map.GetPreviousLinkOnRoute(ramp_info.first->id());

    if (!previous_seg) {
      return;
    }

    SplitSegInfo split_seg_info;
    split_seg_info = MakesureSplitDirection(*previous_seg, sdpro_map);
    route_info_output_.ramp_direction = split_seg_info.split_direction;

  } else if (sapa_info.second > 0) {
    route_info_output_.dis_to_ramp = sapa_info.second;
    auto previous_seg = sdpro_map.GetPreviousLinkOnRoute(sapa_info.first->id());

    if (!previous_seg) {
      return;
    }

    SplitSegInfo split_seg_info;
    split_seg_info = MakesureSplitDirection(*previous_seg, sdpro_map);
    route_info_output_.ramp_direction = split_seg_info.split_direction;
  }
}

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
      ILOG_DEBUG << "out segment is nullptr!!!!!!!!";
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
      ILOG_DEBUG << "enu points error!!!!!!!!!!";
    }
  } else {
    ILOG_DEBUG << "out_link_size != 2!!!!!!!!";
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
      ILOG_DEBUG << "out segment is nullptr!!!!!!!!";
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
      ILOG_DEBUG << "enu points error!!!!!!!!!!";
    }
  } else if (out_link_size == 3) {
    const auto split_next_link = sdpro_map.GetNextLinkOnRoute(split_link.id());
    if (!split_next_link) {
      ILOG_DEBUG << "out segment is nullptr!!!!!!!!";
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
    if (other_link1 == nullptr || other_link2 == nullptr ||
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
      if (result[i] == 'A') {
        if (i == 0) {
          split_seg_info.split_direction = RAMP_ON_RIGHT;
        } else if (i == 1) {
          split_seg_info.split_direction = RAMP_ON_MIDDLE;
        } else if (i == 2) {
          split_seg_info.split_direction = RAMP_ON_LEFT;
        }
      }
    }
  } else {
    ILOG_DEBUG << "out_link_size != 2!!!!!!!!";
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
    ILOG_DEBUG << "merge_info.empty()!!!!!!!";
  }
}

void RouteInfo::CaculateMergeInfo(
    const ad_common::sdpromap::SDProMap& sdpro_map,
    const iflymapdata::sdpro::LinkInfo_Link& link, const double nearest_s,
    const double max_search_length) {
  const auto& merge_info =
      sdpro_map.GetMergeInfoList(link.id(), nearest_s, max_search_length);
  const auto& split_info =
      sdpro_map.GetSplitInfoList(link.id(), nearest_s, max_search_length);

  if (!merge_info.empty()) {
    const iflymapdata::sdpro::LinkInfo_Link* seg_of_first_road_merge = nullptr;
    const iflymapdata::sdpro::LinkInfo_Link* next_seg_of_first_road_merge =
        nullptr;
    const iflymapdata::sdpro::LinkInfo_Link* merge_seg_last_other_seg = nullptr;
    int traverse_num = 0;
    bool is_find_first_merge_onfo = false;
    for (int i = 0; i < merge_info.size(); i++) {
      const auto& merge_info_temp = merge_info[i];
      bool is_road_merged_by_other_lane = false;
      bool is_ramp_road_merge = false;
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
        if (merge_seg->predecessor_link_ids().size() == 2) {
          const auto& merge_seg_last_other_seg_id =
              merge_seg->predecessor_link_ids()[0] == merge_seg_last_seg->id()
                  ? merge_seg->predecessor_link_ids()[1]
                  : merge_seg->predecessor_link_ids()[0];
          merge_seg_last_other_seg =
              sdpro_map.GetLinkOnRoute(merge_seg_last_other_seg_id);
        } else if (merge_seg->predecessor_link_ids().size() >= 3) {
          for (int i = 0; i < merge_seg->predecessor_link_ids().size(); i++) {
            if (merge_seg->predecessor_link_ids()[i] !=
                merge_seg_last_seg->id()) {
              merge_seg_last_other_seg = sdpro_map.GetLinkOnRoute(
                  merge_seg->predecessor_link_ids()[i]);
              if (merge_seg_last_other_seg != nullptr &&
                  (merge_seg_last_other_seg->link_class() ==
                       iflymapdata::sdpro::LinkClass::LC_EXPRESSWAY ||
                   merge_seg_last_other_seg->link_class() ==
                       iflymapdata::sdpro::LinkClass::LC_CITY_EXPRESSWAY)) {
                break;
              }
            }
          }
        }
        if (!merge_seg_last_other_seg) {
          break;
        }
        // 对于other_merge/merge的判断
        if (route_info_output_.is_ego_on_expressway) {
          if (merge_seg_last_seg->lane_num() >
              merge_seg_last_other_seg->lane_num()) {
            route_info_output_.is_road_merged_by_other_lane = true;
            is_road_merged_by_other_lane = true;
            is_ramp_road_merge = false;
          } else if (merge_seg_last_seg->lane_num() ==
                     merge_seg_last_other_seg->lane_num()) {
            if (!sdpro_map.isRamp(merge_seg_last_seg->link_type()) &&
                !sdpro_map.isSaPa(merge_seg_last_seg->link_type())) {
              route_info_output_.is_road_merged_by_other_lane = true;
              is_road_merged_by_other_lane = true;
              is_ramp_road_merge = false;
            } else if (!sdpro_map.isRamp(
                           merge_seg_last_other_seg->link_type()) &&
                       !sdpro_map.isSaPa(
                           merge_seg_last_other_seg->link_type())) {
              route_info_output_.is_road_merged_by_other_lane = false;
              is_road_merged_by_other_lane = false;
              is_ramp_road_merge = true;
            } else {
              route_info_output_.is_road_merged_by_other_lane = true;
              is_road_merged_by_other_lane = true;
              is_ramp_road_merge = false;
            }
          } else {
            route_info_output_.is_road_merged_by_other_lane = false;
            is_road_merged_by_other_lane = false;
            is_ramp_road_merge = true;
          }
        }
        if (!is_find_first_merge_onfo) {
          if (sdpro_map.isRamp(merge_seg_last_seg->link_type()) &&
              sdpro_map.isRamp(merge_seg->link_type()) &&
              route_info_output_.is_on_ramp) {
            route_info_output_.is_ramp_merge_to_ramp_on_expressway = true;
          }
          // 匝道/SAPA合并到普通道路
          if ((sdpro_map.isRamp(merge_seg_last_seg->link_type()) ||
               sdpro_map.isSaPa(merge_seg_last_seg->link_type())) &&
              !sdpro_map.isRamp(merge_seg->link_type()) &&
              (sdpro_map.isSaPa(merge_seg_last_seg->link_type()) ||
               route_info_output_.is_on_ramp)) {
            route_info_output_.is_ramp_merge_to_road_on_expressway = true;
          }
          route_info_output_.first_merge_direction =
              MakesureMergeDirection(*merge_seg, sdpro_map);
          route_info_output_.distance_to_first_road_merge =
              merge_info_temp.second;
          route_info_output_.merge_seg_forward_lane_nums =
              merge_seg->lane_num();
          route_info_output_.merge_last_seg_forward_lane_nums =
              merge_seg_last_seg->lane_num();

          NOASplitRegionInfo first_merge_region_lane_tupo_info;
          const auto merge_region_lane_tupo_info =
              CalculateMergeRegionLaneTupoInfo(
                  *merge_seg, sdpro_map, merge_info, split_info,
                  route_info_output_.distance_to_first_road_merge);
          if (merge_region_lane_tupo_info.is_valid) {
            first_merge_region_lane_tupo_info = merge_region_lane_tupo_info;
            seg_of_first_road_merge = merge_seg;
            next_seg_of_first_road_merge =
                sdpro_map.GetNextLinkOnRoute(seg_of_first_road_merge->id());

            first_merge_region_lane_tupo_info.distance_to_split_point =
                merge_info_temp.second;

            first_merge_region_lane_tupo_info.split_direction =
                static_cast<SplitDirection>(
                    route_info_output_.first_merge_direction);

            first_merge_region_lane_tupo_info.is_other_merge_to_road =
                is_road_merged_by_other_lane;
            first_merge_region_lane_tupo_info.is_ramp_merge =
                is_ramp_road_merge;

            route_info_output_.merge_region_info_list.emplace_back(
                first_merge_region_lane_tupo_info);
            is_find_first_merge_onfo = true;
            traverse_num++;
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
                  *merge_seg, sdpro_map, merge_info, split_info,
                  route_info_output_.distance_to_second_road_merge);

          if (merge_region_lane_tupo_info.is_valid) {
            second_merge_region_lane_tupo_info = merge_region_lane_tupo_info;
            second_merge_region_lane_tupo_info.distance_to_split_point =
                merge_info_temp.second;
            second_merge_region_lane_tupo_info.split_direction =
                static_cast<SplitDirection>(
                    route_info_output_.second_merge_direction);
            second_merge_region_lane_tupo_info.is_other_merge_to_road =
                is_road_merged_by_other_lane;
            second_merge_region_lane_tupo_info.is_ramp_merge =
                is_ramp_road_merge;
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

    if (next_seg_of_first_road_merge != nullptr &&
        seg_of_first_road_merge != nullptr) {
      if (sdpro_map.isRamp(seg_of_first_road_merge->link_type()) &&
          sdpro_map.isRamp(next_seg_of_first_road_merge->link_type())) {
        route_info_output_.is_continuous_ramp = true;
      }
    }
  } else {
    route_info_output_.distance_to_first_road_merge = NL_NMAX;
    ILOG_DEBUG << "merge_info.empty()!!!!!!!";
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
          route_info_output_.split_dir_dis_info_list.emplace_back(
              std::make_pair(static_cast<SplitRelativeDirection>(
                                 route_info_output_.first_split_direction),
                             route_info_output_.distance_to_first_road_split));
        } else if (is_find_first_split_info) {
          route_info_output_.distance_to_second_road_split =
              split_info[i].second;
          SplitSegInfo split_seg_info;
          split_seg_info = MakesureSplitDirection(*split_segment, sd_map);
          route_info_output_.second_split_direction =
              split_seg_info.split_direction;
          route_info_output_.split_dir_dis_info_list.emplace_back(
              std::make_pair(static_cast<SplitRelativeDirection>(
                                 route_info_output_.second_split_direction),
                             route_info_output_.distance_to_second_road_split));
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
    ILOG_DEBUG << "split_info.empty()!!!!!!!";
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
    ILOG_DEBUG << "split_info.empty()!!!!!!!";
  }
}

void RouteInfo::CaculateSplitInfo(
    const ad_common::sdpromap::SDProMap& sdpro_map,
    const iflymapdata::sdpro::LinkInfo_Link& link, const double nearest_s,
    const double max_search_length) {
  route_info_output_.first_split_dir_dis_info = std::make_pair(None, NL_NMAX);
  const auto& split_info =
      sdpro_map.GetSplitInfoList(link.id(), nearest_s, max_search_length);
  route_info_output_.split_dir_dis_info_list.clear();
  if (!split_info.empty()) {
    int traverse_num = 0;
    for (int i = 0; i < split_info.size(); i++) {
      if (split_info[i].second > 3000.0) {
        break;
      }
      const auto split_link = split_info[i].first;
      if (split_link && split_info[i].second > 0) {
        if (!split_link) {
          break;
        }
        const auto split_next_link =
            sdpro_map.GetNextLinkOnRoute(split_link->id());
        if (!split_next_link) {
          break;
        }
        const auto split_last_link =
            sdpro_map.GetPreviousLinkOnRoute(split_link->id());
        if (!split_last_link) {
          break;
        }
        const auto& out_link = split_link->successor_link_ids();
        auto other_link_id =
            out_link[0] == split_next_link->id() ? out_link[1] : out_link[0];
        const auto& other_link = sdpro_map.GetLinkOnRoute(other_link_id);
        if (!other_link) {
          break;
        }
        SplitSegInfo split_seg_info;
        split_seg_info = MakesureSplitDirection(*split_link, sdpro_map);
        if (i == 0) {
          route_info_output_.distance_to_first_road_split =
              split_info[i].second;
          route_info_output_.first_split_direction =
              split_seg_info.split_direction;
          route_info_output_.split_seg_forward_lane_nums =
              split_seg_info.split_seg_forward_lane_nums;
          route_info_output_.split_next_seg_forward_lane_nums =
              split_seg_info.split_next_seg_forward_lane_nums;
        }
        auto split_region_lane_tupo_info = CalculateSplitRegionLaneTupoInfo(
            *split_link, sdpro_map, split_info, split_info[i].second);

        if (split_region_lane_tupo_info.is_valid == true ||
            (split_link != nullptr && other_link != nullptr &&
             split_link->has_link_type() &&
             !sdpro_map.isSaPa(split_link->link_type()) &&
             !sdpro_map.isTollStation(split_link->link_type()) &&
             (other_link->link_class() ==
                  iflymapdata::sdpro::LinkClass::LC_EXPRESSWAY ||
              other_link->link_class() ==
                  iflymapdata::sdpro::LinkClass::LC_CITY_EXPRESSWAY)) &&
                split_next_link->lane_num() <= other_link->lane_num()) {
          split_region_lane_tupo_info.distance_to_split_point =
              split_info[i].second;
          split_region_lane_tupo_info.split_direction =
              static_cast<SplitDirection>(split_seg_info.split_direction);
          route_info_output_.split_region_info_list.emplace_back(
              split_region_lane_tupo_info);
          route_info_output_.first_split_dir_dis_info =
              std::make_pair(static_cast<SplitRelativeDirection>(
                                 route_info_output_.first_split_direction),
                             route_info_output_.distance_to_first_road_split);
          route_info_output_.split_dir_dis_info_list.emplace_back(
              std::make_pair(static_cast<SplitRelativeDirection>(
                                 route_info_output_.first_split_direction),
                             route_info_output_.distance_to_first_road_split));
          traverse_num++;
        }
        double dis_between_first_split_and_merge = 100.0;
        double dis_between_first_and_second_split = -100.0;
        if (!route_info_output_.split_region_info_list.empty() &&
            traverse_num == 1) {
          if (!route_info_output_.split_region_info_list[0].is_valid) {
            if (!route_info_output_.merge_region_info_list.empty()) {
              dis_between_first_split_and_merge =
                  route_info_output_.split_region_info_list[0]
                      .distance_to_split_point -
                  route_info_output_.merge_region_info_list[0]
                      .distance_to_split_point;
            }
            if (route_info_output_.split_region_info_list.size() > 1) {
              dis_between_first_and_second_split =
                  route_info_output_.split_region_info_list[0]
                      .distance_to_split_point -
                  route_info_output_.split_region_info_list[1]
                      .distance_to_split_point;
            }
            // 在存在至少1个交换区fp的情况下，猜测完整的交换区信息
            if ((!route_info_output_.split_region_info_list[0]
                      .start_fp_point.isEmpty() ||
                 !route_info_output_.split_region_info_list[0]
                      .end_fp_point.isEmpty()) &&
                !route_info_output_.split_region_info_list[0].is_valid) {
              route_info_output_.split_region_info_list[0]
                  .recommend_lane_num.emplace_back(split_last_link->lane_num(),
                                                   std::vector<int>{});
              route_info_output_.split_region_info_list[0]
                  .recommend_lane_num.emplace_back(split_link->lane_num(),
                                                   std::vector<int>{});
              route_info_output_.split_region_info_list[0]
                  .recommend_lane_num.emplace_back(split_next_link->lane_num(),
                                                   std::vector<int>{});
              route_info_output_.split_region_info_list[0]
                  .recommend_lane_num.emplace_back(other_link->lane_num(),
                                                   std::vector<int>{});
              route_info_output_.split_region_info_list[0].is_valid = true;
            }
            if (route_info_output_.split_region_info_list[0]
                    .start_fp_point.isEmpty()) {
              route_info_output_.split_region_info_list[0]
                  .start_fp_point.fp_distance_to_split_point =
                  dis_between_first_split_and_merge > 0
                      ? -std::min(dis_between_first_split_and_merge, 100.0)
                      : -100.0;
            }
            if (route_info_output_.split_region_info_list[0]
                    .end_fp_point.isEmpty()) {
              if (dis_between_first_split_and_merge > 0) {
                route_info_output_.split_region_info_list[0]
                    .end_fp_point.fp_distance_to_split_point = std::min(
                    std::abs(dis_between_first_and_second_split), 100.0);
              } else {
                double min_distance =
                    std::min(std::abs(dis_between_first_and_second_split),
                             std::abs(dis_between_first_split_and_merge));
                route_info_output_.split_region_info_list[0]
                    .end_fp_point.fp_distance_to_split_point =
                    std::min(min_distance, 100.0);
              }
            }
          }
        }
      }
    }
  } else {
    route_info_output_.distance_to_first_road_split = NL_NMAX;
    route_info_output_.first_split_direction = RAMP_NONE;
    ILOG_DEBUG << "split_info.empty()!!!!!!!";
  }

  const auto& split_region_info_list =
      route_info_output_.split_region_info_list;
  for (const auto& split_region_info : split_region_info_list) {
    if (split_region_info.is_ramp_split) {
      route_info_output_.dis_to_ramp =
          split_region_info.distance_to_split_point;
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
  if (last_merge_link && last_merge_link->predecessor_link_ids().size() == 2) {
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
  // 计算在主路上距离上一个split点的信息
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
  // TODO(fengwang31)：把这两个合并起来
  // 计算在主路上距离上一个split点的信息
  //  const iflymapdata::sdpro::LinkInfo_Link* last_split_link =
  //      sdpro_map.GetPreviousLinkOnRoute(link.id());
  //  double sum_dis_to_last_split_point = nearest_s;
  //  route_info_output_.sum_dis_to_last_split_point = NL_NMAX;
  //  if (!sdpro_map.isRamp(link.link_type())) {
  //    if (last_split_link != nullptr) {
  //      while (last_split_link->successor_link_ids().size() == 1) {
  //        sum_dis_to_last_split_point =
  //            sum_dis_to_last_split_point + last_split_link->length() * 0.01;
  //        last_split_link =
  //            sdpro_map.GetPreviousLinkOnRoute(last_split_link->id());
  //        if (!last_split_link) {
  //          break;
  //        }
  //      }
  //      if (last_split_link &&
  //          last_split_link->successor_link_ids().size() >= 2) {
  //        if (!sdpro_map.isRamp(last_split_link->link_type())) {
  //          route_info_output_.sum_dis_to_last_split_point =
  //              sum_dis_to_last_split_point;
  //        }
  //      }
  //    }
  //  }

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
      // split_seg_info = MakesureSplitDirection(*temp_last_split_seg,
      // sdpro_map); route_info_output_.last_split_seg_dir =
      // split_seg_info.split_direction;
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
    ILOG_DEBUG << "not find toll station";
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
    ILOG_DEBUG << "not find toll station";
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
      ILOG_DEBUG << "in segment is nullptr!!!!!!!!";
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
      ILOG_DEBUG << "enu points error!!!!!!!!!!";
    }
  } else {
    ILOG_DEBUG << "out_link_size != 2!!!!!!!!";
  }
  return merge_direction;
}

RampDirection RouteInfo::MakesureMergeDirection(
    const iflymapdata::sdpro::LinkInfo_Link& merge_link,
    const ad_common::sdpromap::SDProMap& sdpro_map) {
  const auto in_link_size = merge_link.predecessor_link_ids().size();
  RampDirection merge_direction = RAMP_NONE;
  const auto& in_link = merge_link.predecessor_link_ids();
  const iflymapdata::sdpro::LinkInfo_Link* other_link = nullptr;
  // fengwang31(TODO):暂时假设在merge处只有两个方向
  const auto merge_last_segment =
      sdpro_map.GetPreviousLinkOnRoute(merge_link.id());
  if (!merge_last_segment) {
    ILOG_DEBUG << "in segment is nullptr!!!!!!!!";
    return merge_direction;
  }
  ad_common::math::Vec2d segment_in_route_dir_vec;
  ad_common::math::Vec2d segment_not_in_route_dir_vec;
  Point2D anchor_point_of_cur_seg_to_last_seg = {
      merge_link.points().boot().points().begin()->x(),
      merge_link.points().boot().points().begin()->y()};

  if (in_link_size == 2) {
    auto other_link_id =
        in_link[0] == merge_last_segment->id() ? in_link[1] : in_link[0];
    other_link = sdpro_map.GetLinkOnRoute(other_link_id);
    if (other_link == nullptr) {
      return merge_direction;
    }
  } else if (in_link_size >= 3) {
    for (int i = 0; i < in_link_size; i++) {
      if (in_link[i] != merge_last_segment->id()) {
        other_link = sdpro_map.GetLinkOnRoute(in_link[i]);
        if (other_link != nullptr &&
            (other_link->link_class() ==
                 iflymapdata::sdpro::LinkClass::LC_EXPRESSWAY ||
             other_link->link_class() ==
                 iflymapdata::sdpro::LinkClass::LC_CITY_EXPRESSWAY)) {
          break;
        }
      }
    }
  }
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
    ILOG_DEBUG << "enu points error!!!!!!!!!!";
  }
  return merge_direction;
}

const SdMapSwtx::Segment* RouteInfo::UpdateEgoSegmentInfo(
    const ad_common::sdmap::SDMap& sd_map, double* nearest_s) {
  if (!sdmap_valid_) {
    return nullptr;
  }
  const SdMapSwtx::Segment* segment = nullptr;
  if (!nearest_s) {
    return segment;
  }
  const double max_search_length = 7000.0;  // 搜索7km范围内得地图信息
  const double search_distance = 50.0;
  const double max_heading_diff = PI / 4;
  // 获取当前的segment
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
  ILOG_DEBUG << "current_segment_passed_distance:"
             << route_info_output_.current_segment_passed_distance;
  if (!current_segment) {
    return segment;
  }

  segment = current_segment;
  // debug当前segment的经纬度信息
  if (current_segment->shape_points_size() > 0) {
    const auto& temp_point_LLH = current_segment->shape_points(0);
    ILOG_DEBUG << "lat:" << temp_point_LLH.lat()
               << ",lon:" << temp_point_LLH.lon()
               << ",height:" << temp_point_LLH.height();
    JSON_DEBUG_VALUE("point_LLH_lat", temp_point_LLH.lat());
    JSON_DEBUG_VALUE("point_LLH_lat", temp_point_LLH.lon());
    JSON_DEBUG_VALUE("point_LLH_lat", temp_point_LLH.height());
  }
  JSON_DEBUG_VALUE("current_segment_id", current_segment->id());
  JSON_DEBUG_VALUE("forward_lane_num", current_segment->forward_lane_num());

  // 判断自车当前是否在高速或者高架上
  if (current_segment->has_priority()) {
    if (current_segment->priority() == SdMapSwtx::RoadPriority::EXPRESSWAY ||
        current_segment->priority() ==
            SdMapSwtx::RoadPriority::CITY_EXPRESSWAY) {
      route_info_output_.is_ego_on_expressway = true;
      if (current_segment->priority() == SdMapSwtx::RoadPriority::EXPRESSWAY) {
        route_info_output_.is_ego_on_expressway_hmi = true;
      } else {
        route_info_output_.is_ego_on_city_expressway_hmi = true;
      }
    } else {
      ILOG_DEBUG << "current position not in EXPRESSWAY!!!";
      return segment;
    }
  } else {
    ILOG_DEBUG << "update ego link info failed!!!";
    return segment;
  }

  route_info_output_.is_in_sdmaproad = true;
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
    const ad_common::sdpromap::SDProMap& sdpro_map, double* nearest_s,
    double* nearest_l) {
  const iflymapdata::sdpro::LinkInfo_Link* link = nullptr;
  if (!nearest_s) {
    route_info_output_.reset();
    return link;
  }

  double s = 0.0;
  double l = 0.0;
  auto current_link = CalculateCurrentLink(&s, &l);
  if (!current_link) {
    route_info_output_.reset();
    return link;
  }

  current_link_ = current_link;

  route_info_output_.current_segment_passed_distance = s;
  LOG_DEBUG("current_segment_passed_distance:%f\n",
            route_info_output_.current_segment_passed_distance);

  // debug当前segment的经纬度信息
  if (current_link->llh_points().points().size() > 0) {
    const auto& temp_point_LLH = current_link->llh_points().points(0);
    ILOG_DEBUG << "lat:" << temp_point_LLH.lat()
               << ",lon:" << temp_point_LLH.lon()
               << ",height:" << temp_point_LLH.height();
    JSON_DEBUG_VALUE("point_LLH_lat", temp_point_LLH.lat());
    JSON_DEBUG_VALUE("point_LLH_lat", temp_point_LLH.lon());
    JSON_DEBUG_VALUE("point_LLH_lat", temp_point_LLH.height());
  }
  JSON_DEBUG_VALUE("current_segment_id", current_link->id());
  JSON_DEBUG_VALUE("forward_lane_num", current_link->lane_num());

  if (!route_info_output_.is_in_sdmaproad) {
    // 判断自车当前是否在高速或者高架上
    if (current_link->link_class() ==
            iflymapdata::sdpro::LinkClass::LC_EXPRESSWAY ||
        current_link->link_class() ==
            iflymapdata::sdpro::LinkClass::LC_CITY_EXPRESSWAY) {
      route_info_output_.is_ego_on_expressway = true;
      if (current_link->link_class() ==
          iflymapdata::sdpro::LinkClass::LC_EXPRESSWAY) {
        route_info_output_.is_ego_on_expressway_hmi = true;
      } else {
        route_info_output_.is_ego_on_city_expressway_hmi = true;
      }
      route_info_output_.is_in_sdmaproad = true;
    } else {
      ILOG_DEBUG << "current position not in EXPRESSWAY!!!";
      route_info_output_.reset();
      return link;
    }
  }

  link = current_link;
  *nearest_s = s;
  *nearest_l = l;
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
    // ad_common::sdmap::SDMap sd_map_temp;
    const int res = sd_map_.LoadMapFromProto(local_view.sd_map_info);
    if (res == 0) {
      // sd_map_ = std::move(sd_map_temp);
      sdmap_valid_ = true;
      sd_map_info_updated_timestamp_ = sd_map_info_current_timestamp;
    }
  }
  if (sd_map_info_current_timestamp - sd_map_info_updated_timestamp_ >
      kStaticMapOvertimeThreshold) {
    // 距离上一次更新时间超过阈值，则认为无效报错
    sdmap_valid_ = false;
    ILOG_DEBUG << "error!!! because more than 20s no update hdmap!!!";
  }
  JSON_DEBUG_VALUE("sdmap_valid_", sdmap_valid_)
  return sdmap_valid_;
}

bool RouteInfo::UpdateSdProMap(const LocalView& local_view) {
  const auto& sdpro_map_info = local_view.sdpro_map_info;
  const auto sdpro_map_info_current_timestamp =
      sdpro_map_info.header().timestamp();
  if (sdpro_map_info_current_timestamp != sdpro_map_info_updated_timestamp_) {
    // ad_common::sdpromap::SDProMap sdpro_map_temp;
    int res = sdpro_map_.LoadMapFromProto(sdpro_map_info);
    // 不会返回其他值，先直接用sdpro_map_
    if (res == 0) {
      // sdpro_map_ = std::move(sdpro_map_temp);
      sdpromap_valid_ = true;
      sdpro_map_info_updated_timestamp_ = sdpro_map_info_current_timestamp;
    }
  }
  if (sdpro_map_info_current_timestamp - sdpro_map_info_updated_timestamp_ >
      kStaticMapOvertimeThreshold) {
    // 距离上一次更新时间超过阈值，则认为无效报错
    sdpromap_valid_ = false;
    ILOG_DEBUG << "error!!! because more than 20s no update hdmap!!!";
  }
  JSON_DEBUG_VALUE("sdpromap_valid_", sdpromap_valid_)
  return sdpromap_valid_;
}

bool RouteInfo::UpdateStaticMap(const LocalView& local_view) {
  const auto& static_map_info = local_view.static_map_info;
  const auto static_map_info_current_timestamp =
      static_map_info.header().timestamp();

  if (static_map_info_current_timestamp != static_map_info_updated_timestamp_) {
    ad_common::hdmap::HDMap hd_map_tmp;
    const int res = hd_map_tmp.LoadMapFromProto(static_map_info.road_map());
    if (res == 0) {
      hd_map_ = std::move(hd_map_tmp);
      hdmap_valid_ = true;
      static_map_info_updated_timestamp_ = static_map_info_current_timestamp;
    }
  }
  if (static_map_info_current_timestamp - static_map_info_updated_timestamp_ >
      kStaticMapOvertimeThreshold) {
    // 距离上一次更新时间超过阈值，则认为无效报错
    hdmap_valid_ = false;
    ILOG_DEBUG << "error!!! because more than 20s no update hdmap!!!";
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

  // 判断是否是正在接近匝道
  const double dis_between_first_road_split_and_ramp =
      route_info_output_.distance_to_first_road_split -
      route_info_output_.dis_to_ramp;
  const double allow_error = 5.0;
  route_info_output_.is_nearing_ramp =
      fabs(dis_between_first_road_split_and_ramp) < allow_error &&
      route_info_output_.dis_to_ramp <
          mlc_decider_config_
              .default_pre_triggle_road_to_ramp_distance_threshold_value;

  // 判断哪个场景在前
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
  // 这里是hack匝道延长线800m范围内不在最右侧车道，如果也接近匝道了
  // 根据到匝道的距离判断是匝道延长线汇入在前还是匝道在前
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
  // 常规split场景是前方自车需向左行驶，有其他车道从右边向左边汇入自车道。这种场景，仅需要自车不呆在最右侧车道即可
  // 特殊split场景：自车前方需向右行驶，且当前仅有一个车道(匝道上)，且在split之前有一个右边的汇入。
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
    // 后面的车道数比当前车道数少一条的sceneray，意味着可能有一条车道从当前分流出去
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
  // 目前仅考虑了从右边下匝道的sceneray
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
  // 计算需要变道的次数
  int need_continue_lc_num_on_off_ramp_region = 0;
  if (is_need_continue_lc_on_off_ramp_region) {
    need_continue_lc_num_on_off_ramp_region =
        route_info_output_.last_split_seg_dir == RAMP_ON_LEFT ? -1 : 1;
  }
  route_info_output_.need_continue_lc_num_on_off_ramp_region =
      need_continue_lc_num_on_off_ramp_region;
}

void RouteInfo::UpdateMLCInfoDeciderBaseTencent(
    std::vector<std::shared_ptr<VirtualLane>> relative_id_lanes) {
  mlc_decider_route_info_.feasible_lane_sequence.clear();

  if (relative_id_lanes.empty()) {
    mlc_decider_route_info_.reset();
    return;
  }

  // 1、首先判断自车处在哪个状态上，比如接近ramp、split、merge
  // 1、优先判断split，如果触发了split的条件，需要检查在split之前是否需要处理merge
  // 2、如果split为false，则需要检查是否触发merge。
  auto& split_region_info_list = route_info_output_.split_region_info_list;
  auto& merge_region_info_list = route_info_output_.merge_region_info_list;
  mlc_decider_route_info_.ego_status_on_route = ON_MAIN;  // 每一帧重置状态
  // if (split_region_info_list.empty() && merge_region_info_list.empty()) {
  //   mlc_decider_route_info_.reset();
  //   return;
  // }

  std::vector<int> on_excr_feasible_lane;
  std::vector<int> before_excr_feasible_lane;

  // 2、判断自车状态
  double cur_dis_to_split =
      split_region_info_list.empty()
          ? NL_NMAX
          : split_region_info_list[0].distance_to_split_point;
  // 计算出当前所有交换区的feasible lane
  std::vector<NOASplitRegionInfo> exchange_region_info_list;
  std::vector<int> feasible_lane_sequence;
  std::map<int, double> feasible_lane_distance;
  std::vector<std::pair<std::vector<MLCRequestType>, double>>
      mlc_request_info_list;
  if (!split_region_info_list.empty() &&
      split_region_info_list[0].distance_to_split_point < 500.0 &&
      !split_region_info_list[0].is_valid &&
      (merge_region_info_list.empty() ||
       (!merge_region_info_list.empty() &&
        split_region_info_list[0].distance_to_split_point <
            merge_region_info_list[0].distance_to_split_point))) {
    // 判断前方最近的是不是收费站
    bool is_tollstation_exchange_region = IsClosingTollStationEntrance(
        current_link_, sdpro_map_,
        route_info_output_.current_segment_passed_distance,
        split_region_info_list[0].distance_to_split_point);
    // 处理split信息缺失，向一侧变道的场景
    if (!is_tollstation_exchange_region) {
      for (auto& relative_id_lane : relative_id_lanes) {
        ProcessLaneDistance(relative_id_lane, feasible_lane_distance);
      }
      for (auto& relative_id_lane : relative_id_lanes) {
        if (relative_id_lane->get_relative_id() != 0) {
          continue;
        }
        std::vector<int> task_num;
        if (split_region_info_list[0].split_direction == SPLIT_LEFT) {
          task_num.emplace_back(-1);
          route_info_output_.mlc_request_type_route_info.mlc_request_type =
              OTHER_TYPE_MLC;
          route_info_output_.mlc_request_type_route_info
              .distance_to_exchange_region =
              split_region_info_list[0].distance_to_split_point;
        } else if (split_region_info_list[0].split_direction == SPLIT_RIGHT) {
          task_num.emplace_back(1);
          route_info_output_.mlc_request_type_route_info.mlc_request_type =
              OTHER_TYPE_MLC;
          route_info_output_.mlc_request_type_route_info
              .distance_to_exchange_region =
              split_region_info_list[0].distance_to_split_point;
        }
        if (!task_num.empty()) {
          relative_id_lane->set_current_tasks(task_num);
        }
        feasible_lane_sequence.emplace_back(-1);
        feasible_lane_distance.clear();
        mlc_decider_route_info_.ego_status_on_route = ON_MAIN;
        mlc_decider_route_info_.is_process_split = true;
        mlc_decider_route_info_.feasible_lane_sequence = feasible_lane_sequence;
        mlc_decider_route_info_.static_split_region_info =
            split_region_info_list[0];
        route_info_output_.mlc_decider_route_info = mlc_decider_route_info_;
        return;
      }
    }
  }

  // 判断前方最近的是不是收费站
  double distance_to_next_split = NL_NMAX;
  double distance_to_next_merge = NL_NMAX;
  double distance_to_next_exchange_region = NL_NMAX;
  if (!split_region_info_list.empty()) {
    distance_to_next_split = split_region_info_list[0].distance_to_split_point;
  }
  if (!merge_region_info_list.empty()) {
    distance_to_next_merge = merge_region_info_list[0].distance_to_split_point;
  }
  distance_to_next_exchange_region =
      std::min(distance_to_next_split, distance_to_next_merge);
  double search_dis = std::min(
      std::min(distance_to_next_split, distance_to_next_merge), 3000.0);
  if (!IsClosingTollStationEntrance(
          current_link_, sdpro_map_,
          route_info_output_.current_segment_passed_distance, search_dis) &&
      !IsClosingIntersectionEntrance(
          current_link_, sdpro_map_,
          route_info_output_.current_segment_passed_distance, search_dis)) {
    for (int i = 0; i < split_region_info_list.size(); ++i) {
      auto& split_region_info = split_region_info_list[i];
      if (!split_region_info.is_valid) {
        break;
      }
      mlc_request_info_.clear();
      bool is_calculate_feasible_lane =
          CalculateFeasibleLane(&split_region_info);
      if (i == 0) {
        route_info_output_.distance_to_first_road_split =
            split_region_info.distance_to_split_point;
      } else {
        route_info_output_.distance_to_second_road_split =
            split_region_info.distance_to_split_point;
      }

      if (!is_calculate_feasible_lane) {
        break;
      }
      mlc_request_info_list.emplace_back(
          mlc_request_info_, split_region_info.distance_to_split_point);
      exchange_region_info_list.emplace_back(split_region_info);
    }

    for (int i = 0; i < merge_region_info_list.size(); ++i) {
      auto& merge_region_info = merge_region_info_list[i];
      mlc_request_info_.clear();
      if (i == 0) {
        route_info_output_.distance_to_first_road_merge =
            merge_region_info.distance_to_split_point;
      } else {
        route_info_output_.distance_to_second_road_merge =
            merge_region_info.distance_to_split_point;
      }

      bool is_calculate_feasible_lane = false;
      if (merge_region_info.is_other_merge_to_road) {
        is_calculate_feasible_lane =
            CalculateOtherMergeRoadFeasibleLane(&merge_region_info);
      } else {
        is_calculate_feasible_lane =
            CalculateMergeRegionFeasibleLane(&merge_region_info);
      }
      if (!is_calculate_feasible_lane) {
        break;
      }
      mlc_request_info_list.emplace_back(
          mlc_request_info_, merge_region_info.distance_to_split_point);
      exchange_region_info_list.emplace_back(merge_region_info);
    }
  }

  // 将exchange_region_info_list中按照distance排序
  std::sort(exchange_region_info_list.begin(), exchange_region_info_list.end(),
            [](const NOASplitRegionInfo& exchange_region_temp1,
               const NOASplitRegionInfo& exchange_region_temp2) {
              return exchange_region_temp1.distance_to_split_point <
                     exchange_region_temp2.distance_to_split_point;
            });
  // 将mlc_request_info_list中按照distance排序
  std::sort(mlc_request_info_list.begin(), mlc_request_info_list.end(),
            [](const std::pair<std::vector<MLCRequestType>, double>& a,
               const std::pair<std::vector<MLCRequestType>, double>& b) {
              return a.second < b.second;
            });
  // 根据distance优化feasible lane，考虑3km内的所有exchange
  std::vector<NOASplitRegionInfo> valid_exchange_regions;
  // 筛选3km内的exchange region
  for (const auto& exchange_region : exchange_region_info_list) {
    if (exchange_region.distance_to_split_point <= 3000.0) {
      valid_exchange_regions.emplace_back(exchange_region);
    }
  }

  // 对两种情况下的other_merge取消处理
  // 1. 汇入后车道数变多的场景，无需发起躲避汇流
  // 2. other_merge后近距离存在方向不一致的交换区
  bool is_remove_other_merge = false;
  int other_merge_lane_num = 0;
  if (valid_exchange_regions.size() > 1 &&
      valid_exchange_regions[0].is_other_merge_to_road) {
    bool is_exchange_close =
        valid_exchange_regions[1].distance_to_split_point -
            valid_exchange_regions[0].distance_to_split_point <
        150.0;
    bool is_exchange_direction_conflict =
        valid_exchange_regions[1].is_ramp_merge
            ? valid_exchange_regions[0].split_direction ==
                  valid_exchange_regions[1].split_direction
            : valid_exchange_regions[0].split_direction !=
                  valid_exchange_regions[1].split_direction;

    const auto& merge_seg =
        sdpro_map_.GetLinkOnRoute(valid_exchange_regions[0].split_link_id);
    const auto& merge_seg_last_seg = sdpro_map_.GetPreviousLinkOnRoute(
        valid_exchange_regions[0].split_link_id);
    if (merge_seg != nullptr && merge_seg_last_seg != nullptr &&
        merge_seg->predecessor_link_ids().size() == 2) {
      const auto& merge_seg_last_other_seg_id =
          merge_seg->predecessor_link_ids()[0] == merge_seg_last_seg->id()
              ? merge_seg->predecessor_link_ids()[1]
              : merge_seg->predecessor_link_ids()[0];
      const auto& merge_seg_last_other_seg =
          sdpro_map_.GetLinkOnRoute(merge_seg_last_other_seg_id);
      if (merge_seg_last_other_seg != nullptr) {
        other_merge_lane_num = merge_seg_last_other_seg->lane_num();
      }
    }
    if (is_exchange_close && is_exchange_direction_conflict) {
      if (valid_exchange_regions[0].distance_to_split_point > 150.0) {
        other_merge_lane_num = 0;
      }
      valid_exchange_regions.erase(valid_exchange_regions.begin());
      is_remove_other_merge = true;
    }
  }

  if (!valid_exchange_regions.empty() &&
      valid_exchange_regions[0].is_other_merge_to_road) {
    bool is_no_need_handle_exchange = false;
    const auto& merge_seg =
        sdpro_map_.GetLinkOnRoute(valid_exchange_regions[0].split_link_id);
    const auto& next_merge_seg =
        sdpro_map_.GetNextLinkOnRoute(valid_exchange_regions[0].split_link_id);
    const auto& merge_seg_last_seg = sdpro_map_.GetPreviousLinkOnRoute(
        valid_exchange_regions[0].split_link_id);
    if (merge_seg != nullptr && merge_seg_last_seg != nullptr &&
        next_merge_seg != nullptr &&
        merge_seg->predecessor_link_ids().size() == 2) {
      const auto& merge_seg_last_other_seg_id =
          merge_seg->predecessor_link_ids()[0] == merge_seg_last_seg->id()
              ? merge_seg->predecessor_link_ids()[1]
              : merge_seg->predecessor_link_ids()[0];
      const auto& merge_seg_last_other_seg =
          sdpro_map_.GetLinkOnRoute(merge_seg_last_other_seg_id);
      if (merge_seg_last_other_seg != nullptr) {
        other_merge_lane_num = merge_seg_last_other_seg->lane_num();
        is_no_need_handle_exchange =
            merge_seg_last_seg->lane_num() +
                            merge_seg_last_other_seg->lane_num() <=
                        merge_seg->lane_num() &&
                    merge_seg_last_seg->lane_num() +
                            merge_seg_last_other_seg->lane_num() <=
                        next_merge_seg->lane_num()
                ? true
                : false;
      }
    }

    if (is_no_need_handle_exchange) {
      if (valid_exchange_regions[0].distance_to_split_point > 150.0) {
        other_merge_lane_num = 0;
      }
      valid_exchange_regions.erase(valid_exchange_regions.begin());
      is_remove_other_merge = true;
    }
  }
  // 获取当前地图车道数（不含应急）
  std::vector<int> current_lane_vec;
  iflymapdata::sdpro::FeaturePoint last_fp;
  int map_lane_num = 0;
  int map_emergency_lane_num = 0;
  const double s = route_info_output_.current_segment_passed_distance;
  if (CalculateLastFPInCurrentLink(&last_fp, current_link_, s)) {
    map_lane_num = last_fp.lane_ids().size();
    for (const auto& lane_id : last_fp.lane_ids()) {
      if (IsEmergencyLane(lane_id, sdpro_map_)) {
        map_lane_num--;
        map_emergency_lane_num++;
      }
    }
  } else {
    if (current_link_ != nullptr) {
      map_lane_num = current_link_->lane_num();
    }
  }

  // 可能会出现前方存在车道拓宽点，感知与地图位置不一致的情况
  // 向前搜索25米，看有没有车道拓宽点
  std::map<int, SplitDirection> expand_lane_sequence_vec;
  if (CalculateExpandLaneInfo(expand_lane_sequence_vec, current_link_, s,
                              25.0)) {
    map_lane_num += expand_lane_sequence_vec.size();
  }

  // 获取当前感知车道数
  int perceived_lane_num = 0;
  for (auto& relative_id_lane : relative_id_lanes) {
    if (relative_id_lane->get_relative_id() != 0) {
      continue;
    }
    const auto& lane_nums = relative_id_lane->get_lane_nums();
    int left_lane_num = 0;
    int right_lane_num = 0;
    for (const auto& lane_num : lane_nums) {
      if (lane_num.end > kEpsilon) {
        left_lane_num = lane_num.left_lane_num;
        right_lane_num = lane_num.right_lane_num;
        break;
      }
    }
    perceived_lane_num =
        left_lane_num + right_lane_num + 1 - map_emergency_lane_num;
  }

  // 选取地图车道数与感知车道数中较大者
  int total_lane_num = std::max(map_lane_num, perceived_lane_num);
  if (is_remove_other_merge && other_merge_lane_num > 0 &&
      mlc_decider_route_info_.is_process_split &&
      !exchange_region_info_list.empty() &&
      exchange_region_info_list[0].is_other_merge_to_road &&
      exchange_region_info_list[0].split_direction == SPLIT_RIGHT) {
    total_lane_num += other_merge_lane_num;
  }
  for (int i = 0; i < total_lane_num; ++i) {
    current_lane_vec.emplace_back(i + 1);
  }
  // 为每个exchange region准备数据
  std::vector<std::map<int, double>> exchange_feasible_lane_distances(
      valid_exchange_regions.size());
  int iteration_num = 0;
  double dis_to_last_split_point = NL_NMAX;
  double dis_to_last_merge_point = NL_NMAX;
  double dis_to_last_exchange_point = NL_NMAX;
  double dis_to_last_specific_point = NL_NMAX;
  uint64 last_split_link_id = -1;
  uint64 last_merge_link_id = -1;
  const double passed_dis = route_info_output_.current_segment_passed_distance;

  if (CalculateDistanceToLastSplitPoint(&dis_to_last_split_point, passed_dis,
                                        &last_split_link_id)) {
    dis_to_last_exchange_point =
        std::min(dis_to_last_exchange_point, dis_to_last_split_point);
  }

  if (CalculateDistanceToLastMergePoint(&dis_to_last_merge_point, passed_dis,
                                        &last_merge_link_id)) {
    dis_to_last_exchange_point =
        std::min(dis_to_last_exchange_point, dis_to_last_merge_point);
  }
  // 开始计算feasible lane和对应的distance
  if (valid_exchange_regions.empty()) {
    // 认为前方所有车道均可通行
    feasible_lane_sequence.clear();
    feasible_lane_sequence = current_lane_vec;
    if (mlc_decider_route_info_.first_static_split_region_info.split_link_id !=
        -1) {
      last_exchange_region_info_.last_exchange_info =
          mlc_decider_route_info_.first_static_split_region_info;
      last_exchange_region_info_.is_process_split =
          mlc_decider_route_info_.is_process_split;
      last_exchange_region_info_.is_process_merge =
          mlc_decider_route_info_.is_process_merge;
      last_exchange_region_info_.is_process_other_merge =
          mlc_decider_route_info_.is_process_other_merge;
      if (dis_to_last_exchange_point <
          last_exchange_region_info_.last_exchange_info.end_fp_point
              .fp_distance_to_split_point) {
        feasible_lane_sequence.clear();
        feasible_lane_sequence =
            last_exchange_region_info_.last_exchange_info.recommend_lane_num[1]
                .feasible_lane_sequence;
      }
    }
    for (int i = 0; i < feasible_lane_sequence.size(); i++) {
      feasible_lane_distance[feasible_lane_sequence[i]] = 3000.0;
    }

    // 根据前方merge_fp优化feasible_lane
    std::map<int, SplitDirection> merge_lane;
    std::vector<int> merge_lane_sequence;
    bool is_exist_merge_fp = false;
    if (CalculateMergeLaneInfo(merge_lane, search_dis) && !merge_lane.empty()) {
      for (const auto& [lane_num, split_dir] : merge_lane) {
        merge_lane_sequence.emplace_back(lane_num);
      }

      for (const auto& [lane_num, split_dir] : merge_lane) {
        auto it = std::find(feasible_lane_sequence.begin(),
                            feasible_lane_sequence.end(), lane_num);

        if (it != feasible_lane_sequence.end()) {
          mlc_request_info_.emplace_back(
              MLCRequestType{.lane_num = lane_num,
                             .mlc_request_type = RAMP_TO_MAIN,
                             .split_direction = split_dir});
          feasible_lane_sequence.erase(it);
          is_exist_merge_fp = true;
        }
      }
    }

    // 过滤被删除的feasible_lane_sequence
    for (auto it = feasible_lane_distance.begin();
         it != feasible_lane_distance.end();) {
      auto seq_it = std::find(feasible_lane_sequence.begin(),
                              feasible_lane_sequence.end(), it->first);
      if (seq_it == feasible_lane_sequence.end()) {
        it = feasible_lane_distance.erase(it);
      } else {
        ++it;
      }
    }
    int minVal_seq = 0;
    int maxVal_seq = 0;
    if (!feasible_lane_sequence.empty()) {
      minVal_seq = feasible_lane_sequence[0];
      maxVal_seq = feasible_lane_sequence.back();
    }
    route_info_output_.minVal_seq = minVal_seq;
    route_info_output_.maxVal_seq = maxVal_seq;

    for (auto& relative_id_lane : relative_id_lanes) {
      ProcessLaneDistance(relative_id_lane, feasible_lane_distance);
    }
    for (auto& relative_id_lane : relative_id_lanes) {
      if (relative_id_lane->get_relative_id() != 0) {
        continue;
      }
      const auto& lane_nums = relative_id_lane->get_lane_nums();
      int left_lane_num = 0;
      int right_lane_num = 0;
      EgoMLCRequestType mlc_type = None_MLC;
      for (const auto& lane_num : lane_nums) {
        if (lane_num.end > kEpsilon) {
          left_lane_num = lane_num.left_lane_num;
          right_lane_num = lane_num.right_lane_num;
          break;
        }
      }

      route_info_output_.left_lane_num = left_lane_num;
      route_info_output_.right_lane_num = right_lane_num;
      mlc_decider_route_info_.ego_status_on_route = ON_MAIN;
      mlc_decider_route_info_.feasible_lane_sequence = feasible_lane_sequence;

      int ego_seq = left_lane_num + 1;
      std::vector<int> lc_num_task;
      if (ego_seq >= minVal_seq && ego_seq <= maxVal_seq) {
        mlc_type = None_MLC;
        lc_num_task.clear();
      } else if (ego_seq > maxVal_seq) {
        int err = ego_seq - maxVal_seq;
        for (int i = 0; i < err; i++) {
          lc_num_task.emplace_back(-1);
          mlc_type = RAMP_TO_MAIN;
        }
      } else if (ego_seq < minVal_seq) {
        int err = minVal_seq - ego_seq;
        for (int i = 0; i < err; i++) {
          lc_num_task.emplace_back(1);
          mlc_type = RAMP_TO_MAIN;
        }
      }
      route_info_output_.ego_seq = ego_seq;
      route_info_output_.mlc_request_type_route_info.mlc_request_type =
          mlc_type;
      route_info_output_.mlc_request_type_route_info
          .distance_to_exchange_region =
          route_info_output_.merge_point_info.dis_to_merge_fp;
      relative_id_lane->set_current_tasks(lc_num_task);
      route_info_output_.mlc_decider_route_info = mlc_decider_route_info_;
      return;
    }
  } else {
    std::vector<double> max_distances(valid_exchange_regions.size(), NL_NMAX);

    // 计算每个exchange region的最大距离
    for (size_t i = 0; i < valid_exchange_regions.size(); ++i) {
      double start_fp_distance = std::max(
          valid_exchange_regions[i].start_fp_point.fp_distance_to_split_point,
          -300.0);
      max_distances[i] =
          valid_exchange_regions[i].distance_to_split_point + start_fp_distance;
    }

    // 从最后一个exchange region开始向前优化
    // 限制最多仅优化连续3个，且到3个中最后一个ramp split为止
    bool is_need_optimize = true;
    for (int i = 0; i < valid_exchange_regions.size(); ++i) {
      is_need_optimize =
          !IsClosingTollStationEntrance(
              current_link_, sdpro_map_,
              route_info_output_.current_segment_passed_distance,
              valid_exchange_regions[i].distance_to_split_point) &&
          !IsClosingIntersectionEntrance(
              current_link_, sdpro_map_,
              route_info_output_.current_segment_passed_distance,
              valid_exchange_regions[i].distance_to_split_point);
      bool is_not_merge = !valid_exchange_regions[i].is_ramp_merge &&
                          !valid_exchange_regions[i].is_other_merge_to_road;

      if (valid_exchange_regions[i].is_ramp_merge) {
        break;
      }
      if ((valid_exchange_regions[i].is_ramp_split && is_not_merge) ||
          (valid_exchange_regions[i].split_direction == SPLIT_RIGHT &&
           is_not_merge &&
           valid_exchange_regions[i].recommend_lane_num[3].total_lane_num >=
               valid_exchange_regions[i]
                   .recommend_lane_num[2]
                   .total_lane_num)) {
        if (!is_need_optimize) {
          break;
        }
        iteration_num = i;
        break;
      }
    }
    OptimizeFeasibleLanesByDistance(valid_exchange_regions[0],
                                    exchange_feasible_lane_distances[0],
                                    max_distances[0], current_lane_vec.size());
    if (iteration_num > 0) {
      OptimizeFeasibleLanesByDistance(
          valid_exchange_regions[iteration_num],
          exchange_feasible_lane_distances[iteration_num],
          max_distances[iteration_num], current_lane_vec.size());
    }
  }
  // 针对ramp_split和ramp_merge后存在接近交换区场景
  if (valid_exchange_regions.size() > 1) {
    bool is_first_no_merge = !valid_exchange_regions[0].is_ramp_merge &&
                             !valid_exchange_regions[0].is_other_merge_to_road;
    bool is_second_no_merge = !valid_exchange_regions[1].is_ramp_merge &&
                              !valid_exchange_regions[1].is_other_merge_to_road;
    if (valid_exchange_regions[0].is_ramp_merge && is_second_no_merge &&
        valid_exchange_regions[1].distance_to_split_point -
                valid_exchange_regions[0].distance_to_split_point +
                valid_exchange_regions[1]
                    .start_fp_point.fp_distance_to_split_point -
                valid_exchange_regions[0]
                    .end_fp_point.fp_distance_to_split_point <
            200.0) {
      std::vector<int> temp_feasible_lane;
      if (valid_exchange_regions[1].split_direction == SPLIT_RIGHT) {
        temp_feasible_lane.emplace_back(valid_exchange_regions[0]
                                            .recommend_lane_num[0]
                                            .feasible_lane_sequence.back());
      } else {
        temp_feasible_lane.emplace_back(valid_exchange_regions[0]
                                            .recommend_lane_num[0]
                                            .feasible_lane_sequence.front());
      }
      valid_exchange_regions[0].recommend_lane_num[0].feasible_lane_sequence =
          temp_feasible_lane;
    }
    if (valid_exchange_regions[0].is_ramp_split && is_first_no_merge &&
        is_second_no_merge &&
        valid_exchange_regions[1].distance_to_split_point -
                valid_exchange_regions[0].distance_to_split_point +
                valid_exchange_regions[1]
                    .start_fp_point.fp_distance_to_split_point -
                valid_exchange_regions[0]
                    .end_fp_point.fp_distance_to_split_point <
            200.0) {
      std::vector<int> temp_feasible_lane;
      if (valid_exchange_regions[1].split_direction == SPLIT_RIGHT) {
        temp_feasible_lane.emplace_back(valid_exchange_regions[0]
                                            .recommend_lane_num[0]
                                            .feasible_lane_sequence.back());
      } else {
        temp_feasible_lane.emplace_back(valid_exchange_regions[0]
                                            .recommend_lane_num[0]
                                            .feasible_lane_sequence.front());
      }
      valid_exchange_regions[0].recommend_lane_num[0].feasible_lane_sequence =
          temp_feasible_lane;
    }
  }
  if (valid_exchange_regions.empty()) {
    return;
  }
  const auto first_exchange_region_info = valid_exchange_regions[0];
  if (first_exchange_region_info.split_link_id !=
          mlc_decider_route_info_.first_static_split_region_info
              .split_link_id &&
      mlc_decider_route_info_.first_static_split_region_info.split_link_id !=
          -1) {
    last_exchange_region_info_.last_exchange_info =
        mlc_decider_route_info_.first_static_split_region_info;
    last_exchange_region_info_.is_process_split =
        mlc_decider_route_info_.is_process_split;
    last_exchange_region_info_.is_process_merge =
        mlc_decider_route_info_.is_process_merge;
    last_exchange_region_info_.is_process_other_merge =
        mlc_decider_route_info_.is_process_other_merge;
  }
  // 判断当前处理的场景
  if (!merge_region_info_list.empty() &&
      first_exchange_region_info.split_link_id ==
          merge_region_info_list[0].split_link_id &&
      mlc_decider_route_info_.ego_status_on_route == ON_MAIN) {
    if (first_exchange_region_info.is_other_merge_to_road) {
      mlc_decider_route_info_.is_process_other_merge = true;
      mlc_decider_route_info_.is_process_merge = false;
      mlc_decider_route_info_.is_process_split = false;
      mlc_decider_route_info_.static_merge_region_info =
          first_exchange_region_info;
    } else {
      mlc_decider_route_info_.is_process_merge = true;
      mlc_decider_route_info_.is_process_split = false;
      mlc_decider_route_info_.is_process_other_merge = false;
      mlc_decider_route_info_.static_merge_region_info =
          first_exchange_region_info;
    }
  } else {
    mlc_decider_route_info_.is_process_split = true;
    mlc_decider_route_info_.is_process_merge = false;
    mlc_decider_route_info_.is_process_other_merge = false;
    mlc_decider_route_info_.static_split_region_info =
        first_exchange_region_info;
  }
  // 判断当前处于的状态
  bool is_entery_exchange_region_front =
      first_exchange_region_info.distance_to_split_point <
      std::abs(
          first_exchange_region_info.start_fp_point.fp_distance_to_split_point);
  bool is_entery_exchange_region_rear = false;
  if (last_exchange_region_info_.is_process_split) {
    is_entery_exchange_region_rear =
        dis_to_last_split_point <
            last_exchange_region_info_.last_exchange_info.end_fp_point
                .fp_distance_to_split_point &&
        last_split_link_id ==
            last_exchange_region_info_.last_exchange_info.split_link_id;
    dis_to_last_specific_point = dis_to_last_split_point;
  } else if (last_exchange_region_info_.is_process_merge ||
             last_exchange_region_info_.is_process_other_merge) {
    is_entery_exchange_region_rear =
        dis_to_last_merge_point <
            last_exchange_region_info_.last_exchange_info.end_fp_point
                .fp_distance_to_split_point &&
        last_merge_link_id ==
            last_exchange_region_info_.last_exchange_info.split_link_id;
    dis_to_last_specific_point = dis_to_last_merge_point;
  }

  if (is_entery_exchange_region_front) {
    mlc_decider_route_info_.ego_status_on_route = IN_EXCHANGE_AREA_FRONT;
  } else if (is_entery_exchange_region_rear) {
    mlc_decider_route_info_.ego_status_on_route = IN_EXCHANGE_AREA_REAR;
  }
  mlc_decider_route_info_.first_static_split_region_info =
      first_exchange_region_info;
  route_info_output_.current_exchange_region_info = first_exchange_region_info;

  // 输出到上一个交换区终点的距离，在交换区rear位置计算到前方end_fp距离(负值)
  if (last_exchange_region_info_.is_process_split &&
      last_split_link_id ==
          last_exchange_region_info_.last_exchange_info.split_link_id) {
    route_info_output_.last_split_end_point_distance =
        dis_to_last_split_point - last_exchange_region_info_.last_exchange_info
                                      .end_fp_point.fp_distance_to_split_point;
  } else {
    route_info_output_.last_split_end_point_distance = NL_NMAX;
  }

  // 判断当前是否接近汇入汇出
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const double ego_v = ego_state->ego_v();
  route_info_output_.is_closing_split = false;
  route_info_output_.is_closing_merge = false;
  if (mlc_decider_route_info_.is_process_split &&
      first_exchange_region_info.is_ramp_split) {
    route_info_output_.is_closing_split =
        first_exchange_region_info.distance_to_split_point < ego_v * 10.0;
  }
  if (mlc_decider_route_info_.is_process_merge &&
      !first_exchange_region_info.is_other_merge_to_road) {
    route_info_output_.is_closing_merge =
        first_exchange_region_info.distance_to_split_point < ego_v * 10.0;
  }
  // 状态流转，分配feasible_lane_sequence
  switch (mlc_decider_route_info_.ego_status_on_route) {
    case ON_MAIN: {
      feasible_lane_sequence = first_exchange_region_info.recommend_lane_num[0]
                                   .feasible_lane_sequence;
      std::vector<int> exchange_feasible_lane_sequence =
          first_exchange_region_info.recommend_lane_num[1]
              .feasible_lane_sequence;
      // 根据当前车道数来优化feasible_lane_sequence
      feasible_lane_sequence =
          GetIntersection(current_lane_vec, feasible_lane_sequence);
      for (int i = 0; i < feasible_lane_sequence.size(); i++) {
        auto& lane_distance = feasible_lane_distance[feasible_lane_sequence[i]];
        // 先计算到交换区起点
        if (feasible_lane_sequence.size() == 1) {
          lane_distance = first_exchange_region_info.distance_to_split_point +
                          first_exchange_region_info.start_fp_point
                              .fp_distance_to_split_point;
          break;
        }
        if (!exchange_feasible_lane_distances.empty() &&
            exchange_feasible_lane_distances[0].find(
                feasible_lane_sequence[i]) !=
                exchange_feasible_lane_distances[0].end()) {
          lane_distance =
              exchange_feasible_lane_distances[0][feasible_lane_sequence[i]];
        } else {
          if (feasible_lane_sequence[i] ==
              first_exchange_region_info.avoide_lane_num) {
            lane_distance = first_exchange_region_info.distance_to_split_point +
                            first_exchange_region_info.start_fp_point
                                .fp_distance_to_split_point;
          } else {
            lane_distance = 0.0;
          }
        }
        // 加上起点到终点的距离
        if (std::find(exchange_feasible_lane_sequence.begin(),
                      exchange_feasible_lane_sequence.end(),
                      feasible_lane_sequence[i]) !=
            exchange_feasible_lane_sequence.end()) {
          lane_distance = lane_distance +
                          first_exchange_region_info.end_fp_point
                              .fp_distance_to_split_point -
                          first_exchange_region_info.start_fp_point
                              .fp_distance_to_split_point;
        }
        // 再看后一个交换区的信息
        if (valid_exchange_regions.size() > 1 &&
            !valid_exchange_regions[0].is_ramp_merge &&
            !valid_exchange_regions[0].is_ramp_split) {
          double next_exchange_opt_distance =
              valid_exchange_regions[1].distance_to_split_point -
              first_exchange_region_info.distance_to_split_point +
              valid_exchange_regions[1]
                  .start_fp_point.fp_distance_to_split_point -
              first_exchange_region_info.end_fp_point
                  .fp_distance_to_split_point;
          OptimizeFeasibleLanesByDistance(
              valid_exchange_regions[1], exchange_feasible_lane_distances[1],
              next_exchange_opt_distance, current_lane_vec.size());
          std::vector<int> next_feasible_lane_sequence =
              valid_exchange_regions[1]
                  .recommend_lane_num[0]
                  .feasible_lane_sequence;
          if (std::find(next_feasible_lane_sequence.begin(),
                        next_feasible_lane_sequence.end(),
                        feasible_lane_sequence[i]) !=
              next_feasible_lane_sequence.end()) {
            lane_distance +=
                exchange_feasible_lane_distances[1][feasible_lane_sequence[i]];
          }
        }
      }
      break;
    }

    case IN_EXCHANGE_AREA_FRONT: {
      feasible_lane_sequence =
          mlc_decider_route_info_.first_static_split_region_info
              .recommend_lane_num[1]
              .feasible_lane_sequence;
      for (int i = 0; i < feasible_lane_sequence.size(); i++) {
        feasible_lane_distance[feasible_lane_sequence[i]] =
            mlc_decider_route_info_.first_static_split_region_info
                .distance_to_split_point +
            first_exchange_region_info.end_fp_point.fp_distance_to_split_point;
        // 再看后一个交换区的信息
        if (valid_exchange_regions.size() > 1 &&
            !valid_exchange_regions[0].is_ramp_merge &&
            !valid_exchange_regions[0].is_ramp_split) {
          double next_exchange_opt_distance =
              valid_exchange_regions[1].distance_to_split_point -
              first_exchange_region_info.distance_to_split_point +
              valid_exchange_regions[1]
                  .start_fp_point.fp_distance_to_split_point -
              first_exchange_region_info.end_fp_point
                  .fp_distance_to_split_point;
          OptimizeFeasibleLanesByDistance(
              valid_exchange_regions[1], exchange_feasible_lane_distances[1],
              next_exchange_opt_distance, current_lane_vec.size());
          std::vector<int> next_feasible_lane_sequence =
              valid_exchange_regions[1]
                  .recommend_lane_num[0]
                  .feasible_lane_sequence;
          if (std::find(next_feasible_lane_sequence.begin(),
                        next_feasible_lane_sequence.end(),
                        feasible_lane_sequence[i]) !=
              next_feasible_lane_sequence.end()) {
            feasible_lane_distance[feasible_lane_sequence[i]] +=
                exchange_feasible_lane_distances[1][feasible_lane_sequence[i]];
          }
        }
      }
      break;
    }

    case IN_EXCHANGE_AREA_REAR: {
      feasible_lane_sequence =
          last_exchange_region_info_.last_exchange_info.recommend_lane_num[1]
              .feasible_lane_sequence;
      for (int i = 0; i < feasible_lane_sequence.size(); i++) {
        feasible_lane_distance[feasible_lane_sequence[i]] =
            last_exchange_region_info_.last_exchange_info.end_fp_point
                .fp_distance_to_split_point -
            dis_to_last_specific_point;
        // 再看后一个交换区的信息
        if (!valid_exchange_regions.empty()) {
          std::vector<int> next_feasible_lane_sequence =
              valid_exchange_regions[0]
                  .recommend_lane_num[0]
                  .feasible_lane_sequence;
          if (std::find(next_feasible_lane_sequence.begin(),
                        next_feasible_lane_sequence.end(),
                        feasible_lane_sequence[i]) !=
              next_feasible_lane_sequence.end()) {
            feasible_lane_distance[feasible_lane_sequence[i]] +=
                exchange_feasible_lane_distances[0][feasible_lane_sequence[i]];
          }
        }
      }
      break;
    }
  }
  // 如果存在ramp_split优化，要考虑其剩余距离
  for (int i = 0; i < feasible_lane_sequence.size(); i++) {
    auto& lane_distance = feasible_lane_distance[feasible_lane_sequence[i]];
    if (iteration_num > 0 &&
        exchange_feasible_lane_distances[iteration_num].find(
            feasible_lane_sequence[i]) !=
            exchange_feasible_lane_distances[iteration_num].end()) {
      if (exchange_feasible_lane_distances[iteration_num]
                                          [feasible_lane_sequence[i]] <
          feasible_lane_distance[feasible_lane_sequence[i]]) {
        feasible_lane_distance[feasible_lane_sequence[i]] =
            exchange_feasible_lane_distances[iteration_num]
                                            [feasible_lane_sequence[i]];
      }
    }
  }
  // 根据前方merge_fp优化feasible_lane
  std::map<int, SplitDirection> merge_lane;
  SplitDirection merge_point_direction = SPLIT_NONE;
  std::vector<int> merge_lane_sequence;
  bool is_exist_merge_fp = false;
  if (CalculateMergeLaneInfo(
          merge_lane, first_exchange_region_info.distance_to_split_point) &&
      !merge_lane.empty()) {
    for (const auto& [lane_num, split_dir] : merge_lane) {
      merge_lane_sequence.emplace_back(lane_num);
      merge_point_direction = split_dir;
    }

    mlc_request_info_.clear();
    for (const auto& [lane_num, split_dir] : merge_lane) {
      auto it = std::find(feasible_lane_sequence.begin(),
                          feasible_lane_sequence.end(), lane_num);
      mlc_request_info_.emplace_back(
          MLCRequestType{.lane_num = lane_num,
                         .mlc_request_type = RAMP_TO_MAIN,
                         .split_direction = split_dir});  // 使用map中的方向
      mlc_request_info_list.emplace_back(
          mlc_request_info_,
          route_info_output_.merge_point_info.dis_to_merge_fp);
      is_exist_merge_fp = true;
      if (it != feasible_lane_sequence.end()) {
        feasible_lane_sequence.erase(it);
      }
    }
    if (last_exchange_region_info_.is_process_merge &&
        !last_exchange_region_info_.last_exchange_info.is_other_merge_to_road) {
      route_info_output_.is_closing_merge =
          route_info_output_.merge_point_info.dis_to_merge_fp < ego_v * 10.0;
    }
  }
  // 将mlc_request_info_list中按照distance排序
  std::sort(mlc_request_info_list.begin(), mlc_request_info_list.end(),
            [](const auto& a, const auto& b) { return a.second < b.second; });

  // 在优化feasible lane后，过滤first_feasible_lane_distance
  for (auto it = feasible_lane_distance.begin();
       it != feasible_lane_distance.end();) {
    auto seq_it = std::find(feasible_lane_sequence.begin(),
                            feasible_lane_sequence.end(), it->first);
    if (seq_it == feasible_lane_sequence.end()) {
      it = feasible_lane_distance.erase(it);
    } else {
      ++it;
    }
  }
  // 若因为merge优化导致feasible_lane是空，选取merge_lane外车道
  if (feasible_lane_sequence.empty()) {
    feasible_lane_sequence =
        findMissingElements(current_lane_vec, merge_lane_sequence);
    for (int i = 0; i < feasible_lane_sequence.size(); i++) {
      feasible_lane_distance[feasible_lane_sequence[i]] = 0.0;
    }
  }

  if (feasible_lane_sequence.empty()) {
    return;
  }
  // 向mlc_decider_route_info_赋值
  mlc_decider_route_info_.feasible_lane_sequence = feasible_lane_sequence;

  route_info_output_.mlc_request_type_route_info.reset();
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
  for (auto& relative_id_lane : relative_id_lanes) {
    ProcessLaneDistance(relative_id_lane, feasible_lane_distance);
  }
  for (auto& relative_id_lane : relative_id_lanes) {
    if (relative_id_lane->get_relative_id() != 0) {
      continue;
    }

    const auto& lane_nums = relative_id_lane->get_lane_nums();
    if (lane_nums.empty()) {
      continue;
    }
    // TODO(fengwang31):需要考虑这个车道数的准确性
    // 现在基于感知车道数不准的情况下：当感知车道数与地图车道数对不上时，就往一个方向变道。

    // 1、计算在当前位置的地图提供的可行驶车道数、应急车道数
    iflymapdata::sdpro::FeaturePoint last_fp;
    int emergency_lane_num = 0;
    int total_lane_num = 0;
    const double s = route_info_output_.current_segment_passed_distance;
    if (CalculateLastFPInCurrentLink(&last_fp, current_link_, s)) {
      total_lane_num = last_fp.lane_ids().size();
      emergency_lane_num = 0;

      for (const auto& lane_id : last_fp.lane_ids()) {
        if (IsEmergencyLane(lane_id, sdpro_map_)) {
          emergency_lane_num++;
        }
      }
    }

    // 2、计算当前位置感知提供的车道数，当前感知提供的车道数是默认包含了右边的应急车道的
    int left_lane_num = 0;
    int right_lane_num = 0;
    for (const auto& lane_num : lane_nums) {
      if (lane_num.end > kEpsilon) {
        left_lane_num = lane_num.left_lane_num;
        right_lane_num = lane_num.right_lane_num;
        break;
      }
    }

    route_info_output_.left_lane_num = left_lane_num;
    route_info_output_.right_lane_num = right_lane_num;
    route_info_output_.emergency_lane_num = emergency_lane_num;
    route_info_output_.minVal_seq = minVal_seq;
    route_info_output_.maxVal_seq = maxVal_seq;

    // 3、判断感知车道数和地图车道数是否吻合
    const int perception_lane_num =
        left_lane_num + right_lane_num + 1 - emergency_lane_num;
    const int map_lane_num = total_lane_num - emergency_lane_num;

    const bool is_triggle_continue_lc =
        IsTriggerContinueLCInPerceptionSplitRegion(
            left_lane_num, right_lane_num - emergency_lane_num);

    // 刚经过split，感知车道数可能还带左侧主路车道，这种情况抑制MLC
    if (current_link_ != nullptr && left_lane_num > current_link_->lane_num() &&
        left_lane_num > map_lane_num) {
      continue;
    }

    // 感知车道数远小于地图车道数，且在下匝道，抑制MLC
    if (perception_lane_num + 1 < current_link_->lane_num() &&
        perception_lane_num + 1 < map_lane_num &&
        mlc_decider_route_info_.first_static_split_region_info.is_ramp_split) {
      continue;
    }
    // const bool is_nearing_ramp_scenary =
    //     mlc_decider_route_info_.first_static_split_region_info.is_ramp_split;
    // if ((perception_lane_num != map_lane_num && is_nearing_ramp_scenary) ||
    //     is_triggle_continue_lc) {
    //   mismatch_counter++;
    //   if (mismatch_counter >= MISMATH_THRESHOLD) {
    //     relative_id_lane->set_current_tasks(CalculateMLCTaskNoLaneNum());
    //   }
    //   continue;
    // } else {
    //   mismatch_counter = 0;
    // }

    // 根据前方交换区split的方向，改变使用的感知车道数是左侧还是右侧
    // 总体原则就是选择非车道增加的一侧车道数来计算
    int ego_seq = 0;
    if (emergency_lane_num == 0) {
      if (merge_point_direction == SPLIT_NONE) {
        if (mlc_decider_route_info_.ego_status_on_route ==
            IN_EXCHANGE_AREA_REAR) {
          ego_seq =
              last_exchange_region_info_.last_exchange_info.split_direction ==
                      SPLIT_RIGHT
                  ? map_lane_num - right_lane_num
                  : left_lane_num + 1;
        } else {
          if (!exchange_region_info_list.empty()) {
            if (exchange_region_info_list[0].is_ramp_split &&
                !exchange_region_info_list[0].is_other_merge_to_road &&
                !exchange_region_info_list[0].is_ramp_merge) {
              ego_seq = left_lane_num + 1;
            } else {
              ego_seq =
                  exchange_region_info_list[0].split_direction == SPLIT_RIGHT
                      ? map_lane_num - right_lane_num
                      : left_lane_num + 1;
            }
          }
        }
      } else {
        ego_seq = merge_point_direction == SPLIT_RIGHT
                      ? map_lane_num - right_lane_num
                      : left_lane_num + 1;
      }
    } else {
      ego_seq = left_lane_num + 1;
    }
    if (ego_seq <= 0) {
      ego_seq = left_lane_num + 1;
    }
    if (is_remove_other_merge && other_merge_lane_num > 0 &&
        mlc_decider_route_info_.is_process_split) {
      if (!exchange_region_info_list.empty() &&
          exchange_region_info_list[0].is_other_merge_to_road &&
          exchange_region_info_list[0].split_direction == SPLIT_RIGHT) {
        ego_seq = left_lane_num + 1 + other_merge_lane_num;
      }
    }
    route_info_output_.ego_seq = ego_seq;
    std::vector<int> lc_num_task;
    if (ego_seq >= minVal_seq && ego_seq <= maxVal_seq) {
      lc_task_count_ = 0;
      last_lc_num_task_.clear();
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

    // 检查是否与上一帧相同
    if (!lc_num_task.empty() && !last_lc_num_task_.empty() &&
        lc_num_task[0] == last_lc_num_task_[0]) {
      lc_task_count_++;
    } else {
      lc_task_count_ = 0;
      last_lc_num_task_ = lc_num_task;
    }
    SplitDirection mlc_split_direction = SPLIT_NONE;
    EgoMLCRequestType mlc_type = None_MLC;
    double distance_to_lc_exchange_region = NL_NMAX;
    if (lc_task_count_ >= 2) {
      if (!lc_num_task.empty()) {
        if (lc_num_task[0] == -1) {
          mlc_split_direction = SPLIT_LEFT;
        } else {
          mlc_split_direction = SPLIT_RIGHT;
        }
        bool is_found = false;
        for (int i = 0; i < mlc_request_info_list.size() && !is_found; i++) {
          const auto& mlc_request_info = mlc_request_info_list[i].first;
          for (int n = 0; n < mlc_request_info.size(); n++) {
            auto it = std::find_if(
                mlc_request_info.begin(), mlc_request_info.end(),
                [ego_seq, mlc_split_direction](const MLCRequestType& item) {
                  return item.lane_num == ego_seq &&
                         item.split_direction == mlc_split_direction;
                });
            is_found = (it != mlc_request_info.end());
            if (is_found) {
              mlc_type = it->mlc_request_type;
              distance_to_lc_exchange_region = mlc_request_info_list[i].second;
              break;
            }
          }
        }
        // 搜索完后仍未找到对应的MLC类型
        if (mlc_type == None_MLC) {
          mlc_type = OTHER_TYPE_MLC;
          distance_to_lc_exchange_region = distance_to_next_exchange_region;
        }
        relative_id_lane->set_current_tasks(lc_num_task);
      }
    }
    route_info_output_.mlc_request_type_route_info.mlc_request_type = mlc_type;
    route_info_output_.mlc_request_type_route_info.distance_to_exchange_region =
        distance_to_lc_exchange_region;
  }

  route_info_output_.mlc_decider_route_info = mlc_decider_route_info_;
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
  const auto& local_view = session_->environmental_model().get_local_view();
  const auto& lines = local_view.static_map_info.road_map().lanes();
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
      ILOG_DEBUG << "get nearest lane failed!!";
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
      ILOG_DEBUG << "current pose get projection fail!!";
      return false;
    }

  } else {
    ILOG_DEBUG << "no get nearest lane!!!";
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
  ILOG_DEBUG << "find the nearest lane!!!"
             << "nearest_s_:" << nearest_s
             << ",nearest lane group id:" << nearest_lane->lane_group_id();
  nearest_lane_hpp_ = nearest_lane;
  nearest_s_hpp_ = nearest_s;
  sum_s_hpp_ = sum_s;
  return true;
}

void RouteInfo::CalculateHPPInfo() {
  ConstructBox();
  if (IsOnHPPLane()) {
    ILOG_DEBUG << "is on hpp lane!";
    route_info_output_.is_on_hpp_lane = true;
    const auto& local_view = session_->environmental_model().get_local_view();
    const auto trace_start =
        local_view.static_map_info.parking_assist_info().trace_start();
    const ad_common::math::Vec2d trace_start_point_2d = {trace_start.x(),
                                                         trace_start.y()};
    // get trace_start point projection s
    double trace_start_point_accumulate_s;
    double trace_start_point_lateral;
    if (nearest_lane_hpp_->GetProjection(trace_start_point_2d,
                                         &trace_start_point_accumulate_s,
                                         &trace_start_point_lateral)) {
      ILOG_DEBUG << "trace_start point s:" << trace_start_point_accumulate_s
                 << ",lateral:" << trace_start_point_lateral;
    } else {
      ILOG_DEBUG << " trace_start point get projection fail!! ";
      return;
    }
    // calculate sum distance
    bool is_reached_trace_start_point =
        sum_s_hpp_ >= trace_start_point_accumulate_s;
    const ad_common::math::Vec2d point(current_pose_.x, current_pose_.y);
    if (is_reached_trace_start_point) {
      ILOG_DEBUG << "reached trace start point!!";
      route_info_output_.is_reached_hpp_start_point = true;
      if (last_point_hpp_.x() != NL_NMAX && last_point_hpp_.y() != NL_NMAX) {
        sum_distance_driving_ += point.DistanceTo(last_point_hpp_);
      } else {
        sum_distance_driving_ = 0;
      }
      route_info_output_.sum_distance_driving = sum_distance_driving_;
      last_point_hpp_ = point;
    } else {
      ILOG_DEBUG << "cur point s less than trace start s";
    }
  } else {
    ILOG_DEBUG << "not in hpp lane!!!";
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
  const auto& local_view = session_->environmental_model().get_local_view();
  const auto& lines = local_view.static_map_info.road_map().lanes();
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
    ILOG_DEBUG << "lines is empty from road_map!!!";
  }
}

void RouteInfo::CalculateDistanceToNextSpeedBump() {
  // ehr speed bump
  const auto& local_view = session_->environmental_model().get_local_view();
  const auto& lane_groups = local_view.static_map_info.road_map().lane_groups();
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
          ILOG_DEBUG << "not get speed_bump projection point on line!!!";
          continue;
        } else {
          ILOG_DEBUG << "get s for speed_bump projection point on line:"
                     << speed_bump_nearest_s;
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
  JSON_DEBUG_VALUE("is_in_sdmaproad", (int)route_info_output_.is_in_sdmaproad);
  JSON_DEBUG_VALUE("lsl_length", route_info_output_.lsl_length);

  JSON_DEBUG_VALUE("left_lane_num", route_info_output_.left_lane_num);
  JSON_DEBUG_VALUE("right_lane_num", route_info_output_.right_lane_num);
  JSON_DEBUG_VALUE("emergency_lane_num", route_info_output_.emergency_lane_num);
  JSON_DEBUG_VALUE("minVal_seq", route_info_output_.minVal_seq);
  JSON_DEBUG_VALUE("maxVal_seq", route_info_output_.maxVal_seq);
  JSON_DEBUG_VALUE(
      "ego_status_on_route",
      static_cast<int>(
          route_info_output_.mlc_decider_route_info.ego_status_on_route));
  JSON_DEBUG_VALUE("is_find_exc_fp", (int)route_info_output_.is_find_exc_fp);
  JSON_DEBUG_VALUE("is_miss_split_point",
                   static_cast<int>(route_info_output_.is_miss_split_point));
  JSON_DEBUG_VALUE(
      "mlc_request_type",
      static_cast<int>(
          route_info_output_.mlc_request_type_route_info.mlc_request_type));
  JSON_DEBUG_VALUE("lsl_length", route_info_output_.lsl_length);
  JSON_DEBUG_VALUE("bd_mlc_scene",
                   static_cast<int>(route_info_output_.baidu_mlc_scene));
  JSON_DEBUG_VALUE("ego_seq", route_info_output_.ego_seq);
  JSON_DEBUG_VALUE("left_lane_distance", route_info_output_.left_lane_distance);
  JSON_DEBUG_VALUE("right_lane_distance",
                   route_info_output_.right_lane_distance);
  JSON_DEBUG_VALUE("is_closing_split", route_info_output_.is_closing_split);
  JSON_DEBUG_VALUE("is_closing_merge", route_info_output_.is_closing_merge);
  JSON_DEBUG_VALUE("is_process_split",
                   route_info_output_.mlc_decider_route_info.is_process_split);
  JSON_DEBUG_VALUE("is_process_merge",
                   route_info_output_.mlc_decider_route_info.is_process_merge);
  JSON_DEBUG_VALUE(
      "is_process_other_merge",
      route_info_output_.mlc_decider_route_info.is_process_other_merge);
  JSON_DEBUG_VALUE("last_split_end_point_distance",
                   route_info_output_.last_split_end_point_distance);
}

NOASplitRegionInfo RouteInfo::CalculateSplitRegionLaneTupoInfo(
    const iflymapdata::sdpro::LinkInfo_Link& split_segment,
    const ad_common::sdpromap::SDProMap& sdpro_map,
    const std::vector<std::pair<const iflymapdata::sdpro::LinkInfo_Link*,
                                double>>& split_info_vec,
    const double ego_dis_to_split) {
  NOASplitRegionInfo split_region_info;

  bool is_find_split_region_start = false;
  bool is_find_split_region_end = false;
  double fp_start_length = 0;
  iflymapdata::sdpro::FeaturePoint start_fp;
  iflymapdata::sdpro::FeaturePoint end_fp;
  const iflymapdata::sdpro::LinkInfo_Link* next_split_link = nullptr;
  for (const auto& split_info : split_info_vec) {
    if (split_info.first->id() == split_segment.id()) {
      next_split_link = split_info.first;
      break;
    }
  }

  split_region_info.split_link_id = split_segment.id();

  auto previous_seg = &split_segment;

  if (previous_seg == nullptr) {
    return split_region_info;
  }

  const auto split_seccessor_link =
      sdpro_map.GetNextLinkOnRoute(previous_seg->id());

  if (!split_seccessor_link) {
    return split_region_info;
  }
  const auto& out_link = previous_seg->successor_link_ids();
  if (out_link.size() < 2) {
    return split_region_info;
  }
  auto other_link_id =
      out_link[0] == split_seccessor_link->id() ? out_link[1] : out_link[0];
  const auto& other_link = sdpro_map.GetLinkOnRoute(other_link_id);
  if (!other_link) {
    return split_region_info;
  }
  split_region_info.is_ramp_split =
      sdpro_map.isRamp(split_seccessor_link->link_type()) ||
      sdpro_map.isSaPa(split_seccessor_link->link_type());

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

  double dis_to_last_split_point = 0.0;
  double dis_to_last_merge_point = 0.0;
  double valid_dis = NL_NMAX;

  if (CalculateDistanceNextToLastSplitPoint(&dis_to_last_split_point,
                                            next_split_link)) {
    valid_dis = std::min(valid_dis, dis_to_last_split_point);
  }

  if (CalculateDistanceNextToLastMergePoint(&dis_to_last_merge_point,
                                            next_split_link)) {
    valid_dis = std::min(valid_dis, dis_to_last_merge_point);
  }

  while (!is_find_split_region_start) {
    int fp_point_size = previous_seg->feature_points_size();

    std::vector<iflymapdata::sdpro::FeaturePoint> fp_vec;
    if (!previous_seg->feature_points().empty()) {
      for (const auto& fp : previous_seg->feature_points()) {
        fp_vec.emplace_back(fp);
      }
    }
    std::sort(fp_vec.begin(), fp_vec.end(),
              [](const iflymapdata::sdpro::FeaturePoint& fp_a,
                 const iflymapdata::sdpro::FeaturePoint& fp_b) {
                return fp_a.projection_percent() < fp_b.projection_percent();
              });

    for (int i = fp_point_size - 1; i >= 0; i--) {
      const auto& fp_point = fp_vec[i];
      for (const auto fp_point_type : fp_point.type()) {
        if (fp_point_type ==
            iflymapdata::sdpro::FeaturePointType::EXCHANGE_AREA_START) {
          start_fp = fp_point;
          is_find_split_region_start = true;

          split_region_info.start_fp_point.lane_ids.clear();

          for (const auto& id : fp_point.lane_ids()) {
            if (!IsEmergencyLane(id, sdpro_map)) {
              split_region_info.start_fp_point.lane_ids.push_back(id);
            }
          }
          split_region_info.start_fp_point.link_id = previous_seg->id();
          break;
        } else if (fp_point_type ==
                   iflymapdata::sdpro::FeaturePointType::EXCHANGE_AREA_END) {
          return split_region_info;
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
      break;
    }

    // 计算fp_length的累计长度
    fp_start_length = fp_start_length + previous_seg->length() * 0.01;

    previous_seg = sdpro_map.GetPreviousLinkOnRoute(previous_seg->id());
    if (previous_seg == nullptr || fp_start_length > valid_dis) {
      break;
    }
  }

  double fp_end_length = 0;
  const iflymapdata::sdpro::LinkInfo_Link* split_region_end_link =
      split_seccessor_link;
  while (!is_find_split_region_end) {
    int fp_point_size = split_region_end_link->feature_points_size();

    std::vector<iflymapdata::sdpro::FeaturePoint> fp_vec;
    if (!split_region_end_link->feature_points().empty()) {
      for (const auto& fp : split_region_end_link->feature_points()) {
        fp_vec.emplace_back(fp);
      }
    }
    std::sort(fp_vec.begin(), fp_vec.end(),
              [](const iflymapdata::sdpro::FeaturePoint& fp_a,
                 const iflymapdata::sdpro::FeaturePoint& fp_b) {
                return fp_a.projection_percent() < fp_b.projection_percent();
              });

    for (int i = 0; i < fp_point_size; i++) {
      const auto& fp_point = fp_vec[i];
      for (const auto fp_point_type : fp_point.type()) {
        if (fp_point_type ==
            iflymapdata::sdpro::FeaturePointType::EXCHANGE_AREA_END) {
          end_fp = fp_point;
          is_find_split_region_end = true;

          split_region_info.end_fp_point.lane_ids.clear();

          for (const auto& id : fp_point.lane_ids()) {
            if (!IsEmergencyLane(id, sdpro_map)) {
              split_region_info.end_fp_point.lane_ids.push_back(id);
            }
          }
          split_region_info.end_fp_point.link_id = split_region_end_link->id();
          break;
        } else if (fp_point_type == iflymapdata::sdpro::FeaturePointType::
                                        EXCHANGE_AREA_START &&
                   fp_point.id() != start_fp.id()) {
          return split_region_info;
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

    // 计算出下一个link拓扑变化点的距离search_dis
    double dis = NL_NMAX;
    for (const auto& split_info : split_info_vec) {
      if (split_info.second > ego_dis_to_split) {
        dis = split_info.second;
        break;
      }
    }

    for (const auto& merge_info : route_info_output_.merge_region_info_list) {
      if (merge_info.distance_to_split_point > ego_dis_to_split) {
        dis = std::min(dis, merge_info.distance_to_split_point);
        break;
      }
    }

    // 限制从split向后最多搜索300米
    double search_dis = std::min((dis - ego_dis_to_split), 300.0);
    const double kSearchMargin = 0.1;

    split_region_end_link =
        sdpro_map.GetNextLinkOnRoute(split_region_end_link->id());
    if (split_region_end_link == nullptr ||
        (fp_end_length > (search_dis - kSearchMargin))) {
      break;
    }
  }

  split_region_info.start_fp_point.fp_distance_to_split_point =
      is_find_split_region_start == true ? -fp_start_length : 0.0;
  split_region_info.end_fp_point.fp_distance_to_split_point =
      is_find_split_region_end == true ? fp_end_length : 0.0;

  if (!is_find_split_region_start || !is_find_split_region_end) {
    return split_region_info;
  }

  route_info_output_.is_find_exc_fp = true;

  // 1、计算第一区域的车道总数
  // 计算该split的方向
  SplitSegInfo split_seg_info;
  split_seg_info = MakesureSplitDirection(split_segment, sdpro_map);

  int temp_lane_num1 = 0;
  iflymapdata::sdpro::FeaturePoint last_fp;
  iflymapdata::sdpro::LinkInfo_Link last_fp_link;
  if (CalculateLastFp(&last_fp, &last_fp_link,
                      split_region_info.start_fp_point.link_id, start_fp)) {
    for (const auto& id : last_fp.lane_ids()) {
      if (!IsEmergencyLane(id, sdpro_map)) {
        temp_lane_num1++;
      }
    }

    // 车道数对不上时，取大（小）值
    const int last_fp_link_lane_num = last_fp_link.lane_num();
    if (temp_lane_num1 != last_fp_link_lane_num) {
      if (split_seg_info.split_direction == RAMP_ON_LEFT) {
        temp_lane_num1 = std::min(temp_lane_num1, last_fp_link_lane_num);
      } else if (split_seg_info.split_direction == RAMP_ON_RIGHT) {
        temp_lane_num1 = std::max(temp_lane_num1, last_fp_link_lane_num);
      }
    }
  }

  if (temp_lane_num1 == 0) {
    return split_region_info;
  }

  split_region_info.recommend_lane_num.emplace_back(temp_lane_num1,
                                                    std::vector<int>{});
  // 2、计算第二区域的车道总数
  // 在交换区内部，可能存在车道数变化，因此交换区的车道数以交换区终点的前一个fp的车道数为准
  int temp_lane_num2 = 0;
  iflymapdata::sdpro::FeaturePoint end_last_fp;
  iflymapdata::sdpro::LinkInfo_Link end_last_fp_link;
  if (CalculateLastFp(&end_last_fp, &end_last_fp_link,
                      split_region_info.end_fp_point.link_id, end_fp)) {
    for (const auto& id : end_last_fp.lane_ids()) {
      if (!IsEmergencyLane(id, sdpro_map)) {
        temp_lane_num2++;
      }
    }

    // 先比较交换区终点前一个点的车道数是否能对上
    const int end_last_fp_link_lane_num = end_last_fp_link.lane_num();
    if (temp_lane_num2 != end_last_fp_link_lane_num) {
      if (split_seg_info.split_direction == RAMP_ON_LEFT) {
        temp_lane_num2 = std::min(temp_lane_num2, end_last_fp_link_lane_num);
      } else if (split_seg_info.split_direction == RAMP_ON_RIGHT) {
        temp_lane_num2 = std::max(temp_lane_num2, end_last_fp_link_lane_num);
      }
    }

    // 再与交换区起点的地图车道数比较
    const auto start_fp_link =
        sdpro_map.GetLinkOnRoute(split_region_info.start_fp_point.link_id);
    if (start_fp_link) {
      int start_fp_lane_num = split_region_info.start_fp_point.lane_ids.size();
      const int start_fp_link_lane_num = start_fp_link->lane_num();
      if (start_fp_lane_num != start_fp_link_lane_num) {
        if (split_seg_info.split_direction == RAMP_ON_LEFT) {
          start_fp_lane_num =
              std::min(start_fp_lane_num, start_fp_link_lane_num);
        } else if (split_seg_info.split_direction == RAMP_ON_RIGHT) {
          start_fp_lane_num =
              std::max(start_fp_lane_num, start_fp_link_lane_num);
        }
      }

      if (split_seg_info.split_direction == RAMP_ON_LEFT) {
        temp_lane_num2 = std::min(temp_lane_num2, start_fp_lane_num);
      } else if (split_seg_info.split_direction == RAMP_ON_RIGHT) {
        temp_lane_num2 = std::max(temp_lane_num2, start_fp_lane_num);
      }
    }
  }

  if (temp_lane_num2 == 0) {
    return split_region_info;
  }

  split_region_info.recommend_lane_num.emplace_back(temp_lane_num2,
                                                    std::vector<int>{});

  // 3、计算第三区域的车道总数
  int temp_lane_num3 = 0;
  if (split_region_info.end_fp_point.lane_ids.empty()) {
    return split_region_info;
  } else {
    temp_lane_num3 = split_region_info.end_fp_point.lane_ids.size();
  }
  split_region_info.recommend_lane_num.emplace_back(temp_lane_num3,
                                                    std::vector<int>{});
  // 4、计算第四区域的车道总数
  split_region_info.recommend_lane_num.emplace_back(
      other_successor_link->lane_num(), std::vector<int>{});

  split_region_info.start_fp_point.fp = start_fp;
  split_region_info.end_fp_point.fp = end_fp;

  split_region_info.is_valid = true;
  return split_region_info;
}

NOASplitRegionInfo RouteInfo::CalculateMergeRegionLaneTupoInfo(
    const iflymapdata::sdpro::LinkInfo_Link& merge_segment,
    const ad_common::sdpromap::SDProMap& sdpro_map,
    const std::vector<std::pair<const iflymapdata::sdpro::LinkInfo_Link*,
                                double>>& merge_info_vec,
    const std::vector<std::pair<const iflymapdata::sdpro::LinkInfo_Link*,
                                double>>& split_info_vec,
    const double ego_dis_to_merge) {
  NOASplitRegionInfo merge_region_info;

  bool is_find_merge_region_start = false;
  bool is_find_merge_region_end = false;
  bool fp_start_point_pose_is_rear_merge_point = false;
  double fp_start_length = 0;
  iflymapdata::sdpro::FeaturePoint start_fp;
  iflymapdata::sdpro::FeaturePoint end_fp;

  const iflymapdata::sdpro::LinkInfo_Link* merge_region_start_pre_link =
      nullptr;
  auto temp_seg = &merge_segment;

  if (temp_seg == nullptr) {
    return merge_region_info;
  }

  double dis_to_last_split_point = 0.0;
  double dis_to_last_merge_point = 0.0;
  double valid_dis = NL_NMAX;
  const iflymapdata::sdpro::LinkInfo_Link* next_merge_link = nullptr;
  for (const auto& merge_info : merge_info_vec) {
    if (merge_info.first->id() == merge_segment.id()) {
      next_merge_link = merge_info.first;
      break;
    }
  }
  if (CalculateDistanceNextToLastSplitPoint(&dis_to_last_split_point,
                                            next_merge_link)) {
    valid_dis = std::min(valid_dis, dis_to_last_split_point);
  }

  if (CalculateDistanceNextToLastMergePoint(&dis_to_last_merge_point,
                                            next_merge_link)) {
    valid_dis = std::min(valid_dis, dis_to_last_merge_point);
  }

  while (!is_find_merge_region_start) {
    int fp_point_size = temp_seg->feature_points_size();

    std::vector<iflymapdata::sdpro::FeaturePoint> fp_vec;
    if (!temp_seg->feature_points().empty()) {
      for (const auto& fp : temp_seg->feature_points()) {
        fp_vec.emplace_back(fp);
      }
    }
    std::sort(fp_vec.begin(), fp_vec.end(),
              [](const iflymapdata::sdpro::FeaturePoint& fp_a,
                 const iflymapdata::sdpro::FeaturePoint& fp_b) {
                return fp_a.projection_percent() < fp_b.projection_percent();
              });

    // 增加判断在当前merge link上是否有下一个交换区的起点
    bool is_exist_next_start_fp = false;
    iflymapdata::sdpro::FeaturePoint temp_end_fp;
    if (temp_seg->id() == merge_segment.id()) {
      for (auto fp_point : temp_seg->feature_points()) {
        for (const auto fp_point_type : fp_point.type()) {
          if (fp_point_type ==
              iflymapdata::sdpro::FeaturePointType::EXCHANGE_AREA_END) {
            is_exist_next_start_fp = true;
            temp_end_fp = fp_point;
            break;
          }
        }
        if (is_exist_next_start_fp) {
          break;
        }
      }
    }

    for (int i = fp_point_size - 1; i >= 0; i--) {
      const auto& fp_point = fp_vec[i];
      for (const auto fp_point_type : fp_point.type()) {
        if (fp_point_type ==
            iflymapdata::sdpro::FeaturePointType::EXCHANGE_AREA_START) {
          // 增加判断在当前merge link上是否有下一个交换区的起点
          if (temp_seg->id() == merge_segment.id() && is_exist_next_start_fp &&
              fp_point.projection_percent() >
                  temp_end_fp.projection_percent()) {
            continue;
          }

          is_find_merge_region_start = true;
          start_fp = fp_point;

          merge_region_info.start_fp_point.lane_ids.clear();
          for (const auto& id : fp_point.lane_ids()) {
            if (!IsEmergencyLane(id, sdpro_map)) {
              merge_region_info.start_fp_point.lane_ids.push_back(id);
            }
          }
          merge_region_info.start_fp_point.link_id = temp_seg->id();
          break;
        } else if (temp_seg->id() != merge_segment.id() &&
                   fp_point_type == iflymapdata::sdpro::FeaturePointType::
                                        EXCHANGE_AREA_END) {
          return merge_region_info;
        }
      }
      if (is_find_merge_region_start) {
        double rate = 0;
        if (temp_seg->id() == merge_segment.id()) {
          rate = fp_point.projection_percent();
        } else {
          rate = 1 - fp_point.projection_percent();
        }
        fp_start_length = fp_start_length + temp_seg->length() * 0.01 * rate;
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

    if (temp_seg == nullptr || fp_start_length > valid_dis) {
      return merge_region_info;
    }
    fp_start_point_pose_is_rear_merge_point = true;
  }

  double fp_end_length = 0;
  const iflymapdata::sdpro::LinkInfo_Link* merge_region_end_link =
      &merge_segment;

  if (merge_region_end_link == nullptr) {
    return merge_region_info;
  }

  while (!is_find_merge_region_end) {
    int fp_point_size = merge_region_end_link->feature_points_size();

    std::vector<iflymapdata::sdpro::FeaturePoint> fp_vec;
    if (!merge_region_end_link->feature_points().empty()) {
      for (const auto& fp : merge_region_end_link->feature_points()) {
        fp_vec.emplace_back(fp);
      }
    }
    std::sort(fp_vec.begin(), fp_vec.end(),
              [](const iflymapdata::sdpro::FeaturePoint& fp_a,
                 const iflymapdata::sdpro::FeaturePoint& fp_b) {
                return fp_a.projection_percent() < fp_b.projection_percent();
              });

    for (int i = 0; i < fp_point_size; i++) {
      const auto& fp_point = fp_vec[i];
      for (const auto fp_point_type : fp_point.type()) {
        if (fp_point_type ==
            iflymapdata::sdpro::FeaturePointType::EXCHANGE_AREA_END) {
          is_find_merge_region_end = true;
          end_fp = fp_point;

          merge_region_info.end_fp_point.lane_ids.clear();
          for (const auto& id : fp_point.lane_ids()) {
            if (!IsEmergencyLane(id, sdpro_map)) {
              merge_region_info.end_fp_point.lane_ids.push_back(id);
            }
          }

          merge_region_info.end_fp_point.link_id = merge_region_end_link->id();
          break;
        } else if (fp_point_type == iflymapdata::sdpro::FeaturePointType::
                                        EXCHANGE_AREA_START &&
                   start_fp.id() != fp_point.id()) {
          return merge_region_info;
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

    double dis = NL_NMAX;
    if (!merge_info_vec.empty()) {
      for (const auto& info : merge_info_vec) {
        if (info.second > ego_dis_to_merge) {
          dis = info.second;
          break;
        }
      }
    }

    if (!split_info_vec.empty()) {
      for (const auto& info : split_info_vec) {
        if (info.second > ego_dis_to_merge) {
          dis = std::min(dis, info.second);
          break;
        }
      }
    }

    const double search_dis = dis - ego_dis_to_merge;

    merge_region_end_link =
        sdpro_map.GetNextLinkOnRoute(merge_region_end_link->id());
    if (merge_region_end_link == nullptr || fp_end_length > search_dis) {
      return merge_region_info;
    }
  }

  if (!is_find_merge_region_start || !is_find_merge_region_end) {
    return merge_region_info;
  }

  merge_region_info.is_ramp_split = sdpro_map.isRamp(merge_segment.link_type());
  merge_region_info.split_link_id = merge_segment.id();

  if (fp_start_point_pose_is_rear_merge_point) {
    fp_start_length = -fp_start_length;
  }
  merge_region_info.start_fp_point.fp_distance_to_split_point = fp_start_length;
  merge_region_info.end_fp_point.fp_distance_to_split_point = fp_end_length;

  // 1、计算第一区域的车道总数
  int temp_lane_num1 = 0;
  iflymapdata::sdpro::FeaturePoint last_fp;
  iflymapdata::sdpro::LinkInfo_Link last_fp_link;
  if (CalculateLastFp(&last_fp, &last_fp_link,
                      merge_region_info.start_fp_point.link_id, start_fp)) {
    for (const auto& id : last_fp.lane_ids()) {
      if (!IsEmergencyLane(id, sdpro_map)) {
        temp_lane_num1++;
      }
    }
  }

  if (temp_lane_num1 == 0) {
    return merge_region_info;
  }

  merge_region_info.recommend_lane_num.emplace_back(temp_lane_num1,
                                                    std::vector<int>{});
  // 2、计算第二区域的车道总数
  const auto& temp_link =
      sdpro_map.GetLinkOnRoute(merge_region_info.split_link_id);
  if (temp_link == nullptr) {
    return merge_region_info;
  }

  int temp_lane_num2 = 0;
  if (merge_region_info.start_fp_point.lane_ids.empty()) {
    return merge_region_info;
  } else {
    const int merge_link_lane_num = temp_link->lane_num();
    const int merge_start_fp_num =
        merge_region_info.start_fp_point.lane_ids.size();
    temp_lane_num2 = std::max(merge_start_fp_num, merge_link_lane_num);
  }

  merge_region_info.recommend_lane_num.emplace_back(temp_lane_num2,
                                                    std::vector<int>{});
  // 3、计算第三区域的车道总数
  int temp_lane_num3 = 0;
  if (merge_region_info.end_fp_point.lane_ids.empty()) {
    return merge_region_info;
  } else {
    temp_lane_num3 = merge_region_info.end_fp_point.lane_ids.size();
  }
  merge_region_info.recommend_lane_num.emplace_back(temp_lane_num3,
                                                    std::vector<int>{});

  // 4、计算第四区域的车道数
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

  // 计算merge type
  for (const auto& fp_type : end_fp.type()) {
    if (fp_type == iflymapdata::sdpro::FeaturePointType::LANE_COUNT_CHANGE) {
      iflymapdata::sdpro::LaneChangeType merge_type;
      if (IsMergeFP(&merge_type, end_fp)) {
        if (merge_type ==
            iflymapdata::sdpro::LaneChangeType::LeftTurnMergingLane) {
          merge_region_info.merge_type = LEFT_MERGE;
        } else if (merge_type ==
                   iflymapdata::sdpro::LaneChangeType::RightTurnMergingLane) {
          merge_region_info.merge_type = RIGHT_MERGE;
        } else if (merge_type == iflymapdata::sdpro::LaneChangeType::
                                     BothDirectionMergingLane) {
          merge_region_info.merge_type = BOTH_MERGE;
        } else {
          merge_region_info.merge_type = NO_MERGE;
        }
      }
    }
  }

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

  // 1、获取交换区终点后的lane信息；
  std::vector<const MapLane*> before_fpp_lanes;
  std::vector<const MapLane*> on_fpp_lanes;
  std::vector<const MapLane*> end_fpp_lanes;
  std::vector<std::vector<const MapLane*>> feasible_lane_groups;
  for (const auto lane_id : end_fpp_lane_ids) {
    const auto& lane = sdpro_map.GetLaneInfoByID(lane_id);
    if (lane == nullptr) {
      continue;
    }
    end_fpp_lanes.emplace_back(lane);
  }

  end_fp_lane_sequence_groups.emplace_back(end_fpp_lanes);

  // 2、从交换区终点向前遍历交换区内的lane_groups；
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

      accumulative_distance =
          accumulative_distance + target_lane->length() * 0.01;

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

bool RouteInfo::CalculateFeasibleLane(NOASplitRegionInfo* split_region_info) {
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

  const bool successor_lane_num_condition =
      on_exclnum == successor_exclnum + successor_other_exclnum;
  const bool on_exclnum_lane_num_condition = before_exclnum == on_exclnum;
  const bool is_continue_lane =
      on_exclnum_lane_num_condition && successor_lane_num_condition;

  // 增加判断在自车与split之间，是否有other merge to
  // road，而且是相同方向，那么需要考虑避让other merge。
  bool is_merge_split_same_dir = false;
  int merge_before_exclnum = -1;
  int avoide_num = -1;

  if (!route_info_output_.merge_region_info_list.empty()) {
    const auto& temp_merge_region_info =
        route_info_output_.merge_region_info_list[0];
    bool is_exist_other_merge_between_ego_to_split =
        temp_merge_region_info.distance_to_split_point <
        split_region_info->distance_to_split_point;

    if (is_exist_other_merge_between_ego_to_split) {
      is_merge_split_same_dir = temp_merge_region_info.split_direction ==
                                    split_region_info->split_direction &&
                                temp_merge_region_info.is_other_merge_to_road;

      if (temp_merge_region_info.recommend_lane_num.size() > 0) {
        merge_before_exclnum =
            temp_merge_region_info.recommend_lane_num[0].total_lane_num;
      }
    }
  }

  bool is_split_right =
      split_region_info->split_direction == SplitDirection::SPLIT_RIGHT;
  bool is_split_left =
      split_region_info->split_direction == SplitDirection::SPLIT_LEFT;
  bool is_split_middle =
      split_region_info->split_direction == SplitDirection::SPLIT_MIDDLE;

  // 判断other split是否是ramp
  const auto split_link_id = split_region_info->split_link_id;
  const auto split_link = sdpro_map_.GetLinkOnRoute(split_link_id);
  if (split_link == nullptr) {
    return false;
  }

  const auto next_link = sdpro_map_.GetNextLinkOnRoute(split_link_id);
  if (next_link == nullptr) {
    return false;
  }

  uint64 other_split_link_id = 0;
  uint64 left_split_link_id = 0;
  uint64 middle_split_link_id = 0;
  uint64 right_split_link_id = 0;
  if (split_link->successor_link_ids_size() == 2) {
    const auto& successor_link_ids = split_link->successor_link_ids();
    if (successor_link_ids[0] == next_link->id()) {
      other_split_link_id = successor_link_ids[1];
    } else {
      other_split_link_id = successor_link_ids[0];
    }
  } else if (split_link->successor_link_ids_size() == 3) {
    const auto successor_link_ids = split_link->successor_link_ids();
    std::vector<uint64> other_link_ids;
    for (int i = 0; i < successor_link_ids.size(); ++i) {
      if (successor_link_ids[i] != next_link->id()) {
        other_link_ids.emplace_back(successor_link_ids[i]);
      }
    }

    const auto& other_link1 = sdpro_map_.GetLinkOnRoute(other_link_ids[0]);
    const auto& other_link2 = sdpro_map_.GetLinkOnRoute(other_link_ids[1]);
    if (other_link1 == nullptr || other_link2 == nullptr ||
        other_link1->points().boot().points().size() < 2 ||
        other_link2->points().boot().points().size() < 2) {
      return false;
    }

    // 目标车道的point
    Point2D O{next_link->points().boot().points()[0].x(),
              next_link->points().boot().points()[0].y()};
    Point2D L{next_link->points().boot().points()[1].x(),
              next_link->points().boot().points()[1].y()};

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

    std::vector<uint64> link_ids = {next_link->id(), other_link1->id(),
                                    other_link2->id()};
    std::vector<uint64*> split_links_id = {
        &right_split_link_id, &middle_split_link_id, &left_split_link_id};

    for (int i = 0; i < result.size(); i++) {
      switch (result[i]) {
        case 'A':
          *split_links_id[i] = link_ids[0];
          break;
        case 'B':
          *split_links_id[i] = link_ids[1];
          break;
        case 'C':
          *split_links_id[i] = link_ids[2];
          break;
      }
    }
  }
  std::vector<int> on_excr_feasible_lane;
  std::vector<int> before_excr_feasible_lane;
  std::vector<int> succerssor_excr_feasible_lane;

  if (split_link->successor_link_ids_size() == 2) {
    const auto& other_split_link =
        sdpro_map_.GetLinkOnRoute(other_split_link_id);
    if (other_split_link == nullptr) {
      return false;
    }

    // bool is_other_split_ramp =
    // sdpro_map_.isRamp(other_split_link->link_type());
    bool is_other_split_ramp =
        (other_split_link->link_type() &
         iflymapdata::sdpro::LinkType::LT_MAIN_ROAD) == 0;

    // TODO(fengwang31:需要考虑split的路是否未主路的情况)
    if (is_split_right) {
      // 现在假设交换区终点后的lane都是由交换区分出来的，所以交换区的lane数为on_exclnum
      // = successor_exclnum + successor_other_exclnum
      // 通常认为右边是从主路分出去的路，因此增加的车道属于是右边增加了
      if (on_exclnum == successor_exclnum + successor_other_exclnum) {
        if (successor_other_exclnum == before_exclnum) {
          // int on_excr_feasible_lane_temp = on_exclnum -
          // successor_other_exclnum;
          for (int i = 0; i < successor_exclnum; ++i) {
            on_excr_feasible_lane.emplace_back(successor_other_exclnum + i + 1);
          }
          before_excr_feasible_lane.emplace_back(before_exclnum);
          for (int i = 1; i <= successor_other_exclnum; ++i) {
            mlc_request_info_.emplace_back(
                MLCRequestType{.lane_num = i,
                               .mlc_request_type = MAIN_TO_RAMP,
                               .split_direction = SPLIT_RIGHT});
          }
        } else if (successor_other_exclnum < before_exclnum) {
          int before_excr_feasible_lane_temp =
              before_exclnum - successor_other_exclnum;
          for (int i = 0; i < before_excr_feasible_lane_temp; ++i) {
            before_excr_feasible_lane.emplace_back(successor_other_exclnum + i +
                                                   1);
          }

          int on_excr_feasible_lane_temp = on_exclnum - successor_other_exclnum;
          for (int i = 0; i < on_excr_feasible_lane_temp; ++i) {
            on_excr_feasible_lane.emplace_back(successor_other_exclnum + i + 1);
          }
          for (int i = 1; i <= successor_other_exclnum; ++i) {
            mlc_request_info_.emplace_back(
                MLCRequestType{.lane_num = i,
                               .mlc_request_type = MAIN_TO_RAMP,
                               .split_direction = SPLIT_RIGHT});
          }
        } else {
          on_excr_feasible_lane.emplace_back(on_exclnum);
          before_excr_feasible_lane.emplace_back(before_exclnum);
          for (int i = 1; i <= on_exclnum - 1; ++i) {
            mlc_request_info_.emplace_back(
                MLCRequestType{.lane_num = i,
                               .mlc_request_type = MAIN_TO_RAMP,
                               .split_direction = SPLIT_RIGHT});
          }
        }
      } else if (on_exclnum < successor_exclnum + successor_other_exclnum) {
        // 认为是右边,交换区终点后新增加了车道
        if (successor_other_exclnum == before_exclnum &&
            successor_exclnum < on_exclnum &&
            successor_other_exclnum < on_exclnum) {
          int err = on_exclnum - successor_other_exclnum;
          for (int i = 0; i < err; i++) {
            on_excr_feasible_lane.emplace_back(successor_other_exclnum + i + 1);
          }

          before_excr_feasible_lane.emplace_back(before_exclnum);
          for (int i = 1; i <= successor_other_exclnum; ++i) {
            mlc_request_info_.emplace_back(
                MLCRequestType{.lane_num = i,
                               .mlc_request_type = MAIN_TO_RAMP,
                               .split_direction = SPLIT_RIGHT});
          }
        } else if (before_exclnum == on_exclnum &&
                   on_exclnum == successor_other_exclnum) {
          for (int i = 0; i < successor_exclnum; ++i) {
            on_excr_feasible_lane.emplace_back(on_exclnum + i + 1);
          }
          before_excr_feasible_lane.emplace_back(before_exclnum);
          for (int i = 1; i <= on_exclnum; ++i) {
            mlc_request_info_.emplace_back(
                MLCRequestType{.lane_num = i,
                               .mlc_request_type = MAIN_TO_RAMP,
                               .split_direction = SPLIT_RIGHT});
          }
        } else if (before_exclnum == on_exclnum &&
                   on_exclnum == successor_exclnum) {
          // 分叉，右边是主路的
          for (int i = 0; i < successor_exclnum; ++i) {
            on_excr_feasible_lane.emplace_back(i + 1 + successor_other_exclnum);
            before_excr_feasible_lane.emplace_back(i + 1 +
                                                   successor_other_exclnum);
          }
          for (int i = 0; i < successor_other_exclnum; ++i) {
            mlc_request_info_.emplace_back(
                MLCRequestType{.lane_num = i + 1,
                               .mlc_request_type = KEEP_LEFT,
                               .split_direction = SPLIT_RIGHT});
          }
        } else {
          on_excr_feasible_lane.emplace_back(on_exclnum);
          before_excr_feasible_lane.emplace_back(before_exclnum);
          for (int i = 1; i <= on_exclnum - 1; ++i) {
            mlc_request_info_.emplace_back(
                MLCRequestType{.lane_num = i,
                               .mlc_request_type = MAIN_TO_RAMP,
                               .split_direction = SPLIT_RIGHT});
          }
        }
      } else if (on_exclnum > successor_exclnum + successor_other_exclnum) {
        // 经过交换区后车道数变少，case较少，目前观察消亡车道均是中间车道
        for (int i = successor_exclnum - 1; i >= 0; --i) {
          on_excr_feasible_lane.emplace_back(on_exclnum - i);
          before_excr_feasible_lane.emplace_back(on_exclnum - i);
        }
        for (int i = 1; i <= on_exclnum; ++i) {
          mlc_request_info_.emplace_back(
              MLCRequestType{.lane_num = i,
                             .mlc_request_type = MAIN_TO_RAMP,
                             .split_direction = SPLIT_RIGHT});
        }
      }

      if (is_merge_split_same_dir) {
        RemoveElement(before_excr_feasible_lane, 1);
        RemoveElement(on_excr_feasible_lane, 1);
        mlc_request_info_.emplace_back(
            MLCRequestType{.lane_num = 1,
                           .mlc_request_type = AVOIDE_MERGE,
                           .split_direction = SPLIT_RIGHT});
        avoide_num = 1;
      }
    } else if (is_split_left) {
      // 默认左边都是主路的，后续需要对是否是主路的属性做判断
      // 1、后继车道数等于交换区的车道数；
      // 2、交换区前、交换区、交换区后的车道数都相等；
      // 3、交换区前、交换区车道数相等，交换区后车道数小于或者大于前面的车道数
      if (successor_exclnum <= on_exclnum) {
        if (on_exclnum > before_exclnum ||
            on_exclnum == before_exclnum && successor_exclnum < on_exclnum) {
          // const auto start_link = sdpro_map_.GetLinkOnRoute(
          //     split_region_info->start_fp_point.link_id);
          // if (start_link == nullptr) {
          //   return false;
          // }
          // const auto start_link_is_ramp =
          //     sdpro_map_.isRamp(start_link->link_type());

          // const auto& end_fp_point = split_region_info->end_fp_point;
          // const auto end_link =
          // sdpro_map_.GetLinkOnRoute(end_fp_point.link_id); if (end_link ==
          // nullptr) {
          //   return false;
          // }
          // const auto end_link_is_ramp =
          // sdpro_map_.isRamp(end_link->link_type());

          // if (!end_link_is_ramp && !start_link_is_ramp) {
          //   // 主路上，交换区内、前、后车道都一样
          //   for (int i = 0; i < successor_exclnum; ++i) {
          //     on_excr_feasible_lane.emplace_back(i + 1);
          //     before_excr_feasible_lane.emplace_back(i + 1);
          //   }
          // } else {
          //   // 交换区之后增加车道
          //   on_excr_feasible_lane.emplace_back(1);
          //   before_excr_feasible_lane.emplace_back(1);
          // }

          // 主路上，交换区内、前、后车道都一样
          for (int i = 0; i < successor_exclnum; ++i) {
            before_excr_feasible_lane.emplace_back(i + 1);
            on_excr_feasible_lane.emplace_back(i + 1);
          }
          for (int i = 1; i <= on_exclnum - successor_exclnum; ++i) {
            mlc_request_info_.emplace_back(
                MLCRequestType{.lane_num = successor_exclnum + i,
                               .mlc_request_type = KEEP_LEFT,
                               .split_direction = SPLIT_LEFT});
          }

          if (is_other_split_ramp && !is_continue_lane) {
            RemoveElement(before_excr_feasible_lane, successor_exclnum);
            RemoveElement(on_excr_feasible_lane, successor_exclnum);
            mlc_request_info_.emplace_back(
                MLCRequestType{.lane_num = successor_exclnum,
                               .mlc_request_type = AVOIDE_DIVERGE,
                               .split_direction = SPLIT_LEFT});
            avoide_num = successor_exclnum;
          }
        } else if (on_exclnum == before_exclnum &&
                   successor_exclnum == on_exclnum) {
          if (successor_exclnum == 1) {
            before_excr_feasible_lane.emplace_back(1);
            on_excr_feasible_lane.emplace_back(1);
          } else {
            for (int i = 0; i < successor_exclnum - 1; ++i) {
              before_excr_feasible_lane.emplace_back(i + 1);
              on_excr_feasible_lane.emplace_back(i + 1);
            }
          }
          mlc_request_info_.emplace_back(
              MLCRequestType{.lane_num = successor_exclnum,
                             .mlc_request_type = KEEP_LEFT,
                             .split_direction = SPLIT_LEFT});
        } else if (successor_exclnum <= before_exclnum) {
          for (int i = 0; i < successor_exclnum; ++i) {
            on_excr_feasible_lane.emplace_back(i + 1);
            before_excr_feasible_lane.emplace_back(i + 1);
          }
          for (int i = 1; i <= before_exclnum - successor_exclnum; ++i) {
            mlc_request_info_.emplace_back(
                MLCRequestType{.lane_num = successor_exclnum + i,
                               .mlc_request_type = KEEP_LEFT,
                               .split_direction = SPLIT_LEFT});
          }
          if (is_other_split_ramp && !is_continue_lane) {
            RemoveElement(before_excr_feasible_lane, successor_exclnum);
            RemoveElement(on_excr_feasible_lane, successor_exclnum);
            mlc_request_info_.emplace_back(
                MLCRequestType{.lane_num = successor_exclnum,
                               .mlc_request_type = AVOIDE_DIVERGE,
                               .split_direction = SPLIT_LEFT});
            avoide_num = successor_exclnum;
          }
        } else {
          for (int i = 0; i < before_exclnum; ++i) {
            on_excr_feasible_lane.emplace_back(i + 1);
            before_excr_feasible_lane.emplace_back(i + 1);
          }
        }
      } else {
        // 交换区后面的车道数大于交换区内车道数，由于是走左边，因此取最小车道数
        int valid_lane_num = std::min(before_exclnum, on_exclnum);
        int max_lane_num = std::max(before_exclnum, on_exclnum);
        for (int i = 0; i < valid_lane_num; i++) {
          on_excr_feasible_lane.emplace_back(i + 1);
          before_excr_feasible_lane.emplace_back(i + 1);
        }
        for (int i = 1; i <= max_lane_num - valid_lane_num; ++i) {
          mlc_request_info_.emplace_back(
              MLCRequestType{.lane_num = valid_lane_num + i,
                             .mlc_request_type = KEEP_LEFT,
                             .split_direction = SPLIT_LEFT});
        }
      }
    }
  } else if (split_link->successor_link_ids_size() == 3) {
    const auto& left_split_link = sdpro_map_.GetLinkOnRoute(left_split_link_id);
    const auto& middle_split_link =
        sdpro_map_.GetLinkOnRoute(middle_split_link_id);
    const auto& right_split_link =
        sdpro_map_.GetLinkOnRoute(right_split_link_id);
    if (left_split_link == nullptr || middle_split_link == nullptr ||
        right_split_link == nullptr) {
      return false;
    }

    const int left_split_link_lane_num = left_split_link->lane_num();
    const int middle_split_link_lane_num = middle_split_link->lane_num();
    const int right_split_link_lane_num = right_split_link->lane_num();
    // 检查交换区中有没有车道数变化
    double search_dis =
        std::abs(split_region_info->start_fp_point.fp_distance_to_split_point);
    std::vector<iflymapdata::sdpro::FeaturePoint> merge_expand_fp;
    FindNextMergeExpandTypeFp(split_region_info->start_fp_point,
                              merge_expand_fp, search_dis);
    std::vector<int> merging_lane_num;
    std::vector<int> expanding_lane_num;
    if (!merge_expand_fp.empty()) {
      for (int i = 0; i < merge_expand_fp.size(); i++) {
        const auto& current_fp = merge_expand_fp[i];
        int merge_expand_lane_num = 0;
        for (const auto& lane_id : current_fp.lane_ids()) {
          const auto& lane_info = sdpro_map_.GetLaneInfoByID(lane_id);
          if (lane_info == nullptr) {
            continue;
          }
          merge_expand_lane_num++;
          if (lane_info->change_type() ==
              iflymapdata::sdpro::LaneChangeType::LeftTurnMergingLane) {
            merging_lane_num.emplace_back(merge_expand_lane_num - 1);
          } else if (lane_info->change_type() ==
                     iflymapdata::sdpro::LaneChangeType::RightTurnMergingLane) {
            merging_lane_num.emplace_back(merge_expand_lane_num + 1);
          } else if (lane_info->change_type() ==
                     iflymapdata::sdpro::LaneChangeType::
                         BothDirectionMergingLane) {
            merging_lane_num.emplace_back(1);
            merging_lane_num.emplace_back(merge_expand_lane_num + 1);
          }
          if (lane_info->change_type() ==
              iflymapdata::sdpro::LaneChangeType::LeftTurnExpandingLane) {
            expanding_lane_num.emplace_back(merge_expand_lane_num - 1);
          } else if (lane_info->change_type() ==
                     iflymapdata::sdpro::LaneChangeType::
                         RightTurnExpandingLane) {
            expanding_lane_num.emplace_back(merge_expand_lane_num + 1);
          } else if (lane_info->change_type() ==
                     iflymapdata::sdpro::LaneChangeType::
                         BothDirectionExpandingLane) {
            expanding_lane_num.emplace_back(merge_expand_lane_num - 1);
            expanding_lane_num.emplace_back(merge_expand_lane_num + 1);
          }
        }
      }
    }
    if (is_split_left) {
      if (on_exclnum == left_split_link_lane_num + middle_split_link_lane_num +
                            right_split_link_lane_num) {
        if (before_exclnum >= on_exclnum) {
          for (int i = 0; i < left_split_link_lane_num; ++i) {
            before_excr_feasible_lane.emplace_back(i + 1);
          }
        } else {
          if (!expanding_lane_num.empty()) {
            if (expanding_lane_num[0] <= left_split_link_lane_num) {
              if (left_split_link_lane_num > 1) {
                for (int i = 0; i < left_split_link_lane_num - 1; ++i) {
                  before_excr_feasible_lane.emplace_back(i + 1);
                }
              } else {
                before_excr_feasible_lane.emplace_back(1);
              }
            } else {
              for (int i = 0; i < left_split_link_lane_num; ++i) {
                before_excr_feasible_lane.emplace_back(i + 1);
              }
            }
          } else {
            before_excr_feasible_lane.emplace_back(1);
          }
        }
        for (int i = 0; i < left_split_link_lane_num; ++i) {
          on_excr_feasible_lane.emplace_back(i + 1);
        }
      } else {
        for (int i = 0; i < left_split_link_lane_num; ++i) {
          before_excr_feasible_lane.emplace_back(i + 1);
          on_excr_feasible_lane.emplace_back(i + 1);
        }
      }
    } else if (is_split_middle) {
      if (on_exclnum == left_split_link_lane_num + middle_split_link_lane_num +
                            right_split_link_lane_num) {
        if (before_exclnum >= on_exclnum) {
          for (int i = 0; i < middle_split_link_lane_num; ++i) {
            before_excr_feasible_lane.emplace_back(i + 1 +
                                                   left_split_link_lane_num);
          }
        } else {
          if (!expanding_lane_num.empty()) {
            if (expanding_lane_num[0] <=
                left_split_link_lane_num + middle_split_link_lane_num) {
              if (left_split_link_lane_num > 1) {
                for (int i = 0; i < left_split_link_lane_num - 1; ++i) {
                  before_excr_feasible_lane.emplace_back(
                      i + left_split_link_lane_num);
                }
              } else {
                before_excr_feasible_lane.emplace_back(2);
              }
            } else {
              for (int i = 0; i < middle_split_link_lane_num; ++i) {
                before_excr_feasible_lane.emplace_back(
                    i + 1 + left_split_link_lane_num);
              }
            }
          } else {
            before_excr_feasible_lane.emplace_back(left_split_link_lane_num +
                                                   1);
          }
        }
        for (int i = 0; i < left_split_link_lane_num; ++i) {
          on_excr_feasible_lane.emplace_back(i + 1);
        }
      } else {
        for (int i = 0; i < middle_split_link_lane_num; ++i) {
          before_excr_feasible_lane.emplace_back(i + 1 +
                                                 left_split_link_lane_num);
          on_excr_feasible_lane.emplace_back(i + 1 + left_split_link_lane_num);
        }
      }
    } else if (is_split_right) {
      if (on_exclnum == left_split_link_lane_num + middle_split_link_lane_num +
                            right_split_link_lane_num) {
        if (before_exclnum >= on_exclnum) {
          for (int i = 0; i < right_split_link_lane_num; ++i) {
            before_excr_feasible_lane.emplace_back(before_exclnum - i);
          }
        } else {
          if (!expanding_lane_num.empty()) {
            if (expanding_lane_num[0] <=
                left_split_link_lane_num + middle_split_link_lane_num) {
              for (int i = 0; i < right_split_link_lane_num; ++i) {
                before_excr_feasible_lane.emplace_back(before_exclnum - i);
              }
            } else {
              before_excr_feasible_lane.emplace_back(before_exclnum);
            }
          } else {
            before_excr_feasible_lane.emplace_back(before_exclnum);
          }
        }
        for (int i = 0; i < right_split_link_lane_num; ++i) {
          on_excr_feasible_lane.emplace_back(on_exclnum - i);
        }
      } else {
        for (int i = 0; i < right_split_link_lane_num; ++i) {
          before_excr_feasible_lane.emplace_back(before_exclnum - i);
          on_excr_feasible_lane.emplace_back(on_exclnum - i);
        }
      }
    }
  }
  for (int i = 0; i < successor_exclnum; ++i) {
    succerssor_excr_feasible_lane.emplace_back(i + 1);
  }

  if (before_excr_feasible_lane.empty() || on_excr_feasible_lane.empty() ||
      succerssor_excr_feasible_lane.empty()) {
    return false;
  }

  split_region_info->recommend_lane_num[0].feasible_lane_sequence =
      before_excr_feasible_lane;
  split_region_info->recommend_lane_num[1].feasible_lane_sequence =
      on_excr_feasible_lane;
  split_region_info->recommend_lane_num[2].feasible_lane_sequence =
      succerssor_excr_feasible_lane;
  if (avoide_num > 0) {
    split_region_info->avoide_lane_num = avoide_num;
  }
  return true;
}

bool RouteInfo::CalculateMergeRegionFeasibleLane(
    NOASplitRegionInfo* split_region_info) {
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
  bool is_merge_left =
      split_region_info->split_direction == SplitDirection::SPLIT_LEFT;

  std::vector<int> on_excr_feasible_lane;
  std::vector<int> before_excr_feasible_lane;
  std::vector<int> succerssor_excr_feasible_lane;
  int last_other_seg_lane_num = 0;
  const auto& merge_seg_last_seg =
      sdpro_map_.GetPreviousLinkOnRoute(split_region_info->split_link_id);
  const auto& merge_seg =
      sdpro_map_.GetLinkOnRoute(split_region_info->split_link_id);
  if (merge_seg_last_seg && merge_seg) {
    if (merge_seg->predecessor_link_ids().size() >= 2) {
      const auto& merge_seg_last_other_seg_id =
          merge_seg->predecessor_link_ids()[0] == merge_seg_last_seg->id()
              ? merge_seg->predecessor_link_ids()[1]
              : merge_seg->predecessor_link_ids()[0];
      const auto& merge_seg_last_other_seg =
          sdpro_map_.GetLinkOnRoute(merge_seg_last_other_seg_id);
      if (merge_seg_last_other_seg) {
        last_other_seg_lane_num = merge_seg_last_other_seg->lane_num();
      }
    }
  }
  // TODO(fengwang31:需要考虑是否有merge lane的情况)
  if (is_merge_right) {
    // 现在假设交换区终点后的lane都是由交换区分出来的，所以交换区的lane数为on_exclnum
    // = successor_exclnum + successor_other_exclnum
    // 通常认为右边是从主路分出去的路，因此增加的车道属于是右边增加了
    if (on_exclnum >= successor_exclnum) {
      // 目前假定都是从右边往左边汇入，所以都行驶到左边的车道上去
      // TODO(fengwang31):后续需要考虑左边的车道是否会收窄，如果会的话，则不能继续往左边汇
      if (last_other_seg_lane_num != 0) {
        for (int i = 0; i < last_other_seg_lane_num; ++i) {
          on_excr_feasible_lane.emplace_back(i + 1);
        }
        for (int i = 1; i <= (on_exclnum - last_other_seg_lane_num); ++i) {
          mlc_request_info_.emplace_back(
              MLCRequestType{.lane_num = last_other_seg_lane_num + i,
                             .mlc_request_type = RAMP_TO_MAIN,
                             .split_direction = SPLIT_LEFT});
        }
      } else {
        if (successor_exclnum <= 1) {
          for (int i = 0; i < successor_exclnum; ++i) {
            on_excr_feasible_lane.emplace_back(i + 1);
          }
        } else {
          for (int i = 0; i < successor_exclnum - 1; ++i) {
            on_excr_feasible_lane.emplace_back(i + 1);
          }
        }
        mlc_request_info_.emplace_back(
            MLCRequestType{.lane_num = successor_exclnum + 1,
                           .mlc_request_type = RAMP_TO_MAIN,
                           .split_direction = SPLIT_LEFT});
      }
      if (last_other_seg_lane_num + before_exclnum == on_exclnum) {
        for (int i = 0; i < before_exclnum; i++) {
          before_excr_feasible_lane.emplace_back(i + 1);
        }
      } else {
        before_excr_feasible_lane.emplace_back(1);
      }
    }
  } else if (is_merge_left && (split_region_info->merge_type == LEFT_MERGE ||
                               split_region_info->merge_type == BOTH_MERGE)) {
    if (successor_exclnum < on_exclnum) {
      before_excr_feasible_lane.emplace_back(before_exclnum);
      on_excr_feasible_lane.emplace_back(before_exclnum + 1);
      mlc_request_info_.emplace_back(
          MLCRequestType{.lane_num = on_exclnum,
                         .mlc_request_type = RAMP_TO_MAIN,
                         .split_direction = SPLIT_RIGHT});
      for (int i = 0; i < successor_exclnum; ++i) {
        succerssor_excr_feasible_lane.emplace_back(i + 1);
      }
    }
  }

  for (int i = 0; i < successor_exclnum; ++i) {
    succerssor_excr_feasible_lane.emplace_back(i + 1);
  }

  if (before_excr_feasible_lane.empty() || on_excr_feasible_lane.empty() ||
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

bool RouteInfo::CalculateOtherMergeRoadFeasibleLane(
    NOASplitRegionInfo* split_region_info) {
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

  bool is_other_merge_left =
      split_region_info->split_direction == SplitDirection::SPLIT_LEFT;
  bool is_other_merge_right =
      split_region_info->split_direction == SplitDirection::SPLIT_RIGHT;

  std::vector<int> on_excr_feasible_lane;
  std::vector<int> before_excr_feasible_lane;
  std::vector<int> succerssor_excr_feasible_lane;
  int avoide_num = -1;
  // TODO(fengwang31:目前只考虑了其他路从右边进入交汇区的case)
  if (is_other_merge_left) {
    if (on_exclnum >= successor_exclnum) {
      int min_lane = std::min(before_exclnum, successor_exclnum);
      int max_lane = std::max(before_exclnum, successor_exclnum);
      if (min_lane > 1) {
        for (int i = 0; i < min_lane - 1; ++i) {
          on_excr_feasible_lane.emplace_back(i + 1);
          before_excr_feasible_lane.emplace_back(i + 1);
        }
        for (int i = 0; i <= (max_lane - min_lane); ++i) {
          mlc_request_info_.emplace_back(
              MLCRequestType{.lane_num = min_lane + i,
                             .mlc_request_type = AVOIDE_MERGE,
                             .split_direction = SPLIT_LEFT});
          avoide_num = min_lane + i;
        }
      } else {
        for (int i = 0; i < min_lane; ++i) {
          on_excr_feasible_lane.emplace_back(i + 1);
          before_excr_feasible_lane.emplace_back(i + 1);
        }
      }
    }
  } else if (is_other_merge_right) {
    int min_lane = std::min(before_exclnum, successor_exclnum);
    if (on_exclnum >= successor_exclnum + predecessor_other_exclnum) {
      if (before_exclnum > 1) {
        for (int i = 1; i < before_exclnum; i++) {
          before_excr_feasible_lane.emplace_back(i + 1);
        }
        mlc_request_info_.emplace_back(
            MLCRequestType{.lane_num = 1,
                           .mlc_request_type = AVOIDE_MERGE,
                           .split_direction = SPLIT_RIGHT});
        avoide_num = 1;
      } else {
        before_excr_feasible_lane.emplace_back(1);
      }
      if (min_lane > 1) {
        for (int i = 1; i < min_lane; i++) {
          on_excr_feasible_lane.emplace_back(predecessor_other_exclnum + i + 1);
        }
      } else {
        on_excr_feasible_lane.emplace_back(predecessor_other_exclnum +
                                           min_lane);
      }
    } else {
      if (before_exclnum > 1) {
        for (int i = 1; i < before_exclnum; i++) {
          before_excr_feasible_lane.emplace_back(i + 1);
        }
        mlc_request_info_.emplace_back(
            MLCRequestType{.lane_num = 1,
                           .mlc_request_type = AVOIDE_MERGE,
                           .split_direction = SPLIT_RIGHT});
        avoide_num = 1;
      } else {
        before_excr_feasible_lane.emplace_back(1);
      }
      for (int i = predecessor_other_exclnum; i < successor_exclnum; i++) {
        on_excr_feasible_lane.emplace_back(i + 1);
      }
    }
  }
  for (int i = 0; i < successor_exclnum; ++i) {
    succerssor_excr_feasible_lane.emplace_back(i + 1);
  }

  if (before_excr_feasible_lane.empty() || on_excr_feasible_lane.empty() ||
      succerssor_excr_feasible_lane.empty()) {
    return false;
  }

  split_region_info->recommend_lane_num[0].feasible_lane_sequence =
      before_excr_feasible_lane;
  split_region_info->recommend_lane_num[1].feasible_lane_sequence =
      on_excr_feasible_lane;
  split_region_info->recommend_lane_num[2].feasible_lane_sequence =
      succerssor_excr_feasible_lane;
  split_region_info->avoide_lane_num = avoide_num;
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

std::vector<char> RouteInfo::SortRaysByDirection(
    const std::vector<RayInfo>& rays) {
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
    std::map<int, SplitDirection>& merge_lane_sequence_vec,
    double search_distance) {
  iflymapdata::sdpro::FeaturePoint find_fp;
  uint64 fp_link_id;
  double ego_s_in_cur_link;
  double dis_to_merge_fp = 0.0;
  MergeType merge_type;

  if (!CalculateMergeFP(&merge_type, &find_fp, &fp_link_id, &dis_to_merge_fp,
                        search_distance)) {
    return false;
  }

  route_info_output_.merge_point_info.dis_to_merge_fp = dis_to_merge_fp;
  route_info_output_.merge_point_info.merge_type = merge_type;

  // if (!CalculateLastFp(&last_fp, fp_link_id, find_fp)) {
  //     return false;
  // }

  for (const auto& lane_id : find_fp.lane_ids()) {
    const auto& lane_info = sdpro_map_.GetLaneInfoByID(lane_id);
    if (lane_info == nullptr) {
      continue;
    }

    int find_fp_lane_num = 0;
    for (const auto& lane_id_temp : find_fp.lane_ids()) {
      if (!IsEmergencyLane(lane_id_temp, sdpro_map_)) {
        find_fp_lane_num++;
      }
      if (lane_id_temp == lane_id) {
        break;
      }
    }

    if (lane_info->change_type() ==
        iflymapdata::sdpro::LaneChangeType::LeftTurnMergingLane) {
      merge_lane_sequence_vec.emplace(std::max(1, find_fp_lane_num - 1),
                                      SPLIT_RIGHT);
      return true;
    } else if (lane_info->change_type() ==
               iflymapdata::sdpro::LaneChangeType::RightTurnMergingLane) {
      merge_lane_sequence_vec.emplace(find_fp_lane_num + 1, SPLIT_LEFT);
      return true;
    }
  }
  return false;
}

bool RouteInfo::CalculateMergeFP(MergeType* merge_type,
                                 iflymapdata::sdpro::FeaturePoint* find_fp,
                                 uint64* fp_link_id, double* dis_to_merge_fp,
                                 double search_distance) {
  if (merge_type == nullptr || find_fp == nullptr || fp_link_id == nullptr ||
      dis_to_merge_fp == nullptr) {
    return false;
  }

  double s = 0.0;
  double l = 0.0;
  auto current_link = CalculateCurrentLink(&s, &l);

  if (!current_link) {
    return false;
  }

  double itera_dis = -s;
  const double check_merge_fp_dis = std::min(800.0, search_distance);

  while (itera_dis < check_merge_fp_dis) {
    for (const auto& fp : current_link->feature_points()) {
      for (const auto& fp_type : fp.type()) {
        if (fp_type ==
            iflymapdata::sdpro::FeaturePointType::LANE_COUNT_CHANGE) {
          itera_dis = itera_dis +
                      fp.projection_percent() * current_link->length() * 0.01;
          if (itera_dis < kEpsilon) {
            continue;
          }
          iflymapdata::sdpro::LaneChangeType temp_merge_type;
          if (IsMergeFP(&temp_merge_type, fp)) {
            *find_fp = fp;
            *fp_link_id = current_link->id();
            *dis_to_merge_fp = itera_dis;

            if (temp_merge_type ==
                iflymapdata::sdpro::LaneChangeType::LeftTurnMergingLane) {
              *merge_type = LEFT_MERGE;
            } else if (temp_merge_type == iflymapdata::sdpro::LaneChangeType::
                                              RightTurnMergingLane) {
              *merge_type = RIGHT_MERGE;
            } else if (temp_merge_type == iflymapdata::sdpro::LaneChangeType::
                                              BothDirectionMergingLane) {
              *merge_type = BOTH_MERGE;
            } else {
              *merge_type = NO_MERGE;
            }
            return true;
          }
        }
      }
    }
    itera_dis = itera_dis + current_link->length() * 0.01;

    current_link = sdpro_map_.GetNextLinkOnRoute(current_link->id());
    if (!current_link) {
      return false;
    }
  }
  return false;
}

bool RouteInfo::CalculateLastFp(
    iflymapdata::sdpro::FeaturePoint* last_fp,
    iflymapdata::sdpro::LinkInfo_Link* last_fp_link, const uint64 fp_link_id,
    const iflymapdata::sdpro::FeaturePoint& find_fp) {
  const auto& fp_link = sdpro_map_.GetLinkOnRoute(fp_link_id);
  if (last_fp == nullptr || fp_link == nullptr || last_fp_link == nullptr ||
      fp_link->feature_points().empty()) {
    return false;
  }

  std::vector<iflymapdata::sdpro::FeaturePoint> fp_vec;
  if (!SortFPBaseProjection(fp_vec, fp_link)) {
    return false;
  }

  int fp_index = -1;
  for (int i = 0; i < fp_vec.size(); i++) {
    const auto& fp = fp_vec[i];
    if (fp.id() == find_fp.id()) {
      fp_index = i;
      break;
    }
  }

  if (fp_index < 0) {
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
      if (fp_pre_link == nullptr) {
        return false;
      }

      itear_dis = itear_dis + fp_pre_link->length() * 0.01;
      if (itear_dis > 500.0) {
        return false;
      }
    }

    // 注：因为上面while计算，所以在这个地方feature_points_size不为0
    if (fp_pre_link->feature_points().empty()) {
      return false;
    }

    std::vector<iflymapdata::sdpro::FeaturePoint> temp_fp_vec;
    if (!SortFPBaseProjection(temp_fp_vec, fp_pre_link)) {
      return false;
    }

    if (temp_fp_vec.empty()) {
      return false;
    }

    *last_fp = temp_fp_vec.back();
    *last_fp_link = *fp_pre_link;
    return true;
  } else {
    if (fp_index - 1 >= fp_vec.size()) {
      return false;
    }

    *last_fp = fp_vec[fp_index - 1];
    *last_fp_link = *fp_link;
    return true;
  }
  return false;
}

bool RouteInfo::IsMergeFP(iflymapdata::sdpro::LaneChangeType* merge_type,
                          const iflymapdata::sdpro::FeaturePoint& fp) const {
  if (merge_type == nullptr) {
    return false;
  }

  for (const auto& lane_id : fp.lane_ids()) {
    const auto& lane_info = sdpro_map_.GetLaneInfoByID(lane_id);
    if (lane_info == nullptr) {
      continue;
    }

    if (lane_info->change_type() ==
            iflymapdata::sdpro::LaneChangeType::LeftTurnMergingLane ||
        lane_info->change_type() ==
            iflymapdata::sdpro::LaneChangeType::RightTurnMergingLane ||
        lane_info->change_type() ==
            iflymapdata::sdpro::LaneChangeType::BothDirectionMergingLane) {
      *merge_type = lane_info->change_type();
      return true;
    }
  }
  return false;
}

bool RouteInfo::IsExpandFP(iflymapdata::sdpro::LaneChangeType* expand_type,
                           const iflymapdata::sdpro::FeaturePoint& fp) const {
  if (expand_type == nullptr) {
    return false;
  }

  for (const auto& lane_id : fp.lane_ids()) {
    const auto& lane_info = sdpro_map_.GetLaneInfoByID(lane_id);
    if (lane_info == nullptr) {
      continue;
    }

    if (lane_info->change_type() ==
            iflymapdata::sdpro::LaneChangeType::LeftTurnExpandingLane ||
        lane_info->change_type() ==
            iflymapdata::sdpro::LaneChangeType::RightTurnExpandingLane ||
        lane_info->change_type() ==
            iflymapdata::sdpro::LaneChangeType::BothDirectionExpandingLane) {
      *expand_type = lane_info->change_type();
      return true;
    }
  }
  return false;
}

const iflymapdata::sdpro::LinkInfo_Link* RouteInfo::CalculateCurrentLink(
    double* s, double* l) {
  if (s == nullptr || l == nullptr) {
    return nullptr;
  }

  const double max_search_length = 7000.0;  // 搜索7km范围内得地图信息
  double search_distance = 50.0;
  double max_heading_diff = PI / 4;
  if (is_in_tunnel_) {
    search_distance = 100.0;
  }
  // 获取当前的segment
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
  if (!current_link) {
    return nullptr;
  }

  *s = temp_nearest_s;
  *l = nearest_l;
  return current_link;
}

bool RouteInfo::CalculateLastFPInCurrentLink(
    iflymapdata::sdpro::FeaturePoint* find_fp,
    const iflymapdata::sdpro::LinkInfo_Link* const cur_link, const double s) {
  if (cur_link == nullptr || find_fp == nullptr) {
    return false;
  }

  std::vector<iflymapdata::sdpro::FeaturePoint> fp_vec;
  for (const auto& fp : cur_link->feature_points()) {
    fp_vec.emplace_back(fp);
  }

  std::sort(fp_vec.begin(), fp_vec.end(),
            [](const iflymapdata::sdpro::FeaturePoint& fp_a,
               const iflymapdata::sdpro::FeaturePoint& fp_b) {
              return fp_a.projection_percent() < fp_b.projection_percent();
            });

  for (int i = fp_vec.size() - 1; i >= 0; i--) {
    const auto& fp = fp_vec[i];
    if (fp.projection_percent() * cur_link->length() * 0.01 < s) {
      *find_fp = fp;
      return true;
    }
  }
  return false;
}

std::vector<int> RouteInfo::CalculateMLCTaskNoLaneNum() {
  std::vector<int> task_num;

  bool is_process_split = false;
  bool is_process_split_split = false;
  bool is_process_other_merge_split = false;
  if (mlc_decider_route_info_.is_process_split ||
      mlc_decider_route_info_.is_process_split_split ||
      mlc_decider_route_info_.is_process_other_merge_split) {
    if (mlc_decider_route_info_.first_static_split_region_info
            .split_direction == SPLIT_LEFT) {
      // 暂时由于不知道右侧有几个车道，因此在当前车道上执行一次变道动作
      task_num.emplace_back(-1);

    } else if (mlc_decider_route_info_.first_static_split_region_info
                   .split_direction == SPLIT_RIGHT) {
      task_num.emplace_back(1);
    }
  }
  if (!task_num.empty()) {
    route_info_output_.mlc_request_type_route_info.mlc_request_type =
        OTHER_TYPE_MLC;
  } else {
    route_info_output_.mlc_request_type_route_info.mlc_request_type = None_MLC;
  }
  return task_num;
}

// 搜寻当前link上有没有REGULAR_INTERSECTION_ENTRANCE（普通路口进入点）
bool RouteInfo::IsClosingIntersectionEntrance(
    const iflymapdata::sdpro::LinkInfo_Link* link,
    const ad_common::sdpromap::SDProMap& sdpro_map, double distance_on_link,
    double max_search_distance) {
  const iflymapdata::sdpro::LinkInfo_Link* current_link = link;
  double search_distance = 0.0 - distance_on_link;

  while (current_link != nullptr) {
    if (search_distance > max_search_distance) {
      return false;
    }

    const double current_link_length =
        static_cast<double>(current_link->length());

    std::vector<iflymapdata::sdpro::FeaturePoint> fp_vec;
    for (const auto& fp : current_link->feature_points()) {
      fp_vec.emplace_back(fp);
    }

    // 2、按照距离排序后，由近向远判断当前link上的fp是否有REGULAR_INTERSECTION_ENTRANCE
    std::sort(fp_vec.begin(), fp_vec.end(),
              [](const iflymapdata::sdpro::FeaturePoint& fp_a,
                 const iflymapdata::sdpro::FeaturePoint& fp_b) {
                return fp_a.projection_percent() < fp_b.projection_percent();
              });

    const int fp_point_size = fp_vec.size();
    for (int i = 0; i < fp_point_size; i++) {
      const auto& fp_point = fp_vec[i];

      const double distance_to_this_point =
          search_distance +
          current_link_length * fp_point.projection_percent() * 0.01;

      if (distance_to_this_point > max_search_distance) {
        return false;
      } else if (distance_to_this_point < 0.0) {
        // 说明当前fp点在自车之前，直接跳过
        continue;
      }

      for (const auto fp_point_type : fp_point.type()) {
        if (fp_point_type == iflymapdata::sdpro::FeaturePointType::
                                 REGULAR_INTERSECTION_ENTRANCE ||
            fp_point_type == iflymapdata::sdpro::FeaturePointType::
                                 REGULAR_INTERSECTION_EXIT) {
          const iflymapdata::sdpro::LinkInfo_Link* next_link =
              sdpro_map.GetNextLinkOnRoute(current_link->id());
          if (!next_link) {
            return false;
          }
          bool is_link_class_not_expressway =
              (current_link->link_class() !=
                   iflymapdata::sdpro::LinkClass::LC_EXPRESSWAY &&
               current_link->link_class() !=
                   iflymapdata::sdpro::LinkClass::LC_CITY_EXPRESSWAY) ||
              (next_link->link_class() !=
                   iflymapdata::sdpro::LinkClass::LC_EXPRESSWAY &&
               next_link->link_class() !=
                   iflymapdata::sdpro::LinkClass::LC_CITY_EXPRESSWAY);
          if (is_link_class_not_expressway) {
            return true;
          }
        }
      }
    }
    search_distance += current_link_length * 0.01;
    current_link = sdpro_map.GetNextLinkOnRoute(current_link->id());
  }

  return false;
}

bool RouteInfo::IsMissSplitPoint(const iflymapdata::sdpro::LinkInfo_Link& link,
                                 const double l, const double s) {
  bool current_frame_is_process_split =
      mlc_decider_route_info_.is_process_split ||
      mlc_decider_route_info_.is_process_split_split;

  if (current_frame_is_process_split) {
    split_link_id_ =
        mlc_decider_route_info_.first_static_split_region_info.split_link_id;
  }

  const auto& ego_pose =
      session_->environmental_model().get_ego_state_manager()->ego_pose();

  int lane_num = 0;

  // 1、触发需要判断是否错过split的条件
  if (last_frame_is_process_split_ && !current_frame_is_process_split) {
    is_need_judge_miss_split_ = true;
    split_point_.set_x(ego_pose.GetX());
    split_point_.set_y(ego_pose.GetY());
  }

  // 2、在非noa模式下，把标志位置false
  if (session_->environmental_model().function_info().function_mode() !=
      common::DrivingFunctionInfo::NOA) {
    is_need_judge_miss_split_ = false;
  }

  // 3、给连续帧间判断赋值
  last_frame_is_process_split_ = current_frame_is_process_split;

  // 4、根据标志位做判断
  if (is_need_judge_miss_split_) {
    const double dx = ego_pose.x - split_point_.x();
    const double dy = ego_pose.y - split_point_.y();

    const double accumulate_dis_to_last_split_point =
        std::sqrt(dx * dx + dy * dy);

    // TODO:需要根据场景细分一下这个距离，暂定100m
    if (accumulate_dis_to_last_split_point > 100) {
      is_need_judge_miss_split_ = false;
      return false;
    }

    // 从当前的link向前继搜索，搜到split link，然后找到split link 的other 后继，

    // 获取split link
    const auto split_link = sdpro_map_.GetLinkOnRoute(split_link_id_);
    if (split_link == nullptr) {
      return false;
    }

    if (split_link->successor_link_ids_size() < 2) {
      return false;
    }

    const auto split_next_link = sdpro_map_.GetNextLinkOnRoute(split_link_id_);

    if (split_next_link == nullptr) {
      return false;
    }

    std::vector<uint64> other_link_id_vec;
    for (const auto tep_link_id : split_link->successor_link_ids()) {
      if (tep_link_id == split_next_link->id()) {
        continue;
      }
      other_link_id_vec.emplace_back(tep_link_id);
    }

    // 判断每一个other link与自车当前的横向l是否小于自车与route link的横向距离
    double dis_to_other_link = NL_NMAX;
    for (const auto tep_link_id : other_link_id_vec) {
      const auto tep_link = sdpro_map_.GetLinkOnRoute(tep_link_id);
      if (tep_link == nullptr) {
        continue;
      }

      planning_math::Vec2d point{ego_pose.x, ego_pose.y};

      const auto& tep_link_points = tep_link->points().boot().points();
      if (tep_link_points.empty()) {
        continue;
      }

      planning_math::Vec2d segment_start{tep_link_points.begin()->x(),
                                         tep_link_points.begin()->y()};
      planning_math::Vec2d segment_end{tep_link_points.rbegin()->x(),
                                       tep_link_points.rbegin()->y()};

      double dis_to_other_link =
          DistanceToLine(point, segment_start, segment_end);

      if (dis_to_other_link < std::abs(l)) {
        break;
      }
    }

    iflymapdata::sdpro::FeaturePoint last_fp;

    if (CalculateLastFPInCurrentLink(&last_fp, &link, s)) {
      for (const auto& lane_id : last_fp.lane_ids()) {
        if (IsEmergencyLane(lane_id, sdpro_map_)) {
          continue;
        }

        lane_num++;
      }
    } else {
      lane_num = link.lane_num();
    }

    const double lat_error = lane_num * kStandardLaneWidth;

    if (std::abs(l) > lat_error && dis_to_other_link < std::abs(l)) {
      return true;
    }
  }

  return false;
}

double RouteInfo::DistanceToLine(const planning_math::Vec2d& point,
                                 const planning_math::Vec2d& segment_start,
                                 const planning_math::Vec2d& segment_end) {
  // 计算线段向量
  planning_math::Vec2d segment = segment_end - segment_start;
  // 计算点到线段起点的向量
  planning_math::Vec2d point_to_start = point - segment_start;

  // 计算线段长度
  const double segment_len = segment.Length();

  // 处理线段长度为0的特殊情况（起点和终点重合）
  if (segment_len < planning_math::kMathEpsilon) {
    return point.DistanceTo(segment_start);
  }

  // 计算叉积的绝对值（平行四边形面积）
  double cross_product = std::fabs(point_to_start.CrossProd(segment));

  // 垂直距离 = 平行四边形面积 / 底边长（线段长度）
  return cross_product / segment_len;
}

bool RouteInfo::IsTriggerContinueLCInPerceptionSplitRegion(
    const int perception_left_lane_num,
    const int perception_right_lane_num) const {
  bool is_split_region = session_->planning_context()
                             .ego_lane_road_right_decider_output()
                             .is_split_region;

  if (!is_split_region) {
    return false;
  }

  bool is_split_region_process =
      mlc_decider_route_info_.is_process_split ||
      mlc_decider_route_info_.is_process_other_merge_split ||
      mlc_decider_route_info_.is_process_split_split;

  if (!is_split_region_process) {
    return false;
  }

  if (route_info_output_.split_region_info_list.empty()) {
    return false;
  }

  const auto& split_region_info_list =
      route_info_output_.split_region_info_list[0];

  const auto& split_direction = split_region_info_list.split_direction;

  const auto& virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();

  int split_lane_vitrual_id = session_->planning_context()
                                  .ego_lane_road_right_decider_output()
                                  .split_lane_virtual_id;

  bool is_exist_right_lane = virtual_lane_manager->get_right_lane() != nullptr;
  bool is_exist_left_lane = virtual_lane_manager->get_left_lane() != nullptr;

  if (split_direction == SPLIT_RIGHT && is_exist_right_lane &&
      perception_right_lane_num == 0) {
    return true;
  } else if (split_direction == SPLIT_LEFT && is_exist_left_lane &&
             perception_left_lane_num == 0) {
    return true;
  }
  return false;
}

bool RouteInfo::IsExistLengthSolidLine(
    std::vector<std::pair<const MarkingLineChangeType, double>>&
        mlc_fp_info_list,
    const uint64 fp_link_id, const iflymapdata::sdpro::FeaturePoint cur_fp,
    const double first_distance_to_split_point) {
  iflymapdata::sdpro::FeaturePoint mlc_fp;
  uint64 mlc_link_id;
  double cal_sum_dis;

  uint64 temp_fp_link_id = fp_link_id;
  iflymapdata::sdpro::FeaturePoint temp_cur_fp = cur_fp;

  bool is_continue_find_mlc_fp = true;

  while (is_continue_find_mlc_fp) {
    if (CalculateLastMarkingLineChangeFp(&mlc_fp, &mlc_link_id, &cal_sum_dis,
                                         temp_fp_link_id, temp_cur_fp,
                                         first_distance_to_split_point)) {
      MarkingLineChangeType marking_line_change_type;
      if (IsDashSolidLineTypeChnage(&marking_line_change_type, mlc_fp,
                                    mlc_link_id)) {
        mlc_fp_info_list.emplace_back(marking_line_change_type, cal_sum_dis);
      }

      temp_fp_link_id = mlc_link_id;
      temp_cur_fp = mlc_fp;
    } else {
      if (mlc_fp_info_list.empty()) {
        return false;
      } else {
        return true;
      }
    }

    // todo: 需要根据实际变道距离替换3000m
    is_continue_find_mlc_fp =
        cal_sum_dis < 3000.0 && cal_sum_dis < first_distance_to_split_point;
  }

  if (mlc_fp_info_list.empty()) {
    return false;
  } else {
    return true;
  }

  return false;
}

bool RouteInfo::CalculateLastMarkingLineChangeFp(
    iflymapdata::sdpro::FeaturePoint* mlc_fp, uint64* mlc_link_id,
    double* cal_sum_dis, const uint64 fp_link_id,
    const iflymapdata::sdpro::FeaturePoint cur_fp,
    const double first_distance_to_split_point) {
  const auto& fp_link = sdpro_map_.GetLinkOnRoute(fp_link_id);
  // fp_link->feature_points().empty() 这个条件不对，有可能当前link上就是没有fp
  if (mlc_fp == nullptr || fp_link == nullptr) {
    return false;
  }

  bool is_continue_find_mlc_fp = true;

  iflymapdata::sdpro::FeaturePoint temp_mlc_fp;
  double sum_dis = *cal_sum_dis;
  uint64 cur_link_id = fp_link_id;

  while (is_continue_find_mlc_fp) {
    double temp_sum_dis = 0.0;
    if (cur_link_id == fp_link_id) {
      if (IsExistMarkingLineChangeFPCurLink(&temp_mlc_fp, &temp_sum_dis,
                                            cur_link_id, cur_fp)) {
        *mlc_fp = temp_mlc_fp;
        *mlc_link_id = cur_link_id;

        sum_dis = sum_dis + temp_sum_dis;
        *cal_sum_dis = sum_dis;
        return true;
      } else {
        const auto& pre_link = sdpro_map_.GetPreviousLinkOnRoute(cur_link_id);
        // 如果是nullptr的时候，需要注意处理这种情况
        if (pre_link == nullptr) {
          return false;
        } else {
          cur_link_id = pre_link->id();
          sum_dis = sum_dis + temp_sum_dis;
        }
      }
    } else {
      if (IsExistMarkingLineChangeFP(&temp_mlc_fp, &temp_sum_dis,
                                     cur_link_id)) {
        *mlc_fp = temp_mlc_fp;
        *mlc_link_id = cur_link_id;

        sum_dis = sum_dis + temp_sum_dis;
        *cal_sum_dis = sum_dis;
        return true;

      } else {
        const auto& pre_link = sdpro_map_.GetPreviousLinkOnRoute(cur_link_id);
        // 如果是nullptr的时候，需要注意处理这种情况
        if (pre_link == nullptr) {
          return false;
        } else {
          cur_link_id = pre_link->id();
          sum_dis = sum_dis + temp_sum_dis;
        }
      }
    }

    *cal_sum_dis = sum_dis;
    // todo：后续用需要的变道距离来替换3000m
    is_continue_find_mlc_fp =
        sum_dis < 3000.0 && sum_dis < first_distance_to_split_point;
  }

  return false;
}

bool RouteInfo::IsExistMarkingLineChangeFP(
    iflymapdata::sdpro::FeaturePoint* mlc_fp, double* sum_dis,
    const uint64 cur_link_id) {
  const auto& cur_link = sdpro_map_.GetLinkOnRoute(cur_link_id);
  if (mlc_fp == nullptr || cur_link == nullptr) {
    return false;
  }

  if (cur_link->feature_points().empty()) {
    *sum_dis = cur_link->length() * 0.01;
    // *pre_link_id = pre_link->id();
    return false;
  }

  std::vector<iflymapdata::sdpro::FeaturePoint> fp_vec;
  for (const auto& fp : cur_link->feature_points()) {
    fp_vec.emplace_back(fp);
  }

  std::sort(fp_vec.begin(), fp_vec.end(),
            [](const iflymapdata::sdpro::FeaturePoint& fp_a,
               const iflymapdata::sdpro::FeaturePoint& fp_b) {
              return fp_a.projection_percent() > fp_b.projection_percent();
            });

  for (const auto& fp : fp_vec) {
    for (const auto& fp_type : fp.type()) {
      if (fp_type ==
          iflymapdata::sdpro::FeaturePointType::MARKING_LINE_CHANGE_POINT) {
        *sum_dis = (1 - fp.projection_percent()) * cur_link->length() * 0.01;
        *mlc_fp = fp;
        return true;
      }
    }
  }

  *sum_dis = cur_link->length() * 0.01;

  return false;
}

bool RouteInfo::IsExistMarkingLineChangeFPCurLink(
    iflymapdata::sdpro::FeaturePoint* mlc_fp, double* sum_dis,
    const uint64 cur_link_id, const iflymapdata::sdpro::FeaturePoint cur_fp) {
  const auto& cur_link = sdpro_map_.GetLinkOnRoute(cur_link_id);
  if (mlc_fp == nullptr || cur_link == nullptr || sum_dis == nullptr) {
    return false;
  }

  if (cur_link->feature_points().empty()) {
    // 由于传进来的是当前link和link上的fp，不应该进入到这里来
    *sum_dis = cur_link->length() * 0.01;
    // *pre_link_id = pre_link->id();
    return false;
  }

  std::vector<iflymapdata::sdpro::FeaturePoint> fp_vec;
  for (const auto& fp : cur_link->feature_points()) {
    fp_vec.emplace_back(fp);
  }

  std::sort(fp_vec.begin(), fp_vec.end(),
            [](const iflymapdata::sdpro::FeaturePoint& fp_a,
               const iflymapdata::sdpro::FeaturePoint& fp_b) {
              return fp_a.projection_percent() > fp_b.projection_percent();
            });

  for (const auto& fp : fp_vec) {
    if (fp.projection_percent() > cur_fp.projection_percent() - kEpsilon) {
      continue;
    }

    for (const auto& fp_type : fp.type()) {
      if (fp_type ==
          iflymapdata::sdpro::FeaturePointType::MARKING_LINE_CHANGE_POINT) {
        *sum_dis =
            std::abs((fp.projection_percent() - cur_fp.projection_percent())) *
            cur_link->length() * 0.01;
        *mlc_fp = fp;
        return true;
      }
    }
  }

  *sum_dis = cur_fp.projection_percent() * cur_link->length() * 0.01;

  return false;
}

bool RouteInfo::IsDashSolidLineTypeChnage(
    MarkingLineChangeType* marking_line_change_type,
    const iflymapdata::sdpro::FeaturePoint& mlc_fp, const uint64 mlc_link_id) {
  bool is_solid_front = false;
  bool is_solid_rear = false;

  const int emergency_lane_num = EmergencyLaneNum(mlc_fp);
  for (const auto& lane_id : mlc_fp.lane_ids()) {
    if (IsSolidBoundary(lane_id)) {
      // lane的boundary是否为实线时，需要考虑最左边车道的左边界和最右边车道的右boundary。
      // todo：目前仅考虑了最右边车道的右boundary。
      const auto& lane = sdpro_map_.GetLaneInfoByID(lane_id);
      if (lane == nullptr) {
        continue;
      }

      if (lane->sequence() == 1 + emergency_lane_num) {
        continue;
      }

      is_solid_front = true;
      break;
    }
  }

  iflymapdata::sdpro::FeaturePoint last_fp;
  iflymapdata::sdpro::LinkInfo_Link last_fp_link;
  if (CalculateLastFp(&last_fp, &last_fp_link, mlc_link_id, mlc_fp)) {
    const int emergency_lane_num = EmergencyLaneNum(last_fp);
    for (const auto& lane_id : last_fp.lane_ids()) {
      if (IsSolidBoundary(lane_id)) {
        // lane的boundary是否为实线时，需要考虑最左边车道的左边界和最右边车道的右boundary。
        // todo：目前仅考虑了最右边车道的右boundary。
        const auto& lane = sdpro_map_.GetLaneInfoByID(lane_id);
        if (lane == nullptr) {
          continue;
        }

        if (lane->sequence() == 1 + emergency_lane_num) {
          continue;
        }

        is_solid_rear = true;
        break;
      }
    }
  }

  if ((is_solid_front && is_solid_rear) ||
      (!is_solid_front && !is_solid_rear)) {
    return false;
  } else if (is_solid_front && !is_solid_rear) {
    *marking_line_change_type = MarkingLineChangeType::DASH_TO_SOLID;
    return true;
  } else if (!is_solid_front && is_solid_rear) {
    *marking_line_change_type = MarkingLineChangeType::SOLID_TO_DASH;
    return true;
  }

  return false;
}

bool RouteInfo::IsSolidBoundary(const uint64 lane_id) {
  const auto& lane = sdpro_map_.GetLaneInfoByID(lane_id);
  if (lane == nullptr) {
    return false;
  }

  if (IsEmergencyLane(lane_id, sdpro_map_)) {
    return false;
  }

  for (const auto& boundary : lane->right_boundaries()) {
    if (boundary.divider_marking_type() ==
            iflymapdata::sdpro::LaneBoundary::DivederMarkingType::
                LaneBoundary_DivederMarkingType_DMT_MARKING_SINGLE_SOLID_LINE ||
        boundary.divider_marking_type() ==
            iflymapdata::sdpro::LaneBoundary::DivederMarkingType::
                LaneBoundary_DivederMarkingType_DMT_MARKING_DOUBLE_SOLID_LINE) {
      return true;
    }
  }

  return false;
}

double RouteInfo::LengthSolidLineJudge(
    const uint64 fp_link_id, const iflymapdata::sdpro::FeaturePoint cur_fp,
    const double first_distance_to_split_point) {
  double pre_mlc_dis = 0.0;
  // 从交换区起点到自车按顺序加入虚、实线改变的标线变化点
  std::vector<std::pair<const MarkingLineChangeType, double>> mlc_fp_info_list;

  if (!IsExistLengthSolidLine(mlc_fp_info_list, fp_link_id, cur_fp,
                              first_distance_to_split_point)) {
    return pre_mlc_dis;
  }

  // 1、只有一个虚线到实线的标线变换点
  if (mlc_fp_info_list.size() == 1 &&
      mlc_fp_info_list[0].first == MarkingLineChangeType::DASH_TO_SOLID) {
    return mlc_fp_info_list[0].second;
  }

  // 2、第一个是虚线到实线的变化点，后面的是单独的实线段，需要判断这个两个实线段之间的距离相隔多远。
  // 2.1 首先判断需要变几次道，计算变道所需要的距离
  double lc_need_dis = 0.0;
  const double v_cruise =
      session_->environmental_model().get_ego_state_manager()->ego_v_cruise();

  const auto& cur_lane = session_->environmental_model()
                             .get_virtual_lane_manager()
                             ->get_current_lane();

  if (cur_lane == nullptr) {
    return pre_mlc_dis;
  }

  int left_lane_num = 0;
  int right_lane_num = 0;
  for (const auto& lane_num : cur_lane->get_lane_nums()) {
    if (lane_num.end > kEpsilon) {
      left_lane_num = lane_num.left_lane_num;
      right_lane_num = lane_num.right_lane_num;
      break;
    }
  }

  if (route_info_output_.split_region_info_list.empty()) {
    return pre_mlc_dis;
  }
  const auto& split_link_info = route_info_output_.split_region_info_list[0];

  const auto& split_link =
      sdpro_map_.GetLinkOnRoute(split_link_info.split_link_id);
  if (split_link == nullptr) {
    return pre_mlc_dis;
  }

  int lc_num = 0;
  if (split_link_info.split_direction == SplitDirection::SPLIT_RIGHT) {
    lc_num = split_link->lane_num() - (left_lane_num + 1);
    lc_num = std::max(0, lc_num);
  } else if (split_link_info.split_direction == SplitDirection::SPLIT_LEFT) {
    lc_num = std::max(0, left_lane_num);
  }

  lc_need_dis = lc_num * v_cruise * 6.0;

  int mlc_fp_size = mlc_fp_info_list.size();
  if (mlc_fp_size > 1 &&
      mlc_fp_info_list[0].first == MarkingLineChangeType::DASH_TO_SOLID) {
    // pair<实线起点距离，实线终点距离>
    std::vector<std::pair<double, double>> solid_lines_info;
    for (int i = 1; i + 1 < mlc_fp_size; ++i) {
      if (mlc_fp_info_list[i].first == MarkingLineChangeType::SOLID_TO_DASH &&
          mlc_fp_info_list[i + 1].first ==
              MarkingLineChangeType::DASH_TO_SOLID) {
        solid_lines_info.emplace_back(mlc_fp_info_list[i].second,
                                      mlc_fp_info_list[i + 1].second);
      }
    }

    if (solid_lines_info.empty()) {
      return mlc_fp_info_list[0].second;
    }

    for (int i = 0; i < solid_lines_info.size(); ++i) {
      if (i == 0) {
        if (solid_lines_info[i].first - mlc_fp_info_list[0].second >
            lc_need_dis) {
          return mlc_fp_info_list[0].second;
        }
      } else {
        if (solid_lines_info[i].first - solid_lines_info[i - 1].second >
            lc_need_dis) {
          return solid_lines_info[i - 1].second;
        }
      }
    }

    return solid_lines_info.back().second;

  } else if (mlc_fp_size > 1 && mlc_fp_info_list[0].first ==
                                    MarkingLineChangeType::SOLID_TO_DASH) {
    if (mlc_fp_info_list[0].second > lc_need_dis) {
      return 0.0;
    }

    // pair<实线起点距离，实线终点距离>
    std::vector<std::pair<double, double>> solid_lines_info;
    for (int i = 0; i + 1 < mlc_fp_size; ++i) {
      if (mlc_fp_info_list[i].first == MarkingLineChangeType::SOLID_TO_DASH &&
          mlc_fp_info_list[i + 1].first ==
              MarkingLineChangeType::DASH_TO_SOLID) {
        solid_lines_info.emplace_back(mlc_fp_info_list[i].second,
                                      mlc_fp_info_list[i + 1].second);
      }
    }

    if (solid_lines_info.empty()) {
      return 0.0;
    }

    for (int i = 1; i < solid_lines_info.size(); ++i) {
      if (solid_lines_info[i].first - solid_lines_info[i - 1].second >
          lc_need_dis) {
        return solid_lines_info[i - 1].second;
      }
    }

    return solid_lines_info.back().second;
  }

  return pre_mlc_dis;
}

int RouteInfo::EmergencyLaneNum(
    const iflymapdata::sdpro::FeaturePoint& mlc_fp) {
  int emergency_lane_num = 0;
  for (const auto& lane_id : mlc_fp.lane_ids()) {
    if (IsEmergencyLane(lane_id, sdpro_map_)) {
      emergency_lane_num++;
    }
  }

  return emergency_lane_num;
}

bool RouteInfo::SortFPBaseProjection(
    std::vector<iflymapdata::sdpro::FeaturePoint>& sorted_fp,
    const iflymapdata::sdpro::LinkInfo_Link* link) const {
  if (link == nullptr) {
    return false;
  }
  sorted_fp.clear();

  for (const auto& fp : link->feature_points()) {
    sorted_fp.emplace_back(fp);
  }

  std::sort(sorted_fp.begin(), sorted_fp.end(),
            [](const iflymapdata::sdpro::FeaturePoint& fp_a,
               const iflymapdata::sdpro::FeaturePoint& fp_b) {
              return fp_a.projection_percent() < fp_b.projection_percent();
            });

  return true;
}

bool RouteInfo::CalculateDistanceToLastSplitPoint(double* dis, const double s,
                                                  uint64* split_link_id) const {
  // 计算在匝道上距离上一个split点的信息
  if (current_link_ == nullptr) {
    return false;
  }

  const iflymapdata::sdpro::LinkInfo_Link* temp_last_split_seg =
      sdpro_map_.GetPreviousLinkOnRoute(current_link_->id());

  if (!temp_last_split_seg) {
    return false;
  }

  double accumulate_dis_ego_to_last_split_point = s;

  while (temp_last_split_seg->successor_link_ids().size() == 1) {
    accumulate_dis_ego_to_last_split_point =
        accumulate_dis_ego_to_last_split_point +
        temp_last_split_seg->length() * 0.01;

    temp_last_split_seg =
        sdpro_map_.GetPreviousLinkOnRoute(temp_last_split_seg->id());

    if (!temp_last_split_seg) {
      return false;
    }
  }

  if (temp_last_split_seg &&
      temp_last_split_seg->successor_link_ids().size() >= 2) {
    *dis = accumulate_dis_ego_to_last_split_point;
    *split_link_id = temp_last_split_seg->id();

    return true;
  }

  return false;
}

bool RouteInfo::CalculateDistanceToLastMergePoint(double* dis, const double s,
                                                  uint64* merge_link_id) const {
  if (current_link_ == nullptr) {
    return false;
  }

  const iflymapdata::sdpro::LinkInfo_Link* last_merge_link = current_link_;

  double sum_dis_to_last_merge_point = s;

  while (last_merge_link->predecessor_link_ids().size() == 1) {
    last_merge_link = sdpro_map_.GetPreviousLinkOnRoute(last_merge_link->id());
    // 判断是否为nullptr
    if (last_merge_link == nullptr) {
      return false;
    }

    sum_dis_to_last_merge_point =
        sum_dis_to_last_merge_point + last_merge_link->length() * 0.01;
  }

  if (last_merge_link && last_merge_link->predecessor_link_ids().size() == 2) {
    *dis = sum_dis_to_last_merge_point;
    *merge_link_id = last_merge_link->id();
    return true;
  }

  return false;
}

bool RouteInfo::CalculateDistanceNextToLastSplitPoint(
    double* dis,
    const iflymapdata::sdpro::LinkInfo_Link* next_split_or_merge_link) const {
  if (next_split_or_merge_link == nullptr) {
    return false;
  }
  const iflymapdata::sdpro::LinkInfo_Link* temp_last_split_seg =
      next_split_or_merge_link;
  double accumulate_dis_next_to_last_split_point = 0.0;
  accumulate_dis_next_to_last_split_point +=
      temp_last_split_seg->length() * 0.01;
  temp_last_split_seg =
      sdpro_map_.GetPreviousLinkOnRoute(temp_last_split_seg->id());

  if (temp_last_split_seg == nullptr) {
    return false;
  }
  while (temp_last_split_seg->successor_link_ids().size() == 1) {
    accumulate_dis_next_to_last_split_point =
        accumulate_dis_next_to_last_split_point +
        temp_last_split_seg->length() * 0.01;

    temp_last_split_seg =
        sdpro_map_.GetPreviousLinkOnRoute(temp_last_split_seg->id());

    if (!temp_last_split_seg) {
      return false;
    }
  }

  if (temp_last_split_seg &&
      temp_last_split_seg->successor_link_ids().size() >= 2) {
    *dis = accumulate_dis_next_to_last_split_point;

    return true;
  }

  return false;
}

bool RouteInfo::CalculateDistanceNextToLastMergePoint(
    double* dis,
    const iflymapdata::sdpro::LinkInfo_Link* next_split_or_merge_link) const {
  if (next_split_or_merge_link == nullptr) {
    return false;
  }
  const iflymapdata::sdpro::LinkInfo_Link* temp_last_merge_seg =
      next_split_or_merge_link;
  double accumulate_dis_next_to_last_merge_point = 0.0;

  accumulate_dis_next_to_last_merge_point +=
      temp_last_merge_seg->length() * 0.01;
  temp_last_merge_seg =
      sdpro_map_.GetPreviousLinkOnRoute(temp_last_merge_seg->id());

  if (temp_last_merge_seg == nullptr) {
    return false;
  }
  while (temp_last_merge_seg->predecessor_link_ids().size() == 1) {
    accumulate_dis_next_to_last_merge_point =
        accumulate_dis_next_to_last_merge_point +
        temp_last_merge_seg->length() * 0.01;

    temp_last_merge_seg =
        sdpro_map_.GetPreviousLinkOnRoute(temp_last_merge_seg->id());
    // 判断是否为nullptr
    if (temp_last_merge_seg == nullptr) {
      return false;
    }
  }

  if (temp_last_merge_seg &&
      temp_last_merge_seg->predecessor_link_ids().size() >= 2) {
    *dis = accumulate_dis_next_to_last_merge_point;
    return true;
  }

  return false;
}
void RouteInfo::EraseSplitSplitFeasibleLane(
    NOASplitRegionInfo& first_split_region_info,
    NOASplitRegionInfo& second_split_region_info, int erase_num) {
  if (first_split_region_info.recommend_lane_num.size() != 4) {
    return;
  }

  // 矫正方向不能单从下一个exchange的方向，可能连续两个exchange都是为了后面的ramp_split准备
  // 如果后一个exchange是split_left，但feasible
  // lane中不带有1，认为已经在为后续ramp_split准备
  if (second_split_region_info.split_direction == SplitDirection::SPLIT_LEFT &&
      second_split_region_info.recommend_lane_num[0]
              .feasible_lane_sequence[0] == static_cast<int>(1)) {
    std::vector<int> missing_elements = findMissingElements(
        first_split_region_info.recommend_lane_num[0].feasible_lane_sequence,
        second_split_region_info.recommend_lane_num[0].feasible_lane_sequence);
    if (missing_elements.empty()) {
      return;
    }
    for (int i = 0; i < 3; ++i) {
      auto& lane_sequence =
          first_split_region_info.recommend_lane_num[i].feasible_lane_sequence;
      if (lane_sequence.size() > erase_num) {
        lane_sequence.erase(lane_sequence.end() - erase_num,
                            lane_sequence.end());
      } else if (lane_sequence.size() > 1) {
        lane_sequence.erase(lane_sequence.begin() + 1, lane_sequence.end());
      }
    }
  } else {
    for (int i = 0; i < 3; ++i) {
      auto& lane_sequence =
          first_split_region_info.recommend_lane_num[i].feasible_lane_sequence;
      if (lane_sequence.size() > erase_num) {
        lane_sequence.erase(lane_sequence.begin(),
                            lane_sequence.begin() + erase_num);
      } else if (lane_sequence.size() > 1) {
        lane_sequence.erase(lane_sequence.begin(), lane_sequence.end() - 1);
      }
    }

    if (first_split_region_info.split_direction == SplitDirection::SPLIT_LEFT &&
        second_split_region_info.split_direction ==
            SplitDirection::SPLIT_LEFT) {
      for (int i = 0; i < 3; ++i) {
        auto& first_lane_sequence =
            first_split_region_info.recommend_lane_num[i]
                .feasible_lane_sequence;
        if (first_lane_sequence[0] >
            second_split_region_info.recommend_lane_num[0]
                .feasible_lane_sequence.back()) {
          int num_to_add = first_lane_sequence[0] -
                           second_split_region_info.recommend_lane_num[0]
                               .feasible_lane_sequence.back();
          for (int n = num_to_add - 1; n >= 0; --n) {
            first_lane_sequence.insert(first_lane_sequence.begin(),
                                       first_lane_sequence[0] - n - 1);
          }
        }
      }
    }
  }
}
void RouteInfo::OptimizeFeasibleLanesByDistance(
    NOASplitRegionInfo& exchange_region_info,
    std::map<int, double>& feasible_lane_distance, double max_distance,
    int current_lane_num) {
  const int total_lane_num =
      exchange_region_info.recommend_lane_num[0].total_lane_num;
  bool is_merge_exchange_region = exchange_region_info.is_ramp_merge ||
                                  exchange_region_info.is_other_merge_to_road;

  double opt_distance = 800.0;
  if (!current_link_) {
    opt_distance = 800.0;
  } else {
    opt_distance = current_link_->link_class() ==
                           iflymapdata::sdpro::LinkClass::LC_EXPRESSWAY
                       ? 800.0
                       : 500.0;
  }

  std::vector<int> temp_feasible_lane_seq;
  std::vector<double> temp_feasible_lane_dis;

  auto& feasible_lane_sequence =
      exchange_region_info.recommend_lane_num[0].feasible_lane_sequence;
  for (int i = 0; i < feasible_lane_sequence.size(); i++) {
    feasible_lane_distance[feasible_lane_sequence[i]] = max_distance;
  }
  // 判断是不是下匝道的exchange，是则需要考虑长实线
  double lsl_length = 0;
  if ((!is_merge_exchange_region &&
       exchange_region_info.split_direction == SPLIT_RIGHT) ||
      (is_merge_exchange_region &&
       exchange_region_info.split_direction == SPLIT_LEFT)) {
    for (int i = 1; i < feasible_lane_sequence[0]; i++) {
      // 因为split在右边，所以存在都是右边的车道，seq都比左边的大
      const int lc_num = feasible_lane_sequence[0] - i;
      const double lc_need_dis = opt_distance * std::fabs(lc_num);
      std::vector<std::vector<LSLInfo>> lsl_info_vec;
      bool is_lsl = CalculateLSLDistance(
          current_link_, route_info_output_.current_segment_passed_distance,
          exchange_region_info.distance_to_split_point, lsl_info_vec);
      if (is_lsl) {
        lsl_length = CalculateLSLTotalLength(
            i, lsl_info_vec, exchange_region_info.split_direction);
        route_info_output_.lsl_length = lsl_length;
        max_distance = std::max(0.0, max_distance - lsl_length);
      }
      if (max_distance > lc_need_dis) {
        temp_feasible_lane_seq.emplace_back(i);
        temp_feasible_lane_dis.emplace_back(max_distance - lc_need_dis);
      }
    }

    if (!temp_feasible_lane_seq.empty()) {
      feasible_lane_sequence.insert(feasible_lane_sequence.begin(),
                                    temp_feasible_lane_seq.begin(),
                                    temp_feasible_lane_seq.end());
      for (int i = 0; i < temp_feasible_lane_seq.size(); i++) {
        feasible_lane_distance[temp_feasible_lane_seq[i]] =
            temp_feasible_lane_dis[i];
      }
    }
  } else if ((!is_merge_exchange_region &&
              exchange_region_info.split_direction == SPLIT_LEFT) ||
             (is_merge_exchange_region &&
              exchange_region_info.split_direction == SPLIT_RIGHT)) {
    for (int i = current_lane_num; i > feasible_lane_sequence.back(); i--) {
      const int lc_num = i - feasible_lane_sequence.back();
      const double lc_need_dis = opt_distance * std::fabs(lc_num);
      std::vector<std::vector<LSLInfo>> lsl_info_vec;
      bool is_lsl = CalculateLSLDistance(
          current_link_, route_info_output_.current_segment_passed_distance,
          exchange_region_info.distance_to_split_point, lsl_info_vec);
      if (is_lsl) {
        lsl_length = CalculateLSLTotalLength(
            i, lsl_info_vec, exchange_region_info.split_direction);
        route_info_output_.lsl_length = lsl_length;
        max_distance = std::max(0.0, max_distance - lsl_length);
      }
      if (max_distance > lc_need_dis) {
        temp_feasible_lane_seq.insert(temp_feasible_lane_seq.begin(), i);
        temp_feasible_lane_dis.insert(temp_feasible_lane_dis.begin(),
                                      (max_distance - lc_need_dis));
      }
    }

    if (!temp_feasible_lane_seq.empty()) {
      feasible_lane_sequence.insert(feasible_lane_sequence.end(),
                                    temp_feasible_lane_seq.begin(),
                                    temp_feasible_lane_seq.end());
      for (int i = 0; i < temp_feasible_lane_seq.size(); i++) {
        feasible_lane_distance[temp_feasible_lane_seq[i]] =
            temp_feasible_lane_dis[i];
      }
    }
  }
}
std::vector<int> RouteInfo::GetIntersection(const std::vector<int>& vec1,
                                            const std::vector<int>& vec2) {
  std::vector<int> result;

  for (const auto& item1 : vec1) {
    if (std::find(vec2.begin(), vec2.end(), item1) != vec2.end()) {
      result.emplace_back(item1);
    }
  }

  if (result.empty() && !vec1.empty() && !vec2.empty()) {
    int min_vec2 = *std::min_element(vec2.begin(), vec2.end());
    auto closest_it =
        std::min_element(vec1.begin(), vec1.end(), [min_vec2](int a, int b) {
          return std::abs(a - min_vec2) < std::abs(b - min_vec2);
        });
    result.emplace_back(*closest_it);
  }

  return result;
}
void RouteInfo::FindNextMergeExpandTypeFp(
    const FPPoint& fp_start,
    std::vector<iflymapdata::sdpro::FeaturePoint>& specific_fp,
    double max_search_distance) {
  const iflymapdata::sdpro::LinkInfo_Link* current_link =
      sdpro_map_.GetLinkOnRoute(fp_start.link_id);
  if (current_link == nullptr) {
    return;
  }
  double search_distance = -static_cast<double>(current_link->length()) *
                           fp_start.fp.projection_percent() * 0.01;

  while (current_link != nullptr) {
    if (search_distance - max_search_distance > kEpsilon) {
      return;
    }

    const double current_link_length =
        static_cast<double>(current_link->length());

    std::vector<iflymapdata::sdpro::FeaturePoint> fp_vec;
    for (const auto& fp : current_link->feature_points()) {
      fp_vec.emplace_back(fp);
    }

    // 2、按照距离排序后，由近向远判断当前特定类型的fp
    std::sort(fp_vec.begin(), fp_vec.end(),
              [](const iflymapdata::sdpro::FeaturePoint& fp_a,
                 const iflymapdata::sdpro::FeaturePoint& fp_b) {
                return fp_a.projection_percent() < fp_b.projection_percent();
              });

    const int fp_point_size = fp_vec.size();
    for (int i = 0; i < fp_point_size; i++) {
      const auto& fp_point = fp_vec[i];

      const double distance_to_this_point =
          search_distance +
          current_link_length * fp_point.projection_percent() * 0.01;

      if (distance_to_this_point - max_search_distance > kEpsilon) {
        return;
      } else if (distance_to_this_point < 0.0) {
        // 说明当前fp点在start_fp之前，直接跳过
        continue;
      }
      iflymapdata::sdpro::LaneChangeType fp_type;
      if (IsMergeFP(&fp_type, fp_point)) {
        specific_fp.emplace_back(fp_point);
      }
      if (IsExpandFP(&fp_type, fp_point)) {
        specific_fp.emplace_back(fp_point);
      }
    }
    search_distance += current_link_length * 0.01;
    current_link = sdpro_map_.GetNextLinkOnRoute(current_link->id());
  }
  return;
}
void RouteInfo::ProcessLaneDistance(
    const std::shared_ptr<VirtualLane>& relative_id_lane,
    const std::map<int, double>& feasible_lane_distance) {
  if (!relative_id_lane) {
    return;
  }
  const auto& lane_nums = relative_id_lane->get_lane_nums();
  if (lane_nums.empty()) {
    return;
  }
  int left_lane_num = 0;

  for (const auto& lane_num : lane_nums) {
    if (lane_num.end > kEpsilon) {
      left_lane_num = lane_num.left_lane_num;
      break;
    }
  }

  auto it = feasible_lane_distance.find(left_lane_num + 1);
  std::pair<bool, double> virtual_lane_distance;

  if (it != feasible_lane_distance.end()) {
    virtual_lane_distance = std::make_pair(true, it->second);
  } else {
    virtual_lane_distance = std::make_pair(false, 0.0);
  }

  if (left_lane_num + 1 == route_info_output_.ego_seq - 1) {
    route_info_output_.left_lane_distance = virtual_lane_distance.second;
  } else if (left_lane_num == route_info_output_.ego_seq) {
    route_info_output_.right_lane_distance = virtual_lane_distance.second;
  }
  relative_id_lane->set_feasible_lane_distance(virtual_lane_distance);
}

void RouteInfo::GetStrategy() {
  static bool map_updated = false;
  const auto& local_view = session_->environmental_model().get_local_view();
  if (!map_updated) {
    if (local_view.sdpro_map_info.data_source() ==
        iflymapdata::sdpro::MAP_VENDOR_BAIDU_LD) {
      route_info_strategy_ =
          std::make_shared<LDRouteInfoStrategy>(&mlc_decider_config_, session_);
      map_updated = true;
    } else if (local_view.sdpro_map_info.data_source() ==
               iflymapdata::sdpro::MAP_VENDOR_TENCENT_SD_PRO) {
      route_info_strategy_ = std::make_shared<SDProRouteInfoStrategy>(
          &mlc_decider_config_, session_);
      map_updated = true;
    }
  }
}

void RouteInfo::UpdateMLCInfoDeciderBaseBaidu(
    const std::vector<std::shared_ptr<VirtualLane>>& relative_id_lanes) {
  route_info_strategy_->CalculateMLCDecider(relative_id_lanes,
                                            route_info_output_);
}
// 向前搜索有没有收费站进入点
bool RouteInfo::IsClosingTollStationEntrance(
    const iflymapdata::sdpro::LinkInfo_Link* link,
    const ad_common::sdpromap::SDProMap& sdpro_map, double distance_on_link,
    double max_search_distance) {
  const iflymapdata::sdpro::LinkInfo_Link* current_link = link;
  double search_distance = 0.0 - distance_on_link;

  while (current_link != nullptr) {
    if (search_distance > max_search_distance) {
      return false;
    }

    const double current_link_length =
        static_cast<double>(current_link->length());

    std::vector<iflymapdata::sdpro::FeaturePoint> fp_vec;
    for (const auto& fp : current_link->feature_points()) {
      fp_vec.emplace_back(fp);
    }

    // 2、按照距离排序后，由近向远判断当前link上的fp是否有TOLL_BOOTH_ENTRANCE
    std::sort(fp_vec.begin(), fp_vec.end(),
              [](const iflymapdata::sdpro::FeaturePoint& fp_a,
                 const iflymapdata::sdpro::FeaturePoint& fp_b) {
                return fp_a.projection_percent() < fp_b.projection_percent();
              });

    const int fp_point_size = fp_vec.size();
    for (int i = 0; i < fp_point_size; i++) {
      const auto& fp_point = fp_vec[i];

      const double distance_to_this_point =
          search_distance +
          current_link_length * fp_point.projection_percent() * 0.01;

      if (distance_to_this_point > max_search_distance) {
        return false;
      } else if (distance_to_this_point < 0.0) {
        // 说明当前fp点在自车之前，直接跳过
        continue;
      }

      for (const auto fp_point_type : fp_point.type()) {
        if (fp_point_type ==
            iflymapdata::sdpro::FeaturePointType::TOLL_BOOTH_ENTRANCE) {
          return true;
        }
      }
    }
    search_distance += current_link_length * 0.01;
    current_link = sdpro_map.GetNextLinkOnRoute(current_link->id());
  }

  return false;
}
bool RouteInfo::CalculateLSLDistance(
    const iflymapdata::sdpro::LinkInfo_Link* link, double distance_on_link,
    double max_search_distance,
    std::vector<std::vector<LSLInfo>>& lane_lsl_length) {
  const iflymapdata::sdpro::LinkInfo_Link* current_link = link;
  double search_distance = 0.0 - distance_on_link;
  double distance_to_first_marking_change_point = 0.0;
  double distance_to_second_marking_change_point = 0.0;
  bool is_found_marking_change_point = false;

  if (link == nullptr) {
    return false;
  }
  // 1. 用上一个fp来判断当前的虚实线状态
  iflymapdata::sdpro::FeaturePoint last_fp;
  std::vector<LSLInfo> last_fp_lsl_info_vec;
  if (CalculateLastFPInCurrentLink(&last_fp, current_link_, distance_on_link)) {
    if (CalculateFpLSLInfo(last_fp, last_fp_lsl_info_vec)) {
      for (auto& lsl_info : last_fp_lsl_info_vec) {
        lsl_info.lsl_start_distance = 0.0;
      }
      lane_lsl_length.emplace_back(last_fp_lsl_info_vec);
    }
  }

  // 2. 遍历当前link上的fp，判断虚实变化并记录
  while (current_link != nullptr) {
    if (search_distance > max_search_distance) {
      if (!lane_lsl_length.empty()) {
        for (auto& lsl_info : lane_lsl_length.back()) {
          lsl_info.lsl_end_distance = max_search_distance;
        }
      }
      break;
    }

    const double current_link_length =
        static_cast<double>(current_link->length());

    std::vector<iflymapdata::sdpro::FeaturePoint> fp_vec;
    for (const auto& fp : current_link->feature_points()) {
      fp_vec.emplace_back(fp);
    }

    // 3. 按照距离排序后，由近向远判断当前link上的fp的LSL属性
    std::sort(fp_vec.begin(), fp_vec.end(),
              [](const iflymapdata::sdpro::FeaturePoint& fp_a,
                 const iflymapdata::sdpro::FeaturePoint& fp_b) {
                return fp_a.projection_percent() < fp_b.projection_percent();
              });

    const int fp_point_size = fp_vec.size();
    for (int i = 0; i < fp_point_size; i++) {
      const auto& fp_point = fp_vec[i];

      const double distance_to_this_point =
          search_distance +
          current_link_length * fp_point.projection_percent() * 0.01;

      if (distance_to_this_point > max_search_distance) {
        if (!lane_lsl_length.empty()) {
          for (auto& lsl_info : lane_lsl_length.back()) {
            lsl_info.lsl_end_distance = max_search_distance;
          }
        }
        break;
      } else if (distance_to_this_point < 0.0) {
        // 说明当前fp点在自车之前，直接跳过
        continue;
      }
      std::vector<LSLInfo> lsl_info_vec;
      if (CalculateFpLSLInfo(fp_point, lsl_info_vec)) {
        if (!lane_lsl_length.empty()) {
          for (int n = 0;
               n < std::min(lsl_info_vec.size(), lane_lsl_length.back().size());
               n++) {
            if (lane_lsl_length.back()[n].is_right_lsl !=
                lsl_info_vec[n].is_right_lsl) {
              lane_lsl_length.back()[n].lsl_end_distance =
                  distance_to_this_point;
              lsl_info_vec[n].lsl_start_distance = distance_to_this_point;
              is_found_marking_change_point = true;
            }
          }
        } else {
          for (auto& lsl_info : lsl_info_vec) {
            lsl_info.lsl_start_distance = distance_to_this_point;
          }
          lane_lsl_length.emplace_back(lsl_info_vec);
        }
        if (is_found_marking_change_point) {
          lane_lsl_length.emplace_back(lsl_info_vec);
          is_found_marking_change_point = false;
        }
      }
    }
    search_distance += current_link_length * 0.01;
    current_link = sdpro_map_.GetNextLinkOnRoute(current_link->id());
  }
  if (!lane_lsl_length.empty()) {
    return true;
  } else {
    return false;
  }
}
bool RouteInfo::CalculateFpLSLInfo(const iflymapdata::sdpro::FeaturePoint& fp,
                                   std::vector<LSLInfo>& fp_lsl_info) {
  const auto& lane_ids = fp.lane_ids();
  for (size_t i = 0; i < lane_ids.size() - 1; ++i) {
    const auto& lane_id = lane_ids[i];
    const auto& lane_info = sdpro_map_.GetLaneInfoByID(lane_id);
    if (lane_info == nullptr) {
      continue;
    }
    int lane_seq = 0;
    for (const auto& lane_id_temp : fp.lane_ids()) {
      if (!IsEmergencyLane(lane_id_temp, sdpro_map_)) {
        lane_seq++;
      }
      if (lane_id_temp == lane_id) {
        break;
      }
    }
    LSLInfo lane_seq_lsl_info;
    lane_seq_lsl_info.lane_seq = lane_seq;
    if (lane_info->lsl_type() ==
            iflymapdata::sdpro::Lane_LslType::Lane_LslType_LSL_RIGHT ||
        lane_info->lsl_type() ==
            iflymapdata::sdpro::Lane_LslType::Lane_LslType_LSL_BOTH) {
      lane_seq_lsl_info.is_right_lsl = true;
    }
    fp_lsl_info.emplace_back(lane_seq_lsl_info);
  }
  if (!fp_lsl_info.empty()) {
    return true;
  } else {
    return false;
  }
}
double RouteInfo::CalculateLSLTotalLength(
    int lane_seq, const std::vector<std::vector<LSLInfo>>& lane_lsl_length,
    bool direction) {
  std::map<int, double> total_length;

  if (lane_lsl_length.empty()) {
    return 0.0;
  }
  for (const auto& lsl_info : lane_lsl_length) {
    // 匹配指定的lane_seq和方向
    if (direction == SPLIT_LEFT) {
      // 最左侧车道不用计算
      if (lane_seq == 1) {
        continue;
      }
      for (int i = std::min(lane_seq, static_cast<int>(lsl_info.size())) - 1;
           i > 0; i--) {
        if (lsl_info[i].is_right_lsl) {
          double lsl_length =
              lsl_info[i].lsl_end_distance - lsl_info[i].lsl_start_distance;
          total_length[i] += lsl_length;
        }
      }
    } else {
      // 最右侧车道不用计算
      if (lane_seq == lsl_info.size()) {
        continue;
      }
      for (int i = lane_seq; i < lsl_info.size(); i++) {
        if (lsl_info[i].is_right_lsl) {
          double lsl_length =
              lsl_info[i].lsl_end_distance - lsl_info[i].lsl_start_distance;
          total_length[i] += lsl_length;
        }
      }
    }
  }

  double max_lsl_length = 0.0;
  for (const auto& length : total_length) {
    max_lsl_length = std::max(max_lsl_length, length.second);
  }

  return max_lsl_length;
}
bool RouteInfo::CalculateExpandLaneInfo(
    std::map<int, SplitDirection>& expand_lane_sequence_vec,
    const iflymapdata::sdpro::LinkInfo_Link* link, double distance_on_link,
    double max_search_distance) {
  const iflymapdata::sdpro::LinkInfo_Link* current_link = link;
  double search_distance = 0.0 - distance_on_link;

  while (current_link != nullptr) {
    if (search_distance > max_search_distance) {
      return false;
    }

    const double current_link_length =
        static_cast<double>(current_link->length());

    std::vector<iflymapdata::sdpro::FeaturePoint> fp_vec;
    for (const auto& fp : current_link->feature_points()) {
      fp_vec.emplace_back(fp);
    }

    // 2、按照距离排序后，由近向远判断当前link上的fp是否有TurnExpandingLane
    std::sort(fp_vec.begin(), fp_vec.end(),
              [](const iflymapdata::sdpro::FeaturePoint& fp_a,
                 const iflymapdata::sdpro::FeaturePoint& fp_b) {
                return fp_a.projection_percent() < fp_b.projection_percent();
              });

    const int fp_point_size = fp_vec.size();
    for (int i = 0; i < fp_point_size; i++) {
      const auto& fp_point = fp_vec[i];

      const double distance_to_this_point =
          search_distance +
          current_link_length * fp_point.projection_percent() * 0.01;

      if (distance_to_this_point > max_search_distance) {
        return false;
      } else if (distance_to_this_point < 0.0) {
        // 说明当前fp点在自车之前，直接跳过
        continue;
      }

      for (const auto fp_point_type : fp_point.type()) {
        if (fp_point_type ==
            iflymapdata::sdpro::FeaturePointType::LANE_COUNT_CHANGE) {
          for (const auto& lane_id : fp_point.lane_ids()) {
            const auto& lane_info = sdpro_map_.GetLaneInfoByID(lane_id);
            if (lane_info == nullptr) {
              continue;
            }

            int find_fp_lane_num = 0;
            for (const auto& lane_id_temp : fp_point.lane_ids()) {
              if (!IsEmergencyLane(lane_id_temp, sdpro_map_)) {
                find_fp_lane_num++;
              }
              if (lane_id_temp == lane_id) {
                break;
              }
            }

            if (lane_info->change_type() ==
                iflymapdata::sdpro::LaneChangeType::LeftTurnExpandingLane) {
              expand_lane_sequence_vec.emplace(1, SPLIT_LEFT);
              return true;
            } else if (lane_info->change_type() ==
                       iflymapdata::sdpro::LaneChangeType::
                           RightTurnExpandingLane) {
              expand_lane_sequence_vec.emplace(find_fp_lane_num + 1,
                                               SPLIT_RIGHT);
              return true;
            } else if (lane_info->change_type() ==
                       iflymapdata::sdpro::LaneChangeType::
                           BothDirectionExpandingLane) {
              expand_lane_sequence_vec.emplace(find_fp_lane_num - 1,
                                               SPLIT_LEFT);
              expand_lane_sequence_vec.emplace(find_fp_lane_num + 1,
                                               SPLIT_RIGHT);
              return true;
            }
          }
        }
      }
    }
    search_distance += current_link_length * 0.01;
    current_link = sdpro_map_.GetNextLinkOnRoute(current_link->id());
  }
  return false;
}
void RouteInfo::OptimizeFeasibleLanesForRampSplit(
    NOASplitRegionInfo& first_exchange_region_info,
    NOASplitRegionInfo& ramp_exchange_region_info) {
  // 判断是否需要取消躲避分汇流
  bool is_need_cancel_avoide_mlc = false;
  if (first_exchange_region_info.avoide_lane_num > 0) {
    bool is_two_exchange_close =
        ramp_exchange_region_info.distance_to_split_point -
            first_exchange_region_info.distance_to_split_point +
            ramp_exchange_region_info.start_fp_point
                .fp_distance_to_split_point -
            first_exchange_region_info.end_fp_point.fp_distance_to_split_point <
        500.0;

    bool is_need_cancel_avoide_mlc =
        ramp_exchange_region_info.split_direction == SPLIT_RIGHT
            ? ramp_exchange_region_info.recommend_lane_num[0]
                          .feasible_lane_sequence.front() -
                      first_exchange_region_info.recommend_lane_num[0]
                          .feasible_lane_sequence.back() >
                  ramp_exchange_region_info.recommend_lane_num[0]
                          .feasible_lane_sequence.front() -
                      first_exchange_region_info.avoide_lane_num
            : ramp_exchange_region_info.recommend_lane_num[0]
                          .feasible_lane_sequence.back() -
                      first_exchange_region_info.recommend_lane_num[0]
                          .feasible_lane_sequence.front() <
                  ramp_exchange_region_info.recommend_lane_num[0]
                          .feasible_lane_sequence.front() -
                      first_exchange_region_info.avoide_lane_num;
    if (is_two_exchange_close && is_need_cancel_avoide_mlc) {
      for (int i = 0; i < 3; i++) {
        auto& feasible_sequence =
            first_exchange_region_info.recommend_lane_num[i]
                .feasible_lane_sequence;
        int lane_num = first_exchange_region_info.avoide_lane_num;
        auto it = std::lower_bound(feasible_sequence.begin(),
                                   feasible_sequence.end(), lane_num);
        if (it == feasible_sequence.end() || *it != lane_num) {
          feasible_sequence.insert(it, lane_num);
        }
      }
    }
  }

  // 计算两个交换区共同的feasible lane
  for (int i = 0; i < 3; i++) {
    auto& first_feasible_sequence =
        first_exchange_region_info.recommend_lane_num[i].feasible_lane_sequence;
    auto& ramp_feasible_sequence =
        ramp_exchange_region_info.recommend_lane_num[0].feasible_lane_sequence;
    std::vector<int> common_feasible_lane =
        CommonElements(first_feasible_sequence, ramp_feasible_sequence);
    if (!common_feasible_lane.empty()) {
      first_feasible_sequence = common_feasible_lane;
    } else {
      std::vector<int> temp_feasible_lane;
      if (ramp_exchange_region_info.split_direction == SPLIT_RIGHT) {
        temp_feasible_lane.push_back(first_feasible_sequence.back());
      } else {
        temp_feasible_lane.push_back(first_feasible_sequence.front());
      }
      first_feasible_sequence = temp_feasible_lane;
    }
  }
}
}  // namespace planning
