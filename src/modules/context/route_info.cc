#include "route_info.h"
#include <memory>
#include "config/basic_type.h"
#include "environmental_model.h"

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
}

void RouteInfo::Update() {
  route_info_output_.reset();

  local_view_ = session_->environmental_model().get_local_view();
  if (local_view_.localization.status.status_info.mode ==
      iflyauto::IFLYStatusInfoMode::IFLY_STATUS_INFO_MODE_ERROR) {
    std::cout << "localization invalid" << std::endl;
    return;
  }

  bool is_hpp_scene = session_->get_scene_type() == common::HPP;
  if (!is_hpp_scene) {
    if (UpdateSdMap(local_view_)) {
      UpdateRouteInfoForNOA(sd_map_);
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
  UpdateVisionInfo();
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

void RouteInfo::UpdateRouteInfoForHPP(const ad_common::hdmap::HDMap& hd_map) {
  // if (!GetCurrentNearestLane()) {
  //   std::cout << "GetCurrentNearestLane failed!!!" << std::endl;
  //     return;
  // }
  // void CalculateHPPInfo();
  // void CalculateDistanceToTargetSlot();
  // void CalculateDistanceToNextSpeedBump();
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
  LOG_DEBUG("current_segment_passed_distance:%f\n",
            route_info_output_.current_segment_passed_distance);
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

  route_info_output_.is_in_sdmaproad = true;
  segment = current_segment;
  *nearest_s = temp_nearest_s;
  route_info_output_.is_on_highway =
      current_segment->priority() == SdMapSwtx::RoadPriority::EXPRESSWAY;
  route_info_output_.is_on_ramp =
      current_segment->usage() == SdMapSwtx::RoadUsage::RAMP;
  route_info_output_.cur_seg_forward_lane_num =
      current_segment->forward_lane_num();
  route_info_output_.is_update_segment_success = true;
  return segment;
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
      route_info_output_.distance_to_first_road_merge < 100) {
    route_info_output_.is_leaving_ramp = true;
  }

  //判断是否是正在接近匝道
  const double dis_between_first_road_split_and_ramp =
      route_info_output_.distance_to_first_road_split -
      route_info_output_.dis_to_ramp;
  const double allow_error = 5.0;
  route_info_output_.is_nearing_ramp =
      fabs(dis_between_first_road_split_and_ramp) < allow_error &&
      route_info_output_.dis_to_ramp < 3000.;

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
  std::array<double, 2> fp{0, 100};
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

// for HPP function
// bool RouteInfo::GetCurrentNearestLane() {
//   ad_common::math::Vec2d point;
//   const auto& ego_state =
//       session_->environmental_model().get_ego_state_manager();
//   current_pose_ = ego_state->ego_pose_raw();
//   point.set_x(current_pose_.x);
//   point.set_y(current_pose_.x);
//   // get nearest lane
//   ad_common::hdmap::LaneInfoConstPtr nearest_lane;
//   double nearest_s = 0.0;
//   double nearest_l = 0.0;
//   // const double distance = 10.0;
//   // const double central_heading = pose.heading();
//   // const double max_heading_difference = PI / 4;
//   if (hd_map_.GetNearestLane(point, &nearest_lane, &nearest_s, &nearest_l) !=
//       0) {
//     std::cout << "no get nearest lane!!!" << std::endl;
//     return false;
//   }
//   // const int res = hd_map.GetNearestLaneWithHeading(
//   //     point, distance, central_heading, max_heading_difference,
//   //     &nearest_lane, &nearest_s, &nearest_l);
//   // std::cout << "find current lane to current ego point dis:"
//   //           << nearest_lane->DistanceTo(point) << std::endl;
//   std::cout << "find the nearest lane!!!"
//             << "nearest_s_:" << nearest_s
//             << ",nearest lane group id:" << nearest_lane->lane_group_id()
//             << std::endl;
//   nearest_lane_hpp_ = nearest_lane;
//   nearest_s_hpp_ = nearest_s;
//   return true;
// }

// void RouteInfo::CalculateHPPInfo() {
//   ConstructBox();
//   // if on hpp lane
//   if (nearest_lane_hpp_->IsOnLane(ego_box_hpp_)) {
//     std::cout << "is on hpp lane!" << std::endl;
//     route_info_output_.is_on_hpp_lane = true;
//     const auto trace_start = local_view_->parking_map_info.trace_start();
//     const ad_common::math::Vec2d trace_start_point_2d =
//     {trace_start.enu().x(),
//                                                          trace_start.enu().x()};
//     // get trace_start point projection s
//     double trace_start_point_accumulate_s;
//     double trace_start_point_lateral;
//     if (nearest_lane_hpp_->GetProjection(trace_start_point_2d,
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
//         nearest_s_hpp_ >= trace_start_point_accumulate_s;
//     const ad_common::math::Vec2d point(current_pose_.x, current_pose_.y);
//     if (is_reached_trace_start_point) {
//       std::cout << "reached trace start point!!" << std::endl;
//       route_info_output_.is_reached_hpp_start_point = true;
//       if (last_point_hpp_.x() != NL_NMAX && last_point_hpp_.y() != NL_NMAX) {
//         sum_distance_driving_ += point.DistanceTo(last_point_hpp_);
//       } else {
//         sum_distance_driving_ = 0;
//       }
//       last_point_hpp_ = point;
//     } else {
//       std::cout << "cur point s less than trace start s" << std::endl;
//     }
//   } else {
//     std::cout << "not in hpp lane!!!" << std::endl;
//     ResetHpp();
//   }
// }

// void RouteInfo::ConstructBox() {
//   // ego box
//   double ego_pose_x = current_pose_.x;
//   double ego_pose_y = current_pose_.y;
//   double yaw = current_pose_.theta;
//   const auto& vehicle_param =
//       VehicleConfigurationContext::Instance()->get_vehicle_param();
//   const auto center_x =
//       ego_pose_x + std::cos(yaw) * vehicle_param.rear_axle_to_center;
//   const auto center_y =
//       ego_pose_y + std::sin(yaw) * vehicle_param.rear_axle_to_center;
//   ad_common::math::Box2d ego_box(
//       {center_x, center_y}, yaw, vehicle_param.length, vehicle_param.width);
//   ego_box_hpp_ = ego_box;
// }

// void RouteInfo::ResetHpp() {
//   sum_distance_driving_ = -1;
//   last_point_hpp_.set_x(NL_NMAX);
//   last_point_hpp_.set_y(NL_NMAX);
// }

// void RouteInfo::CalculateDistanceToTargetSlot() {
//   // get target slot projection point on line
//   ad_common::hdmap::LaneInfoConstPtr tar_slot_nearest_lane;
//   double tar_slot_nearest_s = 0.0;
//   double tar_slot_nearest_l = 0.0;

//   const auto& lines = local_view_.static_map_info.road_map().lanes();
//   if (!lines.empty()) {
//     const auto& final_point = lines[0].points_on_central_line().rbegin();
//     const double tar_slot_pose_x = final_point->x();
//     const double tar_slot_pose_y = final_point->y();
//     const int tar_slot_res = hd_map_.GetNearestLane(
//         {tar_slot_pose_x, tar_slot_pose_y}, &tar_slot_nearest_lane,
//         &tar_slot_nearest_s, &tar_slot_nearest_l);
//     if (tar_slot_res != 0) {
//       std::cout << "not get target slot projection point on line!!!"
//                 << std::endl;
//       return;
//     }
//     route_info_output_.distance_to_target_slot = tar_slot_nearest_s -
//     nearest_s_hpp_;
//   } else {
//     std::cout << "lines is empty from road_map!!!" << std::endl;
//   }
// }

// void RouteInfo::CalculateDistanceToNextSpeedBump() {
//   const auto& road_marks =
//   local_view_.parking_map_info.road_tile_info().road_mark();
//   double distance_to_speed_bump_tmp = 0;
//   for (auto& road_mark : road_marks) {
//     if (road_mark.type() == IFLYParkingMap::RoadMark::SPEED_BUMP &&
//         road_mark.shape_size() == 4) {
//       ad_common::hdmap::LaneInfoConstPtr speed_bump_nearest_lane;
//       double speed_bump_nearest_s = 0.0;
//       double speed_bump_nearest_l = 0.0;

//       ad_common::math::Vec2d speed_bump_center_point(
//           (road_mark.shape(0).boot().x() + road_mark.shape(3).boot().x()) *
//           0.5, (road_mark.shape(0).boot().y() +
//           road_mark.shape(3).boot().y()) * 0.5);
//       const int speed_bump_res = hd_map_.GetNearestLane(
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
//       distance_to_speed_bump_tmp = speed_bump_nearest_s - nearest_s_hpp_;
//       if (distance_to_speed_bump_tmp > 0) {  // TODO: 假设挡位为前进档
//         route_info_output_.distance_to_next_speed_bump =
//         distance_to_speed_bump_tmp; break;
//       }
//     }
//   }
// }
void RouteInfo::UpdateVisionInfo() {
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
}
}  // namespace planning