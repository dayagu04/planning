#include "sdmap.h"

#include <cstddef>
#include <optional>

#include "ehr_sdmap.pb.h"
#include "math/aaboxkdtree2d.h"
#include "math/math_utils.h"

namespace ad_common {
namespace sdmap {
using ad_common::math::Vec2d;
int SDMap::LoadMapFromProto(const ::SdMapSwtx::SdMap &map_proto) {
  route_segms_.clear();
  seg_id_to_index_.clear();
  if (map_proto.has_route_map() && map_proto.route_map().segms_size() > 0) {
    for (const auto &seg : map_proto.route_map().segms()) {
      route_segms_.emplace_back(seg);
      seg_id_to_index_[seg.id()] = route_segms_.size() - 1;
    }
  }
  BuildSegmentKdTree();
  if (map_proto.has_route_map() && map_proto.route_map().has_navi_road_info()) {
    navi_road_info_ = map_proto.route_map().navi_road_info();
  } else {
    navi_road_info_ = std::nullopt;
  }
  return 0;
}

const SdMapSwtx::Segment *SDMap::GetRoadSegmentById(
    uint64_t road_seg_id) const {
  auto idx = GetIndexBySegmentId(road_seg_id);
  if (idx.has_value()) {
    return &route_segms_[idx.value()].SegInfo();
  }
  return nullptr;
}

const SdMapSwtx::Segment *SDMap::GetPreviousRoadSegment(
    uint64_t road_seg_id) const {
  auto idx_opt = GetIndexBySegmentId(road_seg_id);
  if (idx_opt.has_value()) {
    size_t idx = idx_opt.value();
    if (idx > 0) {
      return &route_segms_[idx - 1].SegInfo();
    }
  }
  return nullptr;
}

const SdMapSwtx::Segment *SDMap::GetNextRoadSegment(
    uint64_t road_seg_id) const {
  auto idx_opt = GetIndexBySegmentId(road_seg_id);
  if (idx_opt.has_value()) {
    size_t idx = idx_opt.value();
    if (idx < route_segms_.size() - 1) {
      return &route_segms_[idx + 1].SegInfo();
    }
  }
  return nullptr;
}

/*
const ::SdMapSwtx::Segment *SDMap::GetNearestRoad(const Vec2d &p,
                                                  double &nearest_s,
                                                  double &nearest_l) const {
  if (!kdtree_) {
    return nullptr;
  }
  const auto *obj = kdtree_->GetNearestObject(p);
  if (!obj) {
    return nullptr;
  }

  const auto idx = obj->id();
  if (idx < obj->object()->LineSegments().size() &&
      idx < obj->object()->AccumulatedS().size()) {
    const auto &point_segment = obj->object()->LineSegments()[idx];
    Vec2d nearest_pt;
    point_segment.DistanceTo(p, &nearest_pt);
    nearest_s = obj->object()->AccumulatedS()[idx] +
                nearest_pt.DistanceTo(point_segment.start());
    nearest_l =
        point_segment.unit_direction().CrossProd(p - point_segment.start());
  }

  return &obj->object()->SegInfo();
}
*/

/*
std::pair<const ::SdMapSwtx::Segment *, double> SDMap::GetRampInfo(
    const ad_common::math::Vec2d &p) const {
  const ::SdMapSwtx::Segment *ramp_segment = nullptr;
  double dis_to_ramp = 0;
  double nearest_s = 0.0, nearest_l = 0.0;
  const auto *current_road = GetNearestRoad(p, nearest_s, nearest_l);
  if (current_road) {
    if (current_road->usage() == SdMapSwtx::RoadUsage::RAMP) {
      ramp_segment = current_road;
      dis_to_ramp = 0;
      return std::make_pair(ramp_segment, dis_to_ramp);
    }
    auto idx = GetIndexBySegmentId(current_road->id());
    if (idx.has_value()) {
      size_t i = idx.value();
      size_t ramp_idx = route_segms_.size();
      for (; i < route_segms_.size(); ++i) {
        if (route_segms_[i].SegInfo().usage() == SdMapSwtx::RoadUsage::RAMP) {
          ramp_idx = i;
          break;
        }
      }
      if (ramp_idx < route_segms_.size()) {
        ramp_segment = &route_segms_[ramp_idx].SegInfo();
        for (size_t j = idx.value() + 1; j < ramp_idx; ++j) {
          dis_to_ramp += route_segms_[j].SegInfo().dis();
        }
        double current_seg_remain_dis =
            route_segms_[idx.value()].SegInfo().dis() - nearest_s;
        dis_to_ramp += current_seg_remain_dis;
      }
    }
  }
  return std::make_pair(ramp_segment, dis_to_ramp);
}
*/

std::pair<const ::SdMapSwtx::Segment *, double> SDMap::GetRampInfo(
    uint64_t road_seg_id, double accumulated_s, double max_distance) const {
  const ::SdMapSwtx::Segment *ramp_segment = nullptr;
  double dis_to_ramp = 0;
  const auto seginfo_idx_opt = GetIndexBySegmentId(road_seg_id);
  if (seginfo_idx_opt.has_value()) {
    size_t current_seginfo_idx = seginfo_idx_opt.value();
    const auto &current_seg = route_segms_[current_seginfo_idx].SegInfo();
    // 如果已经在匝道上，返回当前Segment，到匝道的距离设为0
    if (current_seg.usage() == SdMapSwtx::RoadUsage::RAMP) {
      ramp_segment = &current_seg;
      return std::make_pair(ramp_segment, 0);
    }

    double current_seg_remain_dis = (accumulated_s > current_seg.dis())
                                        ? 0
                                        : current_seg.dis() - accumulated_s;
    dis_to_ramp += current_seg_remain_dis;

    // search ramp
    size_t ramp_seginfo_idx = route_segms_.size();
    for (size_t i = current_seginfo_idx + 1; i < route_segms_.size(); ++i) {
      if (dis_to_ramp > max_distance) {
        break;
      }
      if (route_segms_[i].SegInfo().usage() == SdMapSwtx::RoadUsage::RAMP) {
        ramp_seginfo_idx = i;
        break;
      }
      dis_to_ramp += route_segms_[i].SegInfo().dis();
    }
    // 如果在距离范围内搜索到了匝道
    if (ramp_seginfo_idx < route_segms_.size()) {
      ramp_segment = &route_segms_[ramp_seginfo_idx].SegInfo();
    }
  }

  return std::make_pair(ramp_segment,
                        (ramp_segment != nullptr) ? dis_to_ramp : 0);
}

std::pair<const ::SdMapSwtx::Segment *, double> SDMap::GetMainRoadMergeInfo(
    uint64_t road_seg_id, double accumulated_s, double max_distance) const {
  const ::SdMapSwtx::Segment *main_road_segment = nullptr;
  double dis_to_main = 0.0;
  const auto seginfo_idx_opt = GetIndexBySegmentId(road_seg_id);
  if (seginfo_idx_opt.has_value()) {
    size_t current_seginfo_idx = seginfo_idx_opt.value();
    const auto &current_seg = route_segms_[current_seginfo_idx].SegInfo();

    double current_seg_remain_dis = (accumulated_s > current_seg.dis())
                                        ? 0
                                        : current_seg.dis() - accumulated_s;
    dis_to_main += current_seg_remain_dis;
    auto previous_road_usage = current_seg.usage();
    for (size_t i = current_seginfo_idx + 1; i < route_segms_.size(); ++i) {
      if (dis_to_main > max_distance) {
        break;
      }
      if (previous_road_usage == SdMapSwtx::RoadUsage::RAMP &&
          route_segms_[i].SegInfo().usage() != SdMapSwtx::RoadUsage::RAMP) {
        main_road_segment = &route_segms_[i - 1].SegInfo();
        break;
      }
      previous_road_usage = route_segms_[i].SegInfo().usage();
      dis_to_main += route_segms_[i].SegInfo().dis();
    }
  }
  return std::make_pair(main_road_segment,
                        (main_road_segment) ? dis_to_main : 0);
}

std::pair<const ::SdMapSwtx::Segment *, double> SDMap::GetTunnelInfo(
    uint64_t road_seg_id, double accumulated_s, double max_distance) const {
  const ::SdMapSwtx::Segment *tunnel_segment = nullptr;
  double dis_to_tunnel = 0.0;
  const auto seginfo_idx_opt = GetIndexBySegmentId(road_seg_id);
  if (seginfo_idx_opt.has_value()) {
    size_t current_seginfo_idx = seginfo_idx_opt.value();
    const auto &current_seg = route_segms_[current_seginfo_idx].SegInfo();
    if (current_seg.is_tunnel() == true) {
      tunnel_segment = &current_seg;
    } else {
      double current_seg_remain_dis = (accumulated_s > current_seg.dis())
                                          ? 0
                                          : current_seg.dis() - accumulated_s;
      dis_to_tunnel += current_seg_remain_dis;
      for (size_t i = current_seginfo_idx + 1; i < route_segms_.size(); ++i) {
        if (dis_to_tunnel > max_distance) {
          break;
        }
        if (route_segms_[i].SegInfo().is_tunnel()) {
          tunnel_segment = &route_segms_[i].SegInfo();
          break;
        }
        dis_to_tunnel += route_segms_[i].SegInfo().dis();
      }
    }
  }
  return std::make_pair(tunnel_segment, (tunnel_segment) ? dis_to_tunnel : 0);
}

std::pair<const ::SdMapSwtx::Segment *, double> SDMap::GetTollStationInfo(
    uint64_t road_seg_id, double accumulated_s, double max_distance) const {
  const ::SdMapSwtx::Segment *toll_station_segment = nullptr;
  double dis_to_toll_station = 0.0;
  const auto seginfo_idx_opt = GetIndexBySegmentId(road_seg_id);
  if (seginfo_idx_opt.has_value()) {
    size_t current_seginfo_idx = seginfo_idx_opt.value();
    const auto &current_seg = route_segms_[current_seginfo_idx].SegInfo();
    double current_seg_remain_dis = (accumulated_s > current_seg.dis())
                                        ? 0
                                        : current_seg.dis() - accumulated_s;
    if (current_seg.has_toll_station()) {
      toll_station_segment = &current_seg;
      dis_to_toll_station = current_seg_remain_dis;
    } else {
      dis_to_toll_station += current_seg_remain_dis;
      for (size_t i = current_seginfo_idx + 1; i < route_segms_.size(); ++i) {
        dis_to_toll_station += route_segms_[i].SegInfo().dis();
        if (dis_to_toll_station > max_distance) {
          break;
        }
        if (route_segms_[i].SegInfo().has_toll_station()) {
          toll_station_segment = &route_segms_[i].SegInfo();
          break;
        }
      }
    }
  }
  return std::make_pair(toll_station_segment,
                        toll_station_segment ? dis_to_toll_station : 0.0);
}

int SDMap::GetDistanceToRouteEnd(uint64_t road_seg_id, double accumulated_s,
                                 double &dis_to_end) const {
  const auto idx_opt = GetIndexBySegmentId(road_seg_id);
  if (!idx_opt.has_value()) {
    return -1;
  }
  const auto &cur_seg = route_segms_[idx_opt.value()].SegInfo();
  dis_to_end =
      (accumulated_s > cur_seg.dis()) ? 0 : cur_seg.dis() - accumulated_s;
  for (size_t i = idx_opt.value() + 1; i < route_segms_.size(); ++i) {
    dis_to_end += route_segms_[i].SegInfo().dis();
  }
  return 0;
}

/*
std::vector<std::pair<const ::SdMapSwtx::Segment *, double>>
SDMap::GetMergeInfoList(const ad_common::math::Vec2d &p) const {
  double dis_s = 0;
  double dis_l = 0;
  auto *current_road = GetNearestRoad(p, dis_s, dis_l);
  if (current_road == nullptr) {
    return {};
  }
  double remain_dis = current_road->dis() - dis_s;
  std::vector<std::pair<const ::SdMapSwtx::Segment *, double>> merge_list{};
  if (current_road->in_link_size() > 1) {
    merge_list.emplace_back(std::make_pair(current_road, -dis_s));
  }

  auto current_idx_opt = GetIndexBySegmentId(current_road->id());
  double accumulate_dis = remain_dis;
  if (current_idx_opt.has_value()) {
    size_t current_road_idx = current_idx_opt.value();
    for (size_t i = current_road_idx + 1; i < route_segms_.size(); ++i) {
      if (route_segms_[i].SegInfo().in_link_size() > 1) {
        merge_list.emplace_back(
            std::make_pair(&route_segms_[i].SegInfo(), accumulate_dis));
      }
      accumulate_dis += route_segms_[i].SegInfo().dis();
    }
  }
  return merge_list;
}
*/

std::vector<std::pair<const ::SdMapSwtx::Segment *, double>>
SDMap::GetMergeInfoList(uint64_t road_seg_id, double accumulated_s,
                        double max_distance) const {
  const auto seginfo_idx_opt = GetIndexBySegmentId(road_seg_id);
  if (!seginfo_idx_opt.has_value()) {
    return {};
  }
  size_t cur_seginfo_idx = seginfo_idx_opt.value();
  const auto &cur_seg = route_segms_[cur_seginfo_idx].SegInfo();
  double cur_seg_remain_dis =
      (accumulated_s > cur_seg.dis()) ? 0 : (cur_seg.dis() - accumulated_s);
  double dis_to_merge = cur_seg_remain_dis;
  std::vector<std::pair<const ::SdMapSwtx::Segment *, double>> merge_list{};

  // 如果当前所在的路段是汇流点，距离设为负数
  if (cur_seg.in_link_size() > 1) {
    merge_list.emplace_back(std::make_pair(&cur_seg, -accumulated_s));
  }

  // search merge
  for (auto i = cur_seginfo_idx + 1; i < route_segms_.size(); ++i) {
    if (dis_to_merge > max_distance) {
      break;
    }
    if (route_segms_[i].SegInfo().in_link_size() > 1) {
      merge_list.emplace_back(
          std::make_pair(&route_segms_[i].SegInfo(), dis_to_merge));
    }
    dis_to_merge += route_segms_[i].SegInfo().dis();
  }

  return merge_list;
}

/*
std::vector<std::pair<const ::SdMapSwtx::Segment *, double>>
SDMap::GetSplitInfoList(const ad_common::math::Vec2d &p) const {
  double dis_s = 0;
  double dis_l = 0;
  auto *current_road = GetNearestRoad(p, dis_s, dis_l);
  if (current_road == nullptr) {
    return {};
  }
  double remain_dis = current_road->dis() - dis_s;
  std::vector<std::pair<const ::SdMapSwtx::Segment *, double>> split_list{};
  if (current_road->out_link_size() > 1) {
    split_list.emplace_back(std::make_pair(current_road, remain_dis));
  }
  double accumulate_dis = remain_dis;
  auto current_idx_opt = GetIndexBySegmentId(current_road->id());
  if (current_idx_opt.has_value()) {
    size_t current_road_idx = current_idx_opt.value();
    for (size_t i = current_road_idx + 1; i < route_segms_.size(); ++i) {
      accumulate_dis += route_segms_[i].SegInfo().dis();
      if (route_segms_[i].SegInfo().out_link_size() > 1) {
        split_list.emplace_back(
            std::make_pair(&route_segms_[i].SegInfo(), accumulate_dis));
      }
    }
  }

  return split_list;
}
*/

std::vector<std::pair<const ::SdMapSwtx::Segment *, double>>
SDMap::GetSplitInfoList(uint64_t road_seg_id, double accumulated_s,
                        double max_distance) const {
  const auto seginfo_idx_opt = GetIndexBySegmentId(road_seg_id);
  if (!seginfo_idx_opt.has_value()) {
    return {};
  }

  size_t cur_seginfo_idx = seginfo_idx_opt.value();
  const auto &cur_seg = route_segms_[cur_seginfo_idx].SegInfo();
  double cur_seg_remain_dis =
      (accumulated_s > cur_seg.dis()) ? 0 : cur_seg.dis() - accumulated_s;
  double dis_to_split = cur_seg_remain_dis;

  std::vector<std::pair<const ::SdMapSwtx::Segment *, double>> split_list{};
  if (cur_seg.crossing()) {
    split_list.emplace_back(std::make_pair(&cur_seg, dis_to_split));
  }
  // search split
  for (auto i = cur_seginfo_idx + 1; i < route_segms_.size(); ++i) {
    dis_to_split += route_segms_[i].SegInfo().dis();
    if (dis_to_split > max_distance) {
      break;
    }
    if (route_segms_[i].SegInfo().crossing()) {
      split_list.emplace_back(
          std::make_pair(&route_segms_[i].SegInfo(), dis_to_split));
    }
  }

  return split_list;
}

std::vector<std::pair<double, double>> SDMap::GetCurvatureList(
    uint64_t road_seg_id, double accumulated_s, double max_distance) const {
  const auto seginfo_idx_opt = GetIndexBySegmentId(road_seg_id);
  if (!seginfo_idx_opt.has_value()) {
    return {};
  }
  std::vector<std::pair<double, double>> curvature_list{};

  size_t cur_seginfo_idx = seginfo_idx_opt.value();

  const auto &cur_seg = route_segms_[cur_seginfo_idx].SegInfo();
  for (const auto &adas : cur_seg.adas_data()) {
    if (adas.has_curvature() && adas.dis_to_seg_start() >= accumulated_s) {
      double dis_to_current_pos = adas.dis_to_seg_start() - accumulated_s;
      if (dis_to_current_pos > max_distance) {
        break;
      }
      curvature_list.emplace_back(std::make_pair(
          dis_to_current_pos, GetCurvatureInReciprocalM(adas.curvature())));
    }
  }
  double dis_to_seg_end =
      (accumulated_s > cur_seg.dis()) ? 0 : cur_seg.dis() - accumulated_s;
  if (cur_seg.has_transition_adas_data() && dis_to_seg_end <= max_distance) {
    curvature_list.emplace_back(std::make_pair(
        dis_to_seg_end,
        GetCurvatureInReciprocalM(cur_seg.transition_adas_data().curvature())));
  }

  bool reach_max_distance = false;
  for (size_t i = cur_seginfo_idx + 1; i < route_segms_.size(); ++i) {
    const auto &seg = route_segms_[i].SegInfo();
    for (const auto &adas : seg.adas_data()) {
      if (adas.has_curvature()) {
        double dis_to_current_pos = dis_to_seg_end + adas.dis_to_seg_start();
        if (dis_to_current_pos > max_distance) {
          reach_max_distance = true;
          break;
        }
        curvature_list.emplace_back(std::make_pair(
            dis_to_current_pos, GetCurvatureInReciprocalM(adas.curvature())));
      }
    }
    if (reach_max_distance) {
      break;
    }
    dis_to_seg_end += seg.dis();
    if (seg.has_transition_adas_data() && dis_to_seg_end <= max_distance) {
      curvature_list.emplace_back(std::make_pair(
          dis_to_seg_end,
          GetCurvatureInReciprocalM(seg.transition_adas_data().curvature())));
    }
  }
  return curvature_list;
}

void SDMap::BuildSegmentKdTree() {
  ad_common::math::AABoxKDTreeParams params;
  params.max_leaf_dimension = 5.0;  // meters.
  params.max_leaf_size = 16;
  road_segment_boxes_.clear();
  for (const auto &seg : route_segms_) {
    const auto &point_segs = seg.LineSegments();
    for (size_t id = 0; id < point_segs.size(); ++id) {
      road_segment_boxes_.emplace_back(
          ad_common::math::AABox2d{point_segs[id].start(),
                                   point_segs[id].end()},
          &seg, point_segs[id], id);
    }
  }
  kdtree_ = std::make_unique<RoadSegmentKDTree>(road_segment_boxes_, params);
}

MyOptional<size_t> SDMap::GetIndexBySegmentId(uint64_t id) const {
  if (seg_id_to_index_.count(id)) {
    return seg_id_to_index_.at(id);
  }
  return {};
}

std::vector<std::pair<uint64_t, uint64_t>> SDMap::SearchObjects(
    const Vec2d &center, double radius) const {
  auto objects = kdtree_->GetObjects(center, radius);
  // {road_id, inner_seg_idx}
  std::vector<std::pair<uint64_t, uint64_t>> result_ids;
  result_ids.reserve(objects.size());
  for (const auto *obj : objects) {
    result_ids.push_back(
        std::make_pair(obj->object()->SegInfo().id(), obj->id()));
  }
  return result_ids;
}

std::vector<std::pair<uint64_t, uint64_t>> SDMap::GetLanesWithHeading(
    const Vec2d &point, double distance, double central_heading,
    double max_heading_diff) const {
  auto result_ids = SearchObjects(point, distance);
  if (result_ids.empty()) {
    return {};
  }
  std::vector<std::pair<uint64_t, uint64_t>> filtered_ids{};
  for (const auto &id_pair : result_ids) {
    uint64_t road_seg_id = id_pair.first;
    uint64_t inner_linesegment_idx = id_pair.second;
    auto seginfo_idx_opt = GetIndexBySegmentId(road_seg_id);
    if (!seginfo_idx_opt.has_value() ||
        seginfo_idx_opt.value() >= route_segms_.size()) {
      continue;
    }
    const auto &current_seginfo = route_segms_[seginfo_idx_opt.value()];
    if (inner_linesegment_idx >= current_seginfo.LineSegments().size() ||
        inner_linesegment_idx >= current_seginfo.Headings().size()) {
      continue;
    }
    const auto &line_segment =
        current_seginfo.LineSegments()[inner_linesegment_idx];
    Vec2d map_point{};
    double dis = line_segment.DistanceTo(point, &map_point);
    if (dis <= distance) {
      double heading_diff = fabs(
          current_seginfo.Headings()[inner_linesegment_idx] - central_heading);
      if (fabs(ad_common::math::NormalizeAngle(heading_diff)) <=
          max_heading_diff) {
        filtered_ids.push_back(id_pair);
      }
    }
  }
  return filtered_ids;
}

const SdMapSwtx::Segment *SDMap::GetNearestRoadWithHeading(
    const Vec2d &point, double distance, double central_heading,
    double max_heading_diff, double &nearest_s, double &nearest_l) const {
  const SdMapSwtx::Segment *nearest_lane = nullptr;
  auto id_list =
      GetLanesWithHeading(point, distance, central_heading, max_heading_diff);
  if (id_list.empty()) {
    return nearest_lane;
  }
  double min_dis = distance;
  for (const auto &id_pair : id_list) {
    uint64_t road_seg_id = id_pair.first;
    uint64_t inner_linesegment_idx = id_pair.second;
    auto seginfo_idx_opt = GetIndexBySegmentId(road_seg_id);
    if (!seginfo_idx_opt.has_value() ||
        seginfo_idx_opt.value() >= route_segms_.size()) {
      continue;
    }
    const auto &current_seginfo = route_segms_[seginfo_idx_opt.value()];
    if (inner_linesegment_idx >= current_seginfo.LineSegments().size() ||
        inner_linesegment_idx >= current_seginfo.Headings().size() ||
        inner_linesegment_idx >= current_seginfo.AccumulatedS().size()) {
      continue;
    }
    const auto &line_segment =
        current_seginfo.LineSegments()[inner_linesegment_idx];
    Vec2d map_point{};
    double dis = line_segment.DistanceTo(point, &map_point);
    if (dis < min_dis) {
      min_dis = dis;
      nearest_lane = &current_seginfo.SegInfo();
      nearest_s = current_seginfo.AccumulatedS()[inner_linesegment_idx] +
                  line_segment.start().DistanceTo(map_point);
      nearest_l =
          line_segment.unit_direction().CrossProd(point - line_segment.start());
    }
  }
  return nearest_lane;
}

std::vector<std::pair<const ::SdMapSwtx::Segment *, double>>
SDMap::GetTrafficLightCountDownInfoList(uint64_t road_seg_id,
                                        double accumulated_s,
                                        double max_distance) const {
  const auto seginfo_idx_opt = GetIndexBySegmentId(road_seg_id);
  if (!seginfo_idx_opt.has_value()) {
    return {};
  }
  size_t cur_seginfo_idx = seginfo_idx_opt.value();
  const auto &cur_seg = route_segms_[cur_seginfo_idx].SegInfo();
  double cur_seg_remain_dis =
      (accumulated_s > cur_seg.dis()) ? 0 : cur_seg.dis() - accumulated_s;
  double dis_to_traffic_light = cur_seg_remain_dis;
  std::vector<std::pair<const ::SdMapSwtx::Segment *, double>>
      traffic_light_list{};
  if (cur_seg.tfl_count_downs_size() > 0) {
    traffic_light_list.emplace_back(
        std::make_pair(&cur_seg, dis_to_traffic_light));
  }

  // search traffic light
  for (auto i = cur_seginfo_idx + 1; i < route_segms_.size(); ++i) {
    dis_to_traffic_light += route_segms_[i].SegInfo().dis();
    if (dis_to_traffic_light > max_distance) {
      break;
    }
    if (route_segms_[i].SegInfo().tfl_count_downs_size() > 0) {
      traffic_light_list.emplace_back(
          std::make_pair(&route_segms_[i].SegInfo(), dis_to_traffic_light));
    }
  }
  return traffic_light_list;
}

std::vector<std::pair<const ::SdMapSwtx::Segment *, double>>
SDMap::GetCameraInfoList(uint64_t road_seg_id, double accumulated_s,
                         double max_distance) const {
  const auto seginfo_idx_opt = GetIndexBySegmentId(road_seg_id);
  if (!seginfo_idx_opt.has_value()) {
    return {};
  }
  size_t cur_seginfo_idx = seginfo_idx_opt.value();
  const auto &cur_seg = route_segms_[cur_seginfo_idx].SegInfo();
  double cur_seg_remain_dis =
      (accumulated_s > cur_seg.dis()) ? 0 : cur_seg.dis() - accumulated_s;
  double dis_to_camera_seg = cur_seg_remain_dis;
  std::vector<std::pair<const ::SdMapSwtx::Segment *, double>> camera_list{};
  static auto HasCameraInSegment = [](const SdMapSwtx::Segment &seg) {
    if (seg.has_camera_info() && seg.camera_info().navi_cameras_size() > 0) {
      return true;
    }
    return false;
  };
  if (HasCameraInSegment(cur_seg)) {
    camera_list.emplace_back(std::make_pair(&cur_seg, 0.0));
  }
  // search cameras
  for (auto i = cur_seginfo_idx + 1; i < route_segms_.size(); ++i) {
    if (dis_to_camera_seg > max_distance) {
      break;
    }
    if (HasCameraInSegment(route_segms_[i].SegInfo())) {
      camera_list.emplace_back(
          std::make_pair(&route_segms_[i].SegInfo(), dis_to_camera_seg));
    }
    dis_to_camera_seg += route_segms_[i].SegInfo().dis();
  }
  return camera_list;
}

std::optional<SdMapSwtx::NaviRoadInfo> SDMap::GetNaviRoadInfo() const {
  return navi_road_info_;
}

bool IsHighWay(const SdMapSwtx::Segment &segment) {
  return segment.has_priority() &&
         segment.priority() == SdMapSwtx::RoadPriority::EXPRESSWAY;
}
bool IsElevated(const SdMapSwtx::Segment &segment) {
  return segment.has_priority() &&
         segment.priority() == SdMapSwtx::RoadPriority::CITY_EXPRESSWAY;
}

bool IsRamp(const SdMapSwtx::Segment &segment) {
  return segment.has_usage() && segment.usage() == SdMapSwtx::RoadUsage::RAMP;
}

}  // namespace sdmap
}  // namespace ad_common