#include "sdpromap.h"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <memory>
#include <queue>
#include <utility>

#include "map_data.pb.h"
#include "math/aaboxkdtree2d.h"
#include "math/math_utils.h"

namespace ad_common {
namespace sdpromap {
using ad_common::math::Vec2d;
int SDProMap::LoadMapFromProto(const iflymapdata::sdpro::MapData& map_proto) {
  map_info_table_.clear();
  std::unordered_set<uint64_t> route_link_ids_set{};
  if (map_proto.has_route()) {
    if (!route_info_ptr_) {
      route_info_ptr_ = std::make_shared<RouteInfo>();
    }
    route_info_ptr_->Init(map_proto.route());
    map_info_table_[MapInfoType::ROUTE_INFO] = route_info_ptr_;
    if (route_info_ptr_) {
      const auto route_link_ids = route_info_ptr_->getRouteLinkIds();
#ifdef BUILD_TEST
      for (const auto& link_id : route_link_ids) {
        std::cout << " route link id: " << link_id << std::endl;
      }
#endif
      if (route_link_ids.empty()) {
        is_noa_status_ = false;
        current_link_id_ = 0;
        current_index_ = -1;
      }  // reset noa status and cur link id
      else {
        if (!is_noa_status_ || history_route_link_ids_.empty()) {
          current_link_id_ = route_link_ids.front();  // init cur link
          current_index_ = 0;
          history_route_link_ids_ = route_link_ids;
          is_noa_status_ = true;
        } else {
          int new_cur_link_index = -1;
          // 将旧轨迹上的index映射到新轨迹上
          if (ad_common::math::ConfirmTargetIndex(
                  history_route_link_ids_, route_link_ids, current_index_,
                  new_cur_link_index)) {
            current_link_id_ = route_link_ids[new_cur_link_index];
            current_index_ = new_cur_link_index;
            history_route_link_ids_ = route_link_ids;
          } else {
            current_link_id_ = route_link_ids.front();  // init cur link
            current_index_ = 0;
            history_route_link_ids_ = route_link_ids;
          }
          is_noa_status_ = true;
        }
      }
      std::copy(route_link_ids.begin(), route_link_ids.end(),
                std::inserter(route_link_ids_set, route_link_ids_set.begin()));
    }
  } else {
    is_noa_status_ = false;
    current_link_id_ = 0;
    current_index_ = -1;
  }
  if (map_proto.has_link_info()) {
    if (!link_info_ptr_) {
      link_info_ptr_ = std::make_shared<LinkInfo>();
    }
    link_info_ptr_->Init(map_proto.link_info(), route_link_ids_set);
    map_info_table_[MapInfoType::LINK_INFO] = link_info_ptr_;
  }
  if (map_proto.has_lanes()) {
    if (!lane_info_ptr_) {
      lane_info_ptr_ = std::make_shared<LaneInfo>();
    }
    lane_info_ptr_->Init(map_proto.lanes());
    map_info_table_[MapInfoType::LANE_INFO] = lane_info_ptr_;
  }
  BuildSegmentKdTree();
  return 0;
}

void SDProMap::BuildSegmentKdTree() {
  ad_common::math::AABoxKDTreeParams params;
  params.max_leaf_dimension = 5.0;  // meters.
  params.max_leaf_size = 16;
  for (const auto& map_info : map_info_table_) {
    map_info.second->BuildSegmentKdTree(params);
  }
}

bool SDProMap::isRamp(const uint32_t& link_type) const {
  return static_cast<bool>(link_type & (iflymapdata::sdpro::LinkType::LT_IC |
                                        iflymapdata::sdpro::LinkType::LT_JCT));
}

bool SDProMap::isRoundAbout(const uint32_t& link_type) const {
  return static_cast<bool>(link_type &
                           (iflymapdata::sdpro::LinkType::LT_ROUNDABOUT));
}

bool SDProMap::isSaPa(const uint32_t& link_type) const {
  return static_cast<bool>(link_type & (iflymapdata::sdpro::LinkType::LT_SAPA));
}

bool SDProMap::isTollStation(const uint32_t& link_type) const {
  return static_cast<bool>(
      link_type & (iflymapdata::sdpro::LinkType::LT_TOLLBOOTH |
                   iflymapdata::sdpro::LinkType::LT_BOOTH_EXIT |
                   iflymapdata::sdpro::LinkType::LT_BOOTH_ENTRANCE |
                   iflymapdata::sdpro::LinkType::LT_BOOTH_EXIT_ENTRANCE));
}

bool SDProMap::isTunnel(const uint32_t& link_type) const {
  return static_cast<bool>(link_type &
                           (iflymapdata::sdpro::LinkType::LT_TUNNEL));
}

bool SDProMap::isNonExpress(
    const iflymapdata::sdpro::LinkClass& link_class) const {
  return link_class != iflymapdata::sdpro::LinkClass::LC_NONE &&
         link_class != iflymapdata::sdpro::LinkClass::LC_EXPRESSWAY &&
         link_class != iflymapdata::sdpro::LinkClass::LC_CITY_EXPRESSWAY;
}

bool SDProMap::isRouteValid() const {
  const RouteInfo::Ptr route_info_ptr = std::dynamic_pointer_cast<RouteInfo>(
      getMapInfoPtr(MapInfoType::ROUTE_INFO));
  if (!route_info_ptr) {
    return false;
  }
  const auto route_link_ids = route_info_ptr->getRouteLinkIds();
  if (route_link_ids.empty()) {
    return false;
  }
  return true;
}

const iflymapdata::sdpro::Lane* SDProMap::GetLaneInfoByID(
    const uint64_t& road_seg_id) const {
  const LaneInfo::Ptr lane_info_ptr = std::dynamic_pointer_cast<LaneInfo>(
      getMapInfoPtr(MapInfoType::LANE_INFO));
  if (!lane_info_ptr) {
    return nullptr;
  }
  const auto lane_ptr = lane_info_ptr->getLaneById(road_seg_id);
  if (!lane_ptr) {
    return nullptr;
  }
  return &lane_ptr->getLane();
}

const iflymapdata::sdpro::LinkInfo_Link* SDProMap::GetLinkOnRoute(
    uint64_t road_seg_id) const {
  const LinkInfo::Ptr link_info_ptr = std::dynamic_pointer_cast<LinkInfo>(
      getMapInfoPtr(MapInfoType::LINK_INFO));
  const RouteInfo::Ptr route_info_ptr = std::dynamic_pointer_cast<RouteInfo>(
      getMapInfoPtr(MapInfoType::ROUTE_INFO));
  if (!route_info_ptr || !link_info_ptr) {
    return nullptr;
  }
  const auto link_ptr = link_info_ptr->getLinkById(road_seg_id);
  if (!link_ptr) {
    return nullptr;
  }
  return &link_ptr->getLink();
}

const iflymapdata::sdpro::LinkInfo_Link* SDProMap::GetPreviousLinkOnRoute(
    uint64_t road_seg_id) const {
  const LinkInfo::Ptr link_info_ptr = std::dynamic_pointer_cast<LinkInfo>(
      getMapInfoPtr(MapInfoType::LINK_INFO));
  const RouteInfo::Ptr route_info_ptr = std::dynamic_pointer_cast<RouteInfo>(
      getMapInfoPtr(MapInfoType::ROUTE_INFO));
  if (!route_info_ptr || !link_info_ptr) {
    return nullptr;
  }
  uint64_t pre_id;
  if (route_info_ptr->getPreviousLinkId(road_seg_id, pre_id)) {
    const auto link_ptr = link_info_ptr->getLinkById(pre_id);
    if (!link_ptr) {
      return nullptr;
    }
    return &link_ptr->getLink();
  }
  return nullptr;
}

const iflymapdata::sdpro::LinkInfo_Link* SDProMap::GetNextLinkOnRoute(
    uint64_t road_seg_id) const {
  const LinkInfo::Ptr link_info_ptr = std::dynamic_pointer_cast<LinkInfo>(
      getMapInfoPtr(MapInfoType::LINK_INFO));
  const RouteInfo::Ptr route_info_ptr = std::dynamic_pointer_cast<RouteInfo>(
      getMapInfoPtr(MapInfoType::ROUTE_INFO));
  if (!route_info_ptr || !link_info_ptr) {
    return nullptr;
  }
  if (!route_info_ptr->isOnRouteLinks(road_seg_id)) {
    return nullptr;
  }
  uint64_t next_id;
  if (route_info_ptr->getNextLinkIdByCurIndex(road_seg_id, current_index_,
                                              next_id)) {
    const auto next_link_ptr = link_info_ptr->getLinkById(next_id);
    if (!next_link_ptr) {
      return nullptr;
    }
    return &next_link_ptr->getLink();
  }
  return nullptr;
}

std::shared_ptr<MapInfoBase> SDProMap::getMapInfoPtr(
    const MapInfoType& info_type) const {
  auto it = map_info_table_.find(info_type);
  return (it != map_info_table_.end()) ? it->second
                                       : std::shared_ptr<MapInfoBase>();
}

const iflymapdata::sdpro::LinkInfo_Link* SDProMap::GetNearestRoad(
    const Vec2d& p, double& nearest_s, double& nearest_l) const {
  const LinkInfo::Ptr link_info_ptr = std::dynamic_pointer_cast<LinkInfo>(
      getMapInfoPtr(MapInfoType::LINK_INFO));
  if (!link_info_ptr) {
    return nullptr;
  }
  const auto* obj = link_info_ptr->getNearestObject(p);
  if (!obj) {
    return nullptr;
  }
  const auto idx = obj->id();
  if (idx < obj->object()->segments().size() &&
      idx < obj->object()->AccumulatedS().size()) {
    const auto& point_segment = obj->object()->segments()[idx];
    Vec2d nearest_pt;
    point_segment.DistanceTo(p, &nearest_pt);
    nearest_s = obj->object()->AccumulatedS()[idx] +
                nearest_pt.DistanceTo(point_segment.start());
    nearest_l =
        point_segment.unit_direction().CrossProd(p - point_segment.start());
  }

  return &obj->object()->getLink();
}

std::pair<const iflymapdata::sdpro::LinkInfo_Link*, double>
SDProMap::GetRampInfo(uint64_t road_seg_id, double accumulated_s,
                      double max_distance) const {
  const iflymapdata::sdpro::LinkInfo_Link* ramp_segment = nullptr;
  double dis_to_ramp = 0;
  const LinkInfo::Ptr link_info_ptr = std::dynamic_pointer_cast<LinkInfo>(
      getMapInfoPtr(MapInfoType::LINK_INFO));
  const RouteInfo::Ptr route_info_ptr = std::dynamic_pointer_cast<RouteInfo>(
      getMapInfoPtr(MapInfoType::ROUTE_INFO));
  if (!route_info_ptr || !link_info_ptr) {
    return {};
  }
  const auto link_ptr = link_info_ptr->getLinkById(road_seg_id);
  if (!link_ptr) {
    return {};
  }
  const auto& current_seg = link_ptr->getLink();
  // 如果已经在匝道上，返回当前Segment，到匝道的距离设为0
  if (isRamp(current_seg.link_type())) {
    ramp_segment = &current_seg;
    return std::make_pair(ramp_segment, 0);
  }

  double current_seg_remain_dis =
      (accumulated_s > current_seg.length() * 0.01)
          ? 0
          : current_seg.length() * 0.01 - accumulated_s;
  dis_to_ramp += current_seg_remain_dis;

  for (const auto& sorted_link_id :
       route_info_ptr->getRemainRouteLinkIds(current_seg.id())) {
    if (dis_to_ramp > max_distance) {
      break;
    }
    const auto next_link_ptr = link_info_ptr->getLinkById(sorted_link_id);
    if (next_link_ptr) {
      const auto& next_link = next_link_ptr->getLink();
      if (isRamp(next_link.link_type())) {
        ramp_segment = &next_link;
        break;
      }
      dis_to_ramp += next_link.length() * 0.01;  // cal dis to ramp
    }
  }  // find next ramp link

  return std::make_pair(ramp_segment,
                        (ramp_segment != nullptr) ? dis_to_ramp : 0);
}

std::vector<std::pair<double, double>> SDProMap::GetCurvatureList(
    uint64_t road_seg_id, double accumulated_s, double max_distance) const {
  // const iflymapdata::sdpro::LinkInfo_Link* ramp_segment = nullptr;
  // double dis_to_ramp = 0;
  const LinkInfo::Ptr link_info_ptr = std::dynamic_pointer_cast<LinkInfo>(
      getMapInfoPtr(MapInfoType::LINK_INFO));
  const RouteInfo::Ptr route_info_ptr = std::dynamic_pointer_cast<RouteInfo>(
      getMapInfoPtr(MapInfoType::ROUTE_INFO));
  if (!route_info_ptr || !link_info_ptr) {
    return {};
  }
  const auto link_ptr = link_info_ptr->getLinkById(road_seg_id);
  if (!link_ptr) {
    return {};
  }
  std::vector<std::pair<double, double>> res;
  constexpr double kCurvatureRatio = 0.00001;  // 将曲率的单位转成1/m
  const auto& current_seg = link_ptr->getLink();
  auto dis_to_cur_link_end =
      (accumulated_s > (current_seg.length() * 0.01))
          ? 0.0
          : (current_seg.length() * 0.01) - accumulated_s;
  for (int i = 0;
       i < current_seg.offset_size() && i < current_seg.curvature_size(); ++i) {
    double offset = current_seg.offset(i) * 0.01;
    if (offset >= accumulated_s) {
      double dis_to_current_pos = offset - accumulated_s;
      if (dis_to_current_pos > max_distance) {
        break;
      }
      res.emplace_back(std::pair<double, double>{
          dis_to_current_pos, kCurvatureRatio * current_seg.curvature(i)});
    }
  }
  double max_search_dis = dis_to_cur_link_end;
  uint64_t cur_link_id = road_seg_id;
  int cur_link_index = current_index_;
  while (max_search_dis < accumulated_s) {
    // 查找下一个link
    uint64_t next_id;
    if (route_info_ptr->getNextLinkIdByCurIndex(cur_link_id, cur_link_index,
                                                next_id, &cur_link_index)) {
      const auto next_link_ptr = link_info_ptr->getLinkById(next_id);
      if (!next_link_ptr) {
        break;
      }
      const auto& next_link_info = next_link_ptr->getLink();
      // 遍历下一个link的曲率信息
      for (int i = 0; i < next_link_info.offset_size() &&
                      i < next_link_info.curvature_size();
           ++i) {
        double dis_to_current_pos =
            next_link_info.offset(i) * 0.01 + max_search_dis;
        // 若曲率信息的位置到当前位置小于检索范围，将曲率信息加入搜索结果
        if (dis_to_current_pos < max_distance) {
          res.emplace_back(std::pair<double, double>{
              dis_to_current_pos,
              kCurvatureRatio * next_link_info.curvature(i)});
        } else {  // 否则结束搜索
          break;
        }
      }

      // 更新搜索信息，为下一轮搜索做准备
      max_search_dis += next_link_info.length() * 0.01;
      cur_link_id = next_id;
    } else {
      break;
    }
  }
  return res;
}

std::vector<std::pair<const iflymapdata::sdpro::LinkInfo_Link*, double>>
SDProMap::GetMergeInfoList(uint64_t road_seg_id, double accumulated_s,
                           double max_distance) const {
  const LinkInfo::Ptr link_info_ptr = std::dynamic_pointer_cast<LinkInfo>(
      getMapInfoPtr(MapInfoType::LINK_INFO));
  const RouteInfo::Ptr route_info_ptr = std::dynamic_pointer_cast<RouteInfo>(
      getMapInfoPtr(MapInfoType::ROUTE_INFO));
  if (!route_info_ptr || !link_info_ptr) {
    return {};
  }
  const auto link_ptr = link_info_ptr->getLinkById(road_seg_id);
  if (!link_ptr) {
    return {};
  }
  const auto& cur_seg = link_ptr->getLink();
  double cur_seg_remain_dis = (accumulated_s > cur_seg.length() * 0.01)
                                  ? 0
                                  : (cur_seg.length() * 0.01 - accumulated_s);
  double dis_to_merge = cur_seg_remain_dis;
  std::vector<std::pair<const iflymapdata::sdpro::LinkInfo_Link*, double>>
      merge_list{};

  // 如果当前所在的路段是汇流点，距离设为负数
  if (cur_seg.predecessor_link_ids_size() > 1) {
    merge_list.emplace_back(std::make_pair(&cur_seg, -accumulated_s));
  }
  // search merge
  for (const auto& sorted_link_id :
       route_info_ptr->getRemainRouteLinkIds(cur_seg.id())) {
    if (dis_to_merge > max_distance) {
      break;
    }
    const auto next_link_ptr = link_info_ptr->getLinkById(sorted_link_id);
    if (next_link_ptr) {
      const auto& next_link = next_link_ptr->getLink();
      if (next_link.predecessor_link_ids_size() > 1) {
        merge_list.emplace_back(std::make_pair(&next_link, dis_to_merge));
      }
      dis_to_merge += next_link.length() * 0.01;
    }
  }

  return merge_list;
}

std::vector<std::pair<const iflymapdata::sdpro::LinkInfo_Link*, double>>
SDProMap::GetSplitInfoList(uint64_t road_seg_id, double accumulated_s,
                           double max_distance) const {
  const LinkInfo::Ptr link_info_ptr = std::dynamic_pointer_cast<LinkInfo>(
      getMapInfoPtr(MapInfoType::LINK_INFO));
  const RouteInfo::Ptr route_info_ptr = std::dynamic_pointer_cast<RouteInfo>(
      getMapInfoPtr(MapInfoType::ROUTE_INFO));
  if (!route_info_ptr || !link_info_ptr) {
    return {};
  }
  const auto link_ptr = link_info_ptr->getLinkById(road_seg_id);
  if (!link_ptr) {
    return {};
  }
  const auto& cur_seg = link_ptr->getLink();
  double cur_seg_remain_dis = (accumulated_s > cur_seg.length() * 0.01)
                                  ? 0
                                  : cur_seg.length() * 0.01 - accumulated_s;
  double dis_to_split = cur_seg_remain_dis;
  std::vector<std::pair<const iflymapdata::sdpro::LinkInfo_Link*, double>>
      split_list{};
  if (cur_seg.successor_link_ids_size() > 1) {
    split_list.emplace_back(std::make_pair(&cur_seg, dis_to_split));
  }

  // search split
  for (const auto& sorted_link_id :
       route_info_ptr->getRemainRouteLinkIds(cur_seg.id())) {
    const auto next_link_ptr = link_info_ptr->getLinkById(sorted_link_id);
    if (next_link_ptr) {
      const auto& next_link = next_link_ptr->getLink();
      dis_to_split += next_link.length() * 0.01;
      if (dis_to_split > max_distance) {
        break;
      }
      if (next_link.successor_link_ids_size() > 1) {
        split_list.emplace_back(std::make_pair(&next_link, dis_to_split));
      }
    }
  }
  return split_list;
}

std::vector<std::pair<const iflymapdata::sdpro::LinkInfo_Link*, double>>
SDProMap::GetRoundAboutList(uint64_t road_seg_id, double accumulated_s,
                            const double& max_distance) const {
  const LinkInfo::Ptr link_info_ptr = std::dynamic_pointer_cast<LinkInfo>(
      getMapInfoPtr(MapInfoType::LINK_INFO));
  const RouteInfo::Ptr route_info_ptr = std::dynamic_pointer_cast<RouteInfo>(
      getMapInfoPtr(MapInfoType::ROUTE_INFO));
  if (!link_info_ptr) {
    return {};
  }
  const auto link_ptr = link_info_ptr->getLinkById(road_seg_id);
  if (!link_ptr) {
    return {};
  }
  std::vector<std::pair<const iflymapdata::sdpro::LinkInfo_Link*, double>>
      roundabout_list{};
  const auto& cur_seg = link_ptr->getLink();
  double dis_to_roundabout = (accumulated_s > cur_seg.length() * 0.01)
                                 ? 0
                                 : cur_seg.length() * 0.01 - accumulated_s;
  if (isRoundAbout(cur_seg.link_type())) {
    roundabout_list.emplace_back(std::make_pair(&cur_seg, dis_to_roundabout));
  } else {  // --cur link is junction
    if (!route_info_ptr) {
      std::queue<std::pair<iflymapdata::sdpro::LinkInfo_Link, double>>
          link_queue;
      std::unordered_set<uint64_t> visited;
      visited.insert(cur_seg.id());
      link_queue.push(std::make_pair(cur_seg, dis_to_roundabout));
      while (!link_queue.empty()) {
        auto [current_link, current_distance] = std::move(link_queue.front());
        link_queue.pop();
        if (isRoundAbout(current_link.link_type())) {
          roundabout_list.emplace_back(
              std::make_pair(&current_link, current_distance));
        } else {
          for (const auto& successor_link_id :
               current_link.successor_link_ids()) {
            const auto next_link_ptr =
                link_info_ptr->getLinkById(successor_link_id);
            if (next_link_ptr) {
              const auto& next_link = next_link_ptr->getLink();
              double new_distance =
                  current_distance + next_link.length() * 0.01;
              if (new_distance <= 500.0 &&
                  visited.find(next_link.id()) == visited.end()) {
                link_queue.push(std::make_pair(next_link, new_distance));
                visited.insert(next_link.id());
              }
            }
          }
        }
      }
    }  // -- no route
    else {
      for (const auto& sorted_link_id :
           route_info_ptr->getRemainRouteLinkIds(cur_seg.id())) {
        const auto next_link_ptr = link_info_ptr->getLinkById(sorted_link_id);
        if (next_link_ptr) {
          const auto& next_link = next_link_ptr->getLink();
          dis_to_roundabout += next_link.length() * 0.01;
          if (isRoundAbout(next_link.link_type())) {
            roundabout_list.emplace_back(
                std::make_pair(&next_link, dis_to_roundabout));
            break;  // only return nearest junction
          }
        }
      }
    }  // -- has route
  }
  return roundabout_list;
}

std::pair<const iflymapdata::sdpro::LinkInfo_Link*, double>
SDProMap::GetTollStationInfo(uint64_t road_seg_id, double accumulated_s,
                             double max_distance) const {
  const iflymapdata::sdpro::LinkInfo_Link* toll_station_segment = nullptr;
  double dis_to_toll_station = 0.0;
  const LinkInfo::Ptr link_info_ptr = std::dynamic_pointer_cast<LinkInfo>(
      getMapInfoPtr(MapInfoType::LINK_INFO));
  const RouteInfo::Ptr route_info_ptr = std::dynamic_pointer_cast<RouteInfo>(
      getMapInfoPtr(MapInfoType::ROUTE_INFO));
  if (!route_info_ptr || !link_info_ptr) {
    return std::make_pair(toll_station_segment,
                          toll_station_segment ? dis_to_toll_station : 0.0);
  }
  const auto link_ptr = link_info_ptr->getLinkById(road_seg_id);
  if (!link_ptr) {
    return std::make_pair(toll_station_segment,
                          toll_station_segment ? dis_to_toll_station : 0.0);
  }

  const auto& current_seg = link_ptr->getLink();
  double current_seg_remain_dis =
      (accumulated_s > current_seg.length() * 0.01)
          ? 0
          : current_seg.length() * 0.01 - accumulated_s;
  if (isTollStation(current_seg.link_type())) {
    toll_station_segment = &current_seg;
    dis_to_toll_station = current_seg_remain_dis;
  } else {
    dis_to_toll_station += current_seg_remain_dis;
    for (const auto& sorted_link_id :
         route_info_ptr->getRemainRouteLinkIds(current_seg.id())) {
      const auto next_link_ptr = link_info_ptr->getLinkById(sorted_link_id);
      if (next_link_ptr) {
        const auto& next_link = next_link_ptr->getLink();
        dis_to_toll_station += next_link.length() * 0.01;
        if (dis_to_toll_station > max_distance) {
          break;
        }
        if (isTollStation(next_link.link_type())) {
          toll_station_segment = &next_link;
          break;
        }
      }
    }
  }
  return std::make_pair(toll_station_segment,
                        toll_station_segment ? dis_to_toll_station : 0.0);
}

std::vector<uint64_t> SDProMap::getRouteLinksIds() const {
  const auto map_info_ptr = getMapInfoPtr(MapInfoType::ROUTE_INFO);
  if (!map_info_ptr) {
    return {};
  }
  const RouteInfo::Ptr route_info_ptr =
      std::dynamic_pointer_cast<RouteInfo>(map_info_ptr);
  if (!route_info_ptr) {
    return {};
  }
  return route_info_ptr->getRouteLinkIds();
}

std::pair<const iflymapdata::sdpro::LinkInfo_Link*, double>
SDProMap::GetTunnelInfo(uint64_t road_seg_id, double accumulated_s,
                        double max_distance) const {
  const iflymapdata::sdpro::LinkInfo_Link* tunnel_segment = nullptr;
  double dis_to_tunnel = 0.0;
  const LinkInfo::Ptr link_info_ptr = std::dynamic_pointer_cast<LinkInfo>(
      getMapInfoPtr(MapInfoType::LINK_INFO));
  const RouteInfo::Ptr route_info_ptr = std::dynamic_pointer_cast<RouteInfo>(
      getMapInfoPtr(MapInfoType::ROUTE_INFO));
  if (!route_info_ptr || !link_info_ptr) {
    return std::make_pair(tunnel_segment, (tunnel_segment) ? dis_to_tunnel : 0);
  }
  const auto link_ptr = link_info_ptr->getLinkById(road_seg_id);
  if (!link_ptr) {
    return std::make_pair(tunnel_segment, (tunnel_segment) ? dis_to_tunnel : 0);
  }
  const auto& current_seg = link_ptr->getLink();
  if (isTunnel(current_seg.link_type())) {
    tunnel_segment = &current_seg;
  } else {
    double current_seg_remain_dis =
        (accumulated_s > current_seg.length() * 0.01)
            ? 0
            : current_seg.length() * 0.01 - accumulated_s;
    dis_to_tunnel += current_seg_remain_dis;
    for (const auto& sorted_link_id :
         route_info_ptr->getRemainRouteLinkIds(current_seg.id())) {
      const auto next_link_ptr = link_info_ptr->getLinkById(sorted_link_id);
      if (next_link_ptr) {
        const auto& next_link = next_link_ptr->getLink();
        if (dis_to_tunnel > max_distance) {
          break;
        }
        if (isTunnel(next_link.link_type())) {
          tunnel_segment = &next_link;
          break;
        }
        dis_to_tunnel += next_link.length() * 0.01;
      }
    }
  }
  return std::make_pair(tunnel_segment, (tunnel_segment) ? dis_to_tunnel : 0);
}

std::pair<const iflymapdata::sdpro::LinkInfo_Link*, double>
SDProMap::GetRoundAboutInfo(uint64_t road_seg_id, double accumulated_s,
                            double max_distance) const {
  const iflymapdata::sdpro::LinkInfo_Link* roundabout_segment = nullptr;
  double dis_to_roundabout = 0.0;
  const LinkInfo::Ptr link_info_ptr = std::dynamic_pointer_cast<LinkInfo>(
      getMapInfoPtr(MapInfoType::LINK_INFO));
  const RouteInfo::Ptr route_info_ptr = std::dynamic_pointer_cast<RouteInfo>(
      getMapInfoPtr(MapInfoType::ROUTE_INFO));
  if (!route_info_ptr || !link_info_ptr) {
    return std::make_pair(roundabout_segment,
                          (roundabout_segment) ? dis_to_roundabout : 0);
  }
  const auto link_ptr = link_info_ptr->getLinkById(road_seg_id);
  if (!link_ptr) {
    return std::make_pair(roundabout_segment,
                          (roundabout_segment) ? dis_to_roundabout : 0);
  }
  const auto& current_seg = link_ptr->getLink();
  if (isRoundAbout(current_seg.link_type())) {
    roundabout_segment = &current_seg;
  } else {
    double current_seg_remain_dis =
        (accumulated_s > current_seg.length() * 0.01)
            ? 0
            : current_seg.length() * 0.01 - accumulated_s;
    dis_to_roundabout += current_seg_remain_dis;
    for (const auto& sorted_link_id :
         route_info_ptr->getRemainRouteLinkIds(current_seg.id())) {
      const auto next_link_ptr = link_info_ptr->getLinkById(sorted_link_id);
      if (next_link_ptr) {
        const auto& next_link = next_link_ptr->getLink();
        if (dis_to_roundabout > max_distance) {
          break;
        }
        if (isRoundAbout(next_link.link_type())) {
          roundabout_segment = &next_link;
          break;
        }
        dis_to_roundabout += next_link.length() * 0.01;
      }
    }
  }
  return std::make_pair(roundabout_segment,
                        (roundabout_segment) ? dis_to_roundabout : 0);
}  // TODO 把所有道路类型的获取整合成一个函数

std::pair<const iflymapdata::sdpro::LinkInfo_Link*, double>
SDProMap::GetSaPaInfo(uint64_t road_seg_id, double accumulated_s,
                      double max_distance) const {
  const iflymapdata::sdpro::LinkInfo_Link* sapa_segment = nullptr;
  double dis_to_sapa = 0.0;
  const LinkInfo::Ptr link_info_ptr = std::dynamic_pointer_cast<LinkInfo>(
      getMapInfoPtr(MapInfoType::LINK_INFO));
  const RouteInfo::Ptr route_info_ptr = std::dynamic_pointer_cast<RouteInfo>(
      getMapInfoPtr(MapInfoType::ROUTE_INFO));
  if (!route_info_ptr || !link_info_ptr) {
    return std::make_pair(sapa_segment, (sapa_segment) ? dis_to_sapa : 0);
  }
  const auto link_ptr = link_info_ptr->getLinkById(road_seg_id);
  if (!link_ptr) {
    return std::make_pair(sapa_segment, (sapa_segment) ? dis_to_sapa : 0);
  }
  const auto& current_seg = link_ptr->getLink();
  if (isSaPa(current_seg.link_type())) {
    sapa_segment = &current_seg;
  } else {
    double current_seg_remain_dis =
        (accumulated_s > current_seg.length() * 0.01)
            ? 0
            : current_seg.length() * 0.01 - accumulated_s;
    dis_to_sapa += current_seg_remain_dis;
    for (const auto& sorted_link_id :
         route_info_ptr->getRemainRouteLinkIds(current_seg.id())) {
      const auto next_link_ptr = link_info_ptr->getLinkById(sorted_link_id);
      if (next_link_ptr) {
        const auto& next_link = next_link_ptr->getLink();
        if (dis_to_sapa > max_distance) {
          break;
        }
        if (isSaPa(next_link.link_type())) {
          sapa_segment = &next_link;
          break;
        }
        dis_to_sapa += next_link.length() * 0.01;
      }
    }
  }
  return std::make_pair(sapa_segment, (sapa_segment) ? dis_to_sapa : 0);
}

std::pair<const iflymapdata::sdpro::LinkInfo_Link*, double>
SDProMap::GetNonExpressInfo(uint64_t road_seg_id, double accumulated_s,
                            double max_distance) const {
  const iflymapdata::sdpro::LinkInfo_Link* non_express_segment = nullptr;
  double dis_to_non_express = 0.0;
  const LinkInfo::Ptr link_info_ptr = std::dynamic_pointer_cast<LinkInfo>(
      getMapInfoPtr(MapInfoType::LINK_INFO));
  const RouteInfo::Ptr route_info_ptr = std::dynamic_pointer_cast<RouteInfo>(
      getMapInfoPtr(MapInfoType::ROUTE_INFO));
  if (!route_info_ptr || !link_info_ptr) {
    return std::make_pair(non_express_segment,
                          (non_express_segment) ? dis_to_non_express : 0);
  }
  const auto link_ptr = link_info_ptr->getLinkById(road_seg_id);
  if (!link_ptr) {
    return std::make_pair(non_express_segment,
                          (non_express_segment) ? dis_to_non_express : 0);
  }
  const auto& current_seg = link_ptr->getLink();
  if (isNonExpress(current_seg.link_class())) {
    non_express_segment = &current_seg;
  } else {
    double current_seg_remain_dis =
        (accumulated_s > current_seg.length() * 0.01)
            ? 0
            : current_seg.length() * 0.01 - accumulated_s;
    dis_to_non_express += current_seg_remain_dis;
    for (const auto& sorted_link_id :
         route_info_ptr->getRemainRouteLinkIds(current_seg.id())) {
      const auto next_link_ptr = link_info_ptr->getLinkById(sorted_link_id);
      if (next_link_ptr) {
        const auto& next_link = next_link_ptr->getLink();
        if (dis_to_non_express > max_distance) {
          break;
        }
        if (isNonExpress(next_link.link_class())) {
          non_express_segment = &next_link;
          break;
        }
        dis_to_non_express += next_link.length() * 0.01;
      }
    }
  }
  return std::make_pair(non_express_segment,
                        (non_express_segment) ? dis_to_non_express : 0);
}

int SDProMap::GetDistanceToRouteEnd(uint64_t road_seg_id, double accumulated_s,
                                    double& dis_to_end) const {
  const LinkInfo::Ptr link_info_ptr = std::dynamic_pointer_cast<LinkInfo>(
      getMapInfoPtr(MapInfoType::LINK_INFO));
  const RouteInfo::Ptr route_info_ptr = std::dynamic_pointer_cast<RouteInfo>(
      getMapInfoPtr(MapInfoType::ROUTE_INFO));
  if (!route_info_ptr || !link_info_ptr) {
    return -1;
  }
  const auto link_ptr = link_info_ptr->getLinkById(road_seg_id);
  if (!link_ptr) {
    return -1;
  }
  const auto& cur_seg = link_ptr->getLink();

  dis_to_end = (accumulated_s > cur_seg.length() * 0.01)
                   ? 0
                   : cur_seg.length() * 0.01 - accumulated_s;

  for (const auto& sorted_link_id :
       route_info_ptr->getRemainRouteLinkIds(cur_seg.id())) {
    const auto next_link_ptr = link_info_ptr->getLinkById(sorted_link_id);
    if (next_link_ptr) {
      const auto& next_link = next_link_ptr->getLink();
      dis_to_end += next_link.length() * 0.01;
    }
  }
  return 0;
}

std::unordered_map<MapInfoType, std::vector<std::pair<uint64_t, uint64_t>>>
SDProMap::SearchObjects(const Vec2d& center, double radius) const {
  std::unordered_map<MapInfoType, std::vector<std::pair<uint64_t, uint64_t>>>
      result_ids;
  for (const auto& map_info : map_info_table_) {
    auto objects = map_info.second->GetObjects(center, radius);
    result_ids[map_info.first] = objects;
  }
  return result_ids;
}

std::vector<std::pair<uint64_t, uint64_t>> SDProMap::SearchObjects(
    const Vec2d& center, double& radius,
    const MapInfoType& map_info_type) const {
  if (!map_info_table_.count(map_info_type)) {
    return std::vector<std::pair<uint64_t, uint64_t>>{};
  }
  auto objects = map_info_table_.at(map_info_type)->GetObjects(center, radius);
  return objects;
}

bool SDProMap::isProjOnSegment(const math::LineSegment2d& segment,
                               const Vec2d& point, double* proj_length) const {
  std::cout << "segment start: " << segment.start().x() << ", "
            << segment.start().y() << std::endl;
  std::cout << "segment end: " << segment.end().x() << ", " << segment.end().y()
            << std::endl;
  std::cout << "point: " << point.x() << ", " << point.y() << std::endl;
  const double x0 = point.x() - segment.start().x();
  const double y0 = point.y() - segment.start().y();
  const double proj =
      x0 * segment.unit_direction().x() + y0 * segment.unit_direction().y();
  if (proj_length) {
    *proj_length = proj;
  }
  /*
   ldmap，1Hz的更新频率过低，会导致更新数据时，boot坐标系下地图位置发生跳变,llh不会
   所以需要给is_proj的判断，增加一个阈值，该阈值实际上取决于ego_motion在1s内的偏移量
  */
  const double threshold = 0.5;
#ifdef BUILD_TEST
  std::cout << " proj: " << proj << " length: " << segment.length()
            << std::endl;
#endif
  if (proj >= (0 - threshold) && proj <= (segment.length() + threshold)) {
    return true;
  }
  return false;
}

std::vector<std::pair<uint64_t, uint64_t>> SDProMap::GetLinksWithHeading(
    const Vec2d& point, double distance, double central_heading,
    double max_heading_diff) const {
  auto result_ids = SearchObjects(point, distance, MapInfoType::LINK_INFO);
  if (result_ids.empty()) {
    return {};
  }
  std::vector<std::pair<uint64_t, uint64_t>> filtered_ids{};
  const auto map_info_ptr = getMapInfoPtr(MapInfoType::LINK_INFO);
  const LinkInfo::Ptr link_info_ptr =
      std::dynamic_pointer_cast<LinkInfo>(map_info_ptr);
  const RouteInfo::Ptr route_info_ptr = std::dynamic_pointer_cast<RouteInfo>(
      getMapInfoPtr(MapInfoType::ROUTE_INFO));
  if (!link_info_ptr) {
    return {};
  }
  if (is_noa_status_) {
    if (!route_info_ptr) {
      return {};
    }
  }
  for (const auto& id_pair : result_ids) {
    uint64_t road_seg_id = id_pair.first;
    uint64_t inner_linesegment_idx = id_pair.second;
    const auto& current_seginfo = link_info_ptr->getLinkById(road_seg_id);
    if (!current_seginfo) {
      continue;
    }
    if (inner_linesegment_idx >= current_seginfo->segments().size() ||
        inner_linesegment_idx >= current_seginfo->Headings().size()) {
      continue;
    }
    const auto& line_segment =
        current_seginfo->segments()[inner_linesegment_idx];
    Vec2d map_point{};
    double dis = line_segment.DistanceTo(point, &map_point);
    if (dis <= distance) {
      double heading_diff = fabs(
          current_seginfo->Headings()[inner_linesegment_idx] - central_heading);
      if (fabs(ad_common::math::NormalizeAngle(heading_diff)) <=
          max_heading_diff) {
        filtered_ids.push_back(id_pair);
      }
    }
  }
  return filtered_ids;
}

/*
  该接口中包含了当前位置在route上递推的功能，但是获取随机点所在link，会影响递推功能
  故增加is_search_cur_link参数，用于控制是否启动递推功能
  该参数默认为true，当需要获取随机点link时，需设置为false
*/
const iflymapdata::sdpro::LinkInfo_Link* SDProMap::GetNearestLinkWithHeading(
    const Vec2d& point, double distance, double central_heading,
    double max_heading_diff, double& nearest_s, double& nearest_l,
    bool is_search_cur_link) const {
#ifdef BUILD_TEST
  std::cout << "get nearest link with heading "
            << "current_link_id: " << current_link_id_
            << " current_index: " << current_index_ << std::endl;
#endif

  const iflymapdata::sdpro::LinkInfo_Link* nearest_lane = nullptr;
  auto id_list =
      GetLinksWithHeading(point, distance, central_heading, max_heading_diff);
  if (id_list.empty()) {
#ifdef BUILD_TEST
    std::cout << "is going off course" << std::endl;
#endif
    is_going_off_course = true;
    return nearest_lane;
  } else {
    is_going_off_course = false;
  }

  const auto map_info_ptr = getMapInfoPtr(MapInfoType::LINK_INFO);
  const LinkInfo::Ptr link_info_ptr =
      std::dynamic_pointer_cast<LinkInfo>(map_info_ptr);
  const RouteInfo::Ptr route_info_ptr = std::dynamic_pointer_cast<RouteInfo>(
      getMapInfoPtr(MapInfoType::ROUTE_INFO));
  std::vector<uint64_t> route_link_ids;
  if (!link_info_ptr) {
    return nearest_lane;
  }

  if (!is_noa_status_ || !route_info_ptr || !is_search_cur_link) {
#ifdef BUILD_TEST
    std::cout << "not in noa status" << std::endl;
#endif
    const iflymapdata::sdpro::LinkInfo_Link* nearest_lane_candidate = nullptr;
    double min_dis = distance, min_dis_candidate = distance;
    bool has_proj_on_segment = false;
    double nearest_s_candidate = nearest_s,
           nearest_l_candidate = nearest_l;  // for proj candidate

    for (const auto& id_pair : id_list) {
      uint64_t road_seg_id = id_pair.first;
      uint64_t inner_linesegment_idx = id_pair.second;

      const auto& current_seginfo = link_info_ptr->getLinkById(road_seg_id);
      if (!current_seginfo) {
        continue;
      }
      if (inner_linesegment_idx >= current_seginfo->segments().size() ||
          inner_linesegment_idx >= current_seginfo->Headings().size() ||
          inner_linesegment_idx >= current_seginfo->AccumulatedS().size()) {
        continue;
      }
      const auto& line_segment =
          current_seginfo->segments()[inner_linesegment_idx];
      Vec2d map_point{};
      double dis = line_segment.DistanceTo(point, &map_point);

      if (isProjOnSegment(line_segment, point)) {
        if (dis < min_dis) {
          has_proj_on_segment = true;
          min_dis = dis;
          nearest_lane = &current_seginfo->getLink();
          nearest_s = current_seginfo->AccumulatedS()[inner_linesegment_idx] +
                      line_segment.start().DistanceTo(map_point);
          nearest_l = line_segment.unit_direction().CrossProd(
              point - line_segment.start());
        }
      }  // filter link, which proj is not on segment
      else {
        if (dis < min_dis_candidate) {
          min_dis_candidate = dis;
          nearest_lane_candidate = &current_seginfo->getLink();
          nearest_s_candidate =
              current_seginfo->AccumulatedS()[inner_linesegment_idx] +
              line_segment.start().DistanceTo(map_point);
          nearest_l_candidate = line_segment.unit_direction().CrossProd(
              point - line_segment.start());
        }
      }
    }
    if (!has_proj_on_segment) {
      nearest_s = nearest_s_candidate;
      nearest_l = nearest_l_candidate;
      return nearest_lane_candidate;
    }
    return nearest_lane;
  }

  if (is_noa_status_) {
    if (!route_info_ptr) {
      return nearest_lane;
    }
    if (route_info_ptr->getRouteLinkIds().empty()) {
      return nearest_lane;
    }
    route_link_ids = route_info_ptr->getRouteLinkIds();
  }

  /*
    下面这些步骤的目的是找出能够投影上的seg段，但是由于一些数据特性
    1、v形线段，坐标点位于v下侧，无法通过isProjOnSegment获得投影结果
    2、baidu地图存在连续link，点集不连续的情况
    所以需要新增策略，确定投影的策略：
    1、直接通过isProjSegment返回true;
    2、同一link上连续的两个seg段，proj_len一个返回+，另一个返回-
    3、连续的不同link衔接处，proj_len一个返回+，一个返回-
    4、暂不考虑坐标点投影到所有Link串两端的外侧，该case视为地图不完整，地图问题
  */

  // 只存储非投影link以及seg的信息
  // first -- link_id,
  // second[x].first -- seg_id,
  // second[x].second -- proj_length
  std::unordered_map<uint64_t, std::vector<std::pair<uint64_t, double>>>
      links_seg_proj_map;
  std::vector<std::pair<uint64_t, uint64_t>>
      proj_id_list;                     // 所有可投影的link_id以及seg_id
  std::set<uint64_t> proj_link_id_set;  // 可投影的link_id集合
  std::set<uint64_t> link_id_set;       // 所有heading符合的link_id集合
  links_seg_proj_map.reserve(id_list.size());
  proj_id_list.reserve(id_list.size());

  // 计算投影值
  for (const auto& id_pair : id_list) {
    uint64_t road_seg_id = id_pair.first;             // link_id
    uint64_t inner_linesegment_idx = id_pair.second;  // seg_index

    const auto& current_seginfo = link_info_ptr->getLinkById(road_seg_id);
    if (!current_seginfo) {
      continue;
    }
    if (inner_linesegment_idx >= current_seginfo->segments().size() ||
        inner_linesegment_idx >= current_seginfo->Headings().size() ||
        inner_linesegment_idx >= current_seginfo->AccumulatedS().size()) {
      continue;
    }
    const auto& line_segment =
        current_seginfo->segments()[inner_linesegment_idx];
#ifdef BUILD_TEST
    std::cout << "link_id: " << road_seg_id
              << " seg_index: " << inner_linesegment_idx;
#endif
    double proj_len = 0.0;
    const auto& is_proj = isProjOnSegment(line_segment, point, &proj_len);
    // 判断投影策略1
    if (is_proj) {
      proj_id_list.push_back(id_pair);
      proj_link_id_set.insert(road_seg_id);
    } else {
      links_seg_proj_map[road_seg_id].emplace_back(
          std::make_pair(inner_linesegment_idx, proj_len));
    }
    link_id_set.insert(road_seg_id);
  }

  // 分析坐标点仍可投影到哪些Link以及seg
  for (auto& link_seg_proj : links_seg_proj_map) {
    const auto& link_id = link_seg_proj.first;
    auto& segs_proj_list = link_seg_proj.second;
    // 进行seg段的排序去重
    std::sort(segs_proj_list.begin(), segs_proj_list.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
    auto last = std::unique(
        segs_proj_list.begin(), segs_proj_list.end(),
        [](const auto& a, const auto& b) { return a.first == b.first; });
    segs_proj_list.erase(last, segs_proj_list.end());

    // 判断投影策略2
    if (segs_proj_list.empty()) {
      continue;
    }
    for (size_t i = 0; i < segs_proj_list.size() - 1; ++i) {
      const auto& cur_seg = segs_proj_list[i];
      const auto& next_seg = segs_proj_list[i + 1];
      if (cur_seg.second * next_seg.second < 0) {
        // 说明坐标点确实投影到了该link上，通过dis决策出最优的seg
        const auto& cur_link = link_info_ptr->getLinkById(link_id);
        if (!cur_link) {
          continue;
        }
        const auto& cur_lineseg = cur_link->segments()[cur_seg.first];
        const auto& next_lineseg = cur_link->segments()[next_seg.first];
        Vec2d cur_map_pt{};
        Vec2d next_map_pt{};
        const double& cur_dis = cur_lineseg.DistanceTo(point, &cur_map_pt);
        const double& next_dis = next_lineseg.DistanceTo(point, &next_map_pt);
        if (cur_dis <= next_dis) {
          proj_id_list.push_back(std::make_pair(link_id, cur_seg.first));
          proj_link_id_set.insert(link_id);
        } else {
          proj_id_list.push_back(std::make_pair(link_id, next_seg.first));
          proj_link_id_set.insert(link_id);
        }
      }
    }
  }

  // 判断投影策略3
  // 先判断seg是否在衔接处，再判断和下一个link的衔接处，proj_length是否符合情况
  for (const auto& link_seg_proj : links_seg_proj_map) {
    const auto& cur_link_id = link_seg_proj.first;
    const auto& cur_link = link_info_ptr->getLinkById(cur_link_id);
    if (!cur_link) {
      continue;
    }
    uint64_t total_segment_size = cur_link->segments().size();

    std::pair<uint64_t, double> cur_target_seg_proj;
    for (const auto& cur_seg_proj : link_seg_proj.second) {
      if (cur_seg_proj.first == total_segment_size - 1) {
        // 说明这个seg确实在末尾边界，需要判断next_link的衔接处起始边界，proj_length是否符合情况
        cur_target_seg_proj = cur_seg_proj;
        break;
      }
    }
    // 寻找next_link
    std::vector<uint64_t> next_link_ids;
    if (route_info_ptr->getNextLinkId(cur_link_id, next_link_ids)) {
      for (const auto& next_id : next_link_ids) {
        const auto& next_link = link_info_ptr->getLinkById(next_id);
        if (!next_link) {
          continue;
        }
        if (links_seg_proj_map.find(next_id) != links_seg_proj_map.end()) {
          // 找出next_link起始边界是否在links_seg_proj_map中
          for (const auto& next_seg_proj : links_seg_proj_map.at(next_id)) {
            if (next_seg_proj.first == 0 &&
                cur_target_seg_proj.second * next_seg_proj.second < 0) {
              // 找到了想要的seg_proj，和末尾的seg_proj一起，并且坐标点是否夹在两者中间
              const auto& cur_seg_info =
                  cur_link->segments()[cur_target_seg_proj.first];
              const auto& next_seg_info =
                  next_link->segments()[next_seg_proj.first];
              Vec2d cur_map_pt{};
              Vec2d next_map_pt{};
              const double& cur_dis =
                  cur_seg_info.DistanceTo(point, &cur_map_pt);
              const double& next_dis =
                  next_seg_info.DistanceTo(point, &next_map_pt);
              if (cur_dis <= next_dis) {
                proj_id_list.emplace_back(cur_link_id,
                                          cur_target_seg_proj.first);
                proj_link_id_set.insert(cur_link_id);
              } else {
                proj_id_list.emplace_back(next_id, next_seg_proj.first);
                proj_link_id_set.insert(next_id);
              }
            }
          }
        }
      }
    }
  }

#ifdef BUILD_TEST
  std::cout << "link_id_set: ";
  for (const auto& link_id : link_id_set) {
    std::cout << link_id << "; ";
  }
  std::cout << std::endl;
#endif

  if (!proj_id_list.empty()) {
#ifdef BUILD_TEST
    std::cout << "can find proj link id: ";
    for (const auto& proj_link_id : proj_link_id_set) {
      std::cout << proj_link_id << "; ";
    }
    std::cout << std::endl;
#endif
    // 在可投影link中找到link_index离cur_link_index最近的
    int min_index_dis = std::numeric_limits<int>::max();
    int target_link_index = -1;
    uint64_t target_link_id = 0;
    for (const auto& proj_link_id : proj_link_id_set) {
      std::vector<int> proj_link_index_list;
      if (route_info_ptr->getRouteLinkIndex(proj_link_id,
                                            proj_link_index_list)) {
        for (const auto& proj_link_index : proj_link_index_list) {
          /*
           此处存在一个问题，ad_common的cur_link的检索，非常依赖cur_index_往前递推，
           如果某一帧输入的定位异常，产生了非常大的index跳变，后续就算输入的定位恢复，
           也会导致cur_index_更新，因为规则上决定cur_index不能倒退
           所以解决该case，需要增加异常状态恢复功能
           该功能设计为：如果当前proj_link_id_set只存在一个元素，则不进行index_dis是否为非负的判断
          */
          /*
            todo: qcchen:
            存在一个问题，坐标点在v型的中间，会始终定位到后一个seg上
          */
          const int index_dis =
              proj_link_index - current_index_;  // 车辆向前行驶，不考虑向后情况
          if (proj_link_id_set.size() == 1) {
            if (index_dis < min_index_dis) {
              min_index_dis = index_dis;
              target_link_index = proj_link_index;
              target_link_id = proj_link_id;
            }
          } else {
            if (index_dis >= 0 && index_dis < min_index_dis) {
              min_index_dis = index_dis;
              target_link_index = proj_link_index;
              target_link_id = proj_link_id;
            }
          }
        }
      }
    }
    if (target_link_index == -1 || target_link_id == 0) {
      return nearest_lane;
    }
    // 找到目标index后，找proj中的目标seg
    double min_proj_dis = distance;
    for (const auto& proj_id_pair : proj_id_list) {
      if (proj_id_pair.first != target_link_id) {
        continue;
      }
      const auto& current_seginfo =
          link_info_ptr->getLinkById(proj_id_pair.first);
      const auto& line_segment =
          current_seginfo->segments()[proj_id_pair.second];
      Vec2d map_point{};
      double dis = line_segment.DistanceTo(point, &map_point);
      if (dis < min_proj_dis) {
        min_proj_dis = dis;
        nearest_lane = &current_seginfo->getLink();
        nearest_s = current_seginfo->AccumulatedS()[proj_id_pair.second] +
                    line_segment.start().DistanceTo(map_point);
        nearest_l = line_segment.unit_direction().CrossProd(
            point - line_segment.start());
        current_link_id_ = target_link_id;
        current_index_ = target_link_index;
      }
    }
    return nearest_lane;
  } else {
#ifdef BUILD_TEST
    std::cout << "cannot find proj link " << std::endl;
#endif
    // 在所有link中找到link_index离cur_link_index最近的
    int min_index_dis = std::numeric_limits<int>::max();
    int target_link_index = -1;
    uint64_t target_link_id = 0;
    for (const auto& link_id : link_id_set) {
      std::vector<int> link_index_list;
      if (route_info_ptr->getRouteLinkIndex(link_id, link_index_list)) {
        for (const auto& link_index : link_index_list) {
          /*
            同上方proj_case相同
          */
          const int index_dis = link_index - current_index_;
          if (link_id_set.size() == 1) {
            if (index_dis < min_index_dis) {
              min_index_dis = index_dis;
              target_link_index = link_index;
              target_link_id = link_id;
            }
          } else {
            if (index_dis >= 0 && index_dis < min_index_dis) {
              min_index_dis = index_dis;
              target_link_index = link_index;
              target_link_id = link_id;
            }
          }
        }
      }
    }
    if (target_link_index == -1 || target_link_id == 0) {
      return nearest_lane;
    }
    // 找到目标index后，找link中的目标seg
    double min_link_dis = distance;
    for (const auto& link_id_pair : id_list) {
      if (link_id_pair.first != target_link_id) {
        continue;
      }
      const auto& current_seginfo =
          link_info_ptr->getLinkById(link_id_pair.first);
      const auto& line_segment =
          current_seginfo->segments()[link_id_pair.second];
      Vec2d map_point{};
      double dis = line_segment.DistanceTo(point, &map_point);
      if (dis < min_link_dis) {
        min_link_dis = dis;
        nearest_lane = &current_seginfo->getLink();
        nearest_s = current_seginfo->AccumulatedS()[link_id_pair.second] +
                    line_segment.start().DistanceTo(map_point);
        nearest_l = line_segment.unit_direction().CrossProd(
            point - line_segment.start());
        current_link_id_ = target_link_id;
        current_index_ = target_link_index;
      }
    }
    return nearest_lane;
  }
  return nearest_lane;
}

bool SDProMap::isOnRouteLinks(uint64_t road_seg_id) const {
  const RouteInfo::Ptr route_info_ptr = std::dynamic_pointer_cast<RouteInfo>(
      getMapInfoPtr(MapInfoType::ROUTE_INFO));
  if (!route_info_ptr) {
    return false;
  }

  return route_info_ptr->isOnRouteLinks(road_seg_id);
}

}  // namespace sdpromap
}  // namespace ad_common
