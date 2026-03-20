#pragma once

#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "./mapinfo/lane_info/lane_info.h"
#include "./mapinfo/link_info/link_info.h"
#include "./mapinfo/route_info/route_info.h"
#include "./mapinfo/sdpromap_utils.hpp"

namespace ad_common {
namespace sdpromap {
using ad_common::math::Vec2d;

class SDProMap {
 public:
  int LoadMapFromProto(const iflymapdata::sdpro::MapData& map_proto);

  void BuildSegmentKdTree();

  const iflymapdata::sdpro::LinkInfo_Link* GetLinkOnRoute(
      uint64_t road_seg_id) const;

  const iflymapdata::sdpro::LinkInfo_Link* GetPreviousLinkOnRoute(
      uint64_t road_seg_id) const;

  const iflymapdata::sdpro::LinkInfo_Link* GetNextLinkOnRoute(
      uint64_t road_seg_id) const;

  std::shared_ptr<MapInfoBase> getMapInfoPtr(
      const MapInfoType& info_type) const;

  std::unordered_map<MapInfoType, std::vector<std::pair<uint64_t, uint64_t>>>
  SearchObjects(const Vec2d& center, double radius) const;

  std::vector<std::pair<uint64_t, uint64_t>> SearchObjects(
      const Vec2d& center, double& radius,
      const MapInfoType& map_info_type) const;

  std::vector<std::pair<uint64_t, uint64_t>> GetLinksWithHeading(
      const Vec2d& point, double distance, double central_heading,
      double max_heading_diff) const;

  const iflymapdata::sdpro::LinkInfo_Link* GetNearestLinkWithHeading(
      const Vec2d& point, double distance, double central_heading,
      double max_heading_diff, double& nearest_s, double& nearest_l,
      bool is_search_cur_link = true) const;

  std::vector<std::pair<double, double>> GetCurvatureList(
      uint64_t road_seg_id, double accumulated_s, double max_distance) const;

  const iflymapdata::sdpro::LinkInfo_Link* GetNearestRoad(
      const Vec2d& p, double& nearest_s, double& nearest_l) const;

  std::pair<const iflymapdata::sdpro::LinkInfo_Link*, double> GetRampInfo(
      uint64_t road_seg_id, double accumulated_s, double max_distance) const;

  std::vector<std::pair<const iflymapdata::sdpro::LinkInfo_Link*, double>>
  GetMergeInfoList(uint64_t road_seg_id, double accumulated_s,
                   double max_distance) const;

  std::vector<std::pair<const iflymapdata::sdpro::LinkInfo_Link*, double>>
  GetSplitInfoList(uint64_t road_seg_id, double accumulated_s,
                   double max_distance) const;

  std::vector<std::pair<const iflymapdata::sdpro::LinkInfo_Link*, double>>
  GetRoundAboutList(uint64_t road_seg_id, double accumulated_s,
                    const double& max_distance = 500) const;

  std::pair<const iflymapdata::sdpro::LinkInfo_Link*, double>
  GetTollStationInfo(uint64_t road_seg_id, double accumulated_s,
                     double max_distance) const;

  std::vector<uint64_t> getRouteLinksIds() const;

  inline bool isGoingOffCourse() const { return is_going_off_course; }

  inline int getCurRouteIndex() const { return current_index_; }

  std::pair<const iflymapdata::sdpro::LinkInfo_Link*, double> GetTunnelInfo(
      uint64_t road_seg_id, double accumulated_s, double max_distance) const;

  std::pair<const iflymapdata::sdpro::LinkInfo_Link*, double> GetRoundAboutInfo(
      uint64_t road_seg_id, double accumulated_s, double max_distance) const;

  std::pair<const iflymapdata::sdpro::LinkInfo_Link*, double> GetSaPaInfo(
      uint64_t road_seg_id, double accumulated_s, double max_distance) const;
  std::pair<const iflymapdata::sdpro::LinkInfo_Link*, double> GetNonExpressInfo(
      uint64_t road_seg_id, double accumulated_s, double max_distance) const;

  const iflymapdata::sdpro::Lane* GetLaneInfoByID(
      const uint64_t& road_seg_id) const;

  bool isRouteValid() const;

  int GetDistanceToRouteEnd(uint64_t road_seg_id, double accumulated_s,
                            double& dis_to_end) const;

  bool isRamp(const uint32_t& link_type) const;

  bool isRoundAbout(const uint32_t& link_type) const;

  bool isSaPa(const uint32_t& link_type) const;

  bool isTollStation(const uint32_t& link_type) const;

  bool isTunnel(const uint32_t& link_type) const;

  bool isNonExpress(const iflymapdata::sdpro::LinkClass& link_class) const;

  bool isProjOnSegment(const math::LineSegment2d& segment, const Vec2d& point,
                       double* proj_length = nullptr) const;
  bool isOnRouteLinks(uint64_t road_seg_id) const;

 private:
  std::unordered_map<MapInfoType, std::shared_ptr<MapInfoBase>> map_info_table_;
  std::shared_ptr<RouteInfo> route_info_ptr_ = nullptr;
  std::shared_ptr<LinkInfo> link_info_ptr_ = nullptr;
  std::shared_ptr<LaneInfo> lane_info_ptr_ = nullptr;
  bool is_noa_status_ = false;
  mutable uint64_t current_link_id_ = 0;
  mutable int current_index_ = -1;
  mutable std::vector<uint64_t> history_route_link_ids_;
  mutable bool is_going_off_course = false;
};

/// @brief 获取以弧度为单位的坡度值
/// @param slope 该点位置处的坡度，正值表示 上坡，负值表示下坡。单位：度*(10^2)
inline double GetSlopeInRad(int32_t slope) {
  double degrees = static_cast<double>(slope) / 100.0;
  return degrees * (M_PI / 180.0);
}

/// @brief 获取以1/m为单位的曲率值
/// @param curvature
/// 曲线半径(单位为：米)的倒数乘以10^6，正值表示左转，负值表示右转。
inline double GetCurvatureInReciprocalM(int32_t curvature) {
  return static_cast<double>(curvature) / 1e6;
}

}  // namespace sdpromap
}  // namespace ad_common
