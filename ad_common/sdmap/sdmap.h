#pragma once

#include <cmath>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>

#include "ehr_sdmap.pb.h"
#include "sdmap_utils.h"

namespace ad_common {
namespace sdmap {
// static_assert(__cplusplus >= 201402L, "requires at least c++14");
// #if __cplusplus >= 201703L
// #include <optional>
// using std::optional;
// #else
// #include <experimental/optional>
// using std::experimental::optional;
// #endif

using ad_common::math::Vec2d;

class SDMap {
 public:
  int LoadMapFromProto(const ::SdMapSwtx::SdMap &map_proto);

  /// @brief 根据id获得对应的道路段的信息
  /// @param road_seg_id
  /// @return  如果存在，返回道路段信息，否则返回nullptr
  const SdMapSwtx::Segment *GetRoadSegmentById(uint64_t road_seg_id) const;

  const SdMapSwtx::Segment *GetPreviousRoadSegment(uint64_t road_seg_id) const;

  const SdMapSwtx::Segment *GetNextRoadSegment(uint64_t road_seg_id) const;

  /// @brief 获取距自车距离最近且与自车朝向相同的道路段信息
  /// @param point 目标点
  /// @param distance 搜索半径
  /// @param central_heading 朝向
  /// @param max_heading_diff 允许的最大朝向误差
  /// @param nearest_s  目标点到结果道路段开头的累加距离
  /// @param nearest_l  目标点到结果道路段中心线的横向距离
  /// @return 距离目标点最近的道路段，如果不存在则返回nullptr
  const SdMapSwtx::Segment *GetNearestRoadWithHeading(
      const Vec2d &point, double distance, double central_heading,
      double max_heading_diff, double &nearest_s, double &nearest_l) const;

  /// @brief 获取距当前位置最近的匝道的信息
  /// @param road_seg_id 自车所在路段id
  /// @param accumulated_s 自车距所在路段起点的距离
  /// @param max_distance 最大搜索距离。
  /// @return pair.first  最近的匝道路段信息，如果不存在则返回nullptr
  /// @return pair.second 到最近的匝道的距离。 如果当前已经在匝道上，距离为0
  std::pair<const ::SdMapSwtx::Segment *, double> GetRampInfo(
      uint64_t road_seg_id, double accumulated_s, double max_distance) const;

  /**
   * @brief 获取匝道汇入主路的信息
   *
   * @return 返回匝道汇入主路的路段信息和距离。如果不存在则返回nullptr,0.
   * pair.first: 主路路段信息，pair.second：到该主路的距离
   */
  std::pair<const ::SdMapSwtx::Segment *, double> GetMainRoadMergeInfo(
      uint64_t road_seg_id, double accumulated_s, double max_distance) const;

  /// @brief 获取当前位置之后且在搜索距离以内的汇流点路段及到汇流点的距离
  /// @param road_seg_id 自车所在路段id
  /// @param accumulated_s 自车距所在路段起点的距离
  /// @param max_distance 最大搜索距离。
  /// @return 汇流点路段信息
  /// pair.first:汇流点所在路段，pair.second：到该汇流点的距离
  /// 如果当前路段起始处正是一个汇流点，则该路段会出现在结果的第一个，且到汇流点的距离为负数
  std::vector<std::pair<const ::SdMapSwtx::Segment *, double>> GetMergeInfoList(
      uint64_t road_seg_id, double accumulated_s, double max_distance) const;

  /// @brief 获取当前位置之后且在搜索距离以内的分流点路段及到分流点的距离
  /// @param road_seg_id 自车所在路段id
  /// @param accumulated_s 自车距所在路段起点的距离
  /// @param max_distance 最大搜索距离。
  /// @return 分流点路段信息
  /// pair.first:分流点所在路段，pair.second：到该分流点的距离
  std::vector<std::pair<const ::SdMapSwtx::Segment *, double>> GetSplitInfoList(
      uint64_t road_seg_id, double accumulated_s, double max_distance) const;

  /// @brief 获取距当前位置最近的隧道的信息
  /// @param road_seg_id 自车所在路段id
  /// @param accumulated_s 自车距所在路段起点的距离
  /// @param max_distance 最大搜索距离。
  /// @return pair.first  最近的隧道路段信息，如果不存在则返回nullptr
  /// @return pair.second 到最近的隧道的距离。 如果当前已经在隧道中，距离为0
  std::pair<const ::SdMapSwtx::Segment *, double> GetTunnelInfo(
      uint64_t road_seg_id, double accumulated_s, double max_distance) const;

  /// @brief 获取距当前位置最近的收费站的信息
  /// @param road_seg_id 自车所在路段id
  /// @param accumulated_s 自车距所在路段起点的距离
  /// @param max_distance 最大搜索距离。
  /// @return pair.first  最近的收费站路段信息，如果不存在则返回nullptr
  /// @return pair.second 到最近的收费站的距离。
  std::pair<const ::SdMapSwtx::Segment *, double> GetTollStationInfo(
      uint64_t road_seg_id, double accumulated_s, double max_distance) const;

  /// @brief 获取当前位置到导航终点的距离
  /// @param road_seg_id 自车所在路段id
  /// @param accumulated_s 自车距所在路段起点的距离
  /// @param dis_to_end 出参，表示当前位置到导航终点的距离。
  /// @return 0表示获取结果成功，否则表示获取失败
  int GetDistanceToRouteEnd(uint64_t road_seg_id, double accumulated_s,
                            double &dis_to_end) const;

  /// @brief 获取当前位置前方一定距离内的曲率信息
  /// @param road_seg_id 自车所在路段id
  /// @param accumulated_s 自车距所在路段起点的距离
  /// @param max_distance 最大搜索距离。
  /// @return 曲率信息列表[{d1,c1}, {d2,c2}, ... {dn,cn}]
  /// 列表中每个元素表示到当前位置的距离和这个位置的曲率值。
  /// 距离单位：米。曲率单位：1/米。正值表示左转，负值表示右转。
  std::vector<std::pair<double, double>> GetCurvatureList(
      uint64_t road_seg_id, double accumulated_s, double max_distance) const;

  /// @brief 获取当前位置前方一定距离内的红绿灯倒计时信息
  /// @param road_seg_id 自车所在路段id
  /// @param accumulated_s 自车距所在路段起点的距离
  /// @param max_distance 最大搜索距离。
  /// @return 带红绿灯倒计时信息的Segment列表[{traffic_light_seg_ptr, d}, ...]
  /// pair.first: 红绿灯所在路段，pair.second：到该红绿灯的距离
  /// 默认红绿灯在Segment的末尾
  std::vector<std::pair<const ::SdMapSwtx::Segment *, double>>
  GetTrafficLightCountDownInfoList(uint64_t road_seg_id, double accumulated_s,
                                   double max_distance) const;

  /// @brief 获取当前位置前方一定距离内的电子眼信息
  /// @param road_seg_id 自车所在路段id
  /// @param accumulated_s 自车距所在路段起点的距离
  /// @param max_distance 最大搜索距离。
  /// @return 带电子眼信息的Segment列表[{camera_seg_ptr, d}, ...]
  /// pair.first: 电子眼所在路段，pair.second：自车到该seg的距离
  /// 具体自车到电子眼的距离，可利用 NaviCameraExt.distance 字段
  /// 如果当前Segment上有电子眼，则该Segment会出现在结果的第一个，且距离为0
  std::vector<std::pair<const ::SdMapSwtx::Segment *, double>>
  GetCameraInfoList(uint64_t road_seg_id, double accumulated_s,
                    double max_distance) const;

  /// @brief 获取当前的高德限速信息
  std::optional<SdMapSwtx::NaviRoadInfo> GetNaviRoadInfo() const;

 private:
  void BuildSegmentKdTree();
  /// @brief 根据id找Segment在所有road_segment数组中的下标
  /// @param id
  /// @return 数组下标
  // std::optional<size_t> GetIndexBySegmentId(uint64_t id) const;
  MyOptional<size_t> GetIndexBySegmentId(uint64_t id) const;

  std::vector<std::pair<uint64_t, uint64_t>> SearchObjects(const Vec2d &center,
                                                           double radius) const;
  std::vector<std::pair<uint64_t, uint64_t>> GetLanesWithHeading(
      const Vec2d &point, double distance, double central_heading,
      double max_heading_diff) const;

 private:
  using RoadSegmentBox =
      ObjectWithAABox<RoadSegInfo, ad_common::math::LineSegment2d>;
  using RoadSegmentKDTree = ad_common::math::AABoxKDTree2d<RoadSegmentBox>;

  std::vector<RoadSegInfo> route_segms_;  // 导航路径上的路径线段集
  std::unordered_map<uint64_t, size_t> seg_id_to_index_;
  std::vector<RoadSegmentBox> road_segment_boxes_;
  std::unique_ptr<RoadSegmentKDTree> kdtree_;

  std::optional<SdMapSwtx::NaviRoadInfo> navi_road_info_;
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

// 是否为匝道
bool IsRamp(const SdMapSwtx::Segment &segment);

// 是否为高速路
bool IsHighWay(const SdMapSwtx::Segment &segment);

// 是否为高架路/城市快速路
bool IsElevated(const SdMapSwtx::Segment &segment);

}  // namespace sdmap
}  // namespace ad_common