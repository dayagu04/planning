#pragma once

#include <memory>
#include <utility>
#include <vector>
#include "modules/context/virtual_lane.h"
#include "map_data.pb.h"
#include "common/utils/kd_path.h"

namespace planning {
namespace context {

// 位置关系类型（用于分段聚合）
enum class PositionRelation {
  BETWEEN_LINKS,      // 在两条link之间
  LEFT_OF_LINK1,      // 在link1左侧
  RIGHT_OF_LINK1,     // 在link1右侧
  LEFT_OF_LINK2,      // 在link2左侧
  RIGHT_OF_LINK2,     // 在link2右侧
  UNKNOWN             // 未知/远离
};

// 中心线分段结果
struct CenterlineSegment {
  double s_start;                // 起始s值（米）
  double s_end;                  // 结束s值（米）
  PositionRelation relation;     // 位置关系
};

// 中心线检查结果
struct CenterlineCheckResult {
  int overlap_start_idx = -1;                     // 中心线上第一个与任一link有overlap的点的索引，-1表示无overlap
  std::vector<CenterlineSegment> segments;        // 分段结果（相邻相同关系的点合并成一段）
  double s_ego = 0.0;                             // 自车在Frenet坐标系中的投影s值
};

/**
 * @brief 中心线与Link关系分析器
 *
 * 功能：
 * - 输入当前车道中心线（VirtualLane）和两条link
 * - 从中心线前方与两条link纵向范围有overlap的第一个点开始
 * - 只在前方110m范围内进行判断
 * - 判断该点往后每个点与两条link的位置关系
 * - 对结果进行分段聚合（相邻相同关系的点合并成一段）
 */
class CenterlineLinkAnalyzer {
 public:
  CenterlineLinkAnalyzer() = default;
  ~CenterlineLinkAnalyzer() = default;

  /**
   * @brief 检查中心线与两条link的位置关系
   *
   * @param current_lane 当前车道中心线
   * @param link1 第一条link
   * @param link2 第二条link
   * @param frenet_coord 当前车道中心线的Frenet坐标系
   * @param ego_x 自车当前位置x坐标（全局坐标系）
   * @param ego_y 自车当前位置y坐标（全局坐标系）
   * @param result 输出结果
   * @return true 检查成功，result有效
   * @return false 检查失败（参数无效或无overlap）
   */
  bool CheckCenterlineRelativeTwoLinks(
      const std::shared_ptr<VirtualLane>& current_lane,
      const iflymapdata::sdpro::LinkInfo_Link* link1,
      const iflymapdata::sdpro::LinkInfo_Link* link2,
      const std::shared_ptr<planning_math::KDPath>& frenet_coord,
      double ego_x,
      double ego_y,
      CenterlineCheckResult& result) const;

 private:
  // 将link的boot坐标系形点转换到车道中心线Frenet坐标系
  std::vector<std::pair<double, double>> TransformLinkBootToFrenet(
      const iflymapdata::sdpro::LinkInfo_Link* link,
      const std::shared_ptr<planning_math::KDPath>& frenet_coord) const;

  // 计算link在Frenet坐标系下的纵向s范围
  std::pair<double, double> CalculateLinkSRange(
      const std::vector<std::pair<double, double>>& frenet_link_points) const;

  // 判断中心线点相对于两条link的位置关系
  // 只对s值在link纵向范围内的link进行判断，通过Frenet l值插值考虑横向误差
  // 根据自车到查询点的纵向距离(s_query - s_ego)动态调整横向容忍度
  PositionRelation DeterminePositionRelation(
      const std::pair<double, double>& point,  // (s, l) in Frenet
      const std::vector<std::pair<double, double>>& link1_points,
      const std::vector<std::pair<double, double>>& link2_points,
      const std::shared_ptr<planning_math::KDPath>& frenet_coord,
      double s_ego) const;

  // 对判断结果进行分段聚合
  std::vector<CenterlineSegment> AggregateSegments(
      const std::vector<PositionRelation>& relations,
      const std::vector<double>& s_values) const;
};

}  // namespace context
}  // namespace planning
