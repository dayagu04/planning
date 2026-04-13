#include "modules/context/route_info_strategy/centerline_link_analyzer.h"
#include <algorithm>
#include <limits>
#include "common/utils/kd_path.h"
#include "library/geometry_lib/include/geometry_math.h"

namespace planning {
namespace context {

bool CenterlineLinkAnalyzer::CheckCenterlineRelativeTwoLinks(
    const std::shared_ptr<VirtualLane>& current_lane,
    const iflymapdata::sdpro::LinkInfo_Link* link1,
    const iflymapdata::sdpro::LinkInfo_Link* link2,
    const std::shared_ptr<planning_math::KDPath>& frenet_coord,
    double ego_x,
    double ego_y,
    CenterlineCheckResult& result) const {
  // 参数有效性检查
  if (current_lane == nullptr || link1 == nullptr || link2 == nullptr || frenet_coord == nullptr) {
    return false;
  }

  // 获取中心线点
  const auto& lane_points = current_lane->lane_points();
  if (lane_points.empty()) {
    return false;
  }

  // 将link形点从boot坐标系转换到车道中心线Frenet坐标系
  auto link1_frenet_points = TransformLinkBootToFrenet(link1, frenet_coord);
  auto link2_frenet_points = TransformLinkBootToFrenet(link2, frenet_coord);

  if (link1_frenet_points.empty() || link2_frenet_points.empty()) {
    return false;
  }

  // 计算link在Frenet坐标系下的纵向s范围
  auto link1_s_range = CalculateLinkSRange(link1_frenet_points);
  auto link2_s_range = CalculateLinkSRange(link2_frenet_points);

  // 取两条link的s范围交集
  double link_s_min = std::max(link1_s_range.first, link2_s_range.first);
  double link_s_max = std::min(link1_s_range.second, link2_s_range.second);

  // 初始化结果
  result.overlap_start_idx = -1;
  result.segments.clear();
  result.s_ego = 0.0;

  // 遍历中心线点，找到第一个与link有overlap的点
  int overlap_start_idx = -1;
  std::vector<double> s_values;
  std::vector<PositionRelation> relations;

  // 获取当前车辆位置在Frenet坐标系中的投影点s值
  double s_ego = 0.0;
  double l_ego = 0.0;
  if (!frenet_coord->XYToSL(ego_x, ego_y, &s_ego, &l_ego)) {
    return false;
  }
  result.s_ego = s_ego;

  // 限制在前方110m范围内（从当前车辆投影点开始）
  const double kMaxS = 110.0;
  const double s_max_range = s_ego + kMaxS;

  for (size_t i = 0; i < lane_points.size(); ++i) {
    const auto& ref_point = lane_points[i];

    // 将local_point(为boot坐标系)转换为Frenet坐标系的s值
    double s_frenet = 0.0;
    double l_frenet = 0.0;
    if (!frenet_coord->XYToSL(ref_point.local_point.x,
                               ref_point.local_point.y,
                               &s_frenet, &l_frenet)) {
      continue;
    }

    // 只考虑自车前方的点且在从当前投影点向前110m范围内
    if (s_frenet <= s_ego || s_frenet > s_max_range) {
      continue;
    }

    double s = s_frenet;

    // 检查是否与link有overlap
    if (s >= link_s_min && s <= link_s_max) {
      // 记录第一个overlap点的索引
      if (overlap_start_idx == -1) {
        overlap_start_idx = static_cast<int>(i);
      }

      // 中心线点在Frenet坐标系下的坐标 (s, l)
      std::pair<double, double> point(s, l_frenet);

      // 判断该点相对于两条link的位置关系
      PositionRelation relation = DeterminePositionRelation(
          point, link1_frenet_points, link2_frenet_points, frenet_coord, s_ego);

      s_values.push_back(s);
      relations.push_back(relation);
    }
  }

  // 如果没有找到overlap点，返回失败
  if (overlap_start_idx == -1 || s_values.empty()) {
    return false;
  }

  result.overlap_start_idx = overlap_start_idx;

  // 对判断结果进行分段聚合
  result.segments = AggregateSegments(relations, s_values);

  return true;
}

std::vector<std::pair<double, double>> CenterlineLinkAnalyzer::TransformLinkBootToFrenet(
    const iflymapdata::sdpro::LinkInfo_Link* link,
    const std::shared_ptr<planning_math::KDPath>& frenet_coord) const {
  std::vector<std::pair<double, double>> frenet_points;

  if (!link->has_points() || !link->points().has_boot() ||
      link->points().boot().points_size() == 0) {
    return frenet_points;
  }

  frenet_points.reserve(link->points().boot().points_size());

  for (int i = 0; i < link->points().boot().points_size(); ++i) {
    const auto& boot_pt = link->points().boot().points(i);

    Point2D cart_point{boot_pt.x(), boot_pt.y()};
    Point2D frenet_point;

    // 使用带范围检查的XYToSL，投影超出范围时会返回false
    if (frenet_coord->XYToSL(cart_point, frenet_point)) {
      frenet_points.emplace_back(frenet_point.x, frenet_point.y);
    }
  }

  return frenet_points;
}

std::pair<double, double> CenterlineLinkAnalyzer::CalculateLinkSRange(
    const std::vector<std::pair<double, double>>& frenet_link_points) const {
  if (frenet_link_points.empty()) {
    return {0.0, 0.0};
  }

  // link是折线，由多个形点按顺序连接而成
  // 取所有形点的s范围作为纵向范围（因为折线可能在多处有纵向投影）
  double s_min = std::numeric_limits<double>::max();
  double s_max = std::numeric_limits<double>::lowest();

  for (const auto& pt : frenet_link_points) {
    s_min = std::min(s_min, pt.first);
    s_max = std::max(s_max, pt.first);
  }

  return {s_min, s_max};
}

PositionRelation CenterlineLinkAnalyzer::DeterminePositionRelation(
    const std::pair<double, double>& point,  // (s, l) in Frenet
    const std::vector<std::pair<double, double>>& link1_points,
    const std::vector<std::pair<double, double>>& link2_points,
    const std::shared_ptr<planning_math::KDPath>& frenet_coord,
    double s_ego) const {
  // 感知车道线横向误差基础容忍度，随自车到查询点距离线性增大
  const double kBaseLateralTolerance = 0.0;
  const double kLateralTolerancePerMeter = 0.00;  // 每米增加0.02m容忍度

  double s_query = point.first;
  double l_query = point.second;
  double longitudinal_dist = s_query - s_ego;
  double lateral_tolerance = kBaseLateralTolerance + kLateralTolerancePerMeter * longitudinal_dist;

  // 在Frenet坐标系下，通过插值获取link在查询点s处的l值
  // 只有当s_query在link的s范围内时才有效，否则返回false
  auto interpolate_link_l_at_s = [](
      const std::vector<std::pair<double, double>>& link_pts,
      double s_q,
      double& out_l) -> bool {
    if (link_pts.empty()) return false;

    double s_min = std::numeric_limits<double>::max();
    double s_max = std::numeric_limits<double>::lowest();
    for (const auto& pt : link_pts) {
      s_min = std::min(s_min, pt.first);
      s_max = std::max(s_max, pt.first);
    }

    // 查询点s不在link纵向范围内，不参与判断
    if (s_q < s_min || s_q > s_max) return false;

    // 找到s_q所在的线段并插值
    for (size_t i = 0; i + 1 < link_pts.size(); ++i) {
      double s1 = link_pts[i].first, l1 = link_pts[i].second;
      double s2 = link_pts[i + 1].first, l2 = link_pts[i + 1].second;
      double seg_s_min = std::min(s1, s2);
      double seg_s_max = std::max(s1, s2);
      if (s_q >= seg_s_min && s_q <= seg_s_max) {
        double ds = s2 - s1;
        double t = (std::abs(ds) > 1e-6) ? (s_q - s1) / ds : 0.0;
        t = std::max(0.0, std::min(1.0, t));
        out_l = l1 + t * (l2 - l1);
        return true;
      }
    }

    // 兜底：使用s最近的端点
    double best_l = link_pts[0].second;
    double best_dist = std::abs(link_pts[0].first - s_q);
    for (const auto& pt : link_pts) {
      double d = std::abs(pt.first - s_q);
      if (d < best_dist) {
        best_dist = d;
        best_l = pt.second;
      }
    }
    out_l = best_l;
    return true;
  };

  double link1_l = 0.0, link2_l = 0.0;
  bool link1_valid = interpolate_link_l_at_s(link1_points, s_query, link1_l);
  bool link2_valid = interpolate_link_l_at_s(link2_points, s_query, link2_l);

  if (!link1_valid || !link2_valid) {
    return PositionRelation::UNKNOWN;
  }

  double diff1 = l_query - link1_l;  // 正值：查询点在link1左侧
  double diff2 = l_query - link2_l;  // 正值：查询点在link2左侧

  // 在两条link之间（l值夹在两者之间）
  if (diff1 * diff2 < 0.0) {
    return PositionRelation::BETWEEN_LINKS;
  }

  // 在某条link横向容忍范围内，视为BETWEEN_LINKS
  if (std::abs(diff1) <= lateral_tolerance || std::abs(diff2) <= lateral_tolerance) {
    return PositionRelation::BETWEEN_LINKS;
  }

  // 明确在两条link同侧，返回距离更近的
  if (diff1 > 0.0 && diff2 > 0.0) {
    return diff1 < diff2 ? PositionRelation::LEFT_OF_LINK1 : PositionRelation::LEFT_OF_LINK2;
  } else {
    return std::abs(diff1) < std::abs(diff2) ? PositionRelation::RIGHT_OF_LINK1
                                              : PositionRelation::RIGHT_OF_LINK2;
  }

  return PositionRelation::UNKNOWN;
}

std::vector<CenterlineSegment> CenterlineLinkAnalyzer::AggregateSegments(
    const std::vector<PositionRelation>& relations,
    const std::vector<double>& s_values) const {
  std::vector<CenterlineSegment> segments;

  if (relations.empty() || relations.size() != s_values.size()) {
    return segments;
  }

  // 遍历关系数组，合并相邻相同关系的点
  size_t i = 0;
  while (i < relations.size()) {
    PositionRelation current_relation = relations[i];
    double s_start = s_values[i];

    // 找到关系不同的点
    size_t j = i + 1;
    while (j < relations.size() && relations[j] == current_relation) {
      ++j;
    }

    double s_end = s_values[j - 1];

    // 创建分段
    CenterlineSegment segment;
    segment.s_start = s_start;
    segment.s_end = s_end;
    segment.relation = current_relation;
    segments.push_back(segment);

    i = j;
  }

  return segments;
}

}  // namespace context
}  // namespace planning
