#include "LD_route_info_strategy.h"

#include <iostream>
#include <utility>

#include "config/basic_type.h"
#include "environmental_model.h"
#include "src/library/geometry_lib/include/geometry_math.h"
#include "local_view.h"
#include "route_info_strategy.h"
#include "modules/context/route_info_strategy/centerline_link_analyzer.h"
#include "virtual_lane_manager.h"

namespace planning {
namespace {
constexpr double kEpsilon = 1.0e-4;
constexpr double kMinFrontSearchDis = 500.0;
}

LDRouteInfoStrategy::LDRouteInfoStrategy(
    const MLCDeciderConfig* config_builder,
    const planning::framework::Session* session)
    : RouteInfoStrategy(config_builder, session) {
  local_view_ = &session_->environmental_model().get_local_view();
}

void LDRouteInfoStrategy::Update(RouteInfoOutput& route_info_output) {
  local_view_ = &session_->environmental_model().get_local_view();
  route_info_output_.reset();
  avoid_link_merge_lane_id_vec_.clear();

  if (!UpdateLDMap()) {
    return;
  }

  if (!CalculateRouteInfo()) {
    if (route_info_output_.is_missed_navi_route) {
      route_info_output.is_missed_navi_route = true;
    }
    return;
  }

  if (!UpdateFeasibleLaneGraph()){
    return;
  }

  route_info_output = route_info_output_;
}

bool LDRouteInfoStrategy::UpdateFeasibleLaneGraph() {
  mlc_decider_scene_type_info_.reset();
  feasible_lane_graph_.lane_topo_groups.clear();

  MLCSceneTypeDecider();

  MLCSceneType mlc_scene_type = mlc_decider_scene_type_info_.mlc_scene_type;

  route_info_output_.mlc_decider_scene_type_info = mlc_decider_scene_type_info_;

  TopoLinkGraph feasible_lane_graph_after_topo_change_vec;
  switch (mlc_scene_type) {
    case SPLIT_SCENE: {
      if (!CalculateFeasibleLaneInRampScene(
              feasible_lane_graph_, feasible_lane_graph_after_topo_change_vec)) {
        return false;
      }
      break;
    }
    case NORMAL_SCENE: {
      if (!CalculateFeasibleLaneInNormalScene(feasible_lane_graph_)) {
        return false;
      }
      break;
    }
    case MERGE_SCENE: {
      if (!CalculateFeasibleLaneInMergeScene(
              feasible_lane_graph_, feasible_lane_graph_after_topo_change_vec)) {
        return false;
      }
      break;
    }
    case NONE_SCENE:
      break;
    case AVOID_MERGE:
    case AVOID_SPLIT:
      break;
  }

  CalculateAvoidMergeFeasibleLane(feasible_lane_graph_);

  // 增加处理在接近匝道时，feasible lane至少有2条车道可达ramp，其中一条是1分2的lane，则从feasible lane中移除这条lane
  if (mlc_scene_type == SPLIT_SCENE) {
    ProcessEraseFeasibleLaneForSplitScene(feasible_lane_graph_);
  }
  // 计算车道减少信息
  double search_distance = 500.0;
  search_distance =
      std::min(search_distance, route_info_output_.mlc_decider_scene_type_info
                                    .dis_to_link_topo_change_point);
  CalculateFrontMergePointInfo(search_distance);

  CalculateFeasibleLaneByMergePoint(feasible_lane_graph_);

  // 把拓扑变化点后面的拓扑信息加入到feasible lane中
  // 供 UpdateLCNumTask & CalLaneCost 使用
  if (!feasible_lane_graph_after_topo_change_vec.lane_topo_groups.empty()) {
      // 插入到最前面
      feasible_lane_graph_.lane_topo_groups.insert(
          feasible_lane_graph_.lane_topo_groups.begin(),
          feasible_lane_graph_after_topo_change_vec.lane_topo_groups.begin(),
          feasible_lane_graph_after_topo_change_vec.lane_topo_groups.end()
      );
  }

  return true;
}

VirtualLanesRouteCost LDRouteInfoStrategy::GetVirtualLaneCostOnRoute(
    const std::vector<std::shared_ptr<VirtualLane>>& relative_id_lanes) {
  // Calculate VirtualLaneRouteCost for each VirtualLane
  auto const& lane_cost =
      CalculateVirtualLaneRouteCost(relative_id_lanes, feasible_lane_graph_);
  return lane_cost;
}

bool LDRouteInfoStrategy::UpdateLDMap() {
  const auto& ld_map_info = local_view_->sdpro_map_info;
  const auto ld_map_info_current_timestamp = ld_map_info.header().timestamp();
  if (ld_map_info_current_timestamp != ld_map_info_updated_timestamp_) {
    int res = ld_map_.LoadMapFromProto(ld_map_info);
    if (res == 0) {
      ldmap_valid_ = true;
      ld_map_info_updated_timestamp_ = ld_map_info_current_timestamp;
    }
  }

  if (ld_map_info_current_timestamp - ld_map_info_updated_timestamp_ >
      kStaticMapOvertimeThreshold) {
    // 距离上一次更新时间超过阈值，则认为无效报错
    ldmap_valid_ = false;
    ILOG_ERROR << "error!!! because more than 20s no update hdmap!!!";
  }
  JSON_DEBUG_VALUE("sdpromap_valid_", ldmap_valid_)
  ILOG_INFO << "ldmap_valid_:" << ldmap_valid_;
  return ldmap_valid_;
}

bool LDRouteInfoStrategy::CalculateRouteInfo() {
  if (!ld_map_.isRouteValid()) {
    return false;
  }

  if (!CalculateCurrentLink()) {
    return false;
  }

  if (!IsInExpressWay()) {
    return false;
  }

  route_info_output_.is_on_ramp = ld_map_.isRamp(current_link_->link_type());
  route_info_output_.current_segment_passed_distance = ego_on_cur_link_s_;
  route_info_output_.is_update_segment_success = true;
  route_info_output_.is_in_tunnel = ld_map_.isTunnel(current_link_->link_type());
  route_info_output_.sum_dis_to_last_link_split_point =
      CalculateDisToLastLinkSplitPoint(current_link_);

  const auto& sdpro_map_info = local_view_->sdpro_map_info;
  route_info_output_.map_vendor = sdpro_map_info.data_source();

  merge_info_vec_.clear();
  split_info_vec_.clear();
  ramp_info_vec_.clear();

  CalculateMergeInfo();

  CalculateSplitInfo();

  // 根据前方的split信息判断，自车是否走错路了
  if (IsMissedNaviRoute()) {
    route_info_output_.reset();
    route_info_output_.is_missed_navi_route = true;
    return false;
  }

  // 一定要先计算split info，再计算ramp info
  CalculateRampInfo();

  CaculateDistanceToRoadEnd(current_link_, ego_on_cur_link_s_);

  CaculateDistanceToTollStation(current_link_, ego_on_cur_link_s_);

  CaculateDistanceToNOAEnd(current_link_, ego_on_cur_link_s_);

  return true;
}

void LDRouteInfoStrategy::UpdateLaneIsOnRouteLinkStatus(
    const std::shared_ptr<VirtualLane>& lane,
    const iflymapdata::sdpro::LinkInfo_Link* split_link) const{
  if (lane == nullptr) {
    return;
  }

  // 场景前置条件检查
  if (split_link == nullptr || split_link->successor_link_ids_size() != 2) {
    lane->set_route_on_link_status(RouteOnLinkStatus::UNKNOWN);
    return;
  }

  const auto& split_next_link = ld_map_.GetNextLinkOnRoute(split_link->id());
  if (split_next_link == nullptr) {
    lane->set_route_on_link_status(RouteOnLinkStatus::UNKNOWN);
    return;
  }

  const auto& out_link_id =
      split_link->successor_link_ids()[0] == split_next_link->id()
          ? split_link->successor_link_ids()[1]
          : split_link->successor_link_ids()[0];
  const auto& out_link = ld_map_.GetLinkOnRoute(out_link_id);
  if (out_link == nullptr) {
    lane->set_route_on_link_status(RouteOnLinkStatus::UNKNOWN);
    return;
  }

  context::CenterlineCheckResult result;
  if (!CheckCenterlineRelativeTwoLinks(lane, split_next_link, out_link, result)) {
    // 数据无效（无overlap等），状态不确定
    lane->set_route_on_link_status(RouteOnLinkStatus::UNKNOWN);
    return;
  }

  const RampDirection split_dir = CalculateSplitDirection(*split_link, ld_map_);
  if (split_dir == RampDirection::RAMP_NONE) {
    lane->set_route_on_link_status(RouteOnLinkStatus::UNKNOWN);
    return;
  }

  // 统计各位置关系的加权分数（近处权重高）
  const double kPerceptionRange = 120.0;
  const double kDecayFactor = 60.0;
  double on_route_weighted = 0.0;
  double total_weighted = 0.0;

  for (const auto& segment : result.segments) {
    double s_start = std::max(0.0, segment.s_start - result.s_ego);
    double s_end = std::max(0.0, segment.s_end - result.s_ego);
    s_start = std::min(s_start, kPerceptionRange);
    s_end = std::min(s_end, kPerceptionRange);

    // 对指数衰减函数做积分，替代中点近似，避免远处长segment权重被高估
    double weighted_len = kDecayFactor * (std::exp(-s_start / kDecayFactor) -
                                          std::exp(-s_end / kDecayFactor));
    total_weighted += weighted_len;

    bool is_on_route = false;
    if (split_dir == RampDirection::RAMP_ON_LEFT) {
      is_on_route = segment.relation == context::PositionRelation::BETWEEN_LINKS;
    } else {
      is_on_route = segment.relation == context::PositionRelation::RIGHT_OF_LINK1;
    }
    if (is_on_route) {
      on_route_weighted += weighted_len;
    }
  }

  if (total_weighted <= 0.1) {
    lane->set_route_on_link_status(RouteOnLinkStatus::UNKNOWN);
    return;
  }

  double distance_to_split = 100;
  for (const auto& split_info: split_info_vec_) {
    if (split_info.first->id() == split_link->id()) {
      distance_to_split = split_info.second;
    }
  }

  double on_route_ratio = on_route_weighted / total_weighted;
  const double kFarDist = 120.0, kNearDist = 60.0;
  const double kFarThreshold = 0.5, kNearThreshold = 0.7;
  double threshold = 0.6;
  if (distance_to_split >= kFarDist) {
    threshold = kFarThreshold;
  } else if (distance_to_split <= kNearDist) {
    threshold = kNearThreshold;
  } else {
    double t = (distance_to_split - kNearDist) / (kFarDist - kNearDist);
    threshold = kNearThreshold + t * (kFarThreshold - kNearThreshold);
  }

  lane->set_route_on_link_status(on_route_ratio > threshold
                                     ? RouteOnLinkStatus::ON_ROUTE
                                     : RouteOnLinkStatus::OFF_ROUTE);
}

void LDRouteInfoStrategy::UpdateLanesOrderOnSplitNextLink(
    const std::shared_ptr<VirtualLane>& cur_lane,
    const std::shared_ptr<VirtualLane>& left_lane,
    const std::shared_ptr<VirtualLane>& right_lane,
    const iflymapdata::sdpro::LinkInfo_Link* split_link) const{
  // 前置条件：split_info_vec_非空，且存在有效的split_next_link
  if (split_link == nullptr || split_link->successor_link_ids_size() != 2) {
    return;
  }
  const auto& split_next_link = ld_map_.GetNextLinkOnRoute(split_link->id());
  if (split_next_link == nullptr) {
    return;
  }

  // 收集split_next_link上有效的（正常行驶）lane，按sequence从大到小排列（左大右小）
  // 同时构��� sequence -> 从左向右序号 的映射
  const auto seq_lane_ids = BuildSeqLaneIds(split_next_link);
  if (seq_lane_ids.empty()) {
    return;
  }
  const int total_normal_lanes = static_cast<int>(seq_lane_ids.size());

  // 检查split_next_link的boot点是否可用
  if (!split_next_link->has_points() || !split_next_link->points().has_boot() ||
      split_next_link->points().boot().points_size() == 0) {
    return;
  }

  // 收集所有ON_ROUTE的virtual lane，对每条lane：
  // 1. 用lane的enu_point构建lane自己的Frenet坐标系
  // 2. 把split_next_link的boot点投影到lane的Frenet上，计算平均l值
  // 3. l值越小（越接近0），说明link和lane越重合；l值为正说明link在lane左侧
  struct LaneWithL {
    std::shared_ptr<VirtualLane> lane;
    double avg_l;  // link相对于lane的平均横向偏移
    double car_y;  // 用于冲突消歧
  };
  std::vector<LaneWithL> lanes_with_l;

  auto collect_lane = [&](const std::shared_ptr<VirtualLane>& vlane) {
    if (vlane == nullptr ||
        vlane->get_route_on_link_status() != RouteOnLinkStatus::ON_ROUTE) {
      return;
    }
    const auto& lane_pts = vlane->lane_points();
    if (lane_pts.size() < 3) {
      return;
    }

    // 用lane的enu_point构建lane的Frenet坐标系
    std::vector<planning_math::PathPoint> lane_path_points;
    for (const auto& pt : lane_pts) {
      if (std::isnan(pt.enu_point.x) || std::isnan(pt.enu_point.y)) {
        continue;
      }
      auto p = planning_math::PathPoint(pt.enu_point.x, pt.enu_point.y);
      if (!lane_path_points.empty()) {
        const auto& last = lane_path_points.back();
        if (planning_math::Vec2d(last.x() - p.x(), last.y() - p.y()).Length() < 1e-2) {
          continue;
        }
      }
      lane_path_points.emplace_back(p);
    }
    if (static_cast<int>(lane_path_points.size()) <
        planning_math::KDPath::kKDPathMinPathPointSize + 1) {
      return;
    }
    auto lane_frenet =
        std::make_shared<planning_math::KDPath>(std::move(lane_path_points));

    // 把split_next_link的boot点投影到lane_frenet上，计算平均l值
    double sum_l = 0.0;
    int cnt = 0;
    for (int i = 0; i < split_next_link->points().boot().points_size(); ++i) {
      const auto& bp = split_next_link->points().boot().points(i);
      double s = 0.0, l = 0.0;
      if (lane_frenet->XYToSL(bp.x(), bp.y(), &s, &l)) {
        sum_l += l;
        cnt++;
      }
    }
    if (cnt == 0) {
      return;
    }
    double avg_l = sum_l / cnt;

    // 计算car_y均值用于消歧
    double sum_y = 0.0;
    int cnt_y = 0;
    for (const auto& pt : lane_pts) {
      if (pt.car_point.x > 0.0 && pt.car_point.x < 200.0) {
        sum_y += pt.car_point.y;
        cnt_y++;
      }
    }
    double avg_car_y = cnt_y > 0 ? sum_y / cnt_y : 0.0;

    lanes_with_l.push_back({vlane, avg_l, avg_car_y});
  };

  collect_lane(cur_lane);
  collect_lane(left_lane);
  collect_lane(right_lane);

  if (lanes_with_l.empty()) {
    return;
  }

  // 按l值从小到大排序（l越小，link越靠右，说明lane越靠左）
  std::sort(lanes_with_l.begin(), lanes_with_l.end(),
            [](const LaneWithL& a, const LaneWithL& b) {
              // 如果l值差异小于0.5m，用car_y消歧
              if (std::abs(a.avg_l - b.avg_l) < 0.5) {
                // car_y差异小于0.3m，用relative_id消歧
                if (std::abs(a.car_y - b.car_y) < 0.3) {
                  return a.lane->get_relative_id() < b.lane->get_relative_id();
                }
                return a.car_y > b.car_y;
              }
              return a.avg_l < b.avg_l;
            });

  // 分配序号：从左到右为1, 2, 3...
  for (int i = 0; i < static_cast<int>(lanes_with_l.size()); ++i) {
    int order = i + 1;
    // 边界限制
    order = std::max(1, std::min(order, total_normal_lanes));
    lanes_with_l[i].lane->set_lane_order_on_split_next_link(order);
  }
}


bool LDRouteInfoStrategy::CalculateCurrentLink() {
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
  bool is_search_cur_link = true;
  current_link_ = ld_map_.GetNearestLinkWithHeading(
      current_point, search_distance, ego_heading_angle, max_heading_diff,
      temp_nearest_s, nearest_l, is_search_cur_link);
  if (!current_link_) {
    return false;
  }

  ego_on_cur_link_s_ = temp_nearest_s;
  ego_on_cur_link_l_ = nearest_l;
  return true;
}

bool LDRouteInfoStrategy::IsInExpressWay() {
  // 判断自车当前是否在高速或者高架上
  if (current_link_->link_class() ==
          iflymapdata::sdpro::LinkClass::LC_EXPRESSWAY ||
      current_link_->link_class() ==
          iflymapdata::sdpro::LinkClass::LC_CITY_EXPRESSWAY ||
      (current_link_->link_type() & iflymapdata::sdpro::LT_IC) != 0 ||
      (current_link_->link_type() & iflymapdata::sdpro::LT_JCT) != 0) {

    route_info_output_.is_ego_on_expressway = true;
    if (current_link_->link_class() ==
        iflymapdata::sdpro::LinkClass::LC_EXPRESSWAY) {
      route_info_output_.is_ego_on_expressway_hmi = true;
    } else {
      route_info_output_.is_ego_on_city_expressway_hmi = true;
    }
    route_info_output_.is_in_sdmaproad = true;
  } else {
    return false;
  }

  return true;
}

bool LDRouteInfoStrategy::CheckEgoPositionRelativeTwoLinks(
    const iflymapdata::sdpro::LinkInfo_Link* link1,
    const iflymapdata::sdpro::LinkInfo_Link* link2,
    EgoPositionResult& result) const{
  if (link1 == nullptr || link2 == nullptr || !local_view_->localization.position.position_boot.available) {
    return false;
  }

  // 初始化结果
  result = EgoPositionResult{};
  result.is_between_links = false;
  result.is_left_of_link1 = false;
  result.is_right_of_link1 = false;
  result.is_left_of_link2 = false;
  result.is_right_of_link2 = false;
  result.dist_to_link1 = -1.0;
  result.dist_to_link2 = -1.0;
  result.min_dist_to_link1 = -1.0;
  result.min_dist_to_link2 = -1.0;
  result.min_dist_idx_link1 = -1;
  result.min_dist_idx_link2 = -1;
  result.proj_ratio_link1 = -1.0;
  result.proj_ratio_link2 = -1.0;

  // 获取自车在boot坐标系下的位置
  Eigen::Vector2d ego_pos(
      local_view_->localization.position.position_boot.x,
      local_view_->localization.position.position_boot.y);

  // 获取两段link的形点（转换为boot坐标系）
  auto get_link_boot_points = [&](const iflymapdata::sdpro::LinkInfo_Link* link) -> std::vector<Eigen::Vector2d> {
    std::vector<Eigen::Vector2d> boot_points;
    if (link->has_points() && link->points().has_boot() &&
               link->points().boot().points_size() > 0) {
      boot_points.reserve(link->points().boot().points_size());
      for (int i = 0; i < link->points().boot().points_size(); ++i) {
        const auto& boot_pt = link->points().boot().points(i);
        boot_points.emplace_back(boot_pt.x(), boot_pt.y());
      }
    }
    return boot_points;
  };

  auto link1_points = get_link_boot_points(link1);
  auto link2_points = get_link_boot_points(link2);

  if (link1_points.empty() || link2_points.empty()) {
    return false;
  }

  // 判断自车相对于两段link的位置
  // 构造两条线段：link1的起点到终点，link2的起点到终点
  pnc::geometry_lib::LineSegment link1_seg(link1_points.front(), link1_points.back());
  pnc::geometry_lib::LineSegment link2_seg(link2_points.front(), link2_points.back());

  // 计算自车到两条线段的距离
  result.dist_to_link1 = pnc::geometry_lib::CalPoint2LineDist(ego_pos, link1_seg);
  result.dist_to_link2 = pnc::geometry_lib::CalPoint2LineDist(ego_pos, link2_seg);

  // 计算自车在每条link上的投影点参数（0到1之间表示在线段内，<0表示在线段起点之前，>1表示在线段终点之后）
  auto calc_projection_ratio = [](const Eigen::Vector2d& point,
                                   const pnc::geometry_lib::LineSegment& seg) -> double {
    Eigen::Vector2d seg_vec = seg.pB - seg.pA;
    Eigen::Vector2d pt_vec = point - seg.pA;
    double seg_len = seg_vec.norm();
    if (seg_len < 1e-6) {
      return 0.0;
    }
    return seg_vec.dot(pt_vec) / (seg_len * seg_len);
  };

  result.proj_ratio_link1 = calc_projection_ratio(ego_pos, link1_seg);
  result.proj_ratio_link2 = calc_projection_ratio(ego_pos, link2_seg);

  // 计算到link每个形点的距离
  auto calc_distances_to_all_points = [](const Eigen::Vector2d& ego_pos,
                                          const std::vector<Eigen::Vector2d>& link_points) {
    std::vector<double> distances;
    distances.reserve(link_points.size());
    for (const auto& pt : link_points) {
      distances.push_back((ego_pos - pt).norm());
    }
    return distances;
  };

  auto dists_to_link1_points = calc_distances_to_all_points(ego_pos, link1_points);
  auto dists_to_link2_points = calc_distances_to_all_points(ego_pos, link2_points);

  // 找到每个link的最小距离和对应的形点索引
  auto min_dist_link1_it = std::min_element(dists_to_link1_points.begin(), dists_to_link1_points.end());
  result.min_dist_to_link1 = *min_dist_link1_it;
  result.min_dist_idx_link1 = std::distance(dists_to_link1_points.begin(), min_dist_link1_it);

  auto min_dist_link2_it = std::min_element(dists_to_link2_points.begin(), dists_to_link2_points.end());
  result.min_dist_to_link2 = *min_dist_link2_it;
  result.min_dist_idx_link2 = std::distance(dists_to_link2_points.begin(), min_dist_link2_it);

  // 判断点在线段的哪一侧（使用叉积）
  auto is_left_of_line = [](const Eigen::Vector2d& point,
                           const pnc::geometry_lib::LineSegment& line) -> bool {
    Eigen::Vector2d line_vec = line.pB - line.pA;
    Eigen::Vector2d point_vec = point - line.pA;
    // 叉积：如果值为正，点在线的左侧；负值，在线的右侧
    double cross_product = line_vec.x() * point_vec.y() - line_vec.y() * point_vec.x();
    return cross_product > 0;
  };

  // 判断自车相对于两段link的位置
  bool left_of_link1 = is_left_of_line(ego_pos, link1_seg);
  bool left_of_link2 = is_left_of_line(ego_pos, link2_seg);

  if (left_of_link1 != left_of_link2) {
    // 自车在两条link之间，两侧标志均有效
    result.is_left_of_link1 = left_of_link1;
    result.is_right_of_link1 = !left_of_link1;
    result.is_left_of_link2 = left_of_link2;
    result.is_right_of_link2 = !left_of_link2;
  } else {
    // 自车在两条link同侧，只保留距离更近的link的标志位
    if (result.dist_to_link1 <= result.dist_to_link2) {
      result.is_left_of_link1 = left_of_link1;
      result.is_right_of_link1 = !left_of_link1;
      result.is_left_of_link2 = false;
      result.is_right_of_link2 = false;
    } else {
      result.is_left_of_link1 = false;
      result.is_right_of_link1 = false;
      result.is_left_of_link2 = left_of_link2;
      result.is_right_of_link2 = !left_of_link2;
    }
  }

  // 情况1：自车在link1和link2之间（横向位置在两条link之间）
  // 判断逻辑：
  // - 横向距离：到两条link的横向距离都小于阈值
  // - 侧向关系：自车在两条link之间（即相对于link1和link2的侧向关系相反）
  result.is_between_links = (result.is_left_of_link1 != result.is_left_of_link2);

  // 输出调试信息
  ILOG_DEBUG << "CheckEgoPositionRelativeTwoLinks: "
             << "ego_pos=(" << ego_pos.x() << "," << ego_pos.y() << "), "
             << "dist_to_link1=" << result.dist_to_link1 << ", "
             << "dist_to_link2=" << result.dist_to_link2 << ", "
             << "min_dist_to_link1=" << result.min_dist_to_link1 << " (idx=" << result.min_dist_idx_link1 << "), "
             << "min_dist_to_link2=" << result.min_dist_to_link2 << " (idx=" << result.min_dist_idx_link2 << "), "
             << "proj_ratio_link1=" << result.proj_ratio_link1 << ", "
             << "proj_ratio_link2=" << result.proj_ratio_link2 << ", "
             << "is_between_links=" << result.is_between_links << ", "
             << "is_left_of_link1=" << result.is_left_of_link1 << ", "
             << "is_right_of_link1=" << result.is_right_of_link1 << ", "
             << "is_left_of_link2=" << result.is_left_of_link2 << ", "
             << "is_right_of_link2=" << result.is_right_of_link2;

  if (result.is_between_links) {
    ILOG_DEBUG << "Ego is between link1 and link2";
  } else if (result.is_left_of_link1 || result.is_right_of_link1) {
    ILOG_DEBUG << "Ego is on same side, beside link1 (dist=" << result.dist_to_link1 << ")";
  } else {
    ILOG_DEBUG << "Ego is on same side, beside link2 (dist=" << result.dist_to_link2 << ")";
  }

  return true;
}

bool LDRouteInfoStrategy::IsMissedNaviRoute() const {
  // 判断自车是否在split点后50m内，且走错了路
  if (current_link_ == nullptr) {
    return true;
  }

  // 从current_link_往回找，寻找最近的split点（有2个后继的link）
  const double kCheckRangeAfterSplit = 50.0;
  double accumulated_distance = 0.0;

  const iflymapdata::sdpro::LinkInfo_Link* iter_link = current_link_;
  const iflymapdata::sdpro::LinkInfo_Link* split_link = nullptr;

  // 往回遍历，累加距离，直到找到split点或超出50m
  while (iter_link != nullptr) {
    // 检查当前link是否是split点
    if ((iter_link->successor_link_ids_size()) == 2 &&
        (iter_link->id() != current_link_->id())) {
      split_link = iter_link;
      break;
    }

    if (iter_link->id() == current_link_->id()) {
      accumulated_distance = accumulated_distance + ego_on_cur_link_s_;
    } else {
      accumulated_distance = accumulated_distance + iter_link->length() * 0.01;
    }

    if (accumulated_distance > kCheckRangeAfterSplit) {
      break;
    }

    // 往回走一个link
    const auto& prev_link = ld_map_.GetPreviousLinkOnRoute(iter_link->id());
    if (prev_link == nullptr) {
      break;
    }

    // 防止找到link_merge往后了
    if (prev_link->predecessor_link_ids_size() != 1) {
      break;
    }

    iter_link = prev_link;
  }

  // 没找到split点，或者距离超过50m
  if (split_link == nullptr) {
    return false;
  }

  // 找到了split点，且在50m范围内
  // 确定哪条分支能到达current_link_（route分支），另一条就是out分支
  if (split_link->successor_link_ids_size() != 2) {
    return false;
  }

  const auto& split_next_link = ld_map_.GetNextLinkOnRoute(split_link->id());
  if (split_next_link == nullptr) {
    return false;
  }

  const auto& out_link_id =
      split_link->successor_link_ids()[0] == split_next_link->id()
          ? split_link->successor_link_ids()[1]
          : split_link->successor_link_ids()[0];

  const auto& out_link = ld_map_.GetLinkOnRoute(out_link_id);
  if (out_link == nullptr)   {
    return false;
  }

  // 使用current_link_和out_branch判断位置关系
  EgoPositionResult ego_pos_result;
  if (!CheckEgoPositionRelativeTwoLinks(current_link_, out_link,
                                        ego_pos_result)) {
    return false;
  }

  const auto& split_dir = CalculateSplitDirection(*split_link, ld_map_);

  if (split_dir == RAMP_ON_LEFT) {
    if (ego_pos_result.is_right_of_link2) {
      return true;
    } else if (ego_pos_result.is_between_links) {
      // 由于自车位置与地图误差，有可能在这个条件下，也是走错的情况，后续根据测试情况继续完善
    }
  } else if (split_dir == RAMP_ON_RIGHT) {
    const double ego_map_lat_err = 1.2;
    if (ego_pos_result.is_left_of_link1 &&
        ego_pos_result.dist_to_link1 > ego_map_lat_err) {
      return true;
    } else if (ego_pos_result.is_between_links) {
      // 由于自车位置与地图误差，有可能在这个条件下，也是走错的情况，后续根据测试情况继续完善
    }
  }

  return false;
}
bool LDRouteInfoStrategy::CheckCenterlineRelativeTwoLinks(
    const std::shared_ptr<VirtualLane>& current_lane,
    const iflymapdata::sdpro::LinkInfo_Link* link1,
    const iflymapdata::sdpro::LinkInfo_Link* link2,
    context::CenterlineCheckResult& result) const {
  // 由于这部分可能在选道之前被调用，因此需要自己建立frenet坐标系
  auto& lane_points = current_lane->lane_points();
  if (lane_points.size() < 3) {
    return false;
  }
  std::shared_ptr<planning_math::KDPath> frenet_coord;
  std::vector<planning_math::PathPoint> path_points;
  path_points.reserve(lane_points.size());

  // point.local_point是boot系
  for (const auto& point : lane_points) {
    if (std::isnan(point.local_point.x) ||
        std::isnan(point.local_point.y)) {
      ILOG_ERROR << "update_lane_points: skip NaN point";
      continue;
    }
    auto pt = planning_math::PathPoint(point.local_point.x,
                                        point.local_point.y);
    if (!path_points.empty()) {
      auto& last_pt = path_points.back();
      if (planning_math::Vec2d(last_pt.x() - pt.x(), last_pt.y() - pt.y())
              .Length() < 1e-2) {
        continue;
      }
    }
    path_points.emplace_back(pt);
  }

  if (path_points.size() <
      planning_math::KDPath::kKDPathMinPathPointSize + 1) {
    return false;
  }

  frenet_coord =
      std::make_shared<planning_math::KDPath>(std::move(path_points));

  // 获取自车当前位置
  // ego_state->ego_pose()是boot系
  const auto& ego_state = session_->environmental_model().get_ego_state_manager();
  double ego_x = ego_state->ego_pose().x;
  double ego_y = ego_state->ego_pose().y;

  // 调用分析器进行判断
  return centerline_link_analyzer_.CheckCenterlineRelativeTwoLinks(
      current_lane, link1, link2, frenet_coord, ego_x, ego_y, result);
}

void LDRouteInfoStrategy::CalculateMLCDecider(
    const std::vector<std::shared_ptr<VirtualLane>>& relative_id_lanes,
    RouteInfoOutput& route_info_output) {
  if (relative_id_lanes.empty()) {
    return;
  }

  UpdateLCNumTask(relative_id_lanes, feasible_lane_graph_);

  // 如果有躲避前方link_merge的mlc，会更新mlc_decider_scene_type_info_.mlc_type，所以在这再次更新这个值
  route_info_output_.mlc_decider_scene_type_info = mlc_decider_scene_type_info_;
  route_info_output = route_info_output_;
}

bool LDRouteInfoStrategy::IsNearingSplit() {
  if (split_info_vec_.empty()) {
    return false;
  }

  bool is_near_split =
      split_info_vec_[0].second <
      mlc_decider_config_
          ->default_pre_triggle_road_to_ramp_distance_threshold_value;
  return is_near_split;
}

bool LDRouteInfoStrategy::IsNearingRamp() {
  if (ramp_info_vec_.empty()) {
    return false;
  }

  // 步骤1：确定目标匝道索引
  const size_t target_ramp_idx = GetTargetRampIndex();
  if (target_ramp_idx >= ramp_info_vec_.size()) {
    return false;
  }
  const auto& [target_link, dis_to_target_link] = ramp_info_vec_[target_ramp_idx];
  const auto& [front_first_link, dis_to_front_first_link] = ramp_info_vec_[0];
  if (target_link == nullptr || front_first_link == nullptr) {
    return false;
  }

  // 步骤2：判断是否接近匝道
  // const bool is_near_ramp = IsDistanceToRampWithinThreshold(dis_to_target_link);
  // if (!is_near_ramp) {
  //   return false;
  // }

  // 步骤3：无合流信息时，直接标记匝道场景并返回
  if (merge_info_vec_.empty()) {
    UpdateSceneInfo(*front_first_link, dis_to_front_first_link, *target_link);
    return true;
  }

  // 步骤4：有合流信息时，判断优先级（合流优先则返回false）
  if (IsMergePriorToRamp(dis_to_target_link)) {
    mlc_decider_scene_type_info_.reset();
    return false;
  }

  // 步骤5：匝道优先，更新决策信息并返回
  UpdateSceneInfo(*front_first_link, dis_to_front_first_link, *target_link);
  return true;
}


size_t LDRouteInfoStrategy::GetTargetRampIndex() {
  const size_t ramp_count = ramp_info_vec_.size();

  for (size_t ramp_idx = 0; ramp_idx < ramp_count; ++ramp_idx) {
    const auto& [ramp_link, ramp_dis] = ramp_info_vec_[ramp_idx];
    if (ramp_link == nullptr) {
      continue;
    }

    // 场景1：匝道距离≥最小搜索距离
    if (ramp_dis > kMinFrontSearchDis) {
      if (ramp_idx == 0) {
        return 0;
      }

      // 检查合流信息，若存在有效合流则取前一个匝道
      if (HasValidMergeBeforeRamp(ramp_dis) ||
          (ramp_info_vec_[ramp_idx].second -
           ramp_info_vec_[ramp_idx - 1].second) > 200.0) {
        return ramp_idx - 1;
      } else {
        return ramp_idx;
      }
    }

    // 场景2：匝道距离<最小搜索距离，检查合流信息
    if (HasValidMergeBeforeRamp(ramp_dis)) {
      return std::max(ramp_idx - 1, static_cast<size_t>(0));
    }
  }

  return ramp_count - 1;
}

bool LDRouteInfoStrategy::HasValidMergeBeforeRamp(const double ramp_dis) {
  if (merge_info_vec_.empty()) {
    return false; // 无合流/第一个匝道，无前置匝道可选
  }

  for (const auto& merge_info : merge_info_vec_) {
    const double merge_distance = merge_info.second;
    // 合流点距离 < 匝道距离 + 不忽略该合流
    const bool is_valid_merge = (merge_distance < ramp_dis - kEpsilon) &&
                                !IsIgnoreMerge(merge_info);
    if (is_valid_merge) {
      return true;
    }
  }
  return false;
}

bool LDRouteInfoStrategy::IsDistanceToRampWithinThreshold(const double dis_to_ramp) {
  if (!mlc_decider_config_) {
    return false;
  }
  return dis_to_ramp < mlc_decider_config_
      ->default_pre_triggle_road_to_ramp_distance_threshold_value;
}

bool LDRouteInfoStrategy::IsMergePriorToRamp(const double dis_to_ramp) {
  for (const auto& merge_info : merge_info_vec_) {
    const double merge_distance = merge_info.second;
    // 合流点距离 ≤ 匝道距离（带容差）且合流有效
    if (merge_distance <= dis_to_ramp + kEpsilon && !IsIgnoreMerge(merge_info)) {
      return true;
    }
  }
  return false;
}

void LDRouteInfoStrategy::UpdateSceneInfo(
    const iflymapdata::sdpro::LinkInfo_Link& front_first_link,
    const double dis_to_front_first_link,
    const iflymapdata::sdpro::LinkInfo_Link& target_link) {
  mlc_decider_scene_type_info_.set_value(
      SPLIT_SCENE, CalculateSplitDirection(front_first_link, ld_map_),
      dis_to_front_first_link, front_first_link.id(), target_link.id());
}

bool LDRouteInfoStrategy::IsNearingMerge() {
  if (merge_info_vec_.empty()) {
    return false;
  }

  for (const auto& merge_info: merge_info_vec_) {
    // bool dis_condition = merge_info.second < 500;

    bool is_ignore_merge = IsIgnoreMerge(merge_info);

    if (!is_ignore_merge) {
      mlc_decider_scene_type_info_.set_value(
          MERGE_SCENE, CalculateMergeDirection(*merge_info.first, ld_map_),
          merge_info.second, merge_info.first->id(), merge_info.first->id());
      return true;
    }
  }

  return false;
}

bool LDRouteInfoStrategy::IsTwoSplitClose() {
  if (split_info_vec_.size() < 2) {
    return false;
  }

  bool is_two_splits_close =
      split_info_vec_[1].second - split_info_vec_[0].second <
      mlc_decider_config_->split_split_gap_threshold;

  return is_two_splits_close;
}

void LDRouteInfoStrategy::MLCSceneTypeDecider() {
  if (IsNearingRamp()) {
    return;
  }

  if (!IsNearingMerge()) {
    mlc_decider_scene_type_info_.is_scene_info_valid = true;
    mlc_decider_scene_type_info_.mlc_scene_type = NORMAL_SCENE;
    mlc_decider_scene_type_info_.dis_to_link_topo_change_point = 3000.0;
  }
}

bool LDRouteInfoStrategy::CalculateFeasibleLaneGraph(
    TopoLinkGraph& feasible_lane_graph,
    const std::vector<iflymapdata::sdpro::Lane>& start_lane_vec,
    const iflymapdata::sdpro::LinkInfo_Link& target_link) {
  // -------------------------- 1. 初始化与输入校验
  feasible_lane_graph.lane_topo_groups.clear();
  if (start_lane_vec.empty()) {
    return false;
  }

  const auto& first_start_lane = start_lane_vec.front();
  const auto* start_link = ld_map_.GetLinkOnRoute(first_start_lane.link_id());
  if (!start_link) {
    return false;
  }

  std::vector<iflymapdata::sdpro::Lane> current_lane_vec = start_lane_vec;
  if (!SortLaneBaseSeq(current_lane_vec)) {
    return false;
  }

  // 关键变量：避免死循环（记录已处理的link ID，防止拓扑环导致无限遍历）
  std::unordered_set<uint64_t> processed_link_ids;
  bool is_target_found = false;
  const uint64_t target_link_id = target_link.id();

  // 增加计算front feasible distance
  double front_sum_distance = 0;

  // -------------------------- 2.遍历拓扑构建车道图
  const iflymapdata::sdpro::LinkInfo_Link* current_link = start_link;
  while (current_link != nullptr) {
    if (processed_link_ids.count(current_link->id()) > 0) {
      return false;
    }

    if (current_link->id() == current_link_->id()) {
      front_sum_distance = front_sum_distance + current_link->length() * 0.01 -
                           ego_on_cur_link_s_;
    } else {
      front_sum_distance = front_sum_distance + current_link->length() * 0.01;
    }

    processed_link_ids.insert(current_link->id());

    // -------------------------- 3.构建当前link的车道拓扑组
    LaneTopoGroup current_topo_group;
    current_topo_group.link_id = current_link->id();
    current_topo_group.lane_nums = current_link->lane_num();

    for (const auto& lane : current_lane_vec) {
      if (lane.link_id() != current_link->id()) {
        return false;
      }

      TopoLane topo_lane;
      topo_lane.id = lane.id();
      topo_lane.link_id = lane.link_id();
      topo_lane.order_id = lane.sequence();
      topo_lane.length = lane.length() * 0.01;
      topo_lane.front_feasible_distance = front_sum_distance;

      // 更新当前topo lane的前继车道
      const int pre_lane_count = lane.predecessor_lane_ids_size();
      if (pre_lane_count == 0) {
        topo_lane.predecessor_lane_ids.clear();
      } else if (pre_lane_count == 1) {
        topo_lane.predecessor_lane_ids.emplace(lane.predecessor_lane_ids()[0]);
      } else {

        // 处理合流车道：判断是否有效，无效则跳过 + 前继存在道路合流
        uint64_t link_id = lane.link_id();
        auto* link = ld_map_.GetLinkOnRoute(link_id);
        // 向前首个拓扑变化是否为合流
        bool pre_topo_change_is_merge = false;
        while (link != nullptr) {
          link_id = link->id();
          // 是否来自合流道路
          if (link->predecessor_link_ids_size() > 1){
            pre_topo_change_is_merge = true;
            break;
          }
          auto* pre_link = ld_map_.GetPreviousLinkOnRoute(link_id);
          if(pre_link == nullptr){
            break;
          }
          if(pre_link->successor_link_ids_size() > 1){
            pre_topo_change_is_merge = false;
            break;
          }
          link = pre_link;
        }
        for (const auto& pre_lane_id : lane.predecessor_lane_ids()) {
          const auto* pre_lane = ld_map_.GetLaneInfoByID(pre_lane_id);
          if (pre_lane == nullptr) {
            continue;
          }

          if (IsMergeLane(pre_lane) && pre_topo_change_is_merge) {
            if (IsInvalidLaneMergeLaneOppositeSide(pre_lane)) {
              continue;
              // merge lane的反方向是无效车道，既这条merge lane不应该在feasible lane中，continue掉，不加入到feasible lane中
            }
          }

          topo_lane.predecessor_lane_ids.emplace(pre_lane_id);
        }
      }

      topo_lane.successor_lane_ids.clear();
      for (int i = 0; i < lane.successor_lane_ids_size(); ++i) {
        topo_lane.successor_lane_ids.emplace(lane.successor_lane_ids()[i]);
      }

      current_topo_group.topo_lanes.emplace_back(std::move(topo_lane));
    }

    // -------------------------- 4. 检查目标link并更新输出
    if (current_link->id() == target_link_id) {
      is_target_found = true;
    }
    if (current_topo_group.topo_lanes.empty()) {
      return false;
    }
    feasible_lane_graph.lane_topo_groups.emplace_back(
        std::move(current_topo_group));

    if (is_target_found) {
      return true;
    }

    // ------------------------- 5. 获取下一个pre link并更新车道列表
    const auto* next_pre_link =
        ld_map_.GetPreviousLinkOnRoute(current_link->id());
    if (!next_pre_link) {
      return false;
    }

    std::vector<iflymapdata::sdpro::Lane> next_lane_vec;
    if (feasible_lane_graph.lane_topo_groups.empty()) {
      return false;
    }

    for (const auto& topo_lane :
         feasible_lane_graph.lane_topo_groups.back().topo_lanes) {
      if (topo_lane.predecessor_lane_ids.empty()) {
        continue;
      }
      //考虑到有多个前继车道，需要遍历所有前继车道
      for (const auto& pre_lane_id : topo_lane.predecessor_lane_ids) {
        const auto* pre_lane = ld_map_.GetLaneInfoByID(pre_lane_id);
        if (!pre_lane) {
          return false;
        }
        // 校验前继车道是否属于routelink的pre link
        if (pre_lane->link_id() == next_pre_link->id()) {
          HandleMainLinkPreLane(pre_lane, next_lane_vec);
        } else {
          // 需要处理前继lane不是在route link上的场景
          HandleOtherMergeLinkPreLane(topo_lane, next_pre_link, current_link,
                                      pre_lane, next_lane_vec);
        }
      }
    }
    if (next_lane_vec.empty()) {
      return false;
    }

    // -------------------------- 6. 更新迭代变量，进入下一轮循环
    current_lane_vec = std::move(next_lane_vec);
    current_link = next_pre_link;
  }

  return false;
}

bool LDRouteInfoStrategy::IsValidInputLanes(
    const iflymapdata::sdpro::LinkInfo_Link* link,
    const std::vector<iflymapdata::sdpro::Lane>& start_lane_vec) {
  if (start_lane_vec.empty()) {
    return false;
  }

  bool is_same_link = true;
  uint64 link_id;

  if (start_lane_vec.size() == 1) {
    link = ld_map_.GetLinkOnRoute(start_lane_vec[0].link_id());
    return true;
  }

  for (int i = 1; i < start_lane_vec.size(); i++) {
    if (start_lane_vec[i].link_id() != start_lane_vec[i - 1].link_id()) {
      return false;
    }
  }

  link = ld_map_.GetLinkOnRoute(start_lane_vec[0].link_id());
  return true;
}

bool LDRouteInfoStrategy::SortLaneBaseSeq(
    std::vector<iflymapdata::sdpro::Lane>& start_lane_vec) {
  if (start_lane_vec.empty()) {
    return false;
  }

  std::sort(start_lane_vec.begin(), start_lane_vec.end(),
            [](const iflymapdata::sdpro::Lane& lane_a,
               const iflymapdata::sdpro::Lane& lane_b) {
              return lane_a.sequence() < lane_b.sequence();
            });

  return true;
}

bool LDRouteInfoStrategy::CalculateExtenedFeasibleLane(
    TopoLinkGraph& before_split_feasible_lane_graph) {
  if (before_split_feasible_lane_graph.lane_topo_groups.empty()) {
    return false;
  }

  const int link_size =
      before_split_feasible_lane_graph.lane_topo_groups.size();
  double link_sum_dis = 0;

  // 计算向前回溯link过程中,是否存在车道偏移
  std::unordered_map<int, int> order_diff;
  IsPreLaneOnOtherLink(before_split_feasible_lane_graph, order_diff);

  const double extend_distance =
      current_link_->link_class() ==
              iflymapdata::sdpro::LinkClass::LC_EXPRESSWAY
          ? 800.
          : 500.;

  for (int i = 0; i < link_size; i++) {
    const auto& temp_link_topo =
        before_split_feasible_lane_graph.lane_topo_groups[i];

    const auto& temp_topo_lanes = temp_link_topo.topo_lanes;

    if (temp_topo_lanes.empty()) {
      return false;
    }

    const auto& temp_link = ld_map_.GetLinkOnRoute(temp_topo_lanes[0].link_id);
    if (temp_link == nullptr) {
      return false;
    }

    // 校验前面计算的feasible lane是否正确的,计算出feasible的范围
    uint32_t min_seq = temp_topo_lanes[0].order_id;
    uint32_t max_seq = temp_topo_lanes[0].order_id;
    for (const auto& temp_topo_lane : temp_topo_lanes) {
      if (temp_topo_lane.link_id != temp_link->id()) {
        return false;
      }
      if (temp_topo_lane.order_id < min_seq) {
        min_seq = temp_topo_lane.order_id;
      }

      if (temp_topo_lane.order_id > max_seq) {
        max_seq = temp_topo_lane.order_id;
      }
    }

    link_sum_dis = temp_topo_lanes[0].front_feasible_distance;

    for (const auto& lane_id : temp_link->lane_ids()) {
      const auto& temp_lane = ld_map_.GetLaneInfoByID(lane_id);
      if (temp_lane == nullptr) {
        continue;
      }

      // 根据车道类型判断车道是否有效
      if (IsInvalidLane(temp_lane)) {
        continue;
      }

      int lc_num = 0;
      uint32_t target_seq = 0;
      if (temp_lane->sequence() <= max_seq &&
          temp_lane->sequence() >= min_seq) {
        continue;
      } else if (temp_lane->sequence() < min_seq) {
        lc_num = min_seq - temp_lane->sequence();
        target_seq = min_seq;
      } else if (temp_lane->sequence() > max_seq) {
        lc_num = temp_lane->sequence() - max_seq;
        target_seq = max_seq;
      }

      // 计算从当前link到目标的整个路径上的实线长度
      double total_lsl_length = 0.0;

      // 找到target_seq对应的lane_id
      uint64_t target_lane_id = 0;
      for (const auto& lane_id : temp_link->lane_ids()) {
        const auto* lane = ld_map_.GetLaneInfoByID(lane_id);
        if (lane && lane->sequence() == target_seq) {
          target_lane_id = lane_id;
          break;
        }
      }

      if (target_lane_id == 0) {
        continue;
      }

      // 当前link上的实线长度
      total_lsl_length += CalculateLSLLengthBetweenLanes(
          temp_link, temp_lane->sequence(), target_seq);

      // 沿着target_lane向前追踪，累计后续link上的实线长度
      const auto* current_trace_lane = temp_lane;
      const auto* target_trace_lane = ld_map_.GetLaneInfoByID(target_lane_id);
      if (!target_trace_lane) continue;

      for (int j = i - 1; j >= 0; --j) {
        const auto& next_link_topo =
            before_split_feasible_lane_graph.lane_topo_groups[j];
        const auto* next_link = ld_map_.GetLinkOnRoute(next_link_topo.link_id);
        if (!next_link) break;

        // 找到当前车道在下一个link上的后继
        uint64_t next_lane_id = 0;
        for (const auto& succ_id : current_trace_lane->successor_lane_ids()) {
          const auto* succ_lane = ld_map_.GetLaneInfoByID(succ_id);
          if (succ_lane && succ_lane->link_id() == next_link->id()) {
            next_lane_id = succ_id;
            break;
          }
        }
        // 找到目标车道在下一个link上的后继
        uint64_t target_next_lane_id = 0;
        for (const auto& succ_id : target_trace_lane->successor_lane_ids()) {
          const auto* succ_lane = ld_map_.GetLaneInfoByID(succ_id);
          if (succ_lane && succ_lane->link_id() == next_link->id()) {
            target_next_lane_id = succ_id;
            break;
          }
        }

        if (next_lane_id == 0 || target_next_lane_id == 0) break;

        const auto* next_lane = ld_map_.GetLaneInfoByID(next_lane_id);
        const auto* target_next_lane =
            ld_map_.GetLaneInfoByID(target_next_lane_id);
        if (!next_lane || !target_next_lane) break;

        // 计算这个link上该车道的边界实线长度
        total_lsl_length += CalculateLSLLengthBetweenLanes(
            next_link, next_lane->sequence(), target_next_lane->sequence());

        // 更新追踪车道
        current_trace_lane = next_lane;
        target_trace_lane = target_next_lane;
      }

      const double lc_need_dis =
          extend_distance * std::fabs(lc_num) + total_lsl_length;

      if (link_sum_dis < lc_need_dis) {
        // 距离ramp点的距离小于变道需要的距离，因此该lane不需要加入进去
        continue;
      }

      if (!IsLaneSuccessorInPlannedRoute(temp_lane)) {
        continue;
      }

      if (IsLaneSuccessorIsMergeLane(temp_lane)) {
        continue;
      }

      TopoLane topo_lane;
      topo_lane.id = temp_lane->id();
      topo_lane.link_id = temp_lane->link_id();
      topo_lane.order_id = temp_lane->sequence();
      topo_lane.length = temp_lane->length() * 0.01;  // cm to m
      topo_lane.front_feasible_distance = link_sum_dis - lc_need_dis;

      // 把符合条件的lane更新到车道组里面去
      before_split_feasible_lane_graph
          .lane_topo_groups[i].link_id = temp_link->id();
      before_split_feasible_lane_graph
          .lane_topo_groups[i].lane_nums = temp_link->lane_num();
      before_split_feasible_lane_graph
          .lane_topo_groups[i]
          .topo_lanes.emplace_back(std::move(topo_lane));

    }
  }

  return true;
}

void LDRouteInfoStrategy::UpdateLCNumTask(
    const std::vector<std::shared_ptr<VirtualLane>>& relative_id_lanes,
    const TopoLinkGraph& feasible_lane_graph) {
  if (feasible_lane_graph.lane_topo_groups.empty()) {
    return;
  }

  const auto& cur_link_feasible_lane =
      feasible_lane_graph.lane_topo_groups.back();
  if (cur_link_feasible_lane.topo_lanes.empty()) {
    return;
  }

  // 计算出原始的order
  std::vector<std::pair<int, double>> origin_order_id_seq;
  origin_order_id_seq.reserve(cur_link_feasible_lane.topo_lanes.size());
  for (const auto& topo_lane : cur_link_feasible_lane.topo_lanes) {
    origin_order_id_seq.emplace_back(topo_lane.order_id,
                                     topo_lane.front_feasible_distance);
  }
  if (origin_order_id_seq.empty()) {
    return;
  }

  // 继续判断是否有导流区车道，如果有的话，需要更新origin_order_id_seq
  int diversion_lane_num = 0;
  int most_left_emergency_lane_num = 0;
  const int cur_lane_size = current_link_->lane_ids_size();
  for (int i = cur_lane_size - 1; i >= 0; i--) {
    const auto& lane_id = current_link_->lane_ids()[i];
    const auto& temp_lane = ld_map_.GetLaneInfoByID(lane_id);
    if (temp_lane == nullptr) {
      continue;
    }

    if (IsDiversionLane(temp_lane)) {
      diversion_lane_num++;
      const int origin_order_size = origin_order_id_seq.size();
      for (int j = origin_order_size - 1; j >= 0; j--) {
        if (origin_order_id_seq[j].first > temp_lane->sequence()) {
          origin_order_id_seq[j].first = origin_order_id_seq[j].first - 1;
        }
      }
    }

    // 默认紧急车道在最右侧，
    // 增加应急车道在左边时，计算车道从左向右序号的逻辑
    if (IsEmergencyLane(temp_lane) &&
        temp_lane->sequence() == cur_lane_size) {
      most_left_emergency_lane_num++;
    }
  }

  const int link_total_lane_num = cur_link_feasible_lane.lane_nums -
                                  diversion_lane_num -
                                  most_left_emergency_lane_num;

  if (link_total_lane_num < 1) {
    return;
  }

  std::vector<std::pair<int, double>> feasible_lane_seq;
  feasible_lane_seq.reserve(origin_order_id_seq.size());
  for (const auto& order_id : origin_order_id_seq) {
    // 把从右向左的顺序转换成从左向右的顺序
    const int seq = link_total_lane_num - order_id.first + 1;
    if (seq > 0) {
      feasible_lane_seq.emplace_back(seq, order_id.second);
    }
  }
  // 更新躲避link_merge车道的从左向右序列
  bool is_exist_need_avoid_link_merge_lane =
      !avoid_link_merge_lane_id_vec_.empty() &&
      avoid_link_merge_lane_id_vec_.front().link_id ==
          cur_link_feasible_lane.link_id;

  int avoid_link_merge_lane_seq = -1;
  if (is_exist_need_avoid_link_merge_lane) {
    avoid_link_merge_lane_seq = link_total_lane_num -
                                avoid_link_merge_lane_id_vec_.front().order_id +
                                1;
  }

  std::unordered_map<int, MapMergePointInfo> map_merge_point_info;
  for (const auto& merge_info: route_info_output_.map_merge_points_info) {
    // 把从右向左的顺序转换成从左向右的顺序
    const int seq = link_total_lane_num - merge_info.merge_lane_sequence + 1;
    map_merge_point_info.insert({seq, merge_info});
  }

  if (feasible_lane_seq.empty()) {
    return;
  }
  int minVal_seq = feasible_lane_seq[0].first;
  int maxVal_seq = feasible_lane_seq[0].first;

  std::vector<int> feasible_lane_seq_vec;
  feasible_lane_seq_vec.reserve(feasible_lane_seq.size());
  for (const auto& num : feasible_lane_seq) {
    if (num.first < minVal_seq) {
      minVal_seq = num.first;
    }
    if (num.first > maxVal_seq) {
      maxVal_seq = num.first;
    }
    feasible_lane_seq_vec.emplace_back(num.first);
  }

  route_info_output_.feasible_lane_sequence =
      std::move(feasible_lane_seq_vec);

  std::unordered_map<int, double> feasible_lane_seq_map;
  for (const auto& temp_feasible_lane_seq : feasible_lane_seq) {
    feasible_lane_seq_map.insert(
        {temp_feasible_lane_seq.first, temp_feasible_lane_seq.second});
  }

  for (auto relative_id_lane : relative_id_lanes) {
    // （fengwang31）TODO:后面把这个函数与route_info中的统一起来
    ProcessLaneDistance(relative_id_lane, feasible_lane_seq_map);
    ProcessLaneMapMergePoint(relative_id_lane, map_merge_point_info);
  }

  for (auto relative_id_lane : relative_id_lanes) {
    if (relative_id_lane == nullptr) {
      continue;
    }

    if (relative_id_lane->get_relative_id() != 0) {
      continue;
    }

    // 计算当前位置感知提供的车道数，当前感知提供的车道数是默认包含了右边的应急车道的
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

    int ego_seq = left_lane_num + 1 - most_left_emergency_lane_num;
    route_info_output_.left_lane_num = left_lane_num;
    route_info_output_.right_lane_num = right_lane_num;
    route_info_output_.ego_seq = ego_seq;

    int real_lane_num = link_total_lane_num;
    // 判断是否有应急车道、加速车道、入口车道
    bool cur_link_is_exist_emergency_lane = false;
    bool cur_link_is_exist_accelerate_lane = false;
    bool cur_link_is_exist_entry_lane = false;
    for (const auto& lane_id : current_link_->lane_ids()) {
      const auto& lane = ld_map_.GetLaneInfoByID(lane_id);
      if (lane == nullptr) {
        continue;
      }

      if (IsEmergencyLane(lane)) {
        // 先假设应急车道都在最右边上，因此seq=1
        if (lane->sequence() == 1) {
          real_lane_num = link_total_lane_num - 1;
        }
        cur_link_is_exist_emergency_lane = true;
      }

      if (IsAccelerateLane(lane)) {
        cur_link_is_exist_accelerate_lane = true;
      }

      if (IsEntryLane(lane)) {
        cur_link_is_exist_entry_lane = true;
      }
    }

    // 如果左、右侧观测车道数，大于道路车道数量。仅左观测比右侧大很多的时候，考虑用右侧观测。
    int const per_total_num = left_lane_num + right_lane_num + 1;
    int const map_total_num =
        cur_link_feasible_lane.lane_nums - diversion_lane_num;

    if (std::abs(per_total_num - map_total_num) > 2) {
      continue;
    }

    if (per_total_num > map_total_num) {
      if (left_lane_num > right_lane_num + 1) {
        ego_seq = link_total_lane_num - right_lane_num;
      }
    }

    // 如果自车左边的车道数大于等于link的总车道数，则认为左侧车道数误检，直接return;
    bool is_per_left_lane_error =
        (left_lane_num - most_left_emergency_lane_num) >= real_lane_num;

    // 如果当前link上只有1条车道，则不需要再触发导航变道。
    bool is_only_one_lane_on_cur_link = real_lane_num == 1;

    if (is_per_left_lane_error || is_only_one_lane_on_cur_link) {
      if ((cur_link_is_exist_emergency_lane && right_lane_num <= 1) ||
          (!cur_link_is_exist_emergency_lane && right_lane_num == 0)) {
        // 当左侧的观测数量大于总车道数时，若右侧车道数量小于一定值时，此时认为我们在地图的最右侧车道。
        ego_seq = real_lane_num;
        route_info_output_.ego_seq = ego_seq;
      } else {
        return;
      }
    }

    if (1 > ego_seq) {
      ego_seq = 1;
      route_info_output_.ego_seq = ego_seq;
    }
    // 当自车位于消亡车道上时，当前处理的场景修改成merge_sence
    for (const auto& merge_point_info :
         route_info_output_.map_merge_points_info) {
      const auto& merge_point_link =
          ld_map_.GetLinkOnRoute(merge_point_info.merge_lane_link_id);
      if (merge_point_link == nullptr) {
        break;
      }
      if (route_info_output_.ego_seq ==
          merge_point_link->lane_num() - merge_point_info.merge_lane_sequence +
              1) {
        mlc_decider_scene_type_info_.mlc_scene_type = MERGE_SCENE;
      }
    }
    // 当自车位于加速车道时，认为是汇入主路
    route_info_output_.is_ego_on_accelerate_lane = false;
    for (const auto& lane_id : current_link_->lane_ids()) {
      const auto& lane = ld_map_.GetLaneInfoByID(lane_id);
      if (lane == nullptr) {
        continue;
      }
      bool is_ego_on_this_lane =
          route_info_output_.ego_seq ==
          current_link_->lane_num() - lane->sequence() + 1;
      bool is_accelerate_lane = IsAccelerateLane(lane) || IsEntryLane(lane);
      bool is_leftest = false;
      bool is_rightest = false;
      if (is_accelerate_lane && is_ego_on_this_lane &&
          IsTheLaneOnSide(lane, is_leftest, is_rightest)) {
        route_info_output_.is_ego_on_accelerate_lane = true;
        break;
      }
    }

    std::vector<int> lc_num_task;

    bool is_nearing_ramp = false;
    const bool is_exist_ramp =
        !ramp_info_vec_.empty() && !split_info_vec_.empty();
    if (is_exist_ramp) {
      const auto& first_ramp = ramp_info_vec_[0].first;
      const auto& first_split = split_info_vec_[0].first;

      if (first_ramp && first_split) {
        const auto& ego_velocity =
            session_->environmental_model().get_ego_state_manager()->ego_v();
        const auto& ramp_distance =
            route_info_output_.mlc_decider_scene_type_info
                .dis_to_link_topo_change_point;
        const int kPredictTimeHorizon = 4;
        const double kMinFrontJudgeDis = 50.0;

        bool type_flag =
            route_info_output_.mlc_decider_scene_type_info.mlc_scene_type ==
            SPLIT_SCENE;
        bool ramp_split_flag = first_ramp->id() == first_split->id();
        bool distance_flag =
            ramp_distance <
            std::max(kMinFrontJudgeDis, ego_velocity * kPredictTimeHorizon);

        is_nearing_ramp = type_flag && ramp_split_flag && distance_flag;

      }
    }
    // 1. 识别当前Relative==0的可变车道，是不是在导航路线上，如果在，那么就不需要进行变道。
    // 2. 取感知前方S米的车道数量，和地图位置进行对比，如果Total、左、右数量均一致，保持当前车道。
    bool current_on_route = false;

    // 用感知车道数量变化点来检查。
    std::vector<float> check_dist_vec;
    for (size_t i = 0; i < lane_nums.size(); ++i) {
      if (lane_nums[i].end < 10.0 || lane_nums[i].begin < 0.0) {
        continue;  // 避开近距离的检查
      }
      if (lane_nums[i].begin > 60.0) {
        break;  // 距离太远的也不检查
      }

      check_dist_vec.push_back(lane_nums[i].begin + 1e-3);
    }

    for (float const check_dist : check_dist_vec) {
      auto find_it = std::find_if(lane_nums.begin(), lane_nums.end(),
                                  [check_dist](const auto& lane_num) {
                                    return lane_num.end > check_dist;
                                  });
      // 感知给出的前方车道数量
      int per_forward_left_num = 0;
      int per_forward_right_num = 0;
      if (find_it != lane_nums.end()) {
        per_forward_left_num = find_it->left_lane_num;
        per_forward_right_num = find_it->right_lane_num;
      } else {
        // 该距离找不到感知的车道数
        current_on_route = false;
        break;
      }

      // 该位置的FeasibleLane
      std::vector<planning::LaneTopoGroup> const& feasible_lane_topo =
          feasible_lane_graph.lane_topo_groups;

      if (feasible_lane_topo.back().link_id != current_link_->id()) {
        // 先不处理前后不一致的情况
        current_on_route = false;
        break;
      }

      double total_length = 0.0;
      // 我们考虑目标位置前后误差距离的所有车道
      double const error_dist = 10.0;

      // std::pair<TotalLaneNum , FeasibleOrder List>
      std::vector<std::pair<int,std::vector<uint32>>> feasible_order;
      for (auto it = feasible_lane_topo.crbegin();
           it != feasible_lane_topo.crend(); ++it) {

        auto link_ptr = ld_map_.GetLinkOnRoute(it->link_id);

        total_length += link_ptr->length() *0.01;

        if (feasible_lane_topo.crbegin() == it) {
          total_length -= ego_on_cur_link_s_;
        }

        if (check_dist - error_dist <= total_length) {
          std::pair<int,std::vector<uint32>> tmp_pair;

          std::vector<uint32> invalid_order;
          for (uint64 const lane_id : link_ptr->lane_ids())
          {
            auto const* lane_ptr = ld_map_.GetLaneInfoByID(lane_id);
            if (IsDiversionLane(lane_ptr))
            {
              invalid_order.emplace_back(lane_ptr->sequence());
            }
          }

          tmp_pair.first = it->lane_nums - invalid_order.size();
          for (auto const& tmp_lane : it->topo_lanes) {
            size_t const invalid_num =
                std::count_if(invalid_order.begin(), invalid_order.end(),
                              [&](int order_id) {
                                return  tmp_lane.order_id > order_id;
                              });
            tmp_pair.second.emplace_back(tmp_lane.order_id - invalid_num);
          }
          feasible_order.emplace_back(tmp_pair);

          if (check_dist + error_dist <= total_length) {
            break;
          }
        }
      }

      // 用check_dist位置的感知车道数量，和目标车道order进行比较
      // 如果检查到数据一致，那么认为自车在导航目标车道上，无需进行变道
      current_on_route = false;
      for (std::pair<int, std::vector<uint32>> const& tmp_order : feasible_order) {
        if (per_forward_left_num + per_forward_right_num + 1 ==
                tmp_order.first &&
            tmp_order.second.end() != std::find(tmp_order.second.begin(),
                                                tmp_order.second.end(),
                                                per_forward_right_num + 1)) {
          current_on_route = true;
          break;
        }
      }
      // 若任一检查不通过时，不再检查。
      if (!current_on_route) {
        break;
      }
    }

    // It is necessary to check whether the ego vehicle is already in the
    // leftmost or rightmost lane within a certain distance ahead. If it is
    // already in that direction, then no lane change will be performed.
    bool forward_in_left_most = false;
    bool forward_in_right_most = false;

    // Normally, it takes about 2 seconds for a vehicle to complete a lane
    // change from initiation to crossing the lane boundary.
    double const ego_v =
        session_->environmental_model().get_ego_state_manager()->ego_v();
    double const check_distance = std::fmax(ego_v * 2.0, 20.0);
    for (iflyauto::LaneNumMsg const& lane_num : lane_nums) {
      if (lane_num.begin <= check_distance && check_distance <= lane_num.end) {
        forward_in_left_most = lane_num.left_lane_num == 0;
        forward_in_right_most = lane_num.right_lane_num == 0;
        break;
      }
    }

    bool cur_lane_on_route_link_base_map_link =
        IsCurrentLaneOnRouteLink(feasible_lane_graph);

    // const bool lane_type_condition =
    //     !cur_link_is_exist_accelerate_lane && !cur_link_is_exist_entry_lane;
    const bool lane_type_condition = true;

    RampDirection front_ramp_dir = route_info_output_.ramp_direction;

    if (maxVal_seq == minVal_seq && maxVal_seq == real_lane_num &&
        !forward_in_right_most && is_nearing_ramp && lane_type_condition &&
        front_ramp_dir == RAMP_ON_RIGHT && !current_on_route &&
        !cur_lane_on_route_link_base_map_link) {
      //split场景，目标车道在最右边的情况，一直向右变道
      // 右边有加速车道或入口车道则需要至少留一个车道
        // 如果是躲避前方merge触发的mlc，则更新mlc_scene_type
      if (ego_seq == avoid_link_merge_lane_seq) {
        mlc_decider_scene_type_info_.mlc_scene_type = AVOID_MERGE;
      }

      lc_num_task.emplace_back(1);
    } else if (maxVal_seq == minVal_seq && maxVal_seq == 1 && is_nearing_ramp &&
               !current_on_route && !forward_in_left_most &&
               !cur_lane_on_route_link_base_map_link) {
      //split场景，目标车道在最左边的情况，一直向左变道
        // 如果是躲避前方merge触发的mlc，则更新mlc_scene_type
      if (ego_seq == avoid_link_merge_lane_seq) {
        mlc_decider_scene_type_info_.mlc_scene_type = AVOID_MERGE;
      }

      lc_num_task.emplace_back(-1);
    } else if (current_on_route || cur_lane_on_route_link_base_map_link) {
      continue;
    } else {
      if (ego_seq >= minVal_seq && ego_seq <= maxVal_seq) {
        continue;
      } else if (ego_seq > maxVal_seq) {
        // 如果是躲避前方merge触发的mlc，则更新mlc_scene_type
        if (ego_seq == avoid_link_merge_lane_seq) {
          mlc_decider_scene_type_info_.mlc_scene_type = AVOID_MERGE;
        }
        // 更新lc_num_task
        int err = ego_seq - maxVal_seq;
        for (int i = 0; i < err; i++) {
          lc_num_task.emplace_back(-1);
        }
      } else if (ego_seq < minVal_seq) {
        // 如果是躲避前方merge触发的mlc，则更新mlc_scene_type
        if (ego_seq == avoid_link_merge_lane_seq) {
          mlc_decider_scene_type_info_.mlc_scene_type = AVOID_MERGE;
        }
        // 更新lc_num_task
        int err = minVal_seq - ego_seq;
        for (int i = 0; i < err; i++) {
          lc_num_task.emplace_back(1);
        }
      }
    }

    if (lc_num_task.empty()) {
      return;
    }

    relative_id_lane->set_current_tasks(lc_num_task);
  }

  return;
}

bool LDRouteInfoStrategy::IsCurrentLaneOnRouteLink(
    const TopoLinkGraph& feasible_lane_graph) const {
  const auto& virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto& cur_lane = virtual_lane_manager->get_current_lane();
  const auto& left_lane = virtual_lane_manager->get_left_lane();
  const auto& right_lane = virtual_lane_manager->get_right_lane();
  const iflymapdata::sdpro::LinkInfo_Link* split_link = nullptr;
  const iflymapdata::sdpro::LinkInfo_Link* split_next_link = nullptr;
  if (split_info_vec_.empty() || cur_lane == nullptr) {
    return false;
  }

  if (split_info_vec_.front().second > 150.0) {
    return false;
  }

  if (split_info_vec_.front().first == nullptr) {
    return false;
  }

  split_link = ld_map_.GetLinkOnRoute(split_info_vec_.front().first->id());

  if (split_link == nullptr) {
    return false;
  }

  split_next_link = ld_map_.GetNextLinkOnRoute(split_link->id());
  if (split_next_link == nullptr) {
    return false;
  }

  UpdateLaneIsOnRouteLinkStatus(cur_lane, split_link);
  UpdateLaneIsOnRouteLinkStatus(left_lane, split_link);
  UpdateLaneIsOnRouteLinkStatus(right_lane, split_link);
  UpdateLanesOrderOnSplitNextLink(cur_lane, left_lane, right_lane, split_link);

  // 与 UpdateLanesOrderOnSplitNextLink 保持一致：只统计正常车道，建立
  // sequence → 从左向右序号（1=最左）的映射
  const auto seq_lane_ids = BuildSeqLaneIds(split_next_link);

  bool cur_lane_on_split_next_link_feasible_lane = false;
  const int cur_lane_order = cur_lane->get_lane_order_on_split_next_link();
  if (cur_lane_order > 0 && !seq_lane_ids.empty()) {
    for (const auto& lane_topo_group : feasible_lane_graph.lane_topo_groups) {
      if (lane_topo_group.link_id != split_next_link->id()) {
        continue;
      }
      for (const auto& topo_lane : lane_topo_group.topo_lanes) {
        // 用与 UpdateLanesOrderOnSplitNextLink 相同的 sequence→序号 映射转换
        const int left_to_right_order =
            SeqToOrder(seq_lane_ids, static_cast<int>(topo_lane.order_id));
        if (left_to_right_order == cur_lane_order) {
          cur_lane_on_split_next_link_feasible_lane = true;
          break;
        }
      }
      break;
    }
  }

  return cur_lane_on_split_next_link_feasible_lane &&
         cur_lane->get_route_on_link_status() == RouteOnLinkStatus::ON_ROUTE;
}

bool LDRouteInfoStrategy::CalculateFrontTargetLinkBaseFixDis(
    iflymapdata::sdpro::LinkInfo_Link* target_link,
    std::vector<iflymapdata::sdpro::Lane>& start_lane_vec,
    const iflymapdata::sdpro::LinkInfo_Link* cur_link,
    const MLCSceneType scene) {
  if (cur_link == nullptr) {
    return false;
  }

  double front_search_dis = 500.0;
  double sum_dis = 0.0;
  if (scene == NORMAL_SCENE) {
    sum_dis = cur_link->length() * 0.01 - ego_on_cur_link_s_;
    front_search_dis = 2000.0;
    // 进入normal场景，有2种情况，
    // 1、不在nearing ramp场景，前面3km内都没有ramp和需要并入的merge，这种情况下向前搜索2000m是合理的
    // 2、不在nearing merge场景,前面500m内，没有需要并入的merge,只能保证在500m内是normal。
    // 此时向前搜索的距离应该在500m内才是合理的，因此需要在此判断更新这个向前搜索的距离值。
    if (!merge_info_vec_.empty()) {
      for (const auto& merge_info : merge_info_vec_) {
        if (!IsIgnoreMerge(merge_info) &&
            merge_info.second <
                mlc_decider_config_
                    ->default_pre_triggle_road_to_ramp_distance_threshold_value) {
          front_search_dis = 500.0;
        }
      }
    }
  } else {
    sum_dis = cur_link->length() * 0.01;
  }
  const iflymapdata::sdpro::LinkInfo_Link* temp_link = cur_link;

  if (temp_link == nullptr) {
    return false;
  }

  while (sum_dis < front_search_dis) {
    const auto& temp_next_link = ld_map_.GetNextLinkOnRoute(temp_link->id());
    if (temp_next_link == nullptr) {
      break;
    }

    // 只在ODD范围内找target_link
    if (!(temp_next_link->link_class() ==
              iflymapdata::sdpro::LinkClass::LC_EXPRESSWAY ||
          temp_next_link->link_class() ==
              iflymapdata::sdpro::LinkClass::LC_CITY_EXPRESSWAY ||
          (temp_next_link->link_type() & iflymapdata::sdpro::LT_IC) != 0 ||
          (temp_next_link->link_type() & iflymapdata::sdpro::LT_JCT) != 0)) {
      break;
    }

    temp_link = temp_next_link;
    sum_dis = sum_dis + temp_next_link->length() * 0.01;
  }

  for (const auto& lane_id : temp_link->lane_ids()) {
    const iflymapdata::sdpro::Lane* lane_info =
        ld_map_.GetLaneInfoByID(lane_id);
    if (lane_info == nullptr || IsEmergencyLane(lane_info)) {
      continue;
    }
    start_lane_vec.emplace_back(*lane_info);
  }

  if (start_lane_vec.empty()) {
    return false;
  }

  *target_link = *temp_link;
  return true;
}

bool LDRouteInfoStrategy::CalculateMergePreFeasibleLane(
    std::vector<iflymapdata::sdpro::Lane>& merge_pre_link_lane_vec,
    const TopoLinkGraph& feasible_lane_graph,
    const iflymapdata::sdpro::LinkInfo_Link* merge_link) {
  if (merge_link == nullptr) {
    return false;
  }

  const auto& merge_pre_link = ld_map_.GetPreviousLinkOnRoute(merge_link->id());
  if (merge_pre_link == nullptr) {
    return false;
  }

  const auto& merge_link_feasible_lane =
      feasible_lane_graph.lane_topo_groups.back();

  for (const auto& feasible_lane : merge_link_feasible_lane.topo_lanes) {
    if (feasible_lane.predecessor_lane_ids.empty()) {
      continue;
    }
    const auto lane_info =
        ld_map_.GetLaneInfoByID(*feasible_lane.predecessor_lane_ids.begin());
    if (lane_info == nullptr) {
      continue;
    }

    if (lane_info->link_id() == merge_pre_link->id()) {
      merge_pre_link_lane_vec.emplace_back(*lane_info);
    }
  }

  if (!merge_pre_link_lane_vec.empty()) {
    return true;
  }

  std::vector<int> merge_link_feasible_lane_order_id_seq;
  merge_link_feasible_lane_order_id_seq.reserve(
      merge_link_feasible_lane.topo_lanes.size());
  for (const auto& feasible_lane : merge_link_feasible_lane.topo_lanes) {
    merge_link_feasible_lane_order_id_seq.emplace_back(
        feasible_lane.order_id);
  }
  std::sort(merge_link_feasible_lane_order_id_seq.begin(),
            merge_link_feasible_lane_order_id_seq.end());

  std::vector<std::pair<int, iflymapdata::sdpro::Lane>> first_merge_lane_info;
  first_merge_lane_info.reserve(merge_link->lane_ids().size());
  for (const auto& lane_id : merge_link->lane_ids()) {
    const auto& temp_lane = ld_map_.GetLaneInfoByID(lane_id);
    if (temp_lane == nullptr) {
      continue;
    }

    first_merge_lane_info.emplace_back(temp_lane->sequence(), *temp_lane);
  }

  std::sort(first_merge_lane_info.begin(), first_merge_lane_info.end(),
            [](const std::pair<int, iflymapdata::sdpro::Lane>& a,
                const std::pair<int, iflymapdata::sdpro::Lane>& b) {
              return a.first < b.first;  // 按 first 升序排列
            });

  // 先检查右边的车道
  const int min_seq = first_merge_lane_info.front().first;
  const int max_seq = first_merge_lane_info.back().first;

  const int min_order = merge_link_feasible_lane_order_id_seq.front();
  if (min_order <= max_seq && min_order >= min_seq) {
    for (int i = min_order; i > 0; i--) {
      for (const auto& [seq, lane] : first_merge_lane_info) {
        if (seq != (i - 1)) {
          continue;
        }

        // 由于是右边的汇入，所以取前继车道中seq较大的lane
        const iflymapdata::sdpro::Lane* max_seq_lane = nullptr;
        uint32_t temp_max_seq = 0;
        for (const auto& pre_lane_id : lane.predecessor_lane_ids()) {
          const auto& pre_lane = ld_map_.GetLaneInfoByID(pre_lane_id);
          if (pre_lane == nullptr) {
            continue;
          }

          if (pre_lane->lane_transiton() == iflymapdata::sdpro::LTS_MERGE ||
              pre_lane->link_id() != merge_pre_link->id()) {
            continue;
          }

          if (pre_lane->sequence() > temp_max_seq) {
            temp_max_seq = pre_lane->sequence();
            max_seq_lane = pre_lane;
          }
        }

        if (max_seq_lane != nullptr) {
          merge_pre_link_lane_vec.emplace_back(*max_seq_lane);
          return true;
        }
      }
    }
  }

  if (!merge_pre_link_lane_vec.empty()) {
    return true;
  }

  // 还是没有找到pre_lane,检查左边的车道
  const int max_order = merge_link_feasible_lane_order_id_seq.back();
  if (max_order > max_seq || min_seq > max_seq) {
    return false;
  }

  for (int i = max_order; i < max_seq; i++) {
    for (const auto& [seq, lane] : first_merge_lane_info) {
      if (seq != (i + 1)) {
        continue;
      }

      // 由于是左边的汇入，所以取前继车道中seq较小的lane
      const iflymapdata::sdpro::Lane* min_seq_lane = nullptr;
      int temp_min_seq = 1000000;// 取一个较大的数
      for (const auto& pre_lane_id : lane.predecessor_lane_ids()) {
        const auto& pre_lane = ld_map_.GetLaneInfoByID(pre_lane_id);
        if (pre_lane == nullptr) {
          continue;
        }

        if (pre_lane->lane_transiton() == iflymapdata::sdpro::LTS_MERGE ||
            pre_lane->link_id() != merge_pre_link->id()) {
          continue;
        }

        if (pre_lane->sequence() < temp_min_seq) {
          temp_min_seq = pre_lane->sequence();
          min_seq_lane = pre_lane;
        }
      }

      if (min_seq_lane != nullptr) {
        merge_pre_link_lane_vec.emplace_back(*min_seq_lane);
        return true;
      }
    }
  }

  return false;
}

bool LDRouteInfoStrategy::IsEmergencyLane(
    const iflymapdata::sdpro::Lane* lane_info) const {
  if (lane_info == nullptr) {
    return false;
  }

  for (const auto& lane_type : lane_info->lane_type()) {
    if (lane_type == iflymapdata::sdpro::LAT_EMERGENCY) {
      return true;
    }
  }

  return false;
}

bool LDRouteInfoStrategy::IsAccelerateLane(
    const iflymapdata::sdpro::Lane* lane_info) const {
  if (lane_info == nullptr) {
    return false;
  }

  for (const auto& lane_type : lane_info->lane_type()) {
    if (lane_type == iflymapdata::sdpro::LAT_ACCELERATE) {
      return true;
    }
  }

  return false;
}

bool LDRouteInfoStrategy::IsDecelerateLane(
    const iflymapdata::sdpro::Lane* lane_info) const {
  if (lane_info == nullptr) {
    return false;
  }

  for (const auto& lane_type : lane_info->lane_type()) {
    if (lane_type == iflymapdata::sdpro::LAT_DECELERATE) {
      return true;
    }
  }

  return false;
}

bool LDRouteInfoStrategy::IsEntryLane(
    const iflymapdata::sdpro::Lane* lane_info) const {
  if (lane_info == nullptr) {
    return false;
  }

  for (const auto& lane_type : lane_info->lane_type()) {
    if (lane_type == iflymapdata::sdpro::LAT_ENTRY) {
      return true;
    }
  }

  return false;
}

bool LDRouteInfoStrategy::IsExitLane(
    const iflymapdata::sdpro::Lane* lane_info) const {
  if (lane_info == nullptr) {
    return false;
  }

  for (const auto& lane_type : lane_info->lane_type()) {
    if (lane_type == iflymapdata::sdpro::LAT_EXIT) {
      return true;
    }
  }

  return false;
}

bool LDRouteInfoStrategy::IsMergeLane(
    const iflymapdata::sdpro::Lane* lane_info) const {
  if (lane_info == nullptr) {
    return false;
  }

  return lane_info->lane_connection() == iflymapdata::sdpro::LAN_STATUS_MERGING;
}

bool LDRouteInfoStrategy::IsDiversionLane(
    const iflymapdata::sdpro::Lane* lane_info) const {
  if (lane_info == nullptr) {
    return false;
  }

  for (const auto& lane_type : lane_info->lane_type()) {
    if (lane_type == iflymapdata::sdpro::LAT_DIVERSION) {
      return true;
    }
  }

  return false;
}

bool LDRouteInfoStrategy::HasLaneId(
    const std::vector<iflymapdata::sdpro::Lane>& lane_vec,
    uint64 target_id) const {
  // 使用 std::find_if 查找是否有 lane 的 id 等于 target_id
  auto it = std::find_if(lane_vec.begin(), lane_vec.end(),
                         [target_id](const iflymapdata::sdpro::Lane& lane) {
                           return lane.id() == target_id;
                         });
  return it != lane_vec.end();
}

bool LDRouteInfoStrategy::IsInvalidLane(
    const iflymapdata::sdpro::Lane* temp_lane) const {
  bool is_emergency_lane = IsEmergencyLane(temp_lane);
  bool is_merge_lane = IsMergeLane(temp_lane);
  bool is_accelerate_lane = IsAccelerateLane(temp_lane);
  bool is_entry_lane = IsEntryLane(temp_lane);
  bool is_diversion_lane = IsDiversionLane(temp_lane);

  // 1、考虑有汇入车道的场景
  if (is_diversion_lane || is_emergency_lane || is_merge_lane ||
      is_accelerate_lane || is_entry_lane) {
    return true;
  }

  // 2、考虑自车当前位置与目标split之间存在分流的场景
  bool is_exit_lane = IsExitLane(temp_lane);
  bool is_decelerate_lane = IsDecelerateLane(temp_lane);
  if (is_decelerate_lane || is_exit_lane) {
    if (route_info_output_.mlc_decider_scene_type_info.mlc_scene_type == SPLIT_SCENE) {
      const double dis_to_ramp = route_info_output_.dis_to_ramp;
      for (const auto& split_info : split_info_vec_) {
        if (split_info.second < dis_to_ramp - kEpsilon) {
          return true;
        }
      }
    }
    // todo:merge场景、normal场景
  }

  return false;
}

bool LDRouteInfoStrategy::IsIgnoreMerge(
    const std::pair<const iflymapdata::sdpro::LinkInfo_Link*, double>&
        merge_info) const {
  const auto& merge_link = merge_info.first;
  if (merge_link == nullptr) {
    return true;
  }

  // 用link上的车道数作为判断可以ignore的条件：
  // 1、在merge
  // link的pre_link上车道数多的link认为是主路，车道数少的link认为是merge进来的；
  // 2、车道数相等的情况下，判断哪边有车道收窄的，则认为收窄那边是merge的，另一边则是主路

  const uint64 merge_link_id = merge_link->id();

  const auto& merge_pre_link = ld_map_.GetPreviousLinkOnRoute(merge_link_id);
  if (merge_pre_link == nullptr) {
    return true;
  }

  if (merge_link->predecessor_link_ids_size() != 2) {
    return true;
  }

  const uint64 other_merge_link_id =
      merge_link->predecessor_link_ids()[0] == merge_pre_link->id()
          ? merge_link->predecessor_link_ids()[1]
          : merge_link->predecessor_link_ids()[0];
  const auto& other_merge_link = ld_map_.GetLinkOnRoute(other_merge_link_id);

  if (other_merge_link == nullptr) {
    return true;
  }

  int other_merge_link_lane_num = 0;
  for (const auto& lane_id : other_merge_link->lane_ids()) {
    const auto& lane = ld_map_.GetLaneInfoByID(lane_id);
    if (!IsEmergencyLane(lane) && !IsDiversionLane(lane)) {
      other_merge_link_lane_num++;
    }
  }

  int merge_pre_link_lane_num = 0;
  for (const auto& lane_id : merge_pre_link->lane_ids()) {
    const auto& lane = ld_map_.GetLaneInfoByID(lane_id);
    if (!IsEmergencyLane(lane) && !IsDiversionLane(lane)) {
      merge_pre_link_lane_num++;
    }
  }

  if (merge_pre_link_lane_num > other_merge_link_lane_num) {
    return true;
  } else if (merge_pre_link_lane_num < other_merge_link_lane_num) {
    return false;
  } else {
    // TODO:后续补充根据收窄车道的判断条件
     return !IsSucMergeLink(merge_pre_link);
  }

  return true;
}



void LDRouteInfoStrategy::CalculateMergeInfo() {
  std::vector<std::pair<const iflymapdata::sdpro::LinkInfo_Link*, double>>
      merge_info_vec = ld_map_.GetMergeInfoList(
          current_link_->id(), ego_on_cur_link_s_, kMaxSearchLength);
  // 筛选掉距离为负的merge信息
  while (!merge_info_vec.empty()) {
    if (merge_info_vec.front().second > kEpsilon) {
      break;
    } else {
      merge_info_vec.erase(merge_info_vec.begin());
    }
  }

  for (const auto& merge_info: merge_info_vec) {
    if (merge_info.first == nullptr) {
      continue;
    }

    const auto& merge_next_link =
        ld_map_.GetNextLinkOnRoute(merge_info.first->id());
    if (merge_next_link == nullptr) {
      continue;
    }

    if (merge_next_link->link_class() ==
            iflymapdata::sdpro::LinkClass::LC_EXPRESSWAY ||
        merge_next_link->link_class() ==
            iflymapdata::sdpro::LinkClass::LC_CITY_EXPRESSWAY ||
        (merge_next_link->link_type() & iflymapdata::sdpro::LT_IC) != 0 ||
        (merge_next_link->link_type() & iflymapdata::sdpro::LT_SAPA) != 0 ||
        (merge_next_link->link_type() & iflymapdata::sdpro::LT_TUNNEL) != 0 ||
        (merge_next_link->link_type() & iflymapdata::sdpro::LT_BRIDGE) != 0 ||
        (merge_next_link->link_type() & iflymapdata::sdpro::LT_TOLLBOOTH) !=
            0 ||
        (merge_next_link->link_type() & iflymapdata::sdpro::LT_TOLLGATE) != 0 ||
        (merge_next_link->link_type() &
         iflymapdata::sdpro::LT_MAINROAD_CONNECTION) != 0) {
      merge_info_vec_.emplace_back(merge_info);
    } else {
      break;
    }
  }

  if (merge_info_vec_.empty()) {
    return;
  }

  route_info_output_.first_merge_direction =
      CalculateMergeDirection(*merge_info_vec_[0].first, ld_map_);
  route_info_output_.distance_to_first_road_merge = merge_info_vec_[0].second;

  // 增加输出split_region_info_list，给下游使用
  route_info_output_.map_merge_region_info_list.reserve(merge_info_vec_.size());
  for (const auto& merge_info : merge_info_vec_) {
    if (merge_info.first == nullptr) {
      continue;
    }

    double distance_to_merge_point = merge_info.second;
    uint64 merge_link_id = merge_info.first->id();
    RampDirection merge_dir =
        CalculateMergeDirection(*merge_info.first, ld_map_);

    MapMergeRegionInfo map_merge_region_info(
        merge_link_id, distance_to_merge_point, merge_dir);

    route_info_output_.map_merge_region_info_list.emplace_back(
        std::move(map_merge_region_info));
  }
}

void LDRouteInfoStrategy::CalculateSplitInfo() {
  std::vector<std::pair<const iflymapdata::sdpro::LinkInfo_Link*, double>>
      split_info_vec = ld_map_.GetSplitInfoList(
          current_link_->id(), ego_on_cur_link_s_, kMaxSearchLength);
  // 筛选掉距离为负的merge信息
  while (!split_info_vec.empty()) {
    if (split_info_vec.front().second > kEpsilon) {
      break;
    } else {
      split_info_vec.erase(split_info_vec.begin());
    }
  }

  for (const auto& split_info : split_info_vec) {
    if (split_info.first == nullptr) {
      continue;
    }

    const auto& split_next_link =
        ld_map_.GetNextLinkOnRoute(split_info.first->id());
    if (split_next_link == nullptr) {
      continue;
    }

    if (split_next_link->link_class() ==
            iflymapdata::sdpro::LinkClass::LC_EXPRESSWAY ||
        split_next_link->link_class() ==
            iflymapdata::sdpro::LinkClass::LC_CITY_EXPRESSWAY ||
        (split_next_link->link_type() & iflymapdata::sdpro::LT_IC) != 0 ||
        (split_next_link->link_type() & iflymapdata::sdpro::LT_SAPA) != 0 ||
        (split_next_link->link_type() & iflymapdata::sdpro::LT_TUNNEL) != 0 ||
        (split_next_link->link_type() & iflymapdata::sdpro::LT_BRIDGE) != 0 ||
        (split_next_link->link_type() & iflymapdata::sdpro::LT_TOLLBOOTH) !=
            0 ||
        (split_next_link->link_type() & iflymapdata::sdpro::LT_TOLLGATE) != 0 ||
        (split_next_link->link_type() &
         iflymapdata::sdpro::LT_MAINROAD_CONNECTION) != 0) {
      split_info_vec_.emplace_back(split_info);
    } else {
      break;
    }
  }

  if (split_info_vec_.empty()) {
    return;
  }

  route_info_output_.distance_to_first_road_split = split_info_vec_[0].second;
  route_info_output_.first_split_direction =
      CalculateSplitDirection(*split_info_vec_[0].first, ld_map_);

  // 增加输出split_region_info_list，给下游使用
  route_info_output_.map_split_region_info_list.reserve(split_info_vec_.size());
  for (const auto& split_info : split_info_vec_) {
    if (split_info.first == nullptr) {
      continue;
    }

    const double distance_to_split_point = split_info.second;
    const uint64 split_link_id = split_info.first->id();
    const SplitDirection split_direction = static_cast<SplitDirection>(
        CalculateSplitDirection(*split_info.first, ld_map_));

    const auto exchange_area_fp =
        CalculateSplitExchangeAreaFP(split_info.first, split_direction);

    const auto& next_split = ld_map_.GetNextLinkOnRoute(split_link_id);
    bool is_ramp_spit =
        next_split ? ld_map_.isRamp(next_split->link_type()) : false;

    MapSplitRegionInfo split_region_info(
        split_link_id, distance_to_split_point, split_direction,
        exchange_area_fp.first, exchange_area_fp.second, is_ramp_spit);

    route_info_output_.map_split_region_info_list.emplace_back(
        std::move(split_region_info));
  }
}

std::pair<FPPoint, FPPoint> LDRouteInfoStrategy::CalculateSplitExchangeAreaFP(
    const iflymapdata::sdpro::LinkInfo_Link* split_link,
    const SplitDirection& split_dir) {
  if (split_link == nullptr) {
    return {FPPoint{}, FPPoint{}};
  }

  const auto& split_next_link = ld_map_.GetNextLinkOnRoute(split_link->id());
  if (split_next_link == nullptr) {
    return {FPPoint{}, FPPoint{}};
  }

  const iflymapdata::sdpro::LinkInfo_Link* iter_link = split_link;
  double start_fp_to_split = std::numeric_limits<double>::max();
  uint64 start_fp_link_id = 0;
  std::vector<uint64> start_fp_lane_ids;
  iflymapdata::sdpro::FeaturePoint end_FP;

  const bool is_highway =
      split_link->link_class() == iflymapdata::sdpro::LinkClass::LC_EXPRESSWAY;
  constexpr double kHighwaySearchDistance = 500.0;
  constexpr double kUrbanSearchDistance = 300.0;
  const double kMaxSearchLength =
      is_highway ? kHighwaySearchDistance : kUrbanSearchDistance;
  double sum_dis = split_link->length() * 0.01;
  iter_link = ld_map_.GetPreviousLinkOnRoute(split_link->id());
  while (iter_link) {
    if (sum_dis > kMaxSearchLength) {
      break;
    }

    bool is_widen_link = false;
    for (const auto lane_id : iter_link->lane_ids()) {
      const auto* lane_info = ld_map_.GetLaneInfoByID(lane_id);
      if (lane_info == nullptr) {
        continue;
      }
      if (lane_info->successor_lane_ids_size() > 1) {
        bool is_leftest = false;
        bool is_rightest = false;
        if (!IsTheLaneOnSide(lane_info, is_leftest, is_rightest)) {
          continue;
        }

        const bool direction_match = (split_dir == SPLIT_LEFT && is_leftest) ||
                                     (split_dir == SPLIT_RIGHT && is_rightest);
        if (direction_match) {
          is_widen_link = true;
          break;
        }
      }
    }
    if (is_widen_link) {
      start_fp_to_split = sum_dis;
      start_fp_link_id = iter_link->id();
      start_fp_lane_ids.clear();
      start_fp_lane_ids.reserve(iter_link->lane_ids_size());
      for (auto lane_id : iter_link->lane_ids()) {
        start_fp_lane_ids.emplace_back(lane_id);
      }
    }

    sum_dis += iter_link->length() * 0.01;
    iter_link = ld_map_.GetPreviousLinkOnRoute(iter_link->id());
  }

  const double dis_to_last_split = CalculateDisToLastLinkSplitPoint(split_link);
  const double dis_to_last_merge = CalculateDisToLastLinkMergePoint(split_link);
  const double default_dis = 300.0;

  if (start_fp_to_split == std::numeric_limits<double>::max()) {
    start_fp_to_split =
        std::min(std::min(dis_to_last_split, dis_to_last_merge), default_dis);
  }

  FPPoint start_fp(start_fp_link_id, -start_fp_to_split,
                   std::move(start_fp_lane_ids), end_FP);

  std::vector<uint64> lane_ids;
  lane_ids.reserve(split_next_link->lane_ids_size());
  for (auto lane_id : split_next_link->lane_ids()) {
    lane_ids.emplace_back(lane_id);
  }

  FPPoint end_fp(split_next_link->id(), 0.0, std::move(lane_ids), end_FP);

  return {std::move(start_fp), std::move(end_fp)};
}

std::pair<FPPoint, FPPoint> LDRouteInfoStrategy::CalculateMergeExchangeAreaFP(
    const iflymapdata::sdpro::LinkInfo_Link* merge_link,
    const SplitDirection& merge_dir) {
  if (merge_link == nullptr) {
    return {FPPoint{}, FPPoint{}};
  }

  iflymapdata::sdpro::FeaturePoint FP;
  std::vector<uint64> merge_lane_ids;
  merge_lane_ids.reserve(merge_link->lane_ids_size());
  for (auto lane_id : merge_link->lane_ids()) {
    merge_lane_ids.emplace_back(lane_id);
  }

  FPPoint start_fp(merge_link->id(), 0.0, std::move(merge_lane_ids), FP);

  const iflymapdata::sdpro::LinkInfo_Link* iter_link = merge_link;
  double end_fp_to_merge_dis = std::numeric_limits<double>::max();
  double sum_dis = 0.0;
  const double kMaxSearchLength = 500.0;
  uint64 end_fp_link_id = 0;
  std::vector<uint64> end_fp_lane_ids;

  while (iter_link) {
    const auto& next_link = ld_map_.GetNextLinkOnRoute(iter_link->id());
    if (next_link == nullptr) {
      break;
    }
    if (next_link->predecessor_link_ids_size() > 1 ||
        next_link->successor_link_ids_size() > 1) {
      // link级别的拓扑已经发生变化，还没找到就认为没有fp
      break;
    }
    sum_dis = sum_dis + iter_link->length() * 0.01;

    if (sum_dis > kMaxSearchLength) {
      break;
    }

    if (next_link->lane_ids_size() >= iter_link->lane_ids_size()) {
      iter_link = next_link;
      continue;
    }

    // 判断是否是加速车道数量减少
    const auto [iter_link_acc_lane_num, iter_link_entry_lane_num] =
        CountAccAndEntryLanes(iter_link);
    const auto [next_link_acc_lane_num, next_link_entry_lane_num] =
        CountAccAndEntryLanes(next_link);

    bool is_acc_lane_num_decrease =
        iter_link_acc_lane_num != 0 &&
        next_link_acc_lane_num < iter_link_acc_lane_num;
    bool is_entery_lane_decrease =
        iter_link_acc_lane_num == 0 && next_link_acc_lane_num == 0 &&
        iter_link_entry_lane_num != 0 &&
        next_link_entry_lane_num < iter_link_entry_lane_num;

    if (is_acc_lane_num_decrease || is_entery_lane_decrease) {
      end_fp_to_merge_dis = -sum_dis;
      end_fp_link_id = next_link->id();
      end_fp_lane_ids.reserve(next_link->lane_ids_size());
      for (auto lane_id : next_link->lane_ids()) {
        end_fp_lane_ids.emplace_back(lane_id);
      }
      break;
    }

    iter_link = next_link;
  }

  FPPoint end_fp(end_fp_link_id, end_fp_to_merge_dis,
                 std::move(end_fp_lane_ids), FP);

  return {std::move(start_fp), std::move(end_fp)};
}

std::tuple<size_t, size_t> LDRouteInfoStrategy::CountAccAndEntryLanes(
    const iflymapdata::sdpro::LinkInfo_Link* link) const {
  size_t acc_lane_num = 0;
  size_t entry_lane_num = 0;
  if (link == nullptr) {
    return {acc_lane_num, entry_lane_num};
  }

  for (auto lane_id : link->lane_ids()) {
    const auto* lane_info = ld_map_.GetLaneInfoByID(lane_id);
    if (lane_info == nullptr) {
      continue;  // 空指针防护
    }
    if (IsAccelerateLane(lane_info)) {
      acc_lane_num++;
    }
    if (IsEntryLane(lane_info)) {
      entry_lane_num++;
    }
  }
  return {acc_lane_num, entry_lane_num};
}

void LDRouteInfoStrategy::CalculateRampInfo() {
  for (const auto& split_info : split_info_vec_) {
    const auto& split_link = split_info.first;
    if (split_link == nullptr) {
      continue;
    }

    uint64 split_link_id = split_link->id();
    const auto& split_next_link = ld_map_.GetNextLinkOnRoute(split_link_id);
    if (split_next_link == nullptr) {
      continue;
    }

    const bool is_ramp = ld_map_.isRamp(split_next_link->link_type());
    const bool is_SAPA =
        (split_next_link->link_type() & iflymapdata::sdpro::LT_SAPA) != 0;

    if (is_ramp || is_SAPA) {
      ramp_info_vec_.emplace_back(split_info);
      continue;
    }

    bool is_filter_split = false;
    for (const auto& suc_link_id: split_link->successor_link_ids()) {
      if (suc_link_id == split_next_link->id()) {
        continue;
      }

      const auto& out_link = ld_map_.GetLinkOnRoute(suc_link_id);
      if (out_link == nullptr) {
        continue;
      }

      std::vector<iflymapdata::sdpro::Lane> exit_lane_vec;
      if (!CalculateSplitLinkExitLane(split_link, out_link, exit_lane_vec)) {
        break;
      }

      for (const auto& lane: exit_lane_vec) {
        if (IsNeedFilterSplit(&lane)) {
          is_filter_split = true;
        } else {
          is_filter_split = false;
        }
      }
    }

    if (!is_filter_split) {
      bool should_skip = false;

      // 找到当前split在split_info_vec_中的索引
      size_t current_split_idx = 0;
      for (size_t i = 0; i < split_info_vec_.size(); ++i) {
        if (split_info_vec_[i].first == split_link) {
          current_split_idx = i;
          break;
        }
      }

      // 如果存在下一个split，检查距离限制
      if (current_split_idx + 1 < split_info_vec_.size()) {
        const auto& next_split_info = split_info_vec_[current_split_idx + 1];
        const double distance_to_next_split = next_split_info.second - split_info.second;

        // 计算当前split到下一个split之间的实线长度
        double total_lsl_length = 0.0;
        const auto* iter_link = ld_map_.GetNextLinkOnRoute(split_info.first->id());
        double accumulated_distance = 0.0;

        while (iter_link && accumulated_distance < distance_to_next_split) {
          for (const auto& lane_id : iter_link->lane_ids()) {
            const auto* lane = ld_map_.GetLaneInfoByID(lane_id);
            if (lane == nullptr || IsEmergencyLane(lane) || IsDiversionLane(lane)) {
              continue;
            }
          }

          double current_link_lsl_distance = 0.0;
          for (const auto& lane_id : iter_link->lane_ids()) {
            const auto* lane = ld_map_.GetLaneInfoByID(lane_id);
            if (lane == nullptr || IsEmergencyLane(lane) || IsDiversionLane(lane)) {
              continue;
            }

            // 跳过最左侧车道
            if (lane->sequence() == iter_link->lane_num()) {
              continue;
            }

            double lane_lsl_length = 0.0;
            for (const auto& boundary : lane->left_boundaries()) {
              if (boundary.divider_marking_type() ==
                      iflymapdata::sdpro::LaneBoundary::DivederMarkingType::
                          LaneBoundary_DivederMarkingType_DMT_MARKING_SINGLE_SOLID_LINE ||
                  boundary.divider_marking_type() ==
                      iflymapdata::sdpro::LaneBoundary::DivederMarkingType::
                          LaneBoundary_DivederMarkingType_DMT_MARKING_DOUBLE_SOLID_LINE) {
                lane_lsl_length += boundary.length() * 0.01;
              }
            }
            current_link_lsl_distance = std::max(current_link_lsl_distance, lane_lsl_length);
          }

          total_lsl_length += current_link_lsl_distance;
          accumulated_distance += iter_link->length() * 0.01;
          iter_link = ld_map_.GetNextLinkOnRoute(iter_link->id());
        }

        // 阈值：下一个 split 前继 link 的 lane_num × 200m（无前继时沿用 200m）
        constexpr double BASE_MIN_GAP_DIS = 200.0;
        const auto* next_split_pre_link =
            ld_map_.GetPreviousLinkOnRoute(next_split_info.first->id());
        const double min_gap_after_lsl_m =
            next_split_pre_link != nullptr
                ? static_cast<double>(next_split_pre_link->lane_num()) *
                      BASE_MIN_GAP_DIS
                : BASE_MIN_GAP_DIS;

        if (distance_to_next_split - total_lsl_length < min_gap_after_lsl_m) {
          should_skip = true;
        }
      }

      if (!should_skip) {
        ramp_info_vec_.emplace_back(split_info);
      }
    }

  }

  if (ramp_info_vec_.empty()) {
    return;
  }

  for (const auto& ramp_info : ramp_info_vec_) {
    const auto& ramp_link = ramp_info.first;
    if (ramp_link == nullptr) {
      continue;
    }

    uint64 ramp_link_id = ramp_link->id();
    const auto& ramp_next_link = ld_map_.GetNextLinkOnRoute(ramp_link_id);
    if (ramp_next_link == nullptr) {
      continue;
    }

    const bool is_ramp = ld_map_.isRamp(ramp_next_link->link_type());
    const bool is_SAPA =
        (ramp_next_link->link_type() & iflymapdata::sdpro::LT_SAPA) != 0;

    if (is_ramp || is_SAPA) {
      route_info_output_.dis_to_ramp = ramp_info.second;
      route_info_output_.ramp_direction =
          CalculateSplitDirection(*ramp_info.first, ld_map_);
      break;
    }
  }
}

bool LDRouteInfoStrategy::get_sdpromap_valid() { return ldmap_valid_; }

const ad_common::sdpromap::SDProMap& LDRouteInfoStrategy::get_sdpro_map() {
  return ld_map_;
}

const iflymapdata::sdpro::LinkInfo_Link* LDRouteInfoStrategy::get_current_link() {
  return current_link_;
}

bool LDRouteInfoStrategy::CalculateFeasibleLaneInRampScene(
    TopoLinkGraph& feasible_lane_graph,
    TopoLinkGraph& feasible_lane_graph_after_topo_change_vec) {
  // TopoLinkGraph before_split_feasible_lane_graph;
  // TopoLinkGraph after_feasible_lane_graph;
  std::vector<iflymapdata::sdpro::Lane> start_lane_vec;

  if (ramp_info_vec_.empty()) {
    return false;
  }
  const iflymapdata::sdpro::LinkInfo_Link* split_link =
      ld_map_.GetLinkOnRoute(mlc_decider_scene_type_info_.target_link_id);
  if (split_link == nullptr) {
    return false;
  }

  // if (route_info_output_.is_on_ramp) {
  //   split_link = FindFrontValidRampSplitLink();
  //   if (split_link == nullptr) {
  //     return false;
  //   }
  // }

  const iflymapdata::sdpro::LinkInfo_Link* split_next_link =
      ld_map_.GetNextLinkOnRoute(split_link->id());
  if (split_next_link == nullptr) {
    return false;
  }

  const iflymapdata::sdpro::LinkInfo_Link* target_link = split_next_link;
  if (target_link == nullptr) {
    return false;
  }

  //在split_next_link再向前计算200m处的link，作为target_link
  double sum_dis = 0.0;
  const iflymapdata::sdpro::LinkInfo_Link* tmp_link = split_next_link;
  while (tmp_link) {
    if (tmp_link->successor_link_ids_size() != 1 ||
        tmp_link->predecessor_link_ids_size() != 1) {
      break;
    }

    target_link = tmp_link;

    sum_dis = sum_dis + tmp_link->length() * 0.01;
    if (sum_dis > 200) {
      break;
    }

    tmp_link = ld_map_.GetNextLinkOnRoute(tmp_link->id());
  }

  for (const auto& lane_id : target_link->lane_ids()) {
    const iflymapdata::sdpro::Lane* lane_info =
        ld_map_.GetLaneInfoByID(lane_id);
    if (lane_info == nullptr || IsEmergencyLane(lane_info) ||
        IsDiversionLane(lane_info)) {
      continue;
    }
    start_lane_vec.emplace_back(*lane_info);
  }

  if (!CalculateFeasibleLaneGraph(feasible_lane_graph_after_topo_change_vec,
                                  start_lane_vec, *split_next_link)) {
    return false;
  }

  // 计算split_next_link上pre_lane在split_link上的lane
  std::vector<iflymapdata::sdpro::Lane> split_link_lane_vec;
  if (feasible_lane_graph_after_topo_change_vec.lane_topo_groups.empty()) {
    return false;
  }

  const auto& split_next_link_lane_info =
      feasible_lane_graph_after_topo_change_vec.lane_topo_groups.back();
  for (const auto& topo_lane : split_next_link_lane_info.topo_lanes) {
    if (topo_lane.predecessor_lane_ids.empty()) {
      continue;
    }
    const auto lane_info =
        ld_map_.GetLaneInfoByID(*topo_lane.predecessor_lane_ids.begin());
    if (lane_info == nullptr) {
      continue;
    }

    if (lane_info->link_id() == split_link->id()) {
      split_link_lane_vec.emplace_back(*lane_info);
    }
  }

  // 反向遍历得到从当前link的lane能直达split_link上targte_lane的feasible
  // lane
  if (!CalculateFeasibleLaneGraph(feasible_lane_graph, split_link_lane_vec,
                                  *current_link_)) {
    return false;
  }

  if (feasible_lane_graph.lane_topo_groups.empty()) {
    return false;
  }

  // 再次反向遍历横向上扩展feasible lane
  // 根据距离把可行驶车道加上
  if (!CalculateExtenedFeasibleLane(feasible_lane_graph)) {
    return false;
  }

  if (feasible_lane_graph.lane_topo_groups.empty()) {
    return false;
  }

  return true;
}

bool LDRouteInfoStrategy::CalculateFeasibleLaneInMergeScene(
    TopoLinkGraph& feasible_lane_graph,
    TopoLinkGraph& feasible_lane_graph_after_topo_change_vec) {
  // 1、计算target link
  if (merge_info_vec_.empty()) {
    return false;
  }
  const auto& first_merge_link_info = ld_map_.GetLinkOnRoute(
      route_info_output_.mlc_decider_scene_type_info.target_link_id);
  if (first_merge_link_info == nullptr) {
    return false;
  }
  const auto& merge_pre_link =
      ld_map_.GetPreviousLinkOnRoute(first_merge_link_info->id());
  if (merge_pre_link == nullptr) {
    return false;
  }

  iflymapdata::sdpro::LinkInfo_Link first_merge_link = *first_merge_link_info;
  iflymapdata::sdpro::LinkInfo_Link* target_link = &first_merge_link;
  std::vector<iflymapdata::sdpro::Lane> start_lane_vec;
  if (!CalculateFrontTargetLinkBaseFixDis(target_link, start_lane_vec,
                                          first_merge_link_info,
                                          route_info_output_.mlc_decider_scene_type_info.mlc_scene_type)) {
    return false;
  }

  // 2、计算merge之后的feasible lane
  // TopoLinkGraph after_merge_feasible_lane_graph;
  if (!CalculateFeasibleLaneGraph(feasible_lane_graph_after_topo_change_vec,
                                  start_lane_vec, *first_merge_link_info)) {
    return false;
  }

  if (feasible_lane_graph_after_topo_change_vec.lane_topo_groups.empty()) {
    return false;
  }

  // 3、计算merge_pre_link上的feasible lane
  std::vector<iflymapdata::sdpro::Lane> merge_pre_link_feasible_lane;
  if (!CalculateMergePreFeasibleLane(merge_pre_link_feasible_lane,
                                     feasible_lane_graph_after_topo_change_vec,
                                     first_merge_link_info)) {
    return false;
  }

  // 4、反向遍历至自车当前位置，计算feasible lane
  // TopoLinkGraph before_merge_feasible_lane_graph;
  if (!CalculateFeasibleLaneGraph(
          feasible_lane_graph, merge_pre_link_feasible_lane, *current_link_)) {
    return false;
  }

  if (feasible_lane_graph.lane_topo_groups.empty()) {
    return false;
  }

  // 5、再次反向遍历横向上扩展feasible lane
  // 根据距离把可行驶车道加上
  if (!CalculateExtenedFeasibleLane(feasible_lane_graph)) {
    return false;
  }

  if (feasible_lane_graph.lane_topo_groups.empty()) {
    return false;
  }

  return true;
}

bool LDRouteInfoStrategy::CalculateFeasibleLaneInNormalScene(
    TopoLinkGraph& feasible_lane_graph) {
  // 1、 确定target_link
  iflymapdata::sdpro::LinkInfo_Link current_link = *current_link_;
  iflymapdata::sdpro::LinkInfo_Link* target_link = &current_link;
  std::vector<iflymapdata::sdpro::Lane> start_lane_vec;
  if (!CalculateFrontTargetLinkBaseFixDis(target_link, start_lane_vec,
                                          current_link_,
                                          route_info_output_.mlc_decider_scene_type_info.mlc_scene_type)) {
    return false;
  }

  // 2、计算feasible lane
  // TopoLinkGraph normal_feasible_lane_graph;
  if (!CalculateFeasibleLaneGraph(feasible_lane_graph, start_lane_vec,
                                  *current_link_)) {
    return false;
  }

  if (feasible_lane_graph.lane_topo_groups.empty()) {
    return false;
  }

  // 再次反向遍历横向上扩展feasible lane
  // 根据距离把可行驶车道加上
  if (!CalculateExtenedFeasibleLane(feasible_lane_graph)) {
    return false;
  }

  if (feasible_lane_graph.lane_topo_groups.empty()) {
    return false;
  }

  return true;
}

void LDRouteInfoStrategy::ProcessLaneDistance(
    const std::shared_ptr<VirtualLane>& relative_id_lane,
    const std::unordered_map<int, double>& feasible_lane_distance) {
  if (relative_id_lane == nullptr) {
    return;
  }

  const auto& lane_nums = relative_id_lane->get_lane_nums();
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
    if (relative_id_lane->get_relative_id() == -1) {
      route_info_output_.left_lane_distance = virtual_lane_distance.second;
    } else if (relative_id_lane->get_relative_id() == 0) {
      route_info_output_.current_lane_distance = virtual_lane_distance.second;
    } else if (relative_id_lane->get_relative_id() == 1) {
      route_info_output_.right_lane_distance = virtual_lane_distance.second;
    }
  } else {
    virtual_lane_distance = std::make_pair(false, 0.0);
  }

  relative_id_lane->set_feasible_lane_distance(virtual_lane_distance);
}

void LDRouteInfoStrategy::ProcessLaneMapMergePoint(
    const std::shared_ptr<VirtualLane>& relative_id_lane,
    const std::unordered_map<int, MapMergePointInfo>& map_merge_points_info) {
  if (relative_id_lane == nullptr || map_merge_points_info.empty()) {
    return;
  }

  const auto& lane_nums = relative_id_lane->get_lane_nums();
  int left_lane_num = 0;

  for (const auto& lane_num : lane_nums) {
    if (lane_num.end > kEpsilon) {
      left_lane_num = lane_num.left_lane_num;
      break;
    }
  }

  auto it = map_merge_points_info.find(left_lane_num + 1);
  MapMergePointInfo map_merge_point_info;

  if (it != map_merge_points_info.end()) {
    map_merge_point_info = it->second;
  }

  relative_id_lane->set_map_merge_point_info(std::move(map_merge_point_info));
}

void LDRouteInfoStrategy::CaculateDistanceToRoadEnd(
    const iflymapdata::sdpro::LinkInfo_Link* segment, const double nearest_s) {
  if (segment == nullptr) {
    return;
  }

  double dis_to_end = NL_NMAX;
  int result =
      ld_map_.GetDistanceToRouteEnd(segment->id(), nearest_s, dis_to_end);
  if (result == 0) {
    route_info_output_.distance_to_route_end = dis_to_end;
  } else {
    route_info_output_.distance_to_route_end = NL_NMAX;
  }
}

void LDRouteInfoStrategy::CaculateDistanceToTollStation(
    const iflymapdata::sdpro::LinkInfo_Link* segment, const double nearest_s) {
  if (segment == nullptr) {
    return;
  }

  const auto& toll_station_info =
      ld_map_.GetTollStationInfo(segment->id(), nearest_s, kMaxSearchLength);
  if (toll_station_info.first != nullptr) {
    route_info_output_.distance_to_toll_station = toll_station_info.second;
    route_info_output_.is_exist_toll_station = true;
  } else {
    route_info_output_.distance_to_toll_station = NL_NMAX;
    route_info_output_.is_exist_toll_station = false;
  }
}

bool LDRouteInfoStrategy::IsLaneSuccessorInPlannedRoute(
    const iflymapdata::sdpro::Lane* lane_info) {
  if (lane_info == nullptr) {
    return false;
  }

  const iflymapdata::sdpro::Lane* iterator_lane = lane_info;
  const double kFrontDis = 500.0;
  double sum_dis = 0;

  while (iterator_lane) {
    if (!ld_map_.isOnRouteLinks(iterator_lane->link_id())) {
      return false;
    }

    const iflymapdata::sdpro::Lane* successor_lane = nullptr;
    for (const auto& lane_id : iterator_lane->successor_lane_ids()) {
      const auto& temp_successor_lane = ld_map_.GetLaneInfoByID(lane_id);
      if (temp_successor_lane == nullptr) {
        continue;
      }

      if (IsDecelerateLane(temp_successor_lane) ||
          IsExitLane(temp_successor_lane)) {
        continue;
      }

      if (!ld_map_.isOnRouteLinks(temp_successor_lane->link_id())) {
        continue;
      }

      successor_lane = temp_successor_lane;
      break;
    }

    if (successor_lane == nullptr) {
      return false;
    }

    if (iterator_lane->link_id() == current_link_->id()) {
      sum_dis = sum_dis + iterator_lane->length() * 0.01 - ego_on_cur_link_s_;
    } else {
      sum_dis = sum_dis + iterator_lane->length() * 0.01;
    }

    if (sum_dis > kFrontDis) {
      break;
    }

    iterator_lane = successor_lane;
  }

  return true;
}

bool LDRouteInfoStrategy::IsLaneSuccessorIsMergeLane(
    const iflymapdata::sdpro::Lane* lane_info) const {
  if (lane_info == nullptr) {
    return false;
  }

  const iflymapdata::sdpro::Lane* iterator_lane = lane_info;
  const double kFrontDis = 500.0;
  double sum_dis = 0.0;

  while (iterator_lane) {
    if (IsMergeLane(iterator_lane)) {
      return true;
    }

    const auto& lane_link = ld_map_.GetLinkOnRoute(iterator_lane->link_id());
    if (lane_link == nullptr) {
      return false;
    }

    if (lane_link->successor_link_ids_size() != 1) {
      return false;
    }

    if (iterator_lane->successor_lane_ids_size() != 1) {
      // 由于目前1分2车道检测不稳定，后继不为1的直接认为不在route上，不会被放进feasible lane中
      // 后续根据测试效果，确定是否需要更精确的判断
      return false;
    }

    const uint64 successor_lane_id = iterator_lane->successor_lane_ids()[0];
    const auto& successor_lane = ld_map_.GetLaneInfoByID(successor_lane_id);
    if (successor_lane == nullptr) {
      return false;
    }

    if (iterator_lane->link_id() == current_link_->id()) {
      sum_dis = sum_dis + iterator_lane->length() * 0.01 - ego_on_cur_link_s_;
    } else {
      sum_dis = sum_dis + iterator_lane->length() * 0.01;
    }

    if (sum_dis > kFrontDis) {
      break;
    }

    iterator_lane = successor_lane;
  }

  return false;
}

const iflymapdata::sdpro::LinkInfo_Link*
LDRouteInfoStrategy::FindFrontValidRampSplitLink() const {
  // 在处理ramp场景时，是以前方split是ramp作为判断条件的，在主路上可以有效计算出feasible lane
  // 自车在匝道上时，前方所有的split都是匝道的话，则只会处理最近的split，这样存在无法处理匝道上有连续split的场景
  // 因此，当自车在匝道上nearing ramp，且前方500m内有多个split时，至少需要判断500m处的ramp。
  if (ramp_info_vec_.empty() || !route_info_output_.is_on_ramp) {
    return nullptr;
  }

  const double min_front_search_dis = 500.0;
  for (size_t ramp_idx = 0; ramp_idx < ramp_info_vec_.size(); ++ramp_idx) {
    const auto& [ramp_link, ramp_dis] = ramp_info_vec_[ramp_idx];
    // 场景1：匝道距离≥最小搜索距离 → 直接取当前匝道链路作为分流链路
    if (ramp_dis > min_front_search_dis) {
      return ramp_link;
    }

    // 场景2：匝道距离小于最小搜索距离 → 检查合流信息
    for (const auto& merge_info : merge_info_vec_) {
      const double merge_distance = merge_info.second;
      // 合流点距离 < 匝道距离 + 不忽略该合流 + 非第一个匝道（避免ramp_idx-1越界）
      const bool is_valid_merge = (merge_distance < ramp_dis) &&
                                  !IsIgnoreMerge(merge_info) &&
                                  (ramp_idx > 0);
      if (is_valid_merge) {
        return ramp_info_vec_[ramp_idx - 1].first;
      }
    }
  }

  return ramp_info_vec_.back().first;
}

void LDRouteInfoStrategy::CalculateAvoidMergeFeasibleLane(
    TopoLinkGraph& feasible_lane_graph) {
  if (feasible_lane_graph.lane_topo_groups.empty()) {
    return;
  }

  // 1. 建立Link到feasible_lane_graph索引的映射
  std::unordered_map<uint64, size_t> link_id_to_index;
  for (size_t i = 0; i < feasible_lane_graph.lane_topo_groups.size(); ++i) {
    link_id_to_index[feasible_lane_graph.lane_topo_groups[i].link_id] = i;
  }

  // 2. 找出所有合流点
  bool has_processed_left = false;
  bool has_processed_right = false;
  for (const auto& merge_info : merge_info_vec_) {
    const auto* merge_ptr = merge_info.first;
    double dist_to_merge = merge_info.second;

    if (merge_ptr == nullptr) {
      continue;
    }
    // 确定合流方向
    const auto merge_direction = CalculateMergeDirection(*merge_ptr, ld_map_);

    // 3. 判断是否需要处理该合流点
    // 3.1 仅处理拓扑变化点之前的合流点
    if (dist_to_merge >
        mlc_decider_scene_type_info_.dis_to_link_topo_change_point - kEpsilon) {
      continue;
    }
    // 3.2 同一方向merge，仅考虑第一个
    if (merge_direction == RAMP_ON_LEFT) {
      if (has_processed_left) {
        continue;
      }
      has_processed_left = true;
    } else if (merge_direction == RAMP_ON_RIGHT) {
      if (has_processed_right) {
        continue;
      }
      has_processed_right = true;
    }
    // 3.3 距离ramp小于300米，且方向相反，不做处理
    if (mlc_decider_scene_type_info_.mlc_scene_type == SPLIT_SCENE &&
        mlc_decider_scene_type_info_.dis_to_link_topo_change_point -
                dist_to_merge <
            300.0) {
      bool is_direction_conflict =
          merge_direction != mlc_decider_scene_type_info_.route_lane_direction;
      if (is_direction_conflict) {
        continue;
      }
    }
    // 3.4 判断平行汇入
    auto it = link_id_to_index.find(merge_ptr->id());
    if (it == link_id_to_index.end()) {
      continue;
    }
    size_t merge_link_idx = it->second;
    if (feasible_lane_graph.lane_topo_groups[merge_link_idx].topo_lanes.size() <
        2) {
      continue;
    }
    bool is_need_avoid = false;
    for (const auto merge_link_lane_id : merge_ptr->lane_ids()) {
      const auto* merge_link_lane = ld_map_.GetLaneInfoByID(merge_link_lane_id);
      if (merge_link_lane == nullptr) {
        continue;
      }
      if (IsEmergencyLane(merge_link_lane) || IsDiversionLane(merge_link_lane)) {
        continue;
      }
      // 找出汇入车道
      if (merge_link_lane->predecessor_lane_ids_size() == 0) {
        continue;
      }
      const auto* pre_merge_link_lane =
          ld_map_.GetLaneInfoByID(merge_link_lane->predecessor_lane_ids()[0]);
      if (pre_merge_link_lane == nullptr) {
        continue;
      }
      if (ld_map_.isOnRouteLinks(pre_merge_link_lane->link_id())) {
        continue;
      }
      // 判断是否是feasible lane旁边车道
      uint32 search_orderid = merge_direction == RAMP_ON_LEFT
                                  ? merge_link_lane->sequence() + 1
                                  : merge_link_lane->sequence() - 1;
      auto it = std::find_if(
          feasible_lane_graph.lane_topo_groups[merge_link_idx]
              .topo_lanes.begin(),
          feasible_lane_graph.lane_topo_groups[merge_link_idx].topo_lanes.end(),
          [search_orderid](const TopoLane& lane) {
            return lane.order_id == search_orderid;
          });
      if (it == feasible_lane_graph.lane_topo_groups[merge_link_idx].topo_lanes.end()) {
        continue;
      }
      // 判断需要躲避的feasible_lane前继link是否在route上
      const auto* avoid_feasible_lane = ld_map_.GetLaneInfoByID(it->id);
      if (avoid_feasible_lane == nullptr) {
        continue;
      }
      if (avoid_feasible_lane->predecessor_lane_ids_size() == 0) {
        continue;
      }
      const auto* avoid_feasible_lane_pre = ld_map_.GetLaneInfoByID(
          avoid_feasible_lane->predecessor_lane_ids()[0]);
      if (avoid_feasible_lane_pre == nullptr) {
        continue;
      }
      if (!ld_map_.isOnRouteLinks(avoid_feasible_lane_pre->link_id())) {
        continue;
      }

      // 判断车道前方是否消亡
      double accumulated_dis = 0.0;
      const auto* itera_lane = merge_link_lane;
      while (itera_lane != nullptr && accumulated_dis < 300.0) {
        if (IsMergeLane(itera_lane)) {
          const auto merge_lane_type = CalculateMergeLaneType(itera_lane);
          if (merge_lane_type == MERGE_TO_LEFT &&
                  merge_direction == RAMP_ON_LEFT ||
              merge_lane_type == MERGE_TO_RIGHT &&
                  merge_direction == RAMP_ON_RIGHT) {
            is_need_avoid = true;
            break;
          }
        }
        if (!ld_map_.isOnRouteLinks(itera_lane->link_id())) {
          is_need_avoid = true;
          break;
        }
        accumulated_dis += itera_lane->length() * 0.01;
        if (itera_lane->successor_lane_ids_size() > 0) {
          itera_lane =
              ld_map_.GetLaneInfoByID(itera_lane->successor_lane_ids()[0]);
        } else {
          is_need_avoid = true;
          break;
        }
      }
    }
    if (!is_need_avoid) {
      continue;
    }
    // 4. 更新该合流点之前所有link的车道可行距离
    for (size_t t = feasible_lane_graph.lane_topo_groups.size() - 1;
         t > merge_link_idx; --t) {
      if (feasible_lane_graph.lane_topo_groups[t].topo_lanes.size() < 2) {
        continue;
      }
      for (auto& lane : feasible_lane_graph.lane_topo_groups[t].topo_lanes) {
        const auto lane_info = ld_map_.GetLaneInfoByID(lane.id);
        const auto link_info = ld_map_.GetLinkOnRoute(lane.link_id);
        if (!lane_info || !link_info) {
          continue;
        }

        bool is_leftest = false;
        bool is_rightest = false;
        IsTheLaneOnSide(lane_info, is_leftest, is_rightest);

        // 判定是否属于需要躲避的侧向车道
        bool should_reduce = false;
        if (merge_direction == RAMP_ON_LEFT && is_rightest) {
          should_reduce = true;
        } else if (merge_direction == RAMP_ON_RIGHT && is_leftest) {
          should_reduce = true;
        }

        if (should_reduce) {
          double link_distance = 0.0;
          if (CalculateDistanceToCertainLink(merge_ptr, link_info,
                                             link_distance)) {
            lane.front_feasible_distance = link_distance - 500.0;
          }
        }
      }
    }
  }

  // 5. 移除不可行的车道
  for (auto& group : feasible_lane_graph.lane_topo_groups) {
    auto& lanes = group.topo_lanes;
    if (lanes.size() <= 1) {
      continue;
    }
    // 记录被移除的lane
    std::copy_if(
        lanes.begin(), lanes.end(),
        std::back_inserter(avoid_link_merge_lane_id_vec_),
        [](const auto& lane) { return lane.front_feasible_distance < 0.0; });
    lanes.erase(std::remove_if(lanes.begin(), lanes.end(),
                               [](const auto& lane) {
                                 return lane.front_feasible_distance < 0.0;
                               }),
                lanes.end());
  }
}

void LDRouteInfoStrategy::Erase1Split2FeasibleLane(
    TopoLinkGraph& feasible_lane_graph) {
  std::vector<TopoLane> max_distance_lanes =
      CalculateMaxDistanceLanes(feasible_lane_graph);
  if (max_distance_lanes.size() < 2) {
    return;
  }

  std::sort(max_distance_lanes.begin(), max_distance_lanes.end(),
            [](const TopoLane& a, const TopoLane& b) {
              return a.order_id < b.order_id;  // 按 order_id 升序排列
            });

  // 判断ramp方向的次车道上是否有1分2的lane，如果有从feasible lane中拿掉
  if (ramp_info_vec_.empty()) {
    return;
  }

  const auto front_first_ramp_info = ramp_info_vec_[0].first;
  if (front_first_ramp_info == nullptr) {
    return;
  }

  const auto& front_first_ramp_dir =
      CalculateSplitDirection(*front_first_ramp_info, ld_map_);
  const auto& split_next_link =
      ld_map_.GetNextLinkOnRoute(front_first_ramp_info->id());
  if (split_next_link == nullptr) {
    return;
  }

  if (front_first_ramp_dir == RAMP_ON_RIGHT) {
    for (size_t idx = 1; idx < max_distance_lanes.size(); ++idx) {
      EraseFeasibleLaneIfNeeded(max_distance_lanes[idx].id, split_next_link,
                                feasible_lane_graph);
    }
  } else if (front_first_ramp_dir == RAMP_ON_LEFT) {
    for (int idx = static_cast<int>(max_distance_lanes.size()) - 2; idx >= 0; --idx) {
      EraseFeasibleLaneIfNeeded(max_distance_lanes[idx].id, split_next_link,
                                feasible_lane_graph);
    }
  }
  return;
}

void LDRouteInfoStrategy::EraseFeasibleLaneIfNeeded(
    uint64_t lane_id, const iflymapdata::sdpro::LinkInfo_Link* split_next_link,
    TopoLinkGraph& feasible_lane_graph) {
  // 从lane开始遍历，一直到split next
  // link之前，是否有1分2的车道，如有的话，则把这条lane从feasible lane中erase
  auto& topo_lanes = feasible_lane_graph.lane_topo_groups.back().topo_lanes;
  bool is_exist_lane_id = false;
  for (const auto& topo_lane : topo_lanes) {
    if (topo_lane.id == lane_id) {
      is_exist_lane_id = true;
    }
  }
  if (!is_exist_lane_id) {
    return;
  }

  const iflymapdata::sdpro::Lane* itera_lane = ld_map_.GetLaneInfoByID(lane_id);
  while (itera_lane) {
    if (itera_lane->link_id() == split_next_link->id() ||
        itera_lane->successor_lane_ids().empty() ||
        !ld_map_.isOnRouteLinks(itera_lane->link_id())) {
      return;
    }

    if (itera_lane->successor_lane_ids_size() > 1) {
      // 如果suc_lane不在route_link上，则从feasible lane中移除这个lane
      bool is_exist_suc_lane_not_in_route_link = false;
      for (const auto& temp_lane_id : itera_lane->successor_lane_ids()) {
        const auto& temp_lane = ld_map_.GetLaneInfoByID(temp_lane_id);
        if (temp_lane == nullptr) {
          continue;
        }

        // suc_lane都不在split_next_link上认为存在后继车道不在route_link上
        if (split_next_link->id() != temp_lane->link_id()) {
          is_exist_suc_lane_not_in_route_link = true;
          break;
        }
      }

      if (is_exist_suc_lane_not_in_route_link) {
        for (auto it = topo_lanes.begin(); it != topo_lanes.end();) {
          if (it->id == lane_id) {
            topo_lanes.erase(it);  // erase returns the next iterator
            return;
          } else {
            ++it;
          }
        }
      }
    }

    itera_lane = ld_map_.GetLaneInfoByID(itera_lane->successor_lane_ids()[0]);
  }
}
std::vector<TopoLane> LDRouteInfoStrategy::CalculateMaxDistanceLanes(
    const TopoLinkGraph& feasible_lane_graph) const {
  std::vector<TopoLane> max_distance_lanes;
  if (feasible_lane_graph.lane_topo_groups.empty()) {
    return max_distance_lanes;
  }

  if (feasible_lane_graph.lane_topo_groups.back().topo_lanes.empty()) {
    return max_distance_lanes;
  }

  // 用int类型，便于后面获取距离相同的lane
  // std::multimap<int, TopoLane> topo_lanes_map;

  // 把当前位置上的所有lane都判断后继是否存在1分2的车道
  for (const auto& topo_lane :
       feasible_lane_graph.lane_topo_groups.back().topo_lanes) {
    max_distance_lanes.push_back(topo_lane);
  }

  // if (topo_lanes_map.empty()) {
  //   return max_distance_lanes;
  // }

  // auto max_key = topo_lanes_map.rbegin()->first;

  // // 使用equal_range获取所有具有最大键值的元素
  // auto range = topo_lanes_map.equal_range(max_key);

  // for (auto it = range.first; it != range.second; ++it) {
  //   max_distance_lanes.push_back(it->second);
  // }

  return max_distance_lanes;
}

const iflymapdata::sdpro::Lane*
LDRouteInfoStrategy::IsEntryLanePresentOnEitherSideOfSuccessorLane(
    const iflymapdata::sdpro::Lane* cur_link_lane_info) {
  if (cur_link_lane_info == nullptr) {
    return nullptr;
  }

  const double front_search_dis = 500.0;
  double sum_dis = 0.0;
  const auto* current_traverse_lane = cur_link_lane_info;
  while (current_traverse_lane) {
    if (current_traverse_lane->link_id() == current_link_->id()) {
      sum_dis += current_traverse_lane->length() * 0.01 - ego_on_cur_link_s_;
    } else {
      sum_dis += current_traverse_lane->length() * 0.01;
    }

    const auto [left_nerghbor_lane, right_nerghbor_lane] =
        FindLaneLeftRightNeighbors(current_traverse_lane);

    if (IsEntryLane(left_nerghbor_lane)) {
      return left_nerghbor_lane;
    }

    if (IsEntryLane(right_nerghbor_lane)) {
      return right_nerghbor_lane;
    }

    if (sum_dis > front_search_dis) {
      break;
    }

    if (current_traverse_lane->successor_lane_ids_size() != 1) {
      return nullptr;
    }

    current_traverse_lane =
        ld_map_.GetLaneInfoByID(current_traverse_lane->successor_lane_ids()[0]);
  }

  return nullptr;
}


std::pair<const iflymapdata::sdpro::Lane*, const iflymapdata::sdpro::Lane*>
LDRouteInfoStrategy::FindLaneLeftRightNeighbors(const iflymapdata::sdpro::Lane* target_lane) {
  if (target_lane == nullptr) {
    return {nullptr, nullptr};
  }

  const auto* lane_link = ld_map_.GetLinkOnRoute(target_lane->link_id());
  if (lane_link == nullptr) {
    return {nullptr, nullptr};
  }

  const int target_seq = target_lane->sequence();
  const int left_neighbor_seq = target_seq + 1;  //sequence左大右小
  const int right_neighbor_seq = target_seq - 1;
  const iflymapdata::sdpro::Lane* left_lane = nullptr;
  const iflymapdata::sdpro::Lane* right_lane = nullptr;

  // 查找左右侧车道，找到后提前终止循环
  for (const auto& lane_id : lane_link->lane_ids()) {
    const auto* lane = ld_map_.GetLaneInfoByID(lane_id);
    if (lane == nullptr) {
      continue;
    }

    if (lane->sequence() == left_neighbor_seq) {
      left_lane = lane;
    } else if (lane->sequence() == right_neighbor_seq) {
      right_lane = lane;
    }

    // 左右都找到，提前退出
    if (left_lane != nullptr && right_lane != nullptr) {
      break;
    }
  }

  return {left_lane, right_lane};
}

bool LDRouteInfoStrategy::IsInvalidNonDrivingLane(const iflymapdata::sdpro::Lane* lane) {
  if (lane == nullptr) {
    return true;
  }
  return IsEmergencyLane(lane) || IsDiversionLane(lane);
}

bool LDRouteInfoStrategy::IsInvalidLaneMergeLaneOppositeSide(
    const iflymapdata::sdpro::Lane* merge_lane) {
  if (merge_lane == nullptr) {
    return false;
  }

  // 步骤1：确定合流方向
  auto merge_lane_type = CalculateMergeLaneType(merge_lane);

  if (merge_lane_type == NONE_MERGE) {
    return false;
  }

  // 步骤2：查找合流车道的左右侧邻居车道
  const auto [left_neighbor, right_neighbor] =
      FindLaneLeftRightNeighbors(merge_lane);

  // 步骤3：根据合流方向判断邻居车道是否有效
  if (merge_lane_type == MERGE_TO_LEFT) {
    return IsInvalidNonDrivingLane(right_neighbor);
  } else if (merge_lane_type == MERGE_TO_RIGHT) {
    return IsInvalidNonDrivingLane(left_neighbor);
  }

  return false;
}

MergeLaneType LDRouteInfoStrategy::CalculateMergeLaneType(const iflymapdata::sdpro::Lane* merge_lane) {
  if (merge_lane == nullptr) {
    return NONE_MERGE;
  }

  if (merge_lane->successor_lane_ids().empty()) {
    return NONE_MERGE;
  }

  const auto& merge_lane_succesor_lane =
      ld_map_.GetLaneInfoByID(merge_lane->successor_lane_ids()[0]);
  if (merge_lane_succesor_lane == nullptr) {
    return NONE_MERGE;
  }

  for (const auto& pre_lane_id : merge_lane_succesor_lane->predecessor_lane_ids()) {
    const auto* pre_lane = ld_map_.GetLaneInfoByID(pre_lane_id);
    if (pre_lane == nullptr || IsMergeLane(pre_lane)) {
      continue;
    }

    if (pre_lane->sequence() < merge_lane->sequence()) {
      return MERGE_TO_RIGHT;
    } else {
      return MERGE_TO_LEFT;
    }
  }

  return NONE_MERGE;
}

void LDRouteInfoStrategy::HandleMainLinkPreLane(
    const iflymapdata::sdpro::Lane* pre_lane,
    std::vector<iflymapdata::sdpro::Lane>& next_lane_vec) {
    if (pre_lane == nullptr) {
      return;
    }

    if (!HasLaneId(next_lane_vec, pre_lane->id())) { // 避免重复添加
        next_lane_vec.emplace_back(*pre_lane);
    }
}

void LDRouteInfoStrategy::HandleOtherMergeLinkPreLane(
    const TopoLane& topo_lane,
    const iflymapdata::sdpro::LinkInfo_Link* next_pre_link,
    const iflymapdata::sdpro::LinkInfo_Link* current_link,
    const iflymapdata::sdpro::Lane* pre_lane,
    std::vector<iflymapdata::sdpro::Lane>& next_lane_vec) {
  if (next_pre_link == nullptr || current_link == nullptr ||
      pre_lane == nullptr) {
    return;
  }

  std::unordered_set<uint64_t> other_link_id_set;
  for (const auto& pre_link_id: current_link->predecessor_link_ids()) {
    if (pre_link_id == next_pre_link->id()) {
      continue;
    }
    other_link_id_set.insert(pre_link_id);
  }

  if (other_link_id_set.empty()) {
    return;
  }

  auto it = other_link_id_set.find(pre_lane->link_id());
  if (it == other_link_id_set.end()) {
    return;
  }

  // 在主link中找到与拓扑车道最匹配的前继车道
  const auto matching_pre_lane =
      FindMatchingPreLaneInMainLink(topo_lane, next_pre_link);

  if (matching_pre_lane.link_id() == next_pre_link->id() &&
      !HasLaneId(next_lane_vec, matching_pre_lane.id())) {
    next_lane_vec.emplace_back(matching_pre_lane);
  }
}

iflymapdata::sdpro::Lane LDRouteInfoStrategy::FindMatchingPreLaneInMainLink(
    const TopoLane& topo_lane,
    const iflymapdata::sdpro::LinkInfo_Link* next_pre_link) {
  int min_order_error = 100;
  iflymapdata::sdpro::Lane best_matching_lane;

  if (next_pre_link == nullptr) {
    return best_matching_lane;
  }

  for (const auto& temp_lane_id : next_pre_link->lane_ids()) {
    const auto* temp_lane = ld_map_.GetLaneInfoByID(temp_lane_id);
    if (!temp_lane || IsEmergencyLane(temp_lane) ||
        IsDiversionLane(temp_lane)) {
      continue;
    }
    for (const auto& temp_suc_lane_id : temp_lane->successor_lane_ids()) {
      const auto* temp_suc_lane = ld_map_.GetLaneInfoByID(temp_suc_lane_id);
      if (!temp_suc_lane || IsEmergencyLane(temp_suc_lane) ||
          IsDiversionLane(temp_suc_lane)) {
        continue;
      }

      // 确保后继车道与拓扑车道属于同一link
      if (temp_suc_lane->link_id() != topo_lane.link_id) {
        continue;
      }

      // 计算序号差，更新最优匹配车道
      const int order_error = std::abs(
          static_cast<int>(temp_suc_lane->sequence() - topo_lane.order_id));
      if (order_error < min_order_error) {
        min_order_error = order_error;
        best_matching_lane = *temp_lane;
      }
    }
  }

  return best_matching_lane;
}

void LDRouteInfoStrategy::CalculateFrontMergePointInfo(double search_dis) {
  if (current_link_ == nullptr) {
    return;
  }

  for (const auto& lane_id: current_link_->lane_ids()) {
    const auto& lane = ld_map_.GetLaneInfoByID(lane_id);
    if (lane == nullptr) {
      continue;
    }
    const iflymapdata::sdpro::Lane* itera_lane = lane;
    double sum_dis = 0.0;
    while (itera_lane) {
      if (itera_lane->link_id() == current_link_->id()) {
        sum_dis = sum_dis + itera_lane->length() * 0.01 - ego_on_cur_link_s_;
      } else {
        sum_dis = sum_dis + itera_lane->length() * 0.01;
      }

      if (itera_lane->successor_lane_ids_size() > 1) {
        // 如果后继车道数大于1，说明道路拓扑有变化，不再考虑merge point的情况
        break;
      }

      if (IsMergeLane(itera_lane)) {
        auto merge_lane_type = CalculateMergeLaneType(itera_lane);
        if (merge_lane_type == NONE_MERGE) {
          break;
        }
        // 避免重复添加
        bool is_element_exists =
            std::any_of(route_info_output_.map_merge_points_info.begin(),
                        route_info_output_.map_merge_points_info.end(),
                        [&](const auto& elem) {
                          return elem.merge_lane_sequence == itera_lane->sequence() &&
                                 elem.merge_lane_link_id == itera_lane->link_id();
                        });
        if (!is_element_exists) {
          route_info_output_.map_merge_points_info.emplace_back(
              sum_dis, merge_lane_type, itera_lane->sequence(),
              itera_lane->link_id());
        }
      }

      if (sum_dis > search_dis) {
        break;
      }

      if (itera_lane->successor_lane_ids().empty()) {
        break;
      }

      itera_lane = ld_map_.GetLaneInfoByID(itera_lane->successor_lane_ids()[0]);
    }
  }

  return;
}

double LDRouteInfoStrategy::CalculateDisToLastLinkSplitPoint(
    const iflymapdata::sdpro::LinkInfo_Link* cur_link) const {
  if (cur_link == nullptr) {
    return std::numeric_limits<double>::max();
  }

  const double search_dis = 2000.0;  // 最多搜索2km，找多了也没必要

  double sum_dis = 0.0;

  const iflymapdata::sdpro::LinkInfo_Link* iterate_link = cur_link;

  while (iterate_link) {
    const auto& pre_link = ld_map_.GetPreviousLinkOnRoute(iterate_link->id());
    if (pre_link == nullptr) {
      return std::numeric_limits<double>::max();
    }

    if (iterate_link->id() == cur_link->id() &&
        iterate_link->id() == current_link_->id()) {
      sum_dis = sum_dis + ego_on_cur_link_s_;
    } else {
      sum_dis = sum_dis + iterate_link->length() * 0.01;
    }

    if (sum_dis > search_dis) {
      return std::numeric_limits<double>::max();
    }

    if (pre_link->successor_link_ids_size() > 1) {
      return sum_dis;
    }

    iterate_link = pre_link;
  }

  return std::numeric_limits<double>::max();
}

double LDRouteInfoStrategy::CalculateDisToLastLinkMergePoint(
    const iflymapdata::sdpro::LinkInfo_Link* cur_link) const {
  if (cur_link == nullptr) {
    return std::numeric_limits<double>::max();
  }

  const double search_dis = 2000.0;  // 最多搜索2km，找多了也没必要

  double sum_dis = 0.0;

  const iflymapdata::sdpro::LinkInfo_Link* iterate_link = cur_link;

  while (iterate_link) {
    if (iterate_link->id() == cur_link->id() &&
        iterate_link->id() == current_link_->id()) {
      sum_dis = sum_dis + ego_on_cur_link_s_;
    } else {
      sum_dis = sum_dis + iterate_link->length() * 0.01;
    }

    if (sum_dis > search_dis) {
      return std::numeric_limits<double>::max();
    }

    if (iterate_link->predecessor_link_ids_size() > 1) {
      return sum_dis;
    }

    const auto& pre_link = ld_map_.GetPreviousLinkOnRoute(iterate_link->id());
    if (pre_link == nullptr) {
      return std::numeric_limits<double>::max();
    }

    iterate_link = pre_link;
  }

  return std::numeric_limits<double>::max();
}

bool LDRouteInfoStrategy::IsSucMergeLink(
    const iflymapdata::sdpro::LinkInfo_Link* link_info) const {
  if (link_info == nullptr) {
    return false;
  }

  for (const auto& temp_lane_id : link_info->lane_ids()) {
    const auto* temp_lane = ld_map_.GetLaneInfoByID(temp_lane_id);
    if (temp_lane == nullptr) {
      continue;
    }

    if (IsLaneSuccessorIsMergeLane(temp_lane)) {
      return true;
    }

  }

  return false;
}

bool LDRouteInfoStrategy::CalculateSplitLinkExitLane(
    const iflymapdata::sdpro::LinkInfo_Link* split_link,
    const iflymapdata::sdpro::LinkInfo_Link* out_link,
    std::vector<iflymapdata::sdpro::Lane>& exit_lane_vec) const {
  if (split_link == nullptr || out_link == nullptr) {
    return false;
  }

  exit_lane_vec.clear();

  for (const auto& lane_id : out_link->lane_ids()) {
    const auto& lane = ld_map_.GetLaneInfoByID(lane_id);
    if (lane == nullptr) {
      continue;
    }

    if (IsDiversionLane(lane) || IsEmergencyLane(lane)) {
      continue;
    }

    for (const auto& pre_lane_id: lane->predecessor_lane_ids()) {
      const auto& pre_lane = ld_map_.GetLaneInfoByID(pre_lane_id);
      if (pre_lane == nullptr) {
        continue;
      }

      if (IsDiversionLane(pre_lane) || IsEmergencyLane(pre_lane)) {
        continue;
      }

      if (pre_lane->link_id() == split_link->id() && IsExitLane(pre_lane) &&
          !HasLaneId(exit_lane_vec, pre_lane->id())) {
        exit_lane_vec.emplace_back(*pre_lane);
      }
    }
  }

  return !exit_lane_vec.empty();
}

bool LDRouteInfoStrategy::IsNeedFilterSplit(
    const iflymapdata::sdpro::Lane* lane) const {
  if (lane == nullptr) {
    return false;
  }

  const iflymapdata::sdpro::Lane* iter_lane = lane;

  while (iter_lane) {
    if (!IsDecelerateLane(iter_lane) && !IsExitLane(iter_lane)) {
      return false;
    }

    if (iter_lane->lane_transiton() == iflymapdata::sdpro::LTS_LANE_ADD_RIGHT ||
        iter_lane->lane_transiton() == iflymapdata::sdpro::LTS_LANE_ADD_LEFT) {
      return true;
    }

    if (iter_lane->predecessor_lane_ids_size() != 1) {
      return false;
    }

    iter_lane = ld_map_.GetLaneInfoByID(iter_lane->predecessor_lane_ids()[0]);
  }

  return false;
}

void LDRouteInfoStrategy::IsPreLaneOnOtherLink(
    TopoLinkGraph& before_split_feasible_lane_graph,
    std::unordered_map<int, int>& order_diff) {
  const int link_size =
      before_split_feasible_lane_graph.lane_topo_groups.size();
  const double extend_distance =
      current_link_->link_class() ==
              iflymapdata::sdpro::LinkClass::LC_EXPRESSWAY
          ? 800.
          : 500.;

  for (int i = 0; i < link_size; i++) {
    auto& temp_link_topo = before_split_feasible_lane_graph.lane_topo_groups[i];
    auto& temp_topo_lanes = temp_link_topo.topo_lanes;

    if (temp_topo_lanes.empty()) {
      continue;
    }

    if (i > 0) {
      for (auto& temp_topo_lane : temp_topo_lanes) {
        int max_inherited_diff = 0;
        bool has_valid_successor = false;

        for (const auto& next_topo_lane :
             before_split_feasible_lane_graph.lane_topo_groups[i - 1]
                 .topo_lanes) {
          if (temp_topo_lane.successor_lane_ids.find(next_topo_lane.id) !=
              temp_topo_lane.successor_lane_ids.end()) {
            has_valid_successor = true;
            if (order_diff.count(next_topo_lane.id)) {
              max_inherited_diff =
                  std::max(max_inherited_diff, order_diff[next_topo_lane.id]);
            }
          }
        }

        int current_lane_diff = 0;

        if (!has_valid_successor) {
          int sequence_diff = std::numeric_limits<int>::max();
          for (const auto& next_topo_lane :
               before_split_feasible_lane_graph.lane_topo_groups[i - 1]
                   .topo_lanes) {
            int diff = std::abs(
                temp_link_topo.lane_nums -
                static_cast<int>(temp_topo_lane.order_id) -
                before_split_feasible_lane_graph.lane_topo_groups[i - 1]
                    .lane_nums +
                static_cast<int>(next_topo_lane.order_id));
            if (diff < sequence_diff) sequence_diff = diff;
          }
          current_lane_diff = (sequence_diff == std::numeric_limits<int>::max())
                                  ? 0
                                  : sequence_diff;
        } else {
          current_lane_diff = max_inherited_diff;
        }

        if (current_lane_diff > 0) {
          order_diff[temp_topo_lane.id] = current_lane_diff;
          temp_topo_lane.front_feasible_distance =
              std::max(temp_topo_lane.front_feasible_distance -
                           current_lane_diff * extend_distance,
                       0.0);
        }
      }
    }
  }
}

bool LDRouteInfoStrategy::IsTheLaneOnSide(const iflymapdata::sdpro::Lane* lane,
                                          bool& is_leftest,
                                          bool& is_rightest) const {
  is_leftest = false;
  is_rightest = false;
  if (lane == nullptr) {
    return false;
  }
  const auto* link = ld_map_.GetLinkOnRoute(lane->link_id());
  if (link == nullptr) {
    return false;
  }
  std::map<int, int> lane_info;
  for (const auto& lane_id : link->lane_ids()) {
    const auto* current_lane = ld_map_.GetLaneInfoByID(lane_id);
    if (current_lane == nullptr) {
      continue;
    }
    if (IsEmergencyLane(current_lane) || IsDiversionLane(current_lane) ||
        IsExitLane(current_lane)) {
      continue;
    }
    lane_info.emplace(current_lane->sequence(), current_lane->id());
  }
  if (lane_info.empty()) {
    return false;
  }

  if (lane_info.rbegin()->second == lane->id()) {
    is_leftest = true;
    is_rightest = false;
  } else if (lane_info.begin()->second == lane->id()) {
    is_leftest = false;
    is_rightest = true;
  }

  return is_leftest || is_rightest;
}

bool LDRouteInfoStrategy::CalculateDistanceToCertainLink(
    const iflymapdata::sdpro::LinkInfo_Link* target_link,
    const iflymapdata::sdpro::LinkInfo_Link* current_link,
    double& link_distance) {
  if (target_link == nullptr || current_link == nullptr) {
    return false;
  }

  if (current_link->id() == current_link_->id() &&
      current_link->id() == target_link->id()) {
    link_distance = -ego_on_cur_link_s_;
    return true;
  }

  double accumulated_dist = 0.0;
  const auto* iter_link = current_link;

  const double MaxSearchDistance = 3000.0;

  while (iter_link != nullptr) {
    if (iter_link->id() == current_link_->id()) {
      double remaining_length = iter_link->length() * 0.01 - ego_on_cur_link_s_;
      accumulated_dist += std::max(0.0, remaining_length);
    } else {
      accumulated_dist += iter_link->length() * 0.01;
    }

    iter_link = ld_map_.GetNextLinkOnRoute(iter_link->id());
    if (iter_link == nullptr) {
      return false;
    }

    if (iter_link->id() == target_link->id()) {
      link_distance = accumulated_dist;
      return true;
    }

    if (accumulated_dist > MaxSearchDistance) {
      return false;
    }
  }

  return false;
}
void LDRouteInfoStrategy::CalculateFeasibleLaneByMergePoint(
    TopoLinkGraph& feasible_lane_graph) {
  if (route_info_output_.map_merge_points_info.empty()) {
    return;
  }

  // 1. 建立Link到index的映射
  std::unordered_map<uint64, size_t> link_id_to_index;
  for (size_t i = 0; i < feasible_lane_graph.lane_topo_groups.size(); ++i) {
    link_id_to_index[feasible_lane_graph.lane_topo_groups[i].link_id] = i;
  }

  for (const auto& merge_point_info :
       route_info_output_.map_merge_points_info) {
    if (merge_point_info.dis_to_merge_fp >
        mlc_decider_scene_type_info_.dis_to_link_topo_change_point - kEpsilon) {
      continue;
    }

    auto it = link_id_to_index.find(merge_point_info.merge_lane_link_id);
    if (it == link_id_to_index.end()) {
      continue;
    }

    size_t merge_point_link_idx = it->second;
    const iflymapdata::sdpro::Lane* current_trace_lane = nullptr;
    double accumulated_dist = 0.0;

    // 2. 处理汇合点所在的Link内的该车道
    for (auto& lane : feasible_lane_graph.lane_topo_groups[merge_point_link_idx]
                          .topo_lanes) {
      if (merge_point_info.merge_lane_sequence == lane.order_id) {
        double lane_len_m = lane.length;

        if (lane.link_id == current_link_->id()) {
          lane.front_feasible_distance =
              std::max(0.0, lane_len_m - ego_on_cur_link_s_);
        } else {
          lane.front_feasible_distance = lane_len_m;
        }

        accumulated_dist = lane_len_m;
        current_trace_lane = ld_map_.GetLaneInfoByID(lane.id);
        break;
      }
    }

    if (current_trace_lane == nullptr) {
      continue;
    }

    // 使用队列处理所有前继车道分支
    std::queue<std::pair<const iflymapdata::sdpro::Lane*, double>> lane_queue;
    lane_queue.push({current_trace_lane, accumulated_dist});

    while (!lane_queue.empty()) {
      auto [current_lane, current_dist] = lane_queue.front();
      lane_queue.pop();

      if (current_lane->link_id() == current_link_->id()) {
        continue;
      }

      for (auto id : current_lane->predecessor_lane_ids()) {
        const auto* pre_lane_info = ld_map_.GetLaneInfoByID(id);
        if (pre_lane_info == nullptr) {
          continue;
        }
        if (!ld_map_.isOnRouteLinks(pre_lane_info->link_id())) {
          continue;
        }

        auto pre_it = link_id_to_index.find(pre_lane_info->link_id());
        if (pre_it == link_id_to_index.end()) {
          continue;
        }

        size_t pre_link_idx = pre_it->second;
        double pre_lane_len = pre_lane_info->length() * 0.01;

        for (auto& lane_node :
             feasible_lane_graph.lane_topo_groups[pre_link_idx].topo_lanes) {
          if (lane_node.id == id) {
            double dist_to_merge;

            if (lane_node.link_id == current_link_->id()) {
              dist_to_merge = (pre_lane_len - ego_on_cur_link_s_) + current_dist;
            } else {
              dist_to_merge = pre_lane_len + current_dist;
            }

            lane_node.front_feasible_distance =
                std::min(lane_node.front_feasible_distance, dist_to_merge);

            lane_queue.push({pre_lane_info, dist_to_merge});
            break;
          }
        }
      }
    }
  }
}

bool LDRouteInfoStrategy::CalculateLinkLaneNum(
    const iflymapdata::sdpro::LinkInfo_Link* link, int& lane_num) {
  if (link == nullptr) {
    return false;
  }
  std::map<int, int> lane_info;
  for (const auto& lane_id : link->lane_ids()) {
    const auto* current_lane = ld_map_.GetLaneInfoByID(lane_id);
    if (current_lane == nullptr) {
      continue;
    }
    if (IsEmergencyLane(current_lane) || IsDiversionLane(current_lane) ||
        IsExitLane(current_lane)) {
      continue;
    }
    lane_info.emplace(current_lane->sequence(), current_lane->id());
  }
  if (lane_info.empty()) {
    return false;
  }
  lane_num = lane_info.size();
  return true;
}

double LDRouteInfoStrategy::CalculateLSLLengthBetweenLanes(
    const iflymapdata::sdpro::LinkInfo_Link* link,
    uint32_t from_seq, uint32_t to_seq) {
  if (link == nullptr || from_seq == to_seq) {
    return 0.0;
  }

  double total_lsl_length = 0.0;
  uint32_t min_seq = std::min(from_seq, to_seq);
  uint32_t max_seq = std::max(from_seq, to_seq);

  for (uint32_t seq = min_seq; seq < max_seq; ++seq) {
    const iflymapdata::sdpro::Lane* lane = nullptr;
    double current_lane_lsl = 0.0;
    for (const auto& lane_id : link->lane_ids()) {
      const auto* temp_lane = ld_map_.GetLaneInfoByID(lane_id);
      if (temp_lane && temp_lane->sequence() == seq) {
        lane = temp_lane;
        break;
      }
    }

    if (!lane) continue;

    for (const auto& boundary : lane->left_boundaries()) {
      const auto type = boundary.divider_marking_type();
      bool is_solid = false;
      if (type ==
              iflymapdata::sdpro::LaneBoundary::DivederMarkingType::
                  LaneBoundary_DivederMarkingType_DMT_MARKING_SINGLE_SOLID_LINE ||
          type ==
              iflymapdata::sdpro::LaneBoundary::DivederMarkingType::
                  LaneBoundary_DivederMarkingType_DMT_MARKING_DOUBLE_SOLID_LINE) {
        is_solid = true;
      } else if (from_seq < to_seq) {
        // 向左变道，穿越左边界右侧，右实左虚算实线
        is_solid =
            (type ==
             iflymapdata::sdpro::LaneBoundary::DivederMarkingType::
                 LaneBoundary_DivederMarkingType_DMT_MARKING_RIGHT_SOLID_LINE_LEFT_DASHED_LINE);
      } else {
        // 向右变道，穿越左边界左侧，左实右虚算实线
        is_solid =
            (type ==
             iflymapdata::sdpro::LaneBoundary::DivederMarkingType::
                 LaneBoundary_DivederMarkingType_DMT_MARKING_LEFT_SOLID_LINE_RIGHT_DASHED_LINE);
      }
      if (is_solid) {
        current_lane_lsl += boundary.length() * 0.01;
      }
    }
    total_lsl_length = std::max(total_lsl_length, current_lane_lsl);
  }

  return total_lsl_length;
}
// Creates a LaneTopoNode for the given link.
// Each lane in the link is classified as on_route or off_route based on
// whether its successor lane's link is on the navigation route.
// Extracts recommended_orders from feasible_orders_by_link (upstream
// FeasibleOrder). DiversionLanes are excluded, and the remaining lane orders
// are adjusted.
LaneTopoNode LDRouteInfoStrategy::CreateNodeFromLink(
    const iflymapdata::sdpro::LinkInfo_Link* link, double begin_dist,
    double end_dist, const std::unordered_set<uint64_t>& route_link_ids,
    const std::unordered_map<uint64_t, std::vector<int>>&
        feasible_orders_by_link) {
  LaneTopoNode node;
  node.link_id = link->id();
  node.begin_dist = begin_dist;
  node.end_dist = end_dist;

  bool link_on_route = (route_link_ids.count(link->id()) > 0);

  // Gather DiversionLane sequences so we can adjust orders
  std::vector<int> diversion_seqs;
  for (uint64_t lane_id : link->lane_ids()) {
    const auto* lane = ld_map_.GetLaneInfoByID(lane_id);
    if (IsDiversionLane(lane)) {
      diversion_seqs.push_back(static_cast<int>(lane->sequence()));
    }
  }

  // Extract recommended orders (FeasibleOrder) from upstream
  std::vector<int> raw_feasible_orders;
  auto it = feasible_orders_by_link.find(link->id());
  if (it != feasible_orders_by_link.end()) {
    raw_feasible_orders = it->second;
  }

  // Adjust feasible orders for DiversionLane offset
  for (int raw_ord : raw_feasible_orders) {
    int adj_ord = raw_ord;
    for (int div_seq : diversion_seqs) {
      if (raw_ord > div_seq) {
        adj_ord--;
      }
    }
    node.recommended_orders.push_back(adj_ord);
  }

  // Classify each non-DiversionLane
  for (uint64_t lane_id : link->lane_ids()) {
    const auto* lane = ld_map_.GetLaneInfoByID(lane_id);
    if (lane == nullptr || IsDiversionLane(lane)) {
      continue;
    }
    int raw_order = static_cast<int>(lane->sequence());

    // Adjust order by subtracting the number of DiversionLane seqs < raw_order
    int adj_order = raw_order;
    for (int div_seq : diversion_seqs) {
      if (raw_order > div_seq) {
        adj_order--;
      }
    }

    if (!link_on_route) {
      // The entire link is off-route
      node.off_route_orders.push_back(adj_order);
    } else {
      // Check if at least one successor lane leads to a route link
      bool successor_on_route = false;
      for (uint64_t suc_lane_id : lane->successor_lane_ids()) {
        const auto* suc_lane = ld_map_.GetLaneInfoByID(suc_lane_id);
        if (suc_lane == nullptr) {
          continue;
        }
        if (route_link_ids.count(suc_lane->link_id()) > 0) {
          successor_on_route = true;
          break;
        }
      }
      // Also handle lanes with no successors (end of route): treat as on_route
      if (lane->successor_lane_ids().empty()) {
        successor_on_route = true;
      }
      if (successor_on_route) {
        node.on_route_orders.push_back(adj_order);
      } else {
        node.off_route_orders.push_back(adj_order);
      }
    }
    node.lane_num++;
  }

  return node;
}

// Builds the complete VirtualLaneTopoGraph covering [-50m, +150m] around ego.
// Uses BFS forward through successor links (up to 150m) and backward through
// predecessor links (up to 50m or ego's position on current link).
// Returns false when ego is on a link that is not on the navigation route
// (wrong_route scenario), in which case topo_graph is left empty.
bool LDRouteInfoStrategy::BuildVirtualLaneTopoGraph(
    const TopoLinkGraph& feasible_lane_graph,
    VirtualLaneTopoGraph& topo_graph) {
  topo_graph.nodes.clear();
  topo_graph.ego_node_index = -1;

  if (current_link_ == nullptr) {
    return false;
  }

  // Ego must be on route
  if (ld_map_.GetLinkOnRoute(current_link_->id()) == nullptr) {
    return false;
  }

  // Collect route link ids and feasible orders from feasible_lane_graph
  std::unordered_set<uint64_t> route_link_ids;
  std::unordered_map<uint64_t, std::vector<int>> feasible_orders_by_link;

  route_link_ids.insert(current_link_->id());
  for (const auto& group : feasible_lane_graph.lane_topo_groups) {
    route_link_ids.insert(group.link_id);

    // Extract feasible orders (recommended lanes) for this link
    std::vector<int> feasible_orders;
    for (const auto& topo_lane : group.topo_lanes) {
      feasible_orders.push_back(static_cast<int>(topo_lane.order_id));
    }
    feasible_orders_by_link[group.link_id] = feasible_orders;
  }

  // Visited set to avoid revisiting (handles diamond merges etc.)
  std::unordered_set<uint64_t> visited_fwd;

  // Create ego node first
  {
    double cur_link_len = current_link_->length() * 0.01;  // cm to m
    double remaining_len = std::max(0.0, cur_link_len - ego_on_cur_link_s_);
    LaneTopoNode ego_node =
        CreateNodeFromLink(current_link_, 0.0, remaining_len, route_link_ids,
                           feasible_orders_by_link);
    topo_graph.ego_node_index = static_cast<int>(topo_graph.nodes.size());
    topo_graph.nodes.push_back(std::move(ego_node));
    visited_fwd.insert(current_link_->id());
  }

  // BFS forward: process a queue of (link_id, begin_dist, parent_node_idx)
  struct FwdEntry {
    uint64_t link_id;
    double begin_dist;
    int parent_index;
  };
  std::queue<FwdEntry> fwd_queue;
  // Seed from current link's successors
  {
    double ego_node_end = topo_graph.nodes[topo_graph.ego_node_index].end_dist;
    for (uint64_t suc_id : current_link_->successor_link_ids()) {
      if (ego_node_end < 150.0) {
        fwd_queue.push({suc_id, ego_node_end, topo_graph.ego_node_index});
      }
    }
  }

  while (!fwd_queue.empty()) {
    auto [link_id, begin_dist, parent_idx] = fwd_queue.front();
    fwd_queue.pop();

    if (visited_fwd.count(link_id)) {
      continue;
    }
    visited_fwd.insert(link_id);

    // Get link ptr — only available for route links via GetLinkOnRoute
    const auto* link_ptr = ld_map_.GetLinkOnRoute(link_id);
    double link_len = 50.0;  // nominal fallback for off-route links
    if (link_ptr != nullptr) {
      link_len = link_ptr->length() * 0.01;
    }
    double end_dist = begin_dist + link_len;

    LaneTopoNode node =
        (link_ptr != nullptr)
            ? CreateNodeFromLink(link_ptr, begin_dist, end_dist, route_link_ids,
                                 feasible_orders_by_link)
            : LaneTopoNode{link_id, begin_dist, end_dist, 0, {},
                           {},      {},         {},       {}};

    int new_idx = static_cast<int>(topo_graph.nodes.size());
    topo_graph.nodes.push_back(std::move(node));

    // Link parent->child edges
    topo_graph.nodes[parent_idx].successor_indices.push_back(new_idx);
    topo_graph.nodes[new_idx].predecessor_indices.push_back(parent_idx);

    // Enqueue successors if within range and link is on-route
    if (end_dist < 150.0 && link_ptr != nullptr) {
      for (uint64_t suc_id : link_ptr->successor_link_ids()) {
        if (!visited_fwd.count(suc_id)) {
          fwd_queue.push({suc_id, end_dist, new_idx});
        }
      }
    }
  }

  // --- Backward BFS from current_link ---
  // Only needed if ego hasn't traveled 50m from link start
  double need_backward = 50.0 - ego_on_cur_link_s_;
  if (need_backward > 0.0) {
    struct BwdEntry {
      uint64_t link_id;
      double end_dist;  // negative (behind ego)
      int child_index;
    };
    std::queue<BwdEntry> bwd_queue;
    std::unordered_set<uint64_t> visited_bwd;
    visited_bwd.insert(current_link_->id());

    for (int i = 0; i < current_link_->predecessor_link_ids_size(); ++i) {
      uint64_t pre_id = current_link_->predecessor_link_ids()[i];
      bwd_queue.push({pre_id, 0.0, topo_graph.ego_node_index});
    }

    double acc_backward = 0.0;
    while (!bwd_queue.empty() && acc_backward < need_backward) {
      auto [link_id, end_dist_neg, child_idx] = bwd_queue.front();
      bwd_queue.pop();

      if (visited_bwd.count(link_id)) {
        continue;
      }
      visited_bwd.insert(link_id);

      const auto* link_ptr = ld_map_.GetLinkOnRoute(link_id);
      double link_len = 50.0;
      if (link_ptr != nullptr) {
        link_len = link_ptr->length() * 0.01;
      }

      double end_dist = end_dist_neg;  // 0 means at ego's link start
      double begin_dist = end_dist - link_len;

      LaneTopoNode node =
          (link_ptr != nullptr)
              ? CreateNodeFromLink(link_ptr, begin_dist, end_dist,
                                   route_link_ids, feasible_orders_by_link)
              : LaneTopoNode{link_id, begin_dist, end_dist, 0, {},
                             {},      {},         {},       {}};

      int new_idx = static_cast<int>(topo_graph.nodes.size());
      topo_graph.nodes.push_back(std::move(node));

      // Edge: predecessor node -> ego node direction
      topo_graph.nodes[new_idx].successor_indices.push_back(child_idx);
      topo_graph.nodes[child_idx].predecessor_indices.push_back(new_idx);

      acc_backward += link_len;
      if (acc_backward < need_backward && link_ptr != nullptr) {
        for (int i = 0; i < link_ptr->predecessor_link_ids_size(); ++i) {
          uint64_t pre_id = link_ptr->predecessor_link_ids()[i];
          if (!visited_bwd.count(pre_id)) {
            bwd_queue.push({pre_id, begin_dist, new_idx});
          }
        }
      }
    }
  }

  return true;
}

// Calculate VirtualLaneRouteCost for each VirtualLane using
// VirtualLaneTopoGraph. The graph covers [-50m, +150m] around ego and
// explicitly classifies each lane's order as on_route (successor leads to
// route) or off_route (successor diverges from route). This enables three-tier
// distance penalties and off_route negative scoring to guarantee cost
// monotonicity.
std::vector<VirtualLaneRouteCost>
LDRouteInfoStrategy::CalculateVirtualLaneRouteCost(
    const std::vector<std::shared_ptr<VirtualLane>>& relative_id_lanes,
    const TopoLinkGraph& feasible_lane_graph) {
  std::vector<VirtualLaneRouteCost> lane_costs;
  if (feasible_lane_graph.lane_topo_groups.empty() ||
      current_link_ == nullptr) {
    return lane_costs;
  }
  const auto& cur_link_topo = feasible_lane_graph.lane_topo_groups.back();
  if (cur_link_topo.topo_lanes.empty()) {
    return lane_costs;
  }

  double max_feasible_distance = 0.0;
  for (const auto& topo_lane : cur_link_topo.topo_lanes) {
    max_feasible_distance =
        std::max(max_feasible_distance, topo_lane.front_feasible_distance);
  }
  int num_groups =
      static_cast<int>(feasible_lane_graph.lane_topo_groups.size());

  // Build VirtualLaneTopoGraph. Returns false if ego is off route
  // (wrong_route).
  VirtualLaneTopoGraph topo_graph;
  bool ego_on_route =
      BuildVirtualLaneTopoGraph(feasible_lane_graph, topo_graph);

  // Infer ego order from perception (rel=0 VirtualLane's first valid
  // LaneNumMsg)
  int ego_order_from_perception = -1;
  for (const auto& vl : relative_id_lanes) {
    if (vl == nullptr || vl->get_relative_id() != 0) {
      continue;
    }
    for (const auto& ln : vl->get_lane_nums()) {
      if (ln.end > ln.begin && ln.left_lane_num <= 20 &&
          ln.right_lane_num <= 20) {
        ego_order_from_perception = ln.right_lane_num + 1;
        break;
      }
    }
    break;
  }

  // Check if ego lane (rel=0) has valid lane_nums; if not, skip this frame
  if (ego_order_from_perception < 0) {
    return lane_costs;
  }

  // Maximum distance covered by forward nodes (for coverage check)
  double max_node_dist = 0.0;
  for (const auto& node : topo_graph.nodes) {
    if (node.end_dist > max_node_dist) {
      max_node_dist = node.end_dist;
    }
  }
  if (max_node_dist < 30.0) {
    return lane_costs;
  }

  // --- Extract all possible topological paths from topo_graph ---
  // This enables lateral relationship matching based on order_id
  std::vector<TopoPath> const& topo_paths = ExtractTopoPaths(topo_graph, 150.0);

  for (const auto& virtual_lane : relative_id_lanes) {
    if (virtual_lane == nullptr) {
      continue;
    }

    VirtualLaneRouteCost cost;
    cost.order_id = virtual_lane->get_order_id();
    cost.relative_id = virtual_lane->get_relative_id();
    const auto& lane_nums = virtual_lane->get_lane_nums();

    // Skip VirtualLane with invalid lane_nums
    bool has_valid_lane_num = false;
    if (lane_nums.size() < 100) {
      for (const auto& ln : lane_nums) {
        if (ln.end > ln.begin && ln.left_lane_num <= 20 &&
            ln.right_lane_num <= 20) {
          has_valid_lane_num = true;
          break;
        }
      }
    }

    if (!has_valid_lane_num) {
      cost.CalculateTotalCost();
      lane_costs.push_back(cost);
      continue;
    }

    // --- Wrong route: penalize everything ---
    if (!ego_on_route) {
      cost.distance_penalty = 1.0;
      cost.CalculateTotalCost();
      lane_costs.push_back(cost);
      continue;
    }

    // --- Lane match confidence via topological path matching ---
    // For each TopoPath, calculate match score. Pick the best-matching path.
    // Then determine on/off route based on the best path's nodes.
    double best_match_score = -1.0;
    int best_path_idx = -1;
    for (size_t pi = 0; pi < topo_paths.size(); ++pi) {
      double const path_score =
          CalculateMatchScore(virtual_lane, topo_paths[pi], 3.0, 120.0);
      if (path_score > 0.0 && path_score > best_match_score) {
        best_match_score = path_score;
        best_path_idx = static_cast<int>(pi);
      }
    }

    // --- Feasible distance reward ---
    // TODO: each FeasibleOrder has its own distance; skip for now
    cost.feasible_distance_reward = 0.0;

    // --- Topo trace score ---
    cost.topo_trace_score =
        std::min(1.0, static_cast<double>(num_groups) / 8.0);

    // --- Distance penalty ---
    // Determined by the best-matching TopoPath:
    //   - If best path's nodes are all on_route → low penalty
    //   - If best path contains off_route nodes → high penalty
    // Additionally use ego node's recommended_orders for fine-grained ordering.
    if (best_path_idx >= 0) {
      const TopoPath& best_path = topo_paths[best_path_idx];

      // Check if the best path is on-route:
      // A path is on-route if all nodes beyond feasible order.
      bool const path_has_on_route =
          std::find_if(best_path.nodes_ptr.begin(), best_path.nodes_ptr.end(),
                       [](std::shared_ptr<planning::LaneTopoNode> const node) {
                         return node->recommended_orders.empty();
                       }) == best_path.nodes_ptr.end();

      cost.on_link_route_score = best_match_score;

      cost.is_on_route = path_has_on_route;

      if (!path_has_on_route) {
        // Path leads entirely off-route
        cost.distance_penalty = 1.0;
      } else {
        // Sample and match observed order with recommended order
        double total_match_error = 0.0;
        int valid_samples = 0;

        for (double dist = 0.0; dist <= 120.0; dist += 5.0) {
          // Get topological node
          std::shared_ptr<LaneTopoNode> node =
              best_path.GetNodeAtDistance(dist);
          if (node == nullptr || node->recommended_orders.empty() ||
              node->lane_num == 0) {
            continue;
          }

          // Get observed values at this distance from lane_nums
          int per_left = -1;
          int per_right = -1;
          for (const auto& ln : lane_nums) {
            if (ln.end <= ln.begin || ln.left_lane_num > 20 ||
                ln.right_lane_num > 20) {
              continue;
            }
            if (dist >= ln.begin && dist < ln.end) {
              per_left = static_cast<int>(ln.left_lane_num);
              per_right = static_cast<int>(ln.right_lane_num);
              break;
            }
          }
          if (per_left < 0 || per_right < 0) {
            continue;  // No observation at this distance
          }

          // Compute observed order (right‑to‑left, 1‑based)
          int per_order_from_right = per_right + 1;
          if (per_order_from_right > node->lane_num) {
            per_order_from_right = node->lane_num;
          }

          int per_order_from_left = node->lane_num - per_left;
          if (per_order_from_left < 1) {
            per_order_from_left = 1;
          }

          // Weighted fusion of left and right observations
          // Smaller observation value indicates higher reliability
          const double epsilon = 1e-6;
          double weight_left = 1.0 / (per_left + 1.0 + epsilon);
          double weight_right = 1.0 / (per_right + 1.0 + epsilon);
          double total_weight = weight_left + weight_right;
          double per_order = 0.0;
          if (total_weight > epsilon) {
            per_order = (weight_left * per_order_from_left +
                         weight_right * per_order_from_right) /
                        total_weight;
          } else {
            // Fallback: use average of the two orders
            per_order = 0.5 * (per_order_from_left + per_order_from_right);
          }

          // Compute distance between observed order and the nearest recommended
          // order
          double min_dist = std::numeric_limits<double>::max();
          for (int rec_ord : node->recommended_orders) {
            min_dist = std::min(min_dist, std::abs(per_order - rec_ord));
          }

          // Compute confidence weight based on lane number difference
          int const per_total = per_left + per_right + 1;
          int const lane_num_diff = std::abs(per_total - node->lane_num);

          // lane_num_diff > 2. no need consider this sample.
          if (lane_num_diff > 2) {
            continue;
          }

          // Exponential decay weight: exp(-diff / 2.0)
          double const lane_num_diff_weight = std::exp(-lane_num_diff / 2.0);

          // Accumulate error (normalized to 0‑1)
          double error =
              lane_num_diff_weight * min_dist / std::max(1, node->lane_num);
          total_match_error += error;
          ++valid_samples;
        }

        // Compute lane_match_confidence (higher confidence when error is
        // smaller)
        if (valid_samples > 0) {
          double avg_error = total_match_error / valid_samples;
          cost.lane_match_confidence = 1.0 - std::min(1.0, avg_error);
        } else {
          cost.lane_match_confidence = 0.0;
        }
      }
    } else {
      // Ambiguous — no clear on/off signal
      cost.distance_penalty = 0.5;
    }

    cost.CalculateTotalCost();
    lane_costs.push_back(cost);
  }

  return lane_costs;
}

// Extract all possible topological paths from topo_graph.
// For each transition between nodes, compute exit_orders: which lane orders
// in the parent node connect (via lane successor) to the child node's link.
std::vector<TopoPath> LDRouteInfoStrategy::ExtractTopoPaths(
    const VirtualLaneTopoGraph& topo_graph, double max_distance) {
  std::vector<TopoPath> paths;

  if (topo_graph.ego_node_index < 0) {
    return paths;
  }

  // DFS to traverse all possible paths from ego node
  std::function<void(int, TopoPath)> dfs = [&](int node_idx,
                                               TopoPath current_path) {
    const auto& node = topo_graph.nodes[node_idx];
    // If no lane, save path
    if (node.lane_num == 0) {
      paths.push_back(current_path);
      return;
    }

    current_path.nodes_ptr.push_back(std::make_shared<LaneTopoNode>(node));

    // If exceeded max distance, save path and return
    if (node.begin_dist >= max_distance) {
      paths.push_back(current_path);
      return;
    }

    // If no successors, save path
    if (node.successor_indices.empty()) {
      paths.push_back(current_path);
      return;
    }

    // For each successor, compute exit_orders: which lane orders in current
    // node connect to the successor node's link via lane-level successor.
    for (int suc_idx : node.successor_indices) {
      const auto& suc_node = topo_graph.nodes[suc_idx];

      // Find which lanes in current link have successor lanes in suc_node's
      // link
      std::vector<int> cur_exit_orders;
      const auto* link_ptr = ld_map_.GetLinkOnRoute(node.link_id);
      if (link_ptr != nullptr) {
        // Gather DiversionLane sequences for order adjustment
        std::vector<int> diversion_seqs;
        for (uint64_t lane_id : link_ptr->lane_ids()) {
          const auto* lane = ld_map_.GetLaneInfoByID(lane_id);
          if (lane != nullptr && IsDiversionLane(lane)) {
            diversion_seqs.push_back(static_cast<int>(lane->sequence()));
          }
        }

        for (uint64_t lane_id : link_ptr->lane_ids()) {
          const auto* lane = ld_map_.GetLaneInfoByID(lane_id);
          if (lane == nullptr || IsDiversionLane(lane)) {
            continue;
          }

          // Check if any successor lane belongs to suc_node's link
          bool connects_to_suc = false;
          for (uint64_t suc_lane_id : lane->successor_lane_ids()) {
            const auto* suc_lane = ld_map_.GetLaneInfoByID(suc_lane_id);
            if (suc_lane != nullptr &&
                suc_lane->link_id() == suc_node.link_id) {
              connects_to_suc = true;
              break;
            }
          }

          if (connects_to_suc) {
            // Adjust order for DiversionLane offset
            int adj_order = static_cast<int>(lane->sequence());
            for (int div_seq : diversion_seqs) {
              if (adj_order > div_seq) {
                adj_order--;
              }
            }
            cur_exit_orders.push_back(adj_order);
          }
        }
      }

      // Sort exit_orders for consistent comparison
      std::sort(cur_exit_orders.begin(), cur_exit_orders.end());

      TopoPath branch_path = current_path;
      branch_path.exit_orders.push_back(cur_exit_orders);
      dfs(suc_idx, branch_path);
    }
  };

  TopoPath initial_path;
  dfs(topo_graph.ego_node_index, initial_path);

  return paths;
}

// Calculate match score between VirtualLane and topological path.
//
// Core matching dimensions:
//   1. Longitudinal lane-count sequence matching
//   2. Exit-order matching at split points (pre-split order in exit_orders?)
//   3. Fork boundary detection: at the split distance, a VirtualLane that sees
//      one side drop to 0 (left=0 or right=0) has clearly entered a branch.
//      - If per_order was in exit_orders AND the opposite side drops to 0
//        → strong match (this VirtualLane follows this path)
//      - If per_order was NOT in exit_orders AND one side drops to 0
//        → strong mismatch (this VirtualLane follows the other branch)
//   4. Post-split on_route/off_route order matching
//
// Fork boundary detection is the strongest signal because it combines two
// independent observations: the lane-level connectivity (exit_orders) and
// the perception geometry (left/right dropping to 0).
double LDRouteInfoStrategy::CalculateMatchScore(
    const std::shared_ptr<VirtualLane>& virtual_lane, const TopoPath& topo_path,
    double sample_step, double max_distance) {
  if (virtual_lane == nullptr || topo_path.nodes_ptr.empty()) {
    return 0.0;
  }

  double total_score = 0.0;
  double total_weight = 0.0;

  // Build split info from node transitions + exit_orders
  struct SplitInfo {
    double dist;                 // begin_dist of the post-split node
    std::vector<int> exit_ords;  // which orders in the pre-split node feed here
    int pre_lane_num;            // lane_num of the pre-split node
  };
  std::vector<SplitInfo> splits;

  for (size_t i = 0; i + 1 < topo_path.nodes_ptr.size(); ++i) {
    int prev_ln = topo_path.nodes_ptr[i]->lane_num;
    int cur_ln = topo_path.nodes_ptr[i + 1]->lane_num;
    if (prev_ln - cur_ln >= 1 && i < topo_path.exit_orders.size()) {
      splits.push_back({topo_path.nodes_ptr[i + 1]->begin_dist,
                        topo_path.exit_orders[i], prev_ln});
    }
  }

  const auto& lane_nums = virtual_lane->get_lane_nums();

  // We need to look at perception data at two distances around each split
  // to detect the fork boundary pattern. Pre-compute perception at each sample.
  struct PerceptionSample {
    double dist;
    int left;
    int right;
    int total;
    int order_from_right;  // right + 1
  };
  std::vector<PerceptionSample> samples;

  for (double dist = 0.0; dist < max_distance; dist += sample_step) {
    int per_left = -1;
    int per_right = -1;
    for (const auto& ln : lane_nums) {
      if (ln.end <= ln.begin || ln.left_lane_num > 20 ||
          ln.right_lane_num > 20) {
        continue;
      }
      if (dist >= ln.begin && dist < ln.end) {
        per_left = static_cast<int>(ln.left_lane_num);
        per_right = static_cast<int>(ln.right_lane_num);
        break;
      }
    }
    if (per_left >= 0 && per_right >= 0) {
      samples.push_back(
          {dist, per_left, per_right, per_left + per_right + 1, per_right + 1});
    }
  }

  // For each split, detect fork boundary pattern across consecutive samples.
  //
  // Key insight: left/right=0 has TWO meanings:
  //   a) Fork: the other branch disappeared (pre>0, post=0) → STRONG signal
  //   b) Edge lane: this VirtualLane is at the leftmost/rightmost → NOT a fork
  // Only (a) is a fork signal. We detect it by checking pre>0 AND post=0.
  //
  // Fork direction determination (independent of order, which may have ±1
  // error):
  //   - right dropped (pre_right>0 → post_right=0): VirtualLane went LEFT
  //   branch
  //   - left dropped  (pre_left>0  → post_left=0):  VirtualLane went RIGHT
  //   branch
  //
  // This direction signal is then compared with the path's exit_orders
  // position:
  //   - exit_orders contain high orders → this path is the LEFT branch
  //   - exit_orders contain low orders  → this path is the RIGHT branch
  //
  // Even when two VirtualLanes have the SAME order (e.g. both order=2),
  // if one has right=0 and the other has left=0, they are on different paths.
  // Accumulators for all splits
  double accumulated_fork_score = 0.0;
  double accumulated_fork_weight = 0.0;
  bool has_any_fork_signal = false;

  for (const auto& split : splits) {
    if (split.exit_ords.empty()) {
      continue;
    }

    // Find samples at split.dist [-5.0, +10.0] m
    const PerceptionSample* pre_sample = nullptr;
    const PerceptionSample* post_sample = nullptr;

    const double target_pre_dist = split.dist - 5.0;
    const double target_post_dist = split.dist + 10.0;

    double min_pre_dist = std::numeric_limits<double>::max();
    double min_post_dist = std::numeric_limits<double>::max();

    for (const auto& sample : samples) {
      double dist_to_pre = std::abs(sample.dist - target_pre_dist);
      double dist_to_post = std::abs(sample.dist - target_post_dist);

      if (dist_to_pre <= min_pre_dist) {
        min_pre_dist = dist_to_pre;
        pre_sample = &sample;
      }
      if (dist_to_post <= min_post_dist) {
        min_post_dist = dist_to_post;
        post_sample = &sample;
      }
    }
    if (pre_sample == nullptr || post_sample == nullptr) {
      continue;
    }

    // Detect fork boundary: one side DROPS from >0 to 0.
    // This distinguishes fork signals from edge-lane signals:
    //   - Fork: pre_right=2, post_right=0 → right dropped → went left branch
    //   - Edge: pre_right=0, post_right=0 → always 0 → just the rightmost lane
    bool right_dropped = (pre_sample->right > 0 && post_sample->right == 0);
    bool left_dropped = (pre_sample->left > 0 && post_sample->left == 0);

    // Determine which side this path's exit_orders are on.
    // Low orders = right side of road, high orders = left side of road.
    int min_exit =
        *std::min_element(split.exit_ords.begin(), split.exit_ords.end());
    int max_exit =
        *std::max_element(split.exit_ords.begin(), split.exit_ords.end());
    double mid = (split.pre_lane_num + 1.0) / 2.0;
    bool path_is_right_branch = (max_exit <= mid);
    bool path_is_left_branch = (min_exit >= mid);
    // If exit_orders span the middle, the path covers both sides (no clear
    // branch)
    bool path_spans_middle = !path_is_right_branch && !path_is_left_branch;

    // Calculate current split match score
    double current_split_match = 0.0;
    bool current_has_signal = false;
    bool is_hard_signal = false;  // Flag for geometric signal

    if (right_dropped || left_dropped) {
      // We have a clear fork signal: one side dropped to 0.
      current_has_signal = true;
      is_hard_signal = true;  // This is a strong geometric signal

      // Determine VirtualLane's branch direction from the drop direction.
      // This is the most reliable signal — independent of order value.
      bool vl_went_left = right_dropped;  // right disappeared → went left
      bool vl_went_right = left_dropped;  // left disappeared → went right

      if (vl_went_left && path_is_left_branch) {
        current_split_match =
            1.0;  // VirtualLane went left, path is left branch → match
      } else if (vl_went_right && path_is_right_branch) {
        current_split_match =
            1.0;  // VirtualLane went right, path is right branch → match
      } else if (vl_went_left && path_is_right_branch) {
        current_split_match =
            -1.0;  // VirtualLane went left, path is right branch → mismatch
      } else if (vl_went_right && path_is_left_branch) {
        current_split_match =
            -1.0;  // VirtualLane went right, path is left branch → mismatch
      } else if (path_spans_middle) {
        // Path spans both sides; use order as secondary signal
        bool pre_order_in_exit =
            std::find(split.exit_ords.begin(), split.exit_ords.end(),
                      pre_sample->order_from_right) != split.exit_ords.end();
        int pre_order_from_left = split.pre_lane_num - pre_sample->left;
        bool pre_order_left_in_exit =
            std::find(split.exit_ords.begin(), split.exit_ords.end(),
                      pre_order_from_left) != split.exit_ords.end();
        current_split_match =
            (pre_order_in_exit || pre_order_left_in_exit) ? 0.5 : -0.5;
      }
    } else {
      // No clear fork boundary (no side dropped to 0).
      // Fall back to order-based matching with lower confidence.
      bool pre_order_in_exit =
          std::find(split.exit_ords.begin(), split.exit_ords.end(),
                    pre_sample->order_from_right) != split.exit_ords.end();
      int pre_order_from_left = split.pre_lane_num - pre_sample->left;
      bool pre_order_left_in_exit =
          std::find(split.exit_ords.begin(), split.exit_ords.end(),
                    pre_order_from_left) != split.exit_ords.end();

      if (pre_order_in_exit && pre_order_left_in_exit) {
        current_split_match = 0.5;
        current_has_signal = true;
      } else if (pre_order_in_exit || pre_order_left_in_exit) {
        current_split_match = 0.3;
        current_has_signal = true;
      } else if (!split.exit_ords.empty()) {
        current_split_match = -0.3;
        current_has_signal = true;
      }
    }

    // Accumulate score with reasonable weight
    if (current_has_signal) {
      has_any_fork_signal = true;

      // 1. Distance Weight: 0-60m consistent, 60m+ decay to 0.3
      double dist_weight = 1.0;
      if (split.dist > 60.0) {
        // Linear decay from 1.0 at 60m to 0.3 at max_distance
        dist_weight =
            std::max(0.3, 1.0 - (split.dist - 60.0) / (max_distance - 60.0));
      }

      // 2. Signal Type Weight: Geometric signals are more reliable than
      // Order-based ones
      double type_weight = is_hard_signal ? 1.5 : 1.0;

      double final_weight = dist_weight * type_weight;

      accumulated_fork_score += current_split_match * final_weight;
      accumulated_fork_weight += final_weight;
    }
  }

  // Calculate final weighted average
  double fork_match = 0.0;
  if (accumulated_fork_weight > 1e-6) {
    fork_match = accumulated_fork_score / accumulated_fork_weight;
  }
  bool has_fork_signal = has_any_fork_signal;

  // Now compute per-sample scores
  for (const auto& sample : samples) {
    // Find node along THIS path at the given distance
    const LaneTopoNode* path_node = nullptr;
    int node_idx = -1;
    for (size_t ni = 0; ni < topo_path.nodes_ptr.size(); ++ni) {
      const auto& n = topo_path.nodes_ptr[ni];
      if (sample.dist >= n->begin_dist && sample.dist < n->end_dist) {
        path_node = n.get();
        node_idx = static_cast<int>(ni);
        break;
      }
    }
    if (path_node == nullptr || path_node->lane_num <= 0) {
      continue;
    }

    // Longitudinal lane-count matching
    double lane_num_score = 0.0;
    int diff = std::abs(sample.total - path_node->lane_num);
    if (diff == 0) {
      lane_num_score = 1.0;
    } else if (diff == 1) {
      lane_num_score = 0.5;
    } else {
      lane_num_score = 0.1;
    }

    // Order matching via exit_orders: does this VirtualLane's observed order
    // fall within the lanes that connect to the next node on this path?
    double order_score = 0.0;
    bool has_order_signal = false;
    if (node_idx >= 0 &&
        node_idx < static_cast<int>(topo_path.exit_orders.size())) {
      const auto& exit_ords = topo_path.exit_orders[node_idx];
      if (!exit_ords.empty()) {
        has_order_signal = true;
        bool right_in = std::find(exit_ords.begin(), exit_ords.end(),
                                  sample.order_from_right) != exit_ords.end();
        int order_from_left = path_node->lane_num - sample.left;
        bool left_in = std::find(exit_ords.begin(), exit_ords.end(),
                                 order_from_left) != exit_ords.end();

        if (right_in && left_in) {
          order_score = (lane_num_score >= 0.5) ? 1.0 : 0.5;
        } else if (!right_in && !left_in) {
          order_score = -1.0;
        } else {
          // Disagreement: trust the side with smaller observation value
          if (sample.right <= sample.left) {
            order_score = right_in ? 0.5 : -0.5;
          } else {
            order_score = left_in ? 0.5 : -0.5;
          }
        }
      }
    }

    // Combine: when order_score is unavailable (last node, no exit_orders),
    // redistribute its weight to other signals to avoid path length bias.
    double point_score = 0.0;
    if (has_fork_signal) {
      if (has_order_signal) {
        point_score =
            0.5 * fork_match + 0.3 * lane_num_score + 0.2 * order_score;
      } else {
        point_score = 0.625 * fork_match + 0.375 * lane_num_score;
      }
    } else {
      if (has_order_signal) {
        point_score = 0.4 * lane_num_score + 0.6 * order_score;
      } else {
        point_score = lane_num_score;
      }
    }

    // Distance weight
    double weight = 1.0;
    if (sample.dist > 20.0) {
      weight = std::max(0.3, 1.0 - (sample.dist - 20.0) / 80.0);
    }

    total_score += weight * point_score;
    total_weight += weight;
  }

  double final_score = 0.0;
  if (total_weight > 1e-6) {
    final_score = total_score / total_weight;
  }
  return final_score;
}
std::vector<std::pair<int, uint64_t>> LDRouteInfoStrategy::BuildSeqLaneIds(
    const iflymapdata::sdpro::LinkInfo_Link* link) const {
  std::vector<std::pair<int, uint64_t>> seq_lane_ids;
  if (link == nullptr) {
    return seq_lane_ids;
  }
  for (auto lane_id : link->lane_ids()) {
    const auto* map_lane = ld_map_.GetLaneInfoByID(lane_id);
    if (map_lane == nullptr) {
      continue;
    }
    if (IsEmergencyLane(map_lane) || IsDiversionLane(map_lane)) {
      continue;
    }
    seq_lane_ids.emplace_back(map_lane->sequence(), lane_id);
  }
  // sequence 从大到小排序（大=靠左），下标+1即为从左向右序号
  std::sort(seq_lane_ids.begin(), seq_lane_ids.end(),
            [](const auto& a, const auto& b) { return a.first > b.first; });
  return seq_lane_ids;
}

int LDRouteInfoStrategy::SeqToOrder(
    const std::vector<std::pair<int, uint64_t>>& seq_lane_ids, int sequence) {
  for (int i = 0; i < static_cast<int>(seq_lane_ids.size()); ++i) {
    if (seq_lane_ids[i].first == sequence) {
      return i + 1;
    }
  }
  return -1;
}

const iflymapdata::sdpro::LinkInfo_Link*
LDRouteInfoStrategy::GetCloserSuccessorLinkByLateralDistance(
    const iflymapdata::sdpro::LinkInfo_Link* input_link,
    const iflymapdata::sdpro::LinkInfo_Link* successor_link1,
    const iflymapdata::sdpro::LinkInfo_Link* successor_link2) const {
  // 参数校验
  if (input_link == nullptr || successor_link1 == nullptr ||
      successor_link2 == nullptr) {
    return nullptr;
  }

  // 检查输入link是否有形点
  if (input_link->points().boot().points_size() < 2) {
    return nullptr;
  }

  // 检查后继link是否有形点
  if (successor_link1->points().boot().points_size() < 1 ||
      successor_link2->points().boot().points_size() < 1) {
    return nullptr;
  }

  // 获取输入link的末端点和末端方向
  const auto& input_points = input_link->points().boot();
  const int last_idx = input_points.points_size() - 1;
  const int second_last_idx = last_idx - 1;

  // 输入link的末端点
  const double end_x = input_points.points(last_idx).x();
  const double end_y = input_points.points(last_idx).y();

  // 输入link的末端方向（从倒数第二个点到最后一个点）
  const double dx = end_x - input_points.points(second_last_idx).x();
  const double dy = end_y - input_points.points(second_last_idx).y();
  const double length = std::sqrt(dx * dx + dy * dy);

  if (length < kEpsilon) {
    return nullptr;
  }

  // 单位方向向量
  const double unit_dx = dx / length;
  const double unit_dy = dy / length;

  // 获取后继link1的起点
  const auto& successor1_points = successor_link1->points().boot();
  const double succ1_start_x = successor1_points.points(0).x();
  const double succ1_start_y = successor1_points.points(0).y();

  // 计算从输入link末端点到后继link1起点的向量
  const double vec1_x = succ1_start_x - end_x;
  const double vec1_y = succ1_start_y - end_y;

  // 计算横向距离（叉积的绝对值）
  // 横向距离 = |vec1 × unit_direction|
  const double lateral_dist1 = std::abs(vec1_x * unit_dy - vec1_y * unit_dx);

  // 获取后继link2的起点
  const auto& successor2_points = successor_link2->points().boot();
  const double succ2_start_x = successor2_points.points(0).x();
  const double succ2_start_y = successor2_points.points(0).y();

  // 计算从输入link末端点到后继link2起点的向量
  const double vec2_x = succ2_start_x - end_x;
  const double vec2_y = succ2_start_y - end_y;

  // 计算横向距离
  const double lateral_dist2 = std::abs(vec2_x * unit_dy - vec2_y * unit_dx);

  // 返回横向距离更小的后继link
  return (lateral_dist1 < lateral_dist2) ? successor_link1 : successor_link2;
}

void LDRouteInfoStrategy::ProcessEraseFeasibleLaneForSplitScene(
    TopoLinkGraph& feasible_lane_graph) {
  // 判断是否为需要移除的那种场景
  // -------------------
  //
  //              -
  //               -
  //                 -
  // 如上，如果是直行则不需要移除，如果是右，则需要移除
  uint64 front_first_topo_change_link_id =
      mlc_decider_scene_type_info_.topo_change_link_id;
  const auto& front_first_topo_change_link =
      ld_map_.GetLinkOnRoute(front_first_topo_change_link_id);

  if (front_first_topo_change_link == nullptr) {
    return;
  }

  const auto& split_next_link =
      ld_map_.GetNextLinkOnRoute(front_first_topo_change_link->id());
  if (split_next_link == nullptr) {
    return;
  }

  if (front_first_topo_change_link->successor_link_ids().size() != 2) {
    return;
  }
  
  uint64 out_link_id =
      front_first_topo_change_link->successor_link_ids()[0] ==
              split_next_link->id()
          ? front_first_topo_change_link->successor_link_ids()[1]
          : front_first_topo_change_link->successor_link_ids()[0];
  const auto& out_link = ld_map_.GetLinkOnRoute((out_link_id));
  if (out_link == nullptr) {
    return;
  }

  const auto& closer_suc_link = GetCloserSuccessorLinkByLateralDistance(
      front_first_topo_change_link, split_next_link, out_link);

  if (closer_suc_link == nullptr) {
    return;
  }

  if (closer_suc_link->id() == split_next_link->id()) {
    return;
  }

  Erase1Split2FeasibleLane(feasible_lane_graph);
}
void LDRouteInfoStrategy::CaculateDistanceToNOAEnd(
    const iflymapdata::sdpro::LinkInfo_Link* segment, const double nearest_s) {
  if (segment == nullptr) {
    return;
  }
  double distance_to_noa_end = 0.0;
  // 当前位于NOA状态，计算到地图信息终点/NOA
  if (route_info_output_.is_ego_on_expressway) {
    const auto* iter_link = segment;
    while (iter_link) {
      if (iter_link->link_class() ==
              iflymapdata::sdpro::LinkClass::LC_EXPRESSWAY ||
          iter_link->link_class() ==
              iflymapdata::sdpro::LinkClass::LC_CITY_EXPRESSWAY ||
          (iter_link->link_type() & iflymapdata::sdpro::LT_IC) != 0) {
        if (iter_link->id() == current_link_->id()) {
          distance_to_noa_end += iter_link->length() * 0.01 - nearest_s;
        } else {
          distance_to_noa_end += iter_link->length() * 0.01;
        }
        // 大于3km直接break
        if (distance_to_noa_end > 3000.0) {
          break;
        }
        iter_link = ld_map_.GetNextLinkOnRoute(iter_link->id());
      } else {
        break;
      }
    }
  } else {
    distance_to_noa_end = 0.0;
  }
  route_info_output_.distance_to_noa_end = distance_to_noa_end;
}
}  // namespace planning