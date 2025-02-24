#include "agent_cost.h"

#include <array>
#include <cstdint>
#include <iostream>

#include "reference_path.h"

namespace planning {
namespace ara_star {

using Vec2d = planning::planning_math::Vec2d;
namespace {
constexpr double kAreaRange = 10.0;  // TODO: may related to speed
constexpr double kRecRange = 6.0;
constexpr double kHardBuffer = 0.0;
constexpr double kSoftBufferMore = 0.6;
constexpr double kSoftBufferLess = 0.6;
constexpr double kAreaCost = 15.0;
constexpr double kDirectlyBehindCost = 20.0;
constexpr double kPassIntervalCost = 12.5;
constexpr double kBendRoadRadius = 30;
constexpr double kBendFrontRange = 7;
constexpr double kBendBackRange = 1;
}  // namespace

AgentCost::AgentCost(
    const double weight, const double ego_wheel_base, const double ego_length,
    const double ego_half_width, const std::vector<SLBox2d>& obstacles,
    const std::shared_ptr<KDPath>& ego_lane,
    const std::shared_ptr<AABoxKDTree2d<GeometryObject>>& agent_box_tree,
    const std::shared_ptr<ReferencePath>& reference_path_ptr,
    const std::vector<std::shared_ptr<planning::FrenetObstacle>>& nudge_agents,
    const double front_edge_to_rear_axle, const double rear_edge_to_rear_axle,
    const std::pair<double, double> pass_interval, const bool left_turn,
    const bool right_turn)
    : BaseCost(BaseCost::CostType::AGENT, weight),
      obstacles_(obstacles),
      ego_lane_(ego_lane),
      agent_box_tree_(agent_box_tree),
      ego_wheel_base_(ego_wheel_base),
      ego_length_(ego_length),
      ego_half_width_(ego_half_width),
      reference_path_ptr_(reference_path_ptr),
      nudge_agents_(nudge_agents),
      front_edge_to_rear_axle_(front_edge_to_rear_axle),
      rear_edge_to_rear_axle_(rear_edge_to_rear_axle),
      pass_interval_(pass_interval),
      left_turn_(left_turn),
      right_turn_(right_turn){};

double AgentCost::MakeCost(Node3D& vertex) const {
  // distance cost，离障碍物越近，cost越大
  double dist = vertex.GetMinDist();
  double dist_cost = 0.0;
  // kHardBuffer = 0.3;
  if (dist < kHardBuffer) {
    dist_cost = 1.0;
    // kSoftBufferMore = 0.9
  } else if (dist < kSoftBufferMore) {
    dist_cost = 1 - (dist - kHardBuffer) / (kSoftBufferMore - kHardBuffer);
  } else {
    dist_cost = 0.0;
  }
  NormalizeCost(dist_cost);

  // ----------------------------------------------------------------------------
  // process area cost
  // 前边缘中心
  double ego_front_s = 0.0;
  double ego_front_l = 0.0;
  double front_x =
      vertex.GetX() + front_edge_to_rear_axle_ * std::cos(vertex.GetPhi());
  double front_y =
      vertex.GetY() + front_edge_to_rear_axle_ * std::sin(vertex.GetPhi());
  if (!ego_lane_->XYToSL(front_x, front_y, &ego_front_s, &ego_front_l)) {
    return 0.0;
  }
  // 后边缘中心
  // double ego_back_s = 0.0;
  // double ego_back_l = 0.0;
  // double back_x = vertex.GetX() - rear_edge_to_rear_axle_ *
  // std::cos(vertex.GetPhi()); double back_y = vertex.GetY() -
  // rear_edge_to_rear_axle_ * std::sin(vertex.GetPhi()); if
  // (!ego_lane_->XYToSL(back_x, back_y, &ego_back_s, &ego_back_l)) {
  //   return 0.0;
  // }
  // 后轴中心
  double ego_back_s = vertex.GetS();
  double ego_back_l = vertex.GetL();

  double area_cost = 0.0;
  double directly_behind_cost = 0.0;

  bool has_set_area_cost = false;

  for (const auto& frenet_obstacle : nudge_agents_) {
    double cost = 0.0;
    has_set_area_cost =
        GetAreaCost(frenet_obstacle, ego_front_s, ego_front_l, ego_back_s,
                    ego_back_l, cost, left_turn_, right_turn_);
    area_cost = area_cost + cost;

    cost = 0.0;
    GetDirectlyBehindCost(frenet_obstacle, ego_front_s, ego_front_l, ego_back_l,
                          cost, left_turn_, right_turn_);
    directly_behind_cost = directly_behind_cost + cost;
  }

  double pass_interval_cost = 0.0;
  GetPassIntervalCost(vertex.GetDeltaS(), ego_front_l, ego_back_l,
                      pass_interval_cost, left_turn_, right_turn_,
                      pass_interval_);

  vertex.SetAgentCost(
      (dist_cost + area_cost + directly_behind_cost + pass_interval_cost) *
      GetCostWeight());
  vertex.SetDistCost(dist_cost * GetCostWeight());
  vertex.SetAreaCost(area_cost * GetCostWeight());
  vertex.SetDirectlyBehindCost(directly_behind_cost * GetCostWeight());
  vertex.SetPassIntervalCost(pass_interval_cost * GetCostWeight());
  return dist_cost + area_cost + directly_behind_cost + pass_interval_cost;
}

void AgentCost::NormalizeCost(double& cost) const { cost = cost * 15.0; }

void AgentCost::GetPassIntervalCost(
    const double delta_s, const double ego_front_l, const double ego_back_l,
    double& cost, const bool left_turn, const bool right_turn,
    const std::pair<double, double> pass_interval) const {
  bool in_bend = left_turn || right_turn;
  if (!in_bend || (pass_interval.first == 0 && pass_interval.second == 0)) {
    cost = 0.0;
    return;
  }

  // 根据delta_s来调整权重，暂时关闭
  std::array<double, 2> xp{4, 20};
  std::array<double, 2> fp{1, 1};
  double times_for_s = interp(delta_s, xp, fp);

  double center_l = (pass_interval.first + pass_interval.second) / 2;
  double l_half_width = (pass_interval.second - pass_interval.first) / 2.0;
  double l_buffer = ego_half_width_ + l_half_width + 0.2;
  double ego_center_l = (ego_back_l + ego_front_l) / 2;
  double l_distance = std::abs(ego_front_l - center_l);
  if (l_distance < l_buffer) {
    cost = -1 * kPassIntervalCost * times_for_s *
           std::max(0.0, (l_buffer - std::pow(l_distance, 2) / l_buffer));
  }
}

void AgentCost::GetDirectlyBehindCost(
    const std::shared_ptr<planning::FrenetObstacle>& frenet_obstacle,
    const double ego_front_s, const double ego_front_l, const double ego_back_l,
    double& cost, const bool left_turn, const bool right_turn) const {
  double min_l = frenet_obstacle->frenet_polygon_sequence()[0].second.min_y();
  double max_l = frenet_obstacle->frenet_polygon_sequence()[0].second.max_y();
  double center_l = (min_l + max_l) / 2;
  double l_half_width = (max_l - min_l) / 2.0;
  double min_s = frenet_obstacle->frenet_polygon_sequence()[0].second.min_x();

  constexpr double kStraightLaneRange = 10;
  constexpr double kInsideBendRange = 10;
  constexpr double kOutsideBendRange = 0.3;
  double directly_behind_range = kStraightLaneRange;
  if (left_turn) {
    if (center_l > 0) {
      directly_behind_range = kInsideBendRange;
    } else {
      directly_behind_range = kOutsideBendRange;
    }
  } else if (right_turn) {
    if (center_l > 0) {
      directly_behind_range = kOutsideBendRange;
    } else {
      directly_behind_range = kInsideBendRange;
    }
  }

  double ego_l = ego_front_l;
  double l_buffer = ego_half_width_ + l_half_width + 0.1;
  if (left_turn || right_turn) {
    // ego_l = (ego_front_l + 3 * ego_back_l) / 4;
    ego_l = (ego_front_l + ego_back_l) / 2;
    l_buffer = ego_half_width_ + l_half_width + 0.2;
  }

  if (ego_front_s < min_s && ego_front_s > min_s - directly_behind_range) {
    double l_distance = std::abs(ego_l - center_l);
    if (l_distance < l_buffer) {
      cost = kDirectlyBehindCost *
             std::max(0.0, (l_buffer - std::pow(l_distance, 2) / l_buffer));
    }
  }
}

bool AgentCost::GetAreaCost(
    const std::shared_ptr<planning::FrenetObstacle>& frenet_obstacle,
    const double ego_front_s, const double ego_front_l, const double ego_back_s,
    const double ego_back_l, double& area_cost, const bool left_turn,
    const bool right_turn) const {
  bool has_set_area_cost = false;

  double min_l = frenet_obstacle->frenet_polygon_sequence()[0].second.min_y();
  double max_l = frenet_obstacle->frenet_polygon_sequence()[0].second.max_y();
  double center_l = (min_l + max_l) / 2;
  double min_s = frenet_obstacle->frenet_polygon_sequence()[0].second.min_x();
  double max_s = frenet_obstacle->frenet_polygon_sequence()[0].second.max_x();
  double center_s = (min_s + max_s) / 2;
  double l_half_width = (max_l - min_l) / 2.0;

  // HACK(zkxie): 0相当于关闭了矩形代价
  constexpr double kRecRangeLBuffer = 0;
  double ellipse_major_axis = 0.0;
  double ellipse_minor_axis = 0.0;
  bool in_bend = left_turn || right_turn;
  if (in_bend) {
    // 直接返回
    area_cost = 0.0;
    return true;
    // 弯道代价
    if (ego_front_s < center_s) {
      ellipse_major_axis = 0.01;
      ellipse_minor_axis = 0.01;
    }
    if (ego_back_s > center_s) {
      ellipse_major_axis = 0.01;
      ellipse_minor_axis = 0.01;
    }
  } else {
    if (ego_front_s < center_s) {
      ellipse_major_axis = 7;
      ellipse_minor_axis = l_half_width + ego_half_width_ + 0.5;
    }
    if (ego_back_s > center_s) {
      ellipse_major_axis = (max_s - min_s) / 4 + 2;
      ellipse_minor_axis = l_half_width + ego_half_width_ + 1;
    }
  }

  auto calculateCost = [&frenet_obstacle, &ellipse_major_axis,
                        &ellipse_minor_axis](const double& s_distance,
                                             const double& l_distance) {
    double area_cost = 0.0;
    bool has_set_area_cost = false;
    // 矩形代价 kRecRange = 6
    if (s_distance < kRecRange && l_distance < kRecRangeLBuffer) {
      // kAreaCost = 10
      area_cost = kAreaCost;
      has_set_area_cost = true;
    } else {
      // 椭圆代价
      double normalizedS = s_distance / ellipse_major_axis;
      double normalizedL = l_distance / ellipse_minor_axis;
      double normalized_factor =
          normalizedS * normalizedS + normalizedL * normalizedL;
      // 在椭圆内
      if (normalized_factor < 1.0) {
        area_cost = kAreaCost * (1.0 - normalized_factor);
        has_set_area_cost = true;
      } else {
        area_cost = 0.0;
        has_set_area_cost = false;
      }
    }
    return std::make_pair(area_cost, has_set_area_cost);
  };

  // 自车在障碍物后面一段距离
  if (ego_front_s < min_s && ego_front_s > min_s - kAreaRange) {
    double s_distance = min_s - ego_front_s;
    double l_distance = std::abs(ego_front_l - center_l);
    // 在障碍物正后方时，加大l的作用
    if (l_distance < 1) {
      // l_distance = std::pow(l_distance, 0.5);
      l_distance = 2 * l_distance - l_distance * l_distance;
    }
    // cost is related to s l
    auto result = calculateCost(s_distance, l_distance);
    area_cost = result.first;
    has_set_area_cost = result.second;
  }
  // 自车在障碍物前面一段距离
  double back_s = min_s + (max_s - min_s) * 3 / 4;
  if (ego_back_s > back_s && ego_back_s < back_s + kAreaRange) {
    double s_distance = ego_back_s - back_s;
    double l_distance = std::abs(ego_back_l - center_l);
    // cost is related to s l
    auto result = calculateCost(s_distance, l_distance);
    area_cost = result.first;
    has_set_area_cost = result.second;
  }
  return has_set_area_cost;
}

}  // namespace ara_star
}  // namespace planning