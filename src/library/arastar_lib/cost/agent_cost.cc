#include "agent_cost.h"

#include <iostream>

#include "reference_path.h"

namespace planning {
namespace ara_star {

using Vec2d = planning::planning_math::Vec2d;
namespace {
constexpr double kAreaRange = 16.0;  // TODO: may related to speed
constexpr double kRecRange = 6.0;
constexpr double kHardBuffer = 0.3;
constexpr double kSoftBufferMore = 0.9;
constexpr double kSoftBufferLess = 0.6;
constexpr double kArearcost = 10.0;
}  // namespace

AgentCost::AgentCost(
    const double weight, const double ego_wheel_base, const double ego_length,
    const double ego_circle_radius, const std::vector<SLBox2d>& obstacles,
    const std::shared_ptr<KDPath>& ego_lane,
    const std::shared_ptr<AABoxKDTree2d<GeometryObject>>& agent_box_tree,
    const std::shared_ptr<ReferencePath>& reference_path_ptr)
    : BaseCost(BaseCost::CostType::AGENT, weight),
      obstacles_(obstacles),
      ego_lane_(ego_lane),
      agent_box_tree_(agent_box_tree),
      ego_wheel_base_(ego_wheel_base),
      ego_length_(ego_length),
      ego_circle_radius_(ego_circle_radius),
      reference_path_ptr_(reference_path_ptr){};

double AgentCost::MakeCost(Node3D& vertex) const {
  size_t node_step_num = vertex.GetStepNum();
  const auto& traversed_x = vertex.GetXs();
  const auto& traversed_y = vertex.GetYs();
  const auto& traversed_phi = vertex.GetPhis();

  // agent box check
  if (agent_box_tree_ == nullptr) {
    return 0.0;
  }

  double distance_less = 10.0;
  double distance_more = 10.0;
  Vec2d back_axis_position;
  Vec2d front_axis_position;
  uint16_t nearest_object_id = UINT16_MAX;
  // since step size is 0.2m for motion integration, we check every 1.0m for
  // collision is enough
  for (int i = node_step_num - 1; i > 0; i = i - 3) {
    back_axis_position.set_x(traversed_x[i]);
    back_axis_position.set_y(traversed_y[i]);
    front_axis_position.set_x(traversed_x[i] +
                              ego_wheel_base_ * std::cos(traversed_phi[i]));
    front_axis_position.set_y(traversed_y[i] +
                              ego_wheel_base_ * std::sin(traversed_phi[i]));

    // 找最近的一个障碍物做校验
    const auto* nearest_object =
        agent_box_tree_->GetNearestObject(front_axis_position);
    if (nearest_object == nullptr) {
      continue;
    }
    nearest_object_id = nearest_object->track_id();

    double distance_back = 100;
    double distance_front = 100;
    if ((nearest_object->obstacle_ptr()->source_type() == SourceType::OCC ||
         nearest_object->obstacle_ptr()->source_type() ==
             SourceType::GroundLine) &&
        nearest_object->obstacle_ptr()->perception_polygon().num_points() >=
            3) {
      // 到后轴中心的距离减一个半车宽
      distance_back =
          nearest_object->obstacle_ptr()->perception_polygon().DistanceTo(
              back_axis_position) -
          ego_circle_radius_;
      // 到前轴中心的距离减一个半车宽
      distance_front =
          nearest_object->obstacle_ptr()->perception_polygon().DistanceTo(
              front_axis_position) -
          ego_circle_radius_;
    } else {
      const auto* nearest_box = nearest_object->box();
      if (nearest_box == nullptr) {
        continue;
      }
      // 到后轴中心的距离减一个半车宽
      distance_back =
          nearest_box->DistanceTo(back_axis_position) - ego_circle_radius_;
      // 到前轴中心的距离减一个半车宽
      distance_front =
          nearest_box->DistanceTo(front_axis_position) - ego_circle_radius_;
    }

    // 区分锥桶，锥桶是less，可以离得更近
    if (nearest_object->obstacle_ptr() != nullptr &&
        nearest_object->obstacle_ptr()->is_traffic_facilities()) {
      distance_less =
          std::min(std::min(distance_back, distance_front), distance_less);
    } else {
      distance_more =
          std::min(std::min(distance_back, distance_front), distance_more);
    }
  }

  // distance cost，离障碍物越近，cost越大
  double dist_cost_more = 0.0;
  // kHardBuffer = 0.3;
  if (distance_more < kHardBuffer) {
    dist_cost_more = 1.0;
    // kSoftBufferMore = 0.9
  } else if (distance_more < kSoftBufferMore) {
    dist_cost_more =
        1 - (distance_more - kHardBuffer) / (kSoftBufferMore - kHardBuffer);
  } else {
    dist_cost_more = 0.0;
  }
  double dist_cost_less = 0.0;
  if (distance_less < kHardBuffer) {
    dist_cost_less = 2.0;
    // kSoftBufferLess = 0.6
  } else if (distance_less < kSoftBufferLess) {
    dist_cost_less =
        1 - (distance_less - kHardBuffer) / (kSoftBufferLess - kHardBuffer);
  } else {
    dist_cost_less = 0.0;
  }
  double dist_cost = std::max(dist_cost_more, dist_cost_less);
  NormalizeCost(dist_cost);

  // ----------------------------------------------------------------------------
  // process area cost
  double ego_front_s = 0.0;
  double ego_front_l = 0.0;
  double ego_back_s = vertex.GetS();
  double ego_back_l = vertex.GetL();
  double front_x = vertex.GetX() + ego_wheel_base_ * std::cos(vertex.GetPhi());
  double front_y = vertex.GetY() + ego_wheel_base_ * std::sin(vertex.GetPhi());
  if (!ego_lane_->XYToSL(front_x, front_y, &ego_front_s, &ego_front_l)) {
    return 0.0;
  }

  double area_cost = 0.0;  // if vertex just behind box, add some cost

  bool has_set_area_cost = false;

  for (const auto& frenet_obstacle : reference_path_ptr_->get_obstacles()) {
    // get area behind box

    // 只计算最近的障碍物
    if (nearest_object_id == frenet_obstacle->id()) {
      has_set_area_cost = GetAreaCost(frenet_obstacle, ego_front_s, ego_front_l,
                                      ego_back_s, ego_back_l, area_cost);
    }

    // 计算所有障碍物
    // double cost = 0.0;
    // has_set_area_cost = GetAreaCost(obstacle, ego_front_s, ego_front_l,
    //                                 ego_back_s, ego_back_l, cost);
    // area_cost = area_cost + cost;
  }

  vertex.SetAgentCost((dist_cost + area_cost) * GetCostWeight());
  return dist_cost + area_cost;
}

void AgentCost::NormalizeCost(double& cost) const { cost = cost * 10.0; }

bool AgentCost::GetAreaCost(
    const std::shared_ptr<planning::FrenetObstacle>& frenet_obstacle,
    const double ego_front_s, const double ego_front_l, const double ego_back_s,
    const double ego_back_l, double& area_cost) const {
  bool has_set_area_cost = false;
  double center_l = 100.0;
  double s = 100.0;
  double min_s = 100.0;
  double max_s = 100.0;
  if ((frenet_obstacle->source_type() == SourceType::OCC ||
       frenet_obstacle->source_type() == SourceType::GroundLine) &&
      frenet_obstacle->obstacle()->perception_polygon().num_points() >= 3 &&
      frenet_obstacle->b_frenet_valid() &&
      !frenet_obstacle->b_frenet_polygon_sequence_invalid()) {
    center_l = (frenet_obstacle->frenet_polygon_sequence()[0].second.min_y() +
                frenet_obstacle->frenet_polygon_sequence()[0].second.max_y()) /
               2;
    min_s = frenet_obstacle->frenet_polygon_sequence()[0].second.min_x();
    max_s = frenet_obstacle->frenet_polygon_sequence()[0].second.max_x();
    s = (min_s + max_s) / 2;
  } else {
    center_l = frenet_obstacle->frenet_l();
    s = frenet_obstacle->frenet_s();
    min_s = frenet_obstacle->frenet_obstacle_boundary().s_start;
    max_s = frenet_obstacle->frenet_obstacle_boundary().s_end;
  }
  // 与障碍物L方向上最近的距离，ego_circle_radius_为自车半车宽
  // HACK(zkxie): 0相当于关闭了矩形代价
  constexpr double kLBuffer = 0;
  constexpr double kMinorAxis = 2.6;
  auto calculateCost = [](const double& s_distance, const double& l_distance) {
    double area_cost = 0.0;
    bool has_set_area_cost = false;
    // 矩形代价 kRecRange = 6
    if (s_distance < kRecRange && l_distance < kLBuffer) {
      // kArearcost = 10
      area_cost = kArearcost;
      has_set_area_cost = true;
    } else {
      // 椭圆代价 kAreaRange = 16
      // (kAreaRange - kRecRange)是长轴，kMinorAxis是短轴
      double normalizedS = s_distance / (kAreaRange - kRecRange);
      double normalizedL = l_distance / kMinorAxis;
      double normalized_factor =
          normalizedS * normalizedS + normalizedL * normalizedL;
      // 在椭圆内
      if (normalized_factor < 1.0) {
        area_cost = kArearcost * (1.0 - normalized_factor);
        has_set_area_cost = true;
      } else {
        area_cost = 0.0;
        has_set_area_cost = false;
      }
    }

    return std::make_pair(area_cost, has_set_area_cost);
  };

  // 自车在障碍物后面一段距离
  if (ego_front_s < s && ego_front_s > min_s - kAreaRange) {
    double s_distance = s - ego_front_s;
    double l_distance = std::abs(ego_front_l - center_l);
    // cost is related to s l
    auto result = calculateCost(s_distance, l_distance);
    area_cost = result.first;
    has_set_area_cost = result.second;
  }
  // 自车在障碍物前面一段距离
  if (ego_back_s > s && ego_back_s < max_s + kAreaRange) {
    double s_distance = ego_back_s - s;
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