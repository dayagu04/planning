#include "scc_agent_cost.h"
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

SccAgentCost::SccAgentCost(
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

double SccAgentCost::MakeCost(Node3d& vertex) const {
  size_t node_step_size = vertex.GetStepNum();
  const auto& traversed_x = vertex.GetXs();
  const auto& traversed_y = vertex.GetYs();
  const auto& traversed_phi = vertex.GetPhis();

  // agent box check
  if (agent_box_tree_ == nullptr) {
    return 0.0;
  }

  double distance_less = 10.0;
  double distance_more = 10.0;
  planning_math::Vec2d back_axis_position;
  planning_math::Vec2d front_axis_position;
  // since step size is 0.2m for motion integration, we check every 1.0m for collision is enough
  for (int i = node_step_size - 1; i > 0; i = i - 5) {
    back_axis_position.set_x(traversed_x[i]);
    back_axis_position.set_y(traversed_y[i]);
    front_axis_position.set_x(traversed_x[i] + ego_wheel_base_ * std::cos(traversed_phi[i]));
    front_axis_position.set_y(traversed_y[i] + ego_wheel_base_ * std::sin(traversed_phi[i]));

    const auto* nearest_object = agent_box_tree_->GetNearestObject(back_axis_position);
    if (nearest_object == nullptr) {
      continue;
    }
    const auto* nearest_box = nearest_object->box();
    if (nearest_box == nullptr) {
      continue;
    }

    double distance_back = nearest_box->DistanceTo(back_axis_position) - ego_half_width_;
    double distance_front = nearest_box->DistanceTo(front_axis_position) - ego_half_width_;
    if (nearest_object->track_id() == 1) {
      distance_less = std::min(std::min(distance_back, distance_front), distance_less);
    } else {
      distance_more = std::min(std::min(distance_back, distance_front), distance_more);
    }


  }
  // process area cost
  double ego_front_s = 0.0;
  double ego_front_l = 0.0;
  double ego_back_s = vertex.GetS();
  double ego_back_l = vertex.GetL();
  double front_x = vertex.GetX() + ego_wheel_base_ * std::cos(vertex.GetPhi());
  double front_y = vertex.GetY() + ego_wheel_base_ * std::sin(vertex.GetPhi());
  if (!reference_path_ptr_->get_frenet_coord()->XYToSL(front_x, front_y, &ego_front_s, &ego_front_l)) {
    return 0.0;
  }

  double area_cost = 0.0; // if vertex just behind box, add some cost

  bool has_set_area_cost = false;

  double nearing_agent_cost = 0.0;
  for (const auto& slbox : obstacles_) {
    double lat_distance = 0.0;
    if (ego_front_l - ego_half_width_ > slbox.max_l) {
      lat_distance = ego_front_l - ego_half_width_ - slbox.max_l;
    } else if (slbox.min_l - ego_front_l - ego_half_width_ > 0) {
      lat_distance = slbox.min_l - ego_front_l - ego_half_width_;
    }

    double nearing_agent_cost_tmp = GetNearingOverlapCost(slbox, ego_front_s, ego_front_l, ego_back_s, ego_back_l, vertex.GetPhi(), lat_distance);
    nearing_agent_cost = std::max(nearing_agent_cost, nearing_agent_cost_tmp);
    // get area behind box
    if (has_set_area_cost) {
      continue;
    }
    if (slbox.nudge_less) {
      continue;
    }
    has_set_area_cost =
        GetAreaCost(slbox, ego_front_s, ego_front_l, ego_back_s, ego_back_l, area_cost);
  }

  // distance cost
  double dist_cost_more = 0.0;
  if (distance_more < kHardBuffer) {
    dist_cost_more = 1.0;
  } else if (distance_more < kSoftBufferMore) {
    dist_cost_more = 1 - (distance_more - kHardBuffer) / (kSoftBufferMore - kHardBuffer);
  } else {
    dist_cost_more = 0.0;
  }
  double dist_cost_less = 0.0;
  if (distance_less < kHardBuffer) {
    dist_cost_less = 2.0;
  } else if (distance_less < kSoftBufferLess) {
    dist_cost_less = 1 - (distance_less - kHardBuffer) / (kSoftBufferLess - kHardBuffer);
  } else {
    dist_cost_less = 0.0;
  }
  double dist_cost = std::max(dist_cost_more, dist_cost_less);
  NormalizeCost(dist_cost);
  vertex.SetAgentCost(dist_cost + area_cost);
  vertex.SetDistCost(dist_cost);
  vertex.SetAreaCost(area_cost);
  vertex.SetNearingAgentCost(nearing_agent_cost);
  return dist_cost + area_cost + nearing_agent_cost;
}

void SccAgentCost::NormalizeCost(double& cost) const { cost = cost * 15.0; }

void SccAgentCost::GetPassIntervalCost(
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

void SccAgentCost::GetDirectlyBehindCost(
    const std::shared_ptr<planning::FrenetObstacle>& frenet_obstacle,
    const double ego_front_s, const double ego_front_l, const double ego_back_l,
    double& cost, const bool left_turn, const bool right_turn) const {
  double min_l = frenet_obstacle->frenet_obstacle_boundary().l_start;
  double max_l = frenet_obstacle->frenet_obstacle_boundary().l_end;
  double center_l = (min_l + max_l) / 2;
  double l_half_width = (max_l - min_l) / 2.0;
  double min_s = frenet_obstacle->frenet_obstacle_boundary().s_start;

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

bool SccAgentCost::GetAreaCost(const SLBox2d& sl_box, const double ego_front_s,
                            const double ego_front_l, const double ego_back_s,
                            const double ego_back_l, double& area_cost) const {
  constexpr double kArearcost = 5.0;
  bool has_set_area_cost = false;
  double center_l = (sl_box.max_l + sl_box.min_l) / 2.0;
  double l_limit = sl_box.max_l - center_l + ego_half_width_ + kSoftBufferMore;
  auto calculateCost = [l_limit](const double s_distance,
                                 const double l_distance) {
    double area_cost = 0.0;
    bool has_set_area_cost = false;
    // 矩形代价
    if (s_distance < kRecRange && l_distance < l_limit) {
      area_cost = kArearcost;
      has_set_area_cost = true;
    } else {
      // 椭圆代价
      double normalizedS = (s_distance - kRecRange) / (kAreaRange - kRecRange);
      double normalizedL = l_distance / l_limit;
      double normalized_factor = normalizedS * normalizedS + normalizedL * normalizedL;

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

  // 自车车头  在障碍物车尾与车尾-10m之间
  if (ego_front_s < sl_box.min_s && ego_front_s > sl_box.min_s - kAreaRange) {
    double s_distance = sl_box.min_s - ego_front_s;
    double l_distance = std::abs(ego_front_l - center_l);
    // cost is related to s l
    auto result = calculateCost(s_distance, l_distance);
    area_cost = result.first;
    has_set_area_cost = result.second;
  }

  // 自车车尾  在障碍物车头与车头+10m之间
  if (ego_back_s > sl_box.max_s && ego_back_s < sl_box.max_s + kAreaRange) {
    double s_distance = ego_back_s - sl_box.max_s;
    double l_distance = std::abs(ego_back_l - center_l);
    // cost is related to s l
    auto result = calculateCost(s_distance, l_distance);
    area_cost = result.first;
    has_set_area_cost = result.second;
  }
  return has_set_area_cost;
}

double SccAgentCost::GetNearingOverlapCost(
    const SLBox2d& sl_box, const double ego_front_s, const double ego_front_l,
    const double ego_back_s, const double ego_back_l, const double vertex_theta,
    double lat_distace) const {
  constexpr double theta_weight = 2;
  double nearing_agent_overlap_cost = 0.0;
  if (lat_distace > 1.5) {
    return nearing_agent_overlap_cost;
  }

  bool lon_overlap =
      std::max(ego_back_s, sl_box.min_s) - std::min(ego_front_s, sl_box.max_s) <
      1e-2;

  if (!lon_overlap) {
    return nearing_agent_overlap_cost;
  }

  double ref_heading =
      (reference_path_ptr_->get_frenet_coord()->GetPathCurveHeading(
           ego_front_s) +
       reference_path_ptr_->get_frenet_coord()->GetPathCurveHeading(
           ego_back_s)) *
      0.5;

  double heading_error = (ego_front_l + ego_back_l) * 0.5 -  (sl_box.min_l +sl_box.max_l) * 0.5 > 1e-2 ?

  std::max(0.0, planning_math::NormalizeAngle(ref_heading - vertex_theta)) : std::min(0.0, planning_math::NormalizeAngle(ref_heading - vertex_theta));
  nearing_agent_overlap_cost = std::min(fabs(heading_error) * theta_weight, 1.0);
  return nearing_agent_overlap_cost;
}
}  // namespace ara_star
}  // namespace planning