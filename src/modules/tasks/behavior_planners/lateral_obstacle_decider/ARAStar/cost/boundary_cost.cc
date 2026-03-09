#include "boundary_cost.h"

#include "common/vec2d.h"
#include "src/modules/context/vehicle_config_context.h"

namespace planning {
namespace ara_star {

BoundaryCost::BoundaryCost(
    const double weight, const double init_v, const double ego_wheel_base,
    const double ego_circle_radius, const double lane_width,
    const std::shared_ptr<planning_math::KDPath>& target_lane,
    const std::shared_ptr<planning::planning_math::KDPath> left_boundary_tree,
    const std::shared_ptr<planning::planning_math::KDPath> right_boundary_tree,
    const double hard_safe_distance, const double soft_safe_distance)
    : BaseCost(BaseCost::CostType::BOUNDARY, weight),
      target_lane_(target_lane),
      ego_wheel_base_(ego_wheel_base),
      ego_circle_radius_(ego_circle_radius),
      init_v_(init_v),
      left_boundary_tree_(left_boundary_tree),
      right_boundary_tree_(right_boundary_tree),
      hard_safe_distance_(hard_safe_distance),
      soft_safe_distance_(soft_safe_distance),
      lane_width_(lane_width) {};

double BoundaryCost::MakeCost(Node3d& vertex) const {
  if (left_boundary_tree_ == nullptr ||
      right_boundary_tree_ == nullptr) {
    return 0.0;
  }
  planning::planning_math::Vec2d back_axis_position(vertex.GetX(), vertex.GetY());
  planning::planning_math::Vec2d front_axis_position(
      vertex.GetX() + ego_wheel_base_ * std::cos(vertex.GetPhi()),
      vertex.GetY() + ego_wheel_base_ * std::sin(vertex.GetPhi()));

  double front_dist_to_left =
      left_boundary_tree_->DistanceTo(front_axis_position);
  double rear_dist_to_left =
      left_boundary_tree_->DistanceTo(back_axis_position);
  double front_dist_to_right =
      right_boundary_tree_->DistanceTo(front_axis_position);
  double rear_dist_to_right =
      right_boundary_tree_->DistanceTo(back_axis_position);

  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  planning_math::Vec2d center(
      vertex.GetX() +
          std::cos(vertex.GetPhi()) * vehicle_param.rear_axle_to_center,
      vertex.GetY() +
          std::sin(vertex.GetPhi()) * vehicle_param.rear_axle_to_center);
  planning_math::Box2d ego_box(center, vertex.GetPhi(), vehicle_param.length,
                               vehicle_param.width);
  ara_star::SLBox2d sl_box;
  double s = 0.0;
  double l = 0.0;
  sl_box.box = ego_box;
  auto corners = ego_box.GetAllCorners();
  // 找到agent的最大和最小的 s l
  for (auto corner : corners) {
    if (!target_lane_->XYToSL(corner.x(), corner.y(), &s, &l)) {
      // std::cout << "box out of lane" << std::endl;
      continue;
    }
    sl_box.min_s = std::min(sl_box.min_s, s);
    sl_box.max_s = std::max(sl_box.max_s, s);
    sl_box.min_l = std::min(sl_box.min_l, l);
    sl_box.max_l = std::max(sl_box.max_l, l);
  }
  bool is_outside_lane =
      (sl_box.max_l > lane_width_) || (sl_box.min_l < -lane_width_);

  // distance cost
  double base_cost = is_outside_lane ? 20.0 : 1.0;

  double distance = std::min(std::min(std::min(front_dist_to_left, rear_dist_to_left), front_dist_to_right), rear_dist_to_right);

  // double distance =
  //     lane_width_ / 2 - (std::fabs(vertex.GetL()) + vehicle_param.width / 2);
  double dist_cost = 0.0;
  if (distance < hard_safe_distance_) {
    dist_cost = base_cost;
  } else if (distance < soft_safe_distance_) {
    dist_cost = base_cost - (distance - hard_safe_distance_) /
                                (soft_safe_distance_ - hard_safe_distance_);
  } else {
    dist_cost = 0.0;
  }

  NormalizeCost(dist_cost);

  vertex.SetBoundaryCost(dist_cost * GetCostWeight());
  return dist_cost;
}

void BoundaryCost::NormalizeCost(double& cost) const { cost = cost * 10.0; }

}  // namespace ara_star
}  // namespace planning