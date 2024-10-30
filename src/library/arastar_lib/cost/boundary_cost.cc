#include "boundary_cost.h"

#include "ad_common/math/vec2d.h"
#include "src/modules/context/vehicle_config_context.h"

namespace planning {
namespace ara_star {

BoundaryCost::BoundaryCost(
    const double weight, const double init_v, const double ego_wheel_base,
    const double ego_circle_radius, const double lane_width,
    const std::shared_ptr<planning_math::KDPath>& target_lane,
    const double& hard_safe_distance, const double& soft_safe_distance)
    : BaseCost(BaseCost::CostType::BOUNDARY, weight),
      target_lane_(target_lane),
      ego_wheel_base_(ego_wheel_base),
      ego_circle_radius_(ego_circle_radius),
      init_v_(init_v),
      hard_safe_distance_(hard_safe_distance),
      soft_safe_distance_(soft_safe_distance),
      lane_width_(lane_width){};

double BoundaryCost::MakeCost(Node3D& vertex) const {
  ad_common::math::Vec2d back_axis_position(vertex.GetX(), vertex.GetY());
  ad_common::math::Vec2d front_axis_position(
      vertex.GetX() + ego_wheel_base_ * std::cos(vertex.GetPhi()),
      vertex.GetY() + ego_wheel_base_ * std::sin(vertex.GetPhi()));

  double distance_lidarRB = 100.0;
  double distance_cameraRB = 100.0;

  // for (const auto& lineseg : Lidar_RB_list_) {
  //   // get distance to box
  //   double distance_back = lineseg.DistanceTo(back_axis_position) -
  //   ego_circle_radius_; double distance_front =
  //   lineseg.DistanceTo(front_axis_position) - ego_circle_radius_;
  //   distance_lidarRB = std::min(std::min(distance_back, distance_front),
  //   distance_lidarRB);
  // }

  // for (const auto& lineseg : Camera_RB_list_) {
  //   // get distance to box
  //   double distance_back = lineseg.DistanceTo(back_axis_position) -
  //   ego_circle_radius_; double distance_front =
  //   lineseg.DistanceTo(front_axis_position) - ego_circle_radius_;
  //   distance_cameraRB = std::min(std::min(distance_back, distance_front),
  //   distance_cameraRB);
  // }

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

  // double distance = std::min(distance_lidarRB, distance_cameraRB);
  double distance =
      lane_width_ / 2 - (std::fabs(vertex.GetL()) + vehicle_param.width / 2);
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