#include "center_cost.h"
#include <cmath>
#include <iostream>
#include "ad_common/math/vec2d.h"

namespace planning {
namespace ara_star {
namespace {
constexpr double kLaneWidthThreshold = 4.0;
}  // namespace

CenterCost::CenterCost(const double weight, const double ego_wheel_base,
                       const std::shared_ptr<planning_math::KDPath>& ego_lane)
    : BaseCost(BaseCost::CostType::CENTER, weight),
      ego_wheel_base_(ego_wheel_base),
      ego_lane_(ego_lane){

      };

double CenterCost::MakeCost(Node3D& vertex) const {
  ad_common::math::Vec2d front_axis_position(
      vertex.GetX() + ego_wheel_base_ * std::cos(vertex.GetPhi()),
      vertex.GetY() + ego_wheel_base_ * std::sin(vertex.GetPhi()));
  double ego_front_s = 0.0;
  double ego_front_l = 0.0;
  if (!ego_lane_->XYToSL(front_axis_position.x(), front_axis_position.y(),
                         &ego_front_s, &ego_front_l)) {
    // std::cout << "ego out of lane" << std::endl;
  }

  // double max_l = std::abs(std::max(vertex.GetL(), ego_front_l));
  double max_l = std::max(std::fabs(vertex.GetL()), std::fabs(ego_front_l));

  // double max_l = (std::abs(vertex.GetL()) + std::abs(ego_front_l)) / 2;

  // double max_l = std::abs(vertex.GetL());

  double cost = max_l;  // use linear cost
  // double cost = std::max(max_l, std::pow(2, max_l));  use square to make the
  // cost more sensitive

  NormalizeCost(cost);

  vertex.SetCenterCost(cost * GetCostWeight());
  return cost;
}

void CenterCost::NormalizeCost(double& cost) const {
  if (cost < kLaneWidthThreshold) {
    cost = cost / 2.0 * 10.0;
  } else {
    cost = 20.0 + (cost - kLaneWidthThreshold);
  }
}

}  // namespace ara_star
}  // namespace planning
