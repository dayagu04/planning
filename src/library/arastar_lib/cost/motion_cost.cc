#include "motion_cost.h"
#include <iostream>

namespace planning {
namespace ara_star {

MotionCost::MotionCost(const double weight, const double max_front_steer_angle, const double last_steering_angle)
    : BaseCost(BaseCost::CostType::MOTION, weight), last_steering_angle_(last_steering_angle),
      max_front_steer_angle_(max_front_steer_angle){
};

// 转向角越大，cost越大；与上一时刻的转角角之差越大，cost越大
// 为什么不考虑距离的cost？
double MotionCost::MakeCost(Node3D& vertex) const {
  // inplement calculation
  double steering = vertex.GetSteer();
  double cost_steering_change = std::abs(steering - last_steering_angle_);
  NormalizeSteeringChange(cost_steering_change);
  double cost_steering_use = std::abs(steering);
  NormalizeSteering(cost_steering_use);
  double cost = cost_steering_change + cost_steering_use;
  vertex.SetMotionCost(cost * GetCostWeight());
  return cost;
}

void MotionCost::NormalizeSteeringChange(double& cost) const {
  cost = cost / (2 * max_front_steer_angle_) * 10.0;
}

void MotionCost::NormalizeSteering(double& cost) const {
  cost = cost / max_front_steer_angle_ * 10.0;
}

}
} // namespace planning