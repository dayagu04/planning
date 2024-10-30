#pragma once
#include "base_cost.h"

namespace planning {
namespace ara_star {

class MotionCost : public BaseCost {
public:
  MotionCost(const double weight, const double max_front_steer_angle, const double last_steering_angle);
  ~MotionCost() = default;

  double MakeCost(Node3D& vertex) const;

  void SetLastSteeringAngle(double last_steering_angle) {
    last_steering_angle_ = last_steering_angle;
  }

private:
  void NormalizeSteering(double& cost) const;
  void NormalizeSteeringChange(double& cost) const;


private:
  double last_steering_angle_ = 0.0;
  double max_front_steer_angle_ = 0.0;
};

} // namespace ara_star
} // namespace planning