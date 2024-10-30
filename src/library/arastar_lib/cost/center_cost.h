#pragma once
#include "base_cost.h"
#include "utils/kd_path.h"

namespace planning {
namespace ara_star {
class CenterCost : public BaseCost {
public:
  CenterCost(const double weight, const double ego_wheel_base,
                         const std::shared_ptr<planning_math::KDPath>& ego_lane);

  ~CenterCost() = default;

  double MakeCost(Node3D& vertex) const;

private:
  void NormalizeCost(double& cost) const;

private :
  double ego_wheel_base_ = 0.0;
  const std::shared_ptr<planning_math::KDPath> ego_lane_;
};

} // namespace ara_star
} // namespace planning
