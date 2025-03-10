#pragma once
#include "base_cost.h"
#include "src/library/arastar_lib/hybrid_ara_data.h"
#include "src/modules/common/math/math_utils.h"
#include "utils/kd_path.h"

namespace planning {
namespace ara_star {

class BoundaryCost : public BaseCost {
 public:
  BoundaryCost(const double weight, const double init_v,
               const double ego_wheel_base, const double ego_circle_radius,
               const double lane_width,
               const std::shared_ptr<planning_math::KDPath>& target_lane,
               const double& hard_safe_distance,
               const double& soft_safe_distance);
  ~BoundaryCost() = default;

  double MakeCost(Node3D& vertex) const;

 private:
  void NormalizeCost(double& cost) const;

 private:
  // agent list;
  std::shared_ptr<planning_math::KDPath> target_lane_;

  double ego_wheel_base_ = 0.0;
  double ego_circle_radius_ = 0.0;
  double init_v_ = 0.0;
  double hard_safe_distance_ = 0.4;
  double soft_safe_distance_ = 0.6;
  double lane_width_ = 4.0;
};

}  // namespace ara_star

}  // namespace planning