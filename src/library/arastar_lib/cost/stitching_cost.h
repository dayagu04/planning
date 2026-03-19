#pragma once
#include "base_cost.h"
#include "src/library/advanced_ctrl_lib/include/spline.h"

namespace planning {
namespace ara_star {
class StitchingCost : public BaseCost {
 public:
  StitchingCost(double weight, 
                const pnc::mathlib::spline& last2cur_spline,
                double spline_s_min, double spline_s_max);

  ~StitchingCost() = default;

  double MakeCost(Node3D& vertex) const override;

 private:
  const pnc::mathlib::spline& last2cur_spline_;
  double spline_s_min_;
  double spline_s_max_;
};

}  // namespace ara_star
}  // namespace planning