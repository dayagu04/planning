#include "stitching_cost.h"
#include <cmath>

namespace planning {
namespace ara_star {

StitchingCost::StitchingCost(double weight,
                             const pnc::mathlib::spline& last2cur_spline,
                             double spline_s_min, double spline_s_max)
    : BaseCost(BaseCost::CostType::STITCHING, weight),
      last2cur_spline_(last2cur_spline),
      spline_s_min_(spline_s_min),
      spline_s_max_(spline_s_max) {}

double StitchingCost::MakeCost(Node3D& vertex) const {
  double s = vertex.GetS();
  if (s < spline_s_min_ || s > spline_s_max_) {
    // 超出样条范围，不施加代价
    vertex.SetStitchingCost(0.0);
    return 0.0;
  }
  
  double last_l = last2cur_spline_(s);      // 上一帧在该s处的l值
  double current_l = vertex.GetL();
  double l_error = std::fabs(current_l - last_l);
  
  // 可选：添加距离衰减（若需要）
  // double decay_factor = 1.0;
  // if (stitching_cost_time_decay_factor_ > 0) {
  //   double delta_s = s - start_s_; // 需要传入起点s
  //   decay_factor = 1.0 - (delta_s * stitching_cost_time_decay_factor_) *
  //                         (delta_s * stitching_cost_time_decay_factor_);
  //   decay_factor = std::max(0.0, decay_factor);
  // }
  // l_error *= decay_factor;
  
  vertex.SetStitchingCost(l_error * GetCostWeight());
  return l_error;
}

}  // namespace ara_star
}  // namespace planning