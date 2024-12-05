#pragma once

#include <vector>

#include "dv_spline.h"
#include "st_spline.h"

namespace planning {
class TargetFollowCurve {
 public:
  TargetFollowCurve(DvPoint target_pt, const double& ego_s,
                    const double& cipv_s, const double& cipv_v);
  ~TargetFollowCurve() = default;
  void generate_target_dv_curve();
  void update_target_st_curve(const double dist, const double& cipv_v,
                              const double& cipv_s);
  const DvSpline& get_target_dv_curve() { return target_dv_curve_; }
  const StSpline& get_target_st_curve() { return target_st_curve_; }

 private:
  DvPoint target_point_;
  DvSpline target_dv_curve_;
  StSpline target_st_curve_;
  const double target_st_acc_ = 0.8;
  const double target_st_dec_ = -0.8;
  const double obj_stop_v_thr_ = 0.5;
};

}  // namespace planning