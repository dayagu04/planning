#ifndef __SPLINE_PROJECTION_H__
#define __SPLINE_PROJECTION_H__

#include "Eigen/Core"
#include "spline.h"

namespace pnc {
namespace spline {

class Projection {
  struct Result {
    double dist_proj = 0.0;
    double s_proj = 0.0;
    Eigen::Vector2d point_proj{};
    bool success = false;
  };

 public:
  void CalProjectionPoint(const mathlib::spline &x_s_spline,
                          const mathlib::spline &y_s_spline,
                          const double s_start, const double s_end,
                          const Eigen::Vector2d &x);
  void Reset();
  const Result &GetOutput() const { return result_; }

 private:
  Result result_;
  double max_iter_ = 5;
  double tol_ = 1e-3;
  double alpha_ = 0.75;
};

}  // namespace spline
}  // namespace pnc

#endif
