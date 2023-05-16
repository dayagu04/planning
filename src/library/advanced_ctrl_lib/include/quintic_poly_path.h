#ifndef __QUINTIC_POLY_PATH_H__
#define __QUINTIC_POLY_PATH_H__

// x = a5_x*t^5 + a4_x*t^4 + a3_x*t^3 + a2_x*t^2 + a1_x*t + a0_x
// y = a5_y*t^5 + a4_y*t^4 + a3_y*t^3 + a2_y*t^2 + a1_y*t + a0_y

#include <Eigen/Core>
#include <vector>

namespace pnc {

namespace spline {

class QuinticPolynominalPath {
public:
  void SetPoints(const Eigen::Vector2d &x0, const Eigen::Vector2d &xT,
                 const Eigen::Vector2d &dx0, const Eigen::Vector2d &dxT,
                 const Eigen::Vector2d &ddx0, const Eigen::Vector2d &ddxT,
                 const double T);

  Eigen::Vector2d operator()(double t) const;

private:
  Eigen::Matrix<double, 6, 1> coef_x_;
  Eigen::Matrix<double, 6, 1> coef_y_;
  Eigen::Matrix<double, 6, 1> T_;
};
} // namespace spline
} // namespace pnc

#endif
