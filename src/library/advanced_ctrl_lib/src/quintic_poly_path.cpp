#include "quintic_poly_path.h"
#include <Eigen/LU>

namespace pnc {

namespace spline {

void QuinticPolynominalPath::SetPoints(
    const Eigen::Vector2d &x0, const Eigen::Vector2d &xT,
    const Eigen::Vector2d &dx0, const Eigen::Vector2d &dxT,
    const Eigen::Vector2d &ddx0, const Eigen::Vector2d &ddxT, const double T) {
  T_[0] = 1.0;
  T_[1] = T;
  T_[2] = T * T_[1];
  T_[3] = T * T_[2];
  T_[4] = T * T_[3];
  T_[5] = T * T_[4];

  Eigen::Matrix3d coef_matrix;
  coef_matrix << T_[5], T_[4], T_[3], 5.0 * T_[4], 4.0 * T_[3], 3.0 * T_[2],
      20.0 * T_[3], 12.0 * T_[2], 6.0 * T;

  // x coef calculation
  coef_x_[0] = x0.x();
  coef_x_[1] = dx0.x();
  coef_x_[2] = 0.5 * ddx0.x();

  Eigen::Vector3d x_vec(
      xT.x() - (coef_x_[2] * T_[2] + coef_x_[1] * T_[1] + coef_x_[0]),
      dxT.x() - (2.0 * T_[1] * coef_x_[2] + coef_x_[1]),
      ddxT.x() - 2.0 * coef_x_[2]);

  const Eigen::Vector3d coef_x_vec = coef_matrix.inverse() * x_vec;
  coef_x_[5] = coef_x_vec[0];
  coef_x_[4] = coef_x_vec[1];
  coef_x_[3] = coef_x_vec[2];

  // y coef calculation
  coef_y_[0] = x0.y();
  coef_y_[1] = dx0.y();
  coef_y_[2] = 0.5 * ddx0.y();

  Eigen::Vector3d y_vec(
      xT.y() - (coef_y_[2] * T_[2] + coef_y_[1] * T_[1] + coef_y_[0]),
      dxT.y() - (2.0 * T_[1] * coef_y_[2] + coef_y_[1]),
      ddxT.y() - 2.0 * coef_y_[2]);

  const Eigen::Vector3d coef_y_vec = coef_matrix.inverse() * y_vec;
  coef_y_[5] = coef_y_vec[0];
  coef_y_[4] = coef_y_vec[1];
  coef_y_[3] = coef_y_vec[2];
}

Eigen::Vector2d QuinticPolynominalPath::operator()(double t) const {
  const double t2 = t * t;
  const double t3 = t2 * t;
  const double t4 = t3 * t;
  const double t5 = t4 * t;

  return Eigen::Vector2d(coef_x_[5] * t5 + coef_x_[4] * t4 + coef_x_[3] * t3 +
                             coef_x_[2] * t2 + coef_x_[1] * t + coef_x_[0],
                         coef_y_[5] * t5 + coef_y_[4] * t4 + coef_y_[3] * t3 +
                             coef_y_[2] * t2 + coef_y_[1] * t + coef_y_[0]);
}
} // namespace spline
} // namespace pnc
