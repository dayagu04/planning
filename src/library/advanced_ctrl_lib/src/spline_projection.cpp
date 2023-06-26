#include "spline_projection.h"
#include <cmath>
#include <vector>
#include "math_lib.h"

namespace pnc {
namespace spline {

void Projection::CalProjectionPoint(const mathlib::spline &x_s_spline,
                                    const mathlib::spline &y_s_spline,
                                    const double s_start, const double s_end,
                                    const Eigen::Vector2d &x) {
  // newton iteration to calculate projection point
  double s_proj = 0.0;
  static const size_t newton_iter = 5;
  static const double tol = 1e-3;
  s_proj = 0.0;
  auto const &x0 = x.x();
  auto const &y0 = x.y();

  // get s_proj
  for (size_t i = 0; i < newton_iter; ++i) {
    double xs = x_s_spline(s_proj);
    double xs_derv_1st = x_s_spline.deriv(1, s_proj);
    double xs_derv_2nd = x_s_spline.deriv(2, s_proj);

    double ys = y_s_spline(s_proj);
    double ys_derv_1st = y_s_spline.deriv(1, s_proj);
    double ys_derv_2nd = y_s_spline.deriv(2, s_proj);

    double deriv_1st = xs * xs_derv_1st + ys * ys_derv_1st - x0 * xs_derv_1st -
                       y0 * ys_derv_1st;

    double deriv_2nd = xs_derv_1st * xs_derv_1st + xs * xs_derv_2nd +
                       ys_derv_1st * ys_derv_1st + ys * ys_derv_2nd -
                       x0 * xs_derv_2nd - y0 * ys_derv_2nd;

    if (mathlib::IsDoubleEqual(deriv_2nd, 0.0, 1e-9)) {
      break;
    }

    double ds = -deriv_1st / deriv_2nd;

    s_proj += ds;
    s_proj = mathlib::Clamp(s_proj, s_start, s_end);

    // terminate when tol achived
    if (std::fabs(ds) < tol) {
      break;
    }
  }

  // cal the euclidean distance between the projection point and the current pos
  Eigen::Vector2d point_proj(x_s_spline(s_proj), y_s_spline(s_proj));
  double dist_proj = (point_proj - x).norm();

  // first point of trajectory
  Eigen::Vector2d first_pos(x_s_spline(s_start), y_s_spline(s_start));
  double dist_first = (first_pos - x).norm();

  // last point of trajectory
  Eigen::Vector2d last_pos(x_s_spline(s_end), y_s_spline(s_end));

  double dist_last = (last_pos - x).norm();

  // projection point can also be first (probably) or last point
  if (dist_first < dist_proj && dist_first < dist_last) {
    point_proj = first_pos;
    s_proj = s_start;
  } else if (dist_last < dist_proj && dist_last < dist_first) {
    point_proj = last_pos;
    s_proj = s_end;
  }

  result_.point_proj = point_proj;
  result_.s_proj = s_proj;
  result_.dist_proj = (point_proj - x).norm();
}

}  // namespace spline
}  // namespace pnc
