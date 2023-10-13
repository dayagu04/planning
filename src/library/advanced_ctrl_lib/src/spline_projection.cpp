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
  Reset();

  if (s_start > s_end) {
    return;
  }

  // newton iteration to calculate projection point
  const double half_length = (s_start + s_end) * 0.5;
  double s_proj = half_length;
  auto const &x0 = x.x();
  auto const &y0 = x.y();

  // get s_proj
  for (size_t i = 0; i < max_iter_; ++i) {
    const double xs = x_s_spline(s_proj);
    const double xs_derv_1st = x_s_spline.deriv(1, s_proj);
    const double xs_derv_2nd = x_s_spline.deriv(2, s_proj);

    const double ys = y_s_spline(s_proj);
    const double ys_derv_1st = y_s_spline.deriv(1, s_proj);
    const double ys_derv_2nd = y_s_spline.deriv(2, s_proj);

    const double deriv_1st = xs * xs_derv_1st + ys * ys_derv_1st -
                             x0 * xs_derv_1st - y0 * ys_derv_1st;

    const double deriv_2nd = xs_derv_1st * xs_derv_1st + xs * xs_derv_2nd +
                             ys_derv_1st * ys_derv_1st + ys * ys_derv_2nd -
                             x0 * xs_derv_2nd - y0 * ys_derv_2nd;

    if (mathlib::IsDoubleEqual(deriv_2nd, 0.0, 1e-9)) {
      return;
    }

    const double ds = pnc::mathlib::Limit(-deriv_1st / deriv_2nd * alpha_,
                                          half_length * 0.95);

    s_proj += ds;
    s_proj = mathlib::Clamp(s_proj, s_start, s_end);

    // terminate when tol achived
    if (std::fabs(ds) < tol_) {
      break;
    }
  }

  // cal the euclidean distance between the projection point and the current pos
  Eigen::Vector2d point_proj(x_s_spline(s_proj), y_s_spline(s_proj));
  const double dist_proj = (point_proj - x).norm();

  // first point of trajectory
  const Eigen::Vector2d first_pos(x_s_spline(s_start), y_s_spline(s_start));
  const double dist_first = (first_pos - x).norm();

  // last point of trajectory
  const Eigen::Vector2d last_pos(x_s_spline(s_end), y_s_spline(s_end));
  const double dist_last = (last_pos - x).norm();

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
  result_.success = true;
}

void Projection::Reset() {
  result_.point_proj.setZero();
  result_.s_proj = 0.0;
  result_.dist_proj = 0.0;
  result_.success = false;
}

}  // namespace spline
}  // namespace pnc
