#include "trajectory/center_line_point.h"
#include "math/linear_interpolation.h"

namespace planning {
namespace trajectory {

CenterLinePoint::CenterLinePoint(double x, double y, double s, double l,
                                 double theta, double kappa, double dkappa,
                                 double ddkappa, double left_lane_width,
                                 double right_lane_width)
    : planning_math::PathPoint(x, y, s, l, theta, kappa, dkappa, ddkappa),
      left_lane_width_(left_lane_width),
      right_lane_width_(right_lane_width) {}

CenterLinePoint::CenterLinePoint(double x, double y, double s, double l,
                                 double theta, double kappa, double dkappa,
                                 double ddkappa, double left_lane_width,
                                 double right_lane_width,
                                 double actual_lane_width)
    : planning_math::PathPoint(x, y, s, l, theta, kappa, dkappa, ddkappa),
      left_lane_width_(left_lane_width),
      right_lane_width_(right_lane_width),
      actual_lane_width_(actual_lane_width) {}

bool CenterLinePoint::get_left_lane_width(double* const left_lane_width) const {
  *left_lane_width = left_lane_width_;
  return has_matched_left_boundary_;
}

bool CenterLinePoint::get_right_lane_width(
    double* const right_lane_width) const {
  *right_lane_width = right_lane_width_;
  return has_matched_right_boundary_;
}

bool CenterLinePoint::get_actual_lane_width(
    double* const actual_lane_width) const {
  *actual_lane_width = actual_lane_width_;
  return has_matched_actual_lane_width_;
}

void CenterLinePoint::set_left_lane_width(const double left_lane_width) {
  left_lane_width_ = left_lane_width;
}

void CenterLinePoint::set_right_lane_width(const double right_lane_width) {
  right_lane_width_ = right_lane_width;
}

bool CenterLinePoint::has_matched_left_boundary() const {
  return has_matched_left_boundary_;
}

bool CenterLinePoint::has_matched_right_boundary() const {
  return has_matched_right_boundary_;
}

bool CenterLinePoint::has_matched_actual_lane_width() const {
  return has_matched_actual_lane_width_;
}

void CenterLinePoint::set_has_matched_left_boundary(
    const bool has_matched_left_boundary) {
  has_matched_left_boundary_ = has_matched_left_boundary;
}

void CenterLinePoint::set_has_matched_right_boundary(
    const bool has_matched_right_boundary) {
  has_matched_right_boundary_ = has_matched_right_boundary;
}

void CenterLinePoint::set_has_matched_actual_lane_width(
    const bool has_matched_actual_lane_width) {
  has_matched_actual_lane_width_ = has_matched_actual_lane_width;
}

CenterLinePoint CenterLinePoint::GetInterpolateByS(const CenterLinePoint& p,
                                                   const double s) const {
  double s0 = s_;
  double s1 = p.s();
  double l = 0.0;

  double r = (s - s0) / (s1 - s0);
  double x = planning_math::lerp(x_, p.x(), r);
  double y = planning_math::lerp(y_, p.y(), r);
  double theta = planning_math::slerp(theta_, s_, p.theta(), p.s(), s);
  double kappa = planning_math::lerp(kappa_, p.kappa(), r);
  double dkappa = planning_math::lerp(dkappa_, p.dkappa(), r);
  double ddkappa = planning_math::lerp(ddkappa_, p.ddkappa(), r);
  double p_left_lane_width = 0.0;
  double p_right_lane_width = 0.0;
  p.get_left_lane_width(&p_left_lane_width);
  p.get_right_lane_width(&p_right_lane_width);

  double left_lane_width =
      planning_math::lerp(left_lane_width_, p_left_lane_width, r);
  double right_lane_width =
      planning_math::lerp(right_lane_width_, p_right_lane_width, r);
  CenterLinePoint res_pt(x, y, s, l, theta, kappa, dkappa, ddkappa,
                         left_lane_width, right_lane_width);
  if (!has_matched_left_boundary_ || !p.has_matched_left_boundary()) {
    res_pt.set_has_matched_left_boundary(false);
  }
  if (!has_matched_right_boundary_ || !p.has_matched_right_boundary()) {
    res_pt.set_has_matched_right_boundary(false);
  }
  return res_pt;
}

}  // namespace trajectory
}  // namespace planning