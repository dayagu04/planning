#pragma once

#include "utils/path_point.h"

namespace planning {
namespace trajectory {

class CenterLinePoint : public planning_math::PathPoint {
 public:
  CenterLinePoint() = default;

  explicit CenterLinePoint(double x, double y, double s, double l, double theta,
                           double kappa, double dkappa, double ddkappa,
                           double left_lane_width, double right_lane_width);
  explicit CenterLinePoint(double x, double y, double s, double l, double theta,
                           double kappa, double dkappa, double ddkappa,
                           double left_lane_width, double right_lane_width,
                           double actual_lane_width);

  ~CenterLinePoint() = default;

  bool get_left_lane_width(double* const left_lane_width) const;

  bool get_right_lane_width(double* const right_lane_width) const;

  bool get_actual_lane_width(double* const actual_lane_width) const;

  bool has_matched_left_boundary() const;

  bool has_matched_right_boundary() const;

  void set_has_matched_left_boundary(const bool has_matched_left_boundary);

  void set_has_matched_right_boundary(const bool has_matched_right_boundary);

  void set_left_lane_width(const double left_lane_width);

  void set_right_lane_width(const double right_lane_width);

  void set_has_matched_actual_lane_width(
      const bool has_matched_actual_lane_width);

  bool has_matched_actual_lane_width() const;

  CenterLinePoint GetInterpolateByS(const CenterLinePoint& p,
                                    const double s) const;

 private:
  double left_lane_width_;
  double right_lane_width_;
  double actual_lane_width_;
  bool has_matched_left_boundary_ = true;
  bool has_matched_right_boundary_ = true;
  bool has_matched_actual_lane_width_ = false;
};

}  // namespace trajectory
}  // namespace planning