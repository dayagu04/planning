#ifndef GROUND_LINE_DECIDER_H
#define GROUND_LINE_DECIDER_H

#include <algorithm>
#include <cmath>
#include <vector>

#include "vec2d.h"

namespace planning {

struct GroundLinePoint {
  enum Status {
    UNCLASSIFIED = 0,
    CLASSIFIED = 1,
    NOISE = 2,
  };

  planning_math::Vec2d point;
  Status status;

  bool operator==(const GroundLinePoint &input) {
    return (this->point == input.point && this->status == input.status);
  }

  bool operator!=(const GroundLinePoint &input) {
    return (!(this->point == input.point) || this->status != input.status);
  }
};

class GroundLineDecider {
 public:
  static void update_params(const int min_pts, const double eps);
  static std::vector<std::vector<planning_math::Vec2d>> execute(
      const std::vector<GroundLinePoint> &ground_line_points);

 private:
  static void update_points(
      const std::vector<GroundLinePoint> &ground_line_points,
      std::vector<GroundLinePoint> &points);
  static std::vector<int> calc_cluster(GroundLinePoint &point,
                                       std::vector<GroundLinePoint> &points);
  static std::vector<planning_math::Vec2d> expand_cluster(
      GroundLinePoint &point, std::vector<GroundLinePoint> &points);

  static int min_pts_;
  static double eps_;
};

}  // namespace planning

#endif  // GROUND_LINE_DECIDER_H
