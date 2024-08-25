#pragma once

#include <memory>
#include <vector>

#include "src/common/vec2d.h"

namespace planning {

class Crosswalk {
 public:
  Crosswalk() = default;

  Crosswalk(int32_t track_id);

  virtual ~Crosswalk() = default;

  int track_id() const;

  void set_id(const int32_t id);

  double width() const;

  void set_width(const double width);

  const std::vector<planning::planning_math::Vec2d>& center_line_points() const;

  void set_center_line_points(
      std::vector<planning::planning_math::Vec2d>&& center_line_points);

  const std::vector<planning::planning_math::Vec2d>& polygon_points() const;

  void set_polygon_points(
      std::vector<planning::planning_math::Vec2d>&& polygon_points);

 protected:
  int track_id_ = -1;

  double width_ = 0.0;

  // The points of center line, which is the center of the crosswalk
  // Tips:the vector is empty now.
  std::vector<planning::planning_math::Vec2d> center_line_points_;

  // The points of polygon.The p0->p1 and p2->p3 are the two sides of the
  // crosswalk.
  std::vector<planning::planning_math::Vec2d> polygon_points_;
};

}  // namespace planning
