#pragma once

#include "src/common/vec2d.h"
#include "src/modules/common/math/line_segment2d.h"

namespace planning {

class StopLine {
 public:
  StopLine() = default;

  StopLine(int id);

  StopLine(int id, const planning::planning_math::LineSegment2d& stop_line);

  StopLine(int id, planning::planning_math::LineSegment2d&& stop_line);

  virtual ~StopLine() = default;

  int track_id() const;

  const planning::planning_math::LineSegment2d& stop_line() const;

  double RawDistanceTo(const planning::planning_math::Vec2d& point) const {
    return stop_line_.RawDistanceTo(point);
  }

  double DistanceTo(const planning::planning_math::Vec2d& point) const {
    return stop_line_.DistanceTo(point);
  }

  double DistanceSquareTo(const planning::planning_math::Vec2d& point) const {
    return stop_line_.DistanceSquareTo(point);
  }

  double min_x() const { return stop_line_.min_x(); }

  double min_y() const { return stop_line_.min_y(); }

  double max_x() const { return stop_line_.max_x(); }

  double max_y() const { return stop_line_.max_y(); }

 protected:
  int id_ = -1;
  planning::planning_math::LineSegment2d stop_line_;
};

}  // namespace planning
