#pragma once

#include "line_segmentf32.h"
#include "pose2d.h"
#include "vecf32.h"

namespace planning {

class ParkReferenceLine {
 public:
  ParkReferenceLine() = default;

  void Process(const Pose2D &start, const Pose2D &end);

  const Vec2df32 &GetStartPoint() const {
    return ref_line_.start();
  }

  const Vec2df32 &GetEndPoint() const { return ref_line_.end(); }

  const float GetHeading() const { return ref_line_.heading(); }

  const Vec2df32 &UnitDirection() const {
    return ref_line_.unit_direction();
  }

  void GetPointByDist(Vec2df32 *point, const float dist);

 private:
  LineSegmentf32 ref_line_;
};

}  // namespace planning