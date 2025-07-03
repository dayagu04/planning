#pragma once

#include "line_segmentf32.h"
#include "pose2d.h"

namespace planning {

class ParkReferenceLine {
 public:
  ParkReferenceLine() = default;

  void Process(const Pose2f &start, const Pose2f &end);

  const Vec2f &GetStartPoint() const {
    return ref_line_.start();
  }

  const Vec2f &GetEndPoint() const { return ref_line_.end(); }

  const float GetHeading() const { return ref_line_.heading(); }

  const Vec2f &UnitDirection() const {
    return ref_line_.unit_direction();
  }

  void GetPointByDist(Vec2f *point, const float dist);

 private:
  LineSegmentf32 ref_line_;
};

}  // namespace planning