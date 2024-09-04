#pragma once

#include "pose2d.h"
#include "ad_common/math/line_segment2d.h"

namespace planning {

class ParkReferenceLine {
 public:
  ParkReferenceLine() = default;

  void Init(const Pose2D &start, const Pose2D &end);

  const ad_common::math::Vec2d &GetStartPoint() const {
    return ref_line_.start();
  }

  const ad_common::math::Vec2d &GetEndPoint() const { return ref_line_.end(); }

  const double GetHeading() const { return ref_line_.heading(); }

  const ad_common::math::Vec2d &UnitDirection() const {
    return ref_line_.unit_direction();
  }

  void GetPointByDist(ad_common::math::Vec2d *point, const double dist);

 private:
  ad_common::math::LineSegment2d ref_line_;
};

}  // namespace planning