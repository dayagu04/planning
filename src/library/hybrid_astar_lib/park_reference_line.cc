
#include "park_reference_line.h"

namespace planning {

void ParkReferenceLine::Process(const Pose2D &start, const Pose2D &end) {
  ad_common::math::Vec2d line_start = ad_common::math::Vec2d(start.x, start.y);
  ad_common::math::Vec2d line_end = ad_common::math::Vec2d(end.x, end.y);

  ref_line_ = ad_common::math::LineSegment2d(line_start, line_end);

  return;
}

void ParkReferenceLine::GetPointByDist(ad_common::math::Vec2d *point,
                                       const double dist) {
  *point = ref_line_.start() + dist * ref_line_.unit_direction();

  return;
}

}  // namespace planning