
#include "park_reference_line.h"

namespace planning {

void ParkReferenceLine::Process(const Pose2f &start, const Pose2f &end) {
  Vec2f line_start = Vec2f(start.x, start.y);
  Vec2f line_end = Vec2f(end.x, end.y);

  ref_line_ = LineSegmentf32 (line_start, line_end);

  return;
}

void ParkReferenceLine::GetPointByDist(Vec2f *point,
                                       const float dist) {
  *point = ref_line_.start() + dist * ref_line_.unit_direction();

  return;
}

}  // namespace planning