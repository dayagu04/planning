
#include "park_reference_line.h"
#include "line_segmentf32.h"

namespace planning {

void ParkReferenceLine::Process(const Pose2D &start, const Pose2D &end) {
  Vec2df32 line_start = Vec2df32(start.x, start.y);
  Vec2df32 line_end = Vec2df32(end.x, end.y);

  ref_line_ = LineSegmentf32(line_start, line_end);

  return;
}

void ParkReferenceLine::GetPointByDist(Vec2df32 *point,
                                       const float dist) {
  *point = ref_line_.start() + dist * ref_line_.unit_direction();

  return;
}

}  // namespace planning