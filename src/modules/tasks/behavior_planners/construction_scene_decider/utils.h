#include "common.h"
#include "vector"

namespace planning {
namespace construction_scene_utils {

double Orientation(const Point2d& a, const Point2d& b, const Point2d& c);

bool OnSegment(const Point2d& a, const Point2d& b, const Point2d& p);

bool SegmentsIntersect(const Point2d& p1, const Point2d& p2, const Point2d& q1,
                       const Point2d& q2);

// 折线是否相交：任意两段相交即相交
bool PolylinesIntersect(const std::vector<Point2d>& A,
                        const std::vector<Point2d>& B);
}  // namespace construction_scene_utils
}  // namespace planning
