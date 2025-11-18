#include "geometry_utils.h"
#include <algorithm>
#include "math.h"
#include "tuple"
namespace planning {
namespace {
constexpr double EPS = 1e-12;
}  // namespace

double Orientation(const Point2d& a, const Point2d& b, const Point2d& c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

bool OnSegment(const Point2d& a, const Point2d& b, const Point2d& p) {
  if (std::fabs(Orientation(a, b, p)) > EPS) return false;
  return (std::min(a.x, b.x) - EPS <= p.x && p.x <= std::max(a.x, b.x) + EPS &&
          std::min(a.y, b.y) - EPS <= p.y && p.y <= std::max(a.y, b.y) + EPS);
}

bool SegmentsIntersect(const Point2d& p1, const Point2d& p2,
                       const Point2d& q1, const Point2d& q2) {
  double o1 = Orientation(p1, p2, q1);
  double o2 = Orientation(p1, p2, q2);
  double o3 = Orientation(q1, q2, p1);
  double o4 = Orientation(q1, q2, p2);

  if ((o1 * o2 < 0) && (o3 * o4 < 0)) return true;

  if (std::fabs(o1) <= EPS && OnSegment(p1, p2, q1)) return true;
  if (std::fabs(o2) <= EPS && OnSegment(p1, p2, q2)) return true;
  if (std::fabs(o3) <= EPS && OnSegment(q1, q2, p1)) return true;
  if (std::fabs(o4) <= EPS && OnSegment(q1, q2, p2)) return true;

  return false;
}

bool PolylinesIntersect(const std::vector<Point2d>& A,
                        const std::vector<Point2d>& B) {
  for (size_t i = 0; i + 1 < A.size(); ++i) {
    for (size_t j = 0; j + 1 < B.size(); ++j) {
      if (SegmentsIntersect(A[i], A[i + 1], B[j], B[j + 1])) return true;
    }
  }
  return false;
}

}  // namespace planning
