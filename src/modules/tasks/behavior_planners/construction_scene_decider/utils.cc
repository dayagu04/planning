#include "utils.h"
#include <algorithm>  // 包含std::min_element所需的头文件
#include "math.h"
#include "tuple"
namespace planning {
namespace construction_scene_utils {
static constexpr double EPS = 1e-12;

double orientation(const Point2d& a, const Point2d& b, const Point2d& c) {
  // >0: c 在 ab 的左侧；<0: 右侧；=0: 共线
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

bool onSegment(const Point2d& a, const Point2d& b, const Point2d& p) {
  if (fabs(orientation(a, b, p)) > EPS) return false;
  return (std::min(a.x, b.x) - EPS <= p.x && p.x <= std::max(a.x, b.x) + EPS &&
          std::min(a.y, b.y) - EPS <= p.y && p.y <= std::max(a.y, b.y) + EPS);
}

bool segmentsIntersect(const Point2d& p1, const Point2d& p2, const Point2d& q1,
                       const Point2d& q2) {
  double o1 = orientation(p1, p2, q1);
  double o2 = orientation(p1, p2, q2);
  double o3 = orientation(q1, q2, p1);
  double o4 = orientation(q1, q2, p2);

  if ((o1 * o2 < 0) && (o3 * o4 < 0)) return true;

  if (fabs(o1) <= EPS && onSegment(p1, p2, q1)) return true;
  if (fabs(o2) <= EPS && onSegment(p1, p2, q2)) return true;
  if (fabs(o3) <= EPS && onSegment(q1, q2, p1)) return true;
  if (fabs(o4) <= EPS && onSegment(q1, q2, p2)) return true;

  return false;
}

// 折线是否相交：任意两段相交即相交
bool polylinesIntersect(const std::vector<Point2d>& A,
                        const std::vector<Point2d>& B) {
  for (size_t i = 0; i + 1 < A.size(); ++i) {
    for (size_t j = 0; j + 1 < B.size(); ++j) {
      if (segmentsIntersect(A[i], A[i + 1], B[j], B[j + 1])) return true;
    }
  }
  return false;
}
}  // namespace construction_scene_utils
}  // namespace planning
