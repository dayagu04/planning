#pragma once
#include <vector>
#include "common.h"

namespace planning {

double Orientation(const Point2d& a, const Point2d& b, const Point2d& c);

bool OnSegment(const Point2d& a, const Point2d& b, const Point2d& p);

bool SegmentsIntersect(const Point2d& p1, const Point2d& p2,
                       const Point2d& q1, const Point2d& q2);

bool PolylinesIntersect(const std::vector<Point2d>& A,
                        const std::vector<Point2d>& B);

}  // namespace planning
