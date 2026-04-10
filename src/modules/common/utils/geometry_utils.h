#pragma once
#include <vector>
#include "common.h"
#include "math/vec2d.h"
#include "utils/path_point.h"

namespace planning {

double Orientation(const Point2d& a, const Point2d& b, const Point2d& c);

bool OnSegment(const Point2d& a, const Point2d& b, const Point2d& p);

bool SegmentsIntersect(const Point2d& p1, const Point2d& p2,
                       const Point2d& q1, const Point2d& q2);

bool PolylinesIntersect(const std::vector<Point2d>& A,
                        const std::vector<Point2d>& B);

// 计算自车在给定位姿下的4个角点（世界坐标系）
// 返回顺序：左前、右前、右后、左后
std::vector<planning_math::Vec2d> GetEgoCorners(
    double x, double y, double theta,
    double half_w, double front, double rear);

// 重载：从 PathPoint 计算
inline std::vector<planning_math::Vec2d> GetEgoCorners(
    const planning_math::PathPoint& pt,
    double half_w, double front, double rear) {
  return GetEgoCorners(pt.x(), pt.y(), pt.theta(), half_w, front, rear);
}

}  // namespace planning
