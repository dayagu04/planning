#ifndef __GEOMETERY_MATH_H__
#define __GEOMETERY_MATH_H__

#include <utility>

#include "Eigen/Core"

namespace pnc {

namespace geometry_lib {

struct LineSegment {
  void SetPoints(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2) {
    pA = p1;
    pB = p2;
  }
  Eigen::Vector2d pA = Eigen::Vector2d::Zero();
  Eigen::Vector2d pB = Eigen::Vector2d::Zero();
  double length = 0.0;
};

struct Circle {
  Eigen::Vector2d center = Eigen::Vector2d::Zero();
  double radius = 0.0;
};

struct Arc {
  Circle circle_info;
  Eigen::Vector2d pA;
  Eigen::Vector2d pB;
  double length;
};

struct TangentOutput {
  std::pair<Eigen::Vector2d, Eigen::Vector2d> tagent_points_a;
  std::pair<Eigen::Vector2d, Eigen::Vector2d> tagent_points_b;
  Eigen::Vector2d cross_point;
};

const double CalPoint2LineDist(const Eigen::Vector2d &pO,
                               const LineSegment &line);

const bool CheckLineSegmentInCircle(const LineSegment &line, const Circle &c);

const bool CalTangentPointsOfEqualCircles(TangentOutput &output,
                                          const Circle &c1, const Circle &c2);

const Eigen::Matrix2d GetRotm2dFromTwoVec(const Eigen::Vector2d &a,
                                          const Eigen::Vector2d &b);

}  // namespace geometry_lib
}  // namespace pnc

#endif