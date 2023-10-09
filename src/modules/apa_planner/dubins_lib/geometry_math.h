#ifndef __GEOMETERY_MATH_H__
#define __GEOMETERY_MATH_H__

#include <math.h>

#include <cmath>
#include <utility>

#include "Eigen/Core"
#include "Eigen/src/Core/Matrix.h"

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

struct GlobalToLocalTf {
  Eigen::Vector2d pos_n = Eigen::Vector2d::Zero();
  Eigen::Matrix2d rot_m = Eigen::Matrix2d::Identity();

  void Init(Eigen::Vector2d &p_n, double theta) {
    pos_n = p_n;
    const double cos_theta = std::cos(theta);
    const double sin_theta = std::sin(theta);
    rot_m << cos_theta, sin_theta, -sin_theta, cos_theta;
  }

  const Eigen::Vector2d GetPos(const Eigen::Vector2d &pn) const {
    return rot_m * (pn - pos_n);
  }

  const Eigen::Vector2d GetHeadingVec(const double heading) const {
    return rot_m * Eigen::Vector2d(cos(heading), sin(heading));
  }
};

struct LocalToGlobalTf {
  Eigen::Vector2d pos_n = Eigen::Vector2d::Zero();
  Eigen::Matrix2d rot_m = Eigen::Matrix2d::Identity();

  void Init(const Eigen::Vector2d &p_n, double theta) {
    pos_n = p_n;
    const double cos_theta = std::cos(theta);
    const double sin_theta = std::sin(theta);
    rot_m << cos_theta, -sin_theta, sin_theta, cos_theta;
  }

  const Eigen::Vector2d GetPos(Eigen::Vector2d &pn) const {
    return (rot_m * pn + pos_n);
  }

  const Eigen::Vector2d GetHeadingVec(const double heading) const {
    return rot_m * Eigen::Vector2d(cos(heading), sin(heading));
  }
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