#ifndef __GEOMETERY_MATH_H__
#define __GEOMETERY_MATH_H__

#include <math.h>

#include <cmath>
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
  Eigen::Vector2d pA = Eigen::Vector2d::Zero();
  Eigen::Vector2d pB = Eigen::Vector2d::Zero();
  double length = 0.0;
  double headingA = 0.0;
  double headingB = 0.0;
  bool is_anti_clockwise = true;
};

struct TangentOutput {
  std::pair<Eigen::Vector2d, Eigen::Vector2d> tagent_points_a;
  std::pair<Eigen::Vector2d, Eigen::Vector2d> tagent_points_b;
  Eigen::Vector2d cross_point;
};

const double NormalizeAngle(const double angle);
struct GlobalToLocalTf {
  Eigen::Vector2d pos_n_ori = Eigen::Vector2d::Zero();
  Eigen::Matrix2d rot_m = Eigen::Matrix2d::Identity();
  double heading_ori = 0.0;

  void Init(const Eigen::Vector2d &p_n_ori, double theta_ori) {
    pos_n_ori = p_n_ori;
    heading_ori = theta_ori;
    const double cos_theta = std::cos(theta_ori);
    const double sin_theta = std::sin(theta_ori);
    rot_m << cos_theta, sin_theta, -sin_theta, cos_theta;
  }

  const Eigen::Vector2d GetPos(const Eigen::Vector2d &p_n) const {
    return rot_m * (p_n - pos_n_ori);
  }

  const double GetHeading(const double heading) const {
    return NormalizeAngle(heading - heading_ori);
  }
};
struct LocalToGlobalTf {
  Eigen::Vector2d pos_n_ori = Eigen::Vector2d::Zero();
  Eigen::Matrix2d rot_m = Eigen::Matrix2d::Identity();
  double heading_ori = 0.0;

  void Init(const Eigen::Vector2d &p_n_ori, double theta_ori) {
    pos_n_ori = p_n_ori;
    heading_ori = theta_ori;
    const double cos_theta = std::cos(theta_ori);
    const double sin_theta = std::sin(theta_ori);
    rot_m << cos_theta, -sin_theta, sin_theta, cos_theta;
  }

  const Eigen::Vector2d GetPos(Eigen::Vector2d &p_n) const {
    return (rot_m * p_n + pos_n_ori);
  }

  const double GetHeading(const double heading) const {
    return NormalizeAngle(heading + heading_ori);
  }
};

double GetAngleFromTwoVec(const Eigen::Vector2d &a, const Eigen::Vector2d &b);

const Eigen::Matrix2d GetRotm2dFromTheta(const double theta);

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