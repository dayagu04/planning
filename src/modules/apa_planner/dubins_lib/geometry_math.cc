#include "geometry_math.h"

#include <cmath>
#include <iostream>
#include <utility>

#include "Eigen/Geometry"
#include "math_lib.h"

namespace pnc {
namespace geometry_lib {

const double NormalizeAngle(const double angle) {
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  return a - M_PI;
}

const double NormSquareOfVector2d(const Eigen::Vector2d &p1) {
  return p1.x() * p1.x() + p1.y() * p1.y();
}

const double NormSquareOfTwoVector2d(const Eigen::Vector2d &p1,
                                     const Eigen::Vector2d &p2) {
  return NormSquareOfVector2d(p1 - p2);
}

const double CalPoint2LineDist(const Eigen::Vector2d &pO,
                               const LineSegment &line) {
  return std::sqrt(CalPoint2LineDistSquare(pO, line));
}

const double CalPoint2LineDistSquare(const Eigen::Vector2d &pO,
                                     const LineSegment &line) {
  Eigen::Vector2d v_AB = line.pB - line.pA;
  Eigen::Vector2d v_AO = pO - line.pA;

  const double v_AO_dot_v_AB = v_AO.dot(v_AB);

  return NormSquareOfVector2d(v_AO) -
         v_AO_dot_v_AB * v_AO_dot_v_AB / NormSquareOfVector2d(v_AB);
}

const bool CheckLineSegmentInCircle(const LineSegment &line, const Circle &c) {
  const auto &R = c.radius;
  const auto &pO = c.center;
  const double d2 = CalPoint2LineDistSquare(pO, line);

  if (d2 > R * R) {
    return false;
  } else {
    const auto OA_norm = (pO - line.pA).norm();
    const auto OB_norm = (pO - line.pB).norm();

    return (OA_norm < R || OB_norm < R);
  }
}

const bool CalTangentPointsOfEqualCircles(TangentOutput &output,
                                          const Circle &c1, const Circle &c2) {
  const auto &pO1 = c1.center;
  const auto &pO2 = c2.center;
  const auto &R = c1.radius;
  const auto v_O1O2 = pO2 - pO1;
  const auto d_O1O2 = v_O1O2.norm();

  if (!pnc::mathlib::IsDoubleEqual(c1.radius, c2.radius)) {
    std::cout << "two inequal radius!" << std::endl;
  }

  bool inner_tagent = true;
  if (d_O1O2 < c1.radius + c2.radius) {
    std::cout << "outter tangent!" << std::endl;
    inner_tagent = false;
  } else {
    std::cout << "inner tangent!" << std::endl;
    inner_tagent = true;
  }

  if (inner_tagent) {
    // for inner case
    const auto L = 0.5 * d_O1O2;

    const auto sin_theta = R / L;
    const auto cos_theta = std::sqrt(1.0 - sin_theta * sin_theta);

    const auto pM = 0.5 * (pO1 + pO2);

    Eigen::Matrix2d rot_m;
    rot_m << cos_theta, -sin_theta, sin_theta, cos_theta;

    const auto normal_v_MO1 = (pO1 - pM).normalized();
    const auto norm_MTa1 = L * cos_theta;

    const auto v_MTa1 = rot_m * normal_v_MO1 * norm_MTa1;
    const auto v_MTb1 = rot_m.transpose() * normal_v_MO1 * norm_MTa1;

    output.tagent_points_a.first = pM + v_MTa1;
    output.tagent_points_a.second = pM - v_MTa1;

    output.tagent_points_b.first = pM + v_MTb1;
    output.tagent_points_b.second = pM - v_MTb1;
  } else {
    // too close, for outer case
    const auto normal_v_O1O2 = v_O1O2.normalized();
    Eigen::Vector2d n_O1(-normal_v_O1O2.y(), normal_v_O1O2.x());

    output.tagent_points_a.first = pO1 + n_O1 * R;
    output.tagent_points_a.second = pO2 + n_O1 * R;

    output.tagent_points_b.first = pO1 - n_O1 * R;
    output.tagent_points_b.second = pO2 - n_O1 * R;
  }

  return true;
}

const Eigen::Matrix2d GetRotm2dFromTwoVec(const Eigen::Vector2d &a,
                                          const Eigen::Vector2d &b) {
  Eigen::Matrix2d rotm;

  Eigen::Vector3d a_3d(a.x(), a.y(), 0.0);
  Eigen::Vector3d b_3d(b.x(), b.y(), 0.0);

  a_3d.normalize();
  b_3d.normalize();

  // TODO: consider very small a and b

  const auto cos_theta = a_3d.dot(b_3d);
  const auto cross_a_b = a_3d.cross(b_3d);
  const double sign = cross_a_b.z() >= 0.0 ? 1.0 : -1.0;
  const double sin_theta = cross_a_b.norm() * sign;

  rotm << cos_theta, -sin_theta, sin_theta, cos_theta;

  return rotm;
}

double GetAngleFromTwoVec(const Eigen::Vector2d &a, const Eigen::Vector2d &b) {
  if (a.norm() < 0.00001 || b.norm() < 0.00001) {
    return 0.0;
  } else {
    Eigen::Vector3d a_3d(a(0), a(1), 0.0);
    Eigen::Vector3d b_3d(b(0), b(1), 0.0);

    a_3d.normalize();
    b_3d.normalize();

    double cos_theta = a_3d.dot(b_3d);
    Eigen::Vector3d cross_a_b = a_3d.cross(b_3d);
    double sign = cross_a_b.z() >= 0.0 ? 1.0 : -1.0;
    double sin_theta = cross_a_b.norm() * sign;
    double theta = std::atan2(sin_theta, cos_theta);

    return theta;
  }
}

const Eigen::Matrix2d GetRotm2dFromTheta(const double theta) {
  const double cos_theta = std::cos(theta);
  const double sin_theta = std::sin(theta);
  Eigen::Matrix2d rot_m;
  rot_m << cos_theta, -sin_theta, sin_theta, cos_theta;

  return rot_m;
}

}  // namespace geometry_lib
}  // namespace pnc