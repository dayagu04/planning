#include "geometry_math.h"

#include <cmath>
#include <iostream>

#include "Eigen/Geometry"
#include "math_lib.h"

namespace pnc {
namespace geometry_lib {

const double CalPoint2LineDist(const Eigen::Vector2d &pO,
                               const LineSegment &line) {
  Eigen::Vector2d vec_AB = line.pB - line.pA;
  Eigen::Vector2d vec_AO = pO - line.pA;
  const double AC_norm = vec_AO.dot(vec_AB) / vec_AB.norm();

  return std::sqrt(AC_norm * AC_norm - vec_AO.dot(vec_AO));
}

const bool CheckLineSegmentInCircle(const LineSegment &line, const Circle &c) {
  const auto &R = c.radius;
  const auto &pO = c.center;
  const double d = CalPoint2LineDist(pO, line);

  if (d > R) {
    return false;
  } else {
    const auto OA_norm = (pO - line.pA).norm();
    const auto OB_norm = (pO - line.pB).norm();

    return (OA_norm < R || OB_norm < R);
  }
}

const bool CalInnerTangentPointsOfEqualCircles(TangentOutput &output,
                                               const Circle &c1,
                                               const Circle &c2) {
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

const Eigen::Matrix2d GetRotm2dFromTwoVec(Eigen::Vector2d &a,
                                          Eigen::Vector2d &b) {
  Eigen::Matrix2d rotm;

  Eigen::Vector3d a_3d(a.x(), a.y(), 0.0);
  Eigen::Vector3d b_3d(b.x(), b.y(), 0.0);

  a_3d.normalize();
  b_3d.normalize();

  const auto cos_theta = a.dot(b);
  const auto cross_a_b = a_3d.cross(b_3d);
  const double sign = cross_a_b.z() >= 0.0 ? 1.0 : -1.0;
  const double sin_theta = cross_a_b.norm() * sign;

  rotm << cos_theta, -sin_theta, cos_theta, sin_theta;

  return rotm;
}

}  // namespace geometry_lib
}  // namespace pnc