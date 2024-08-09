#include "geometry_math.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <type_traits>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Eigenvalues"
#include "Eigen/Geometry"
#include "math_lib.h"

double kDeg2Rad = M_PI / 180;
double kRad2Deg = 180 / M_PI;

namespace pnc {
namespace geometry_lib {

static const double kRadiusCalcTolerance = 1e-6;
static const double kSameHeadingEps = 0.5 * kDeg2Rad;

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

const Eigen::Vector2d GenHeadingVec(const double heading) {
  return Eigen::Vector2d(std::cos(heading), std::sin(heading));
}

const double CalPoint2LineDist(const Eigen::Vector2d &pO,
                               const LineSegment &line) {
  double dist_square = CalPoint2LineDistSquare(pO, line);
  if (dist_square < 1e-6) {
    return 0.0;
  }
  return std::sqrt(dist_square);
}

const double CalPoint2LineDistSquare(const Eigen::Vector2d &pO,
                                     const LineSegment &line) {
  Eigen::Vector2d v_AB = line.pB - line.pA;
  Eigen::Vector2d v_AO = pO - line.pA;

  const double v_AO_dot_v_AB = v_AO.dot(v_AB);

  if (NormSquareOfVector2d(v_AB) < 1e-4) {
    return 0.0;
  }

  return NormSquareOfVector2d(v_AO) -
         v_AO_dot_v_AB * v_AO_dot_v_AB / NormSquareOfVector2d(v_AB);
}

const bool CheckLineSegmentInCircle(const LineSegment &line, const Circle &c) {
  const auto &R = c.radius;
  const auto &pO = c.center;
  const double d2 = CalPoint2LineDistSquare(pO, line);
  const auto R2 = R * R;
  const auto v_OA = pO - line.pA;
  const auto v_OB = pO - line.pB;

  if (d2 >= R2) {
    return false;
  }

  const auto OA_norm2 = NormSquareOfVector2d(v_OA);
  const auto OB_norm2 = NormSquareOfVector2d(v_OB);

  if (OA_norm2 < R2 || OB_norm2 < R2) {
    return true;
  }

  if (OA_norm2 > R2 && OB_norm2 > R2) {
    const auto v_AB = line.pB - line.pA;

    const auto n_AB = Eigen::Vector2d(-v_AB.y(), v_AB.x());

    const auto cross_OA_n = v_OA.x() * n_AB.y() - n_AB.x() * v_OA.y();
    const auto cross_OB_n = v_OB.x() * n_AB.y() - n_AB.x() * v_OB.y();

    if (cross_OA_n * cross_OB_n < 0.0) {
      return true;
    }
  }

  return false;
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
    // std::cout << "outter tangent!" << std::endl;
    inner_tagent = false;
  } else {
    // std::cout << "inner tangent!" << std::endl;
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

const LineSegment GetEgoHeadingLine(const Eigen::Vector2d &ego_pos,
                                    const double ego_heading) {
  const Eigen::Vector2d next_pos =
      ego_pos + Eigen::Vector2d(std::cos(ego_heading), std::sin(ego_heading));

  return pnc::geometry_lib::LineSegment(ego_pos, next_pos);
}

const bool GetIntersectionFromTwoLineSeg(
    Eigen::Vector2d &intersection, pnc::geometry_lib::LineSegment &line_seg1,
    pnc::geometry_lib::LineSegment &line_seg2) {
  using namespace mathlib;
  Eigen::Vector2d AB = line_seg1.pB - line_seg1.pA;
  Eigen::Vector2d CD = line_seg2.pB - line_seg2.pA;
  if (!IsDoublePositive(AB.norm()) || !IsDoublePositive(CD.norm())) {
    return false;
  }

  if (!IsDoublePositive(line_seg1.length) ||
      !IsDoublePositive(line_seg2.length)) {
    line_seg1.length = AB.norm();
    line_seg2.length = CD.norm();
  }

  if (IsDoubleEqual(GetCrossFromTwoVec2d(AB, CD), 0.0)) {
    Eigen::Vector2d AC = line_seg2.pA - line_seg1.pA;
    if (!IsDoubleEqual(GetCrossFromTwoVec2d(AB, AC), 0.0)) {
      // two line parallel
      return false;
    }
    // two line collinear
    // determine if two line segments overlap
    double min1 = std::min(line_seg1.pA.x(), line_seg1.pB.x());
    double max1 = std::max(line_seg1.pA.x(), line_seg1.pB.x());
    double min2 = std::min(line_seg2.pA.x(), line_seg2.pB.x());
    double max2 = std::max(line_seg2.pA.x(), line_seg2.pB.x());
    bool overlap = (max1 >= min2) && (max2 >= min1);
    if (overlap == true) {
      double dist1 = (line_seg1.pA - line_seg2.pA).norm();
      double dist2 = (line_seg1.pB - line_seg2.pA).norm();
      if (dist1 <= dist2) {
        intersection = line_seg1.pA;
      } else {
        intersection = line_seg1.pB;
      }
      return true;
    } else {
      return false;
    }
  }

  Eigen::Vector2d AC = line_seg2.pA - line_seg1.pA;

  // P is the intersection of line segment AB and CD
  // P = A + t1 * AB;  P = C + t2 * CD
  // t1 * AB - t2 * CD = C - A
  // t1 * AB.x - t2 * CD.x = AC.x
  // t1 * AB.y - t2 * CD.y = AC.y
  // D = | AB.x  -CD.x |
  //     | AB.y  -CD.y |
  double t1 = 0.0;
  double t2 = 0.0;
  double determinant = AB.x() * (-CD.y()) - (-CD.x()) * AB.y();
  if (pnc::mathlib::IsDoubleEqual(determinant, 0.0)) {
    return false;
  } else {
    t1 = (AC.x() * (-CD.y()) - (-CD.x() * AC.y())) / determinant;
    t2 = (AB.x() * AC.y() - AC.x() * AB.y()) / determinant;
  }

  if (IsInBound(t1, 0.0, 1.0) && IsInBound(t2, 0.0, 1.0)) {
    intersection = line_seg1.pA + t1 * AB;
    return true;
  } else {
    return false;
  }
}

const bool GetIntersectionFromTwoLine(Eigen::Vector2d &intersection,
                                      LineSegment &line1, LineSegment &line2) {
  using namespace mathlib;
  Eigen::Vector2d AB = line1.pB - line1.pA;
  Eigen::Vector2d CD = line2.pB - line2.pA;

  if (!IsDoublePositive(AB.norm()) || !IsDoublePositive(CD.norm())) {
    return LogErr(__func__, 0);
  }

  if (!IsDoublePositive(line1.length) || !IsDoublePositive(line2.length)) {
    line1.length = AB.norm();
    line2.length = CD.norm();
  }

  if (IsDoubleEqual(GetCrossFromTwoVec2d(AB, CD), 0.0)) {
    std::cout
        << "two lines are parallel or overlapping, which must be ruled out\n";
    return LogErr(__func__, 1);
  }

  Eigen::Vector2d AC = line2.pA - line1.pA;

  // P is the intersection of line segment AB and CD
  // P = A + t1 * AB;  P = C + t2 * CD
  // t1 * AB - t2 * CD = C - A
  // t1 * AB.x - t2 * CD.x = AC.x
  // t1 * AB.y - t2 * CD.y = AC.y
  // D = | AB.x  -CD.x |
  //     | AB.y  -CD.y |
  double t1 = 0.0;
  // double t2 = 0.0;
  double determinant = AB.x() * (-CD.y()) - (-CD.x()) * AB.y();
  if (IsDoubleEqual(determinant, 0.0)) {
    return LogErr(__func__, 2);
  } else {
    t1 = (AC.x() * (-CD.y()) - (-CD.x() * AC.y())) / determinant;
    // t2 = (AB.x() * AC.y() - AC.x() * AB.y()) / determinant;
  }

  intersection = line1.pA + t1 * AB;

  return true;
}

const bool CheckPointLiesOnArc(const pnc::geometry_lib::Arc &arc,
                               const Eigen::Vector2d &pC) {
  const auto &pO = arc.circle_info.center;
  const auto v_OA = arc.pA - pO;
  const auto v_OB = arc.pB - pO;
  const auto v_OC = pC - pO;

  const auto norm_v_OA = v_OA.norm();
  const auto norm_v_OB = v_OB.norm();
  const auto norm_v_OC = v_OC.norm();

  const auto cos_AOC = v_OA.dot(v_OC) / (norm_v_OA * norm_v_OC);
  const auto cos_BOC = v_OB.dot(v_OC) / (norm_v_OB * norm_v_OC);
  const auto cos_AOB = v_OA.dot(v_OB) / (norm_v_OA * norm_v_OB);

  return (cos_AOC > cos_AOB && cos_BOC > cos_AOB);
}

const bool GetArcLineIntersection(
    Eigen::Vector2d &intersection, const pnc::geometry_lib::Arc &arc,
    const pnc::geometry_lib::LineSegment &line_seg) {
  const auto AB = line_seg.pB - line_seg.pA;
  const Eigen::Vector2d OA(line_seg.pA.x() - arc.circle_info.center.x(),
                           line_seg.pA.y() - arc.circle_info.center.y());
  // |OA + lambda * AB|^2 = r^2 means that line segment AB intersects at circle
  // OA·OA + 2*OA·AB*lambda + AB·AB*lambda^2 = r^2
  const auto a = AB.dot(AB);
  const auto b = 2 * OA.dot(AB);
  const auto c = OA.dot(OA) - pow(arc.circle_info.radius, 2);

  const auto delta = b * b - 4.0 * a * c;

  if (delta < 0.0) {
    // line does not intersect with a circle
    return false;
  }

  if (mathlib::IsDoubleEqual(delta, 0.0)) {
    // line has one intersection with a circle
    const auto lambda = -b / (2.0 * a);
    // if lambda ∈ [0, 1], line segment AB intersects with a circle
    if (!pnc::mathlib::IsInBound(lambda, 0.0, 1.0)) {
      // the intersection is not on line_segment
      return false;
    }
    // AC = lamdba * AB -> pC - pA = lamdba * (pB - pA)
    // pC = lamdba * pB + (1 - lambda) * pA
    intersection = lambda * line_seg.pB + (1.0 - lambda) * line_seg.pA;
    // determine whether the intersection point is on an arc
    if (CheckPointLiesOnArc(arc, intersection)) {
      return true;
    }
    return false;
  }

  // delta > 0 -> line has two intersection with a circle
  const auto lambda1 = (-b + sqrt(delta)) / (2.0 * a);
  const auto lambda2 = (-b - sqrt(delta)) / (2.0 * a);
  // due to motion limitations, line segment and arc can only have at most one
  // intersection
  if (pnc::mathlib::IsInBound(lambda1, 0.0, 1.0)) {
    intersection = lambda1 * line_seg.pB + (1.0 - lambda1) * line_seg.pA;
    if (CheckPointLiesOnArc(arc, intersection)) {
      return true;
    }
  }
  if (pnc::mathlib::IsInBound(lambda2, 0.0, 1.0)) {
    intersection = lambda2 * line_seg.pB + (1.0 - lambda2) * line_seg.pA;
    if (CheckPointLiesOnArc(arc, intersection)) {
      return true;
    }
  }
  return false;
}

const size_t GetArcLineIntersection(
    std::pair<Eigen::Vector2d, Eigen::Vector2d> &intersections,
    const pnc::geometry_lib::Arc &arc,
    const pnc::geometry_lib::LineSegment &line_seg) {
  const auto AB = line_seg.pB - line_seg.pA;
  const Eigen::Vector2d OA(line_seg.pA.x() - arc.circle_info.center.x(),
                           line_seg.pA.y() - arc.circle_info.center.y());
  // |OA + lambda * AB|^2 = r^2 means that line segment AB intersects at circle
  // OA·OA + 2*OA·AB*lambda + AB·AB*lambda^2 = r^2
  const auto a = AB.dot(AB);
  const auto b = 2 * OA.dot(AB);
  const auto c = OA.dot(OA) - pow(arc.circle_info.radius, 2);

  const auto delta = b * b - 4.0 * a * c;

  if (delta < 0.0) {
    // line does not intersect with a circle
    return 0;
  }

  if (mathlib::IsDoubleEqual(delta, 0.0)) {
    // line has one intersection with a circle
    const auto lambda = -b / (2.0 * a);
    // if lambda ∈ [0, 1], line segment AB intersects with a circle
    if (!pnc::mathlib::IsInBound(lambda, 0.0, 1.0)) {
      // the intersection is not on line_segment
      return 0;
    }
    // AC = lamdba * AB -> pC - pA = lamdba * (pB - pA)
    // pC = lamdba * pB + (1 - lambda) * pA
    intersections.first = lambda * line_seg.pB + (1.0 - lambda) * line_seg.pA;
    // determine whether the intersection point is on an arc
    if (CheckPointLiesOnArc(arc, intersections.first)) {
      return 1;
    }
    return 0;
  }

  size_t number = 0;
  // delta > 0 -> line has two intersection with a circle
  const auto lambda1 = (-b + sqrt(delta)) / (2.0 * a);
  const auto lambda2 = (-b - sqrt(delta)) / (2.0 * a);
  // due to motion limitations, line segment and arc can only have at most one
  // intersection
  if (pnc::mathlib::IsInBound(lambda1, 0.0, 1.0)) {
    intersections.first = lambda1 * line_seg.pB + (1.0 - lambda1) * line_seg.pA;
    if (CheckPointLiesOnArc(arc, intersections.first)) {
      number++;
    }
  }
  if (pnc::mathlib::IsInBound(lambda2, 0.0, 1.0)) {
    if (number == 0) {
      intersections.first =
          lambda2 * line_seg.pB + (1.0 - lambda2) * line_seg.pA;
      if (CheckPointLiesOnArc(arc, intersections.first)) {
        number++;
      }
    } else {
      intersections.second =
          lambda2 * line_seg.pB + (1.0 - lambda2) * line_seg.pA;
      if (CheckPointLiesOnArc(arc, intersections.second)) {
        number++;
      }
    }
  }
  return number;
}

const bool CheckTwoCircleIntersection(const Circle &c1, const Circle &c2) {
  // due to car lat expand, when tangential, it can be regarded as no collision
  const double d = (c1.center - c2.center).norm();
  if (d < c1.radius + c2.radius &&
      d > std::abs(c1.radius - c2.radius)) {  // maybe need some buffer here
    return true;
  } else {
    return false;
  }
}

const std::pair<Eigen::Vector2d, Eigen::Vector2d> GetTwoCircleIntersection(
    const Circle &c1, const Circle &c2) {
  const double d = (c1.center - c2.center).norm();

  // TODO: should consider very small d

  const double r1 = c1.radius;
  const double r2 = c2.radius;
  const double a = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d);
  const double h = sqrt(r1 * r1 - a * a);

  const double x3 = c1.center.x() + a * (c2.center.x() - c1.center.x()) / d;
  const double y3 = c1.center.y() + a * (c2.center.y() - c1.center.y()) / d;

  std::pair<Eigen::Vector2d, Eigen::Vector2d> intersects;
  intersects.first =
      Eigen::Vector2d(x3 + h * (c2.center.y() - c1.center.y()) / d,
                      y3 - h * (c2.center.x() - c1.center.x()) / d);

  intersects.second =
      Eigen::Vector2d(x3 - h * (c2.center.y() - c1.center.y()) / d,
                      y3 + h * (c2.center.x() - c1.center.x()) / d);

  return intersects;
}

const bool GetTwoArcIntersection(Eigen::Vector2d &intersection, const Arc &arc1,
                                 const Arc &arc2) {
  // step 1: consider intersection between two circles rather than arcs
  if (!CheckTwoCircleIntersection(arc1.circle_info, arc2.circle_info)) {
    return false;
  }

  // step 2: if two circles intersect, there must be two intersections.
  // Calculate these two intersections
  const auto intersection_pair =
      GetTwoCircleIntersection(arc1.circle_info, arc2.circle_info);

  // step 3: due to car motion limit, only have at most one
  // intersection
  if (CheckPointLiesOnArc(arc1, intersection_pair.first) &&
      CheckPointLiesOnArc(arc2, intersection_pair.first)) {
    intersection = intersection_pair.first;
    return true;
  } else if (CheckPointLiesOnArc(arc1, intersection_pair.second) &&
             CheckPointLiesOnArc(arc2, intersection_pair.second)) {
    intersection = intersection_pair.second;
    return true;
  } else {
    return false;
  }
}

const double GetCrossFromTwoVec2d(const Eigen::Vector2d &vec0,
                                  const Eigen::Vector2d &vec1) {
  return vec0.x() * vec1.y() - vec0.y() * vec1.x();
}

const size_t CalcCrossPointsOfLineSegAndCircle(
    const LineSegment &line, const Circle &circle,
    std::vector<Eigen::Vector2d> &cross_points) {
  size_t cross_point_nums = 0;
  cross_points.clear();

  const double dist = CalPoint2LineDist(circle.center, line);

  const Eigen::Vector2d line_vec = (line.pB - line.pA).normalized();
  const Eigen::Vector2d n(-line_vec.y(), line_vec.x());

  if (dist > circle.radius + kRadiusCalcTolerance) {
    cross_point_nums = 0;

  } else {
    Eigen::Vector2d foot_pt = circle.center + dist * n;
    if (CalPoint2LineDist(foot_pt, line) > kRadiusCalcTolerance) {
      foot_pt = circle.center - dist * n;
    }

    if (dist > circle.radius - kRadiusCalcTolerance) {
      if ((line.pA - foot_pt).norm() + (line.pB - foot_pt).norm() <=
          (line.pA - line.pB).norm() + 0.001) {
        cross_points.emplace_back(foot_pt);
        cross_point_nums += 1;
      }

    } else {
      const double dist_cross_to_foot =
          std::sqrt(circle.radius * circle.radius - dist * dist);
      const auto tmp = dist_cross_to_foot * line_vec;
      const auto p0 = foot_pt - tmp;
      const auto p1 = foot_pt + tmp;

      if ((line.pA - p0).norm() + (line.pB - p0).norm() <=
          (line.pA - line.pB).norm() + 0.001) {
        cross_points.emplace_back(p0);
        cross_point_nums += 1;
      }
      if ((line.pA - p1).norm() + (line.pB - p1).norm() <=
          (line.pA - line.pB).norm() + 0.001) {
        cross_points.emplace_back(p1);
        cross_point_nums += 1;
      }
    }
  }
  return cross_point_nums;
}

const size_t CalcCrossPointsOfLineAndCircle(
    const LineSegment &line, const Circle &circle,
    std::vector<Eigen::Vector2d> &cross_points) {
  size_t cross_point_nums = 0;
  cross_points.clear();

  const double dist = CalPoint2LineDist(circle.center, line);

  const Eigen::Vector2d line_vec = (line.pB - line.pA).normalized();
  const Eigen::Vector2d n(-line_vec.y(), line_vec.x());

  if (dist > circle.radius + kRadiusCalcTolerance) {
    cross_point_nums = 0;

  } else {
    Eigen::Vector2d foot_pt = circle.center + dist * n;
    if (CalPoint2LineDist(foot_pt, line) > kRadiusCalcTolerance) {
      foot_pt = circle.center - dist * n;
    }

    if (dist > circle.radius - kRadiusCalcTolerance) {
      cross_points.emplace_back(foot_pt);
      cross_point_nums = 1;
    } else {
      const double dist_cross_to_foot =
          std::sqrt(circle.radius * circle.radius - dist * dist);

      cross_points.emplace_back(foot_pt - dist_cross_to_foot * line_vec);
      cross_points.emplace_back(foot_pt + dist_cross_to_foot * line_vec);

      cross_point_nums = 2;
    }
  }
  return cross_point_nums;
}

const size_t CalcTangentPtOfCircleAndLinePassingThroughAGivenPt(
    const Circle &circle, Eigen::Vector2d pt,
    std::vector<Eigen::Vector2d> &tangent_pts) {
  tangent_pts.clear();
  tangent_pts.reserve(2);

  size_t res = 0;
  Eigen::Matrix2d rotm;

  const Eigen::Vector2d pt_to_center_vec = circle.center - pt;
  const double dist_pt_to_center = pt_to_center_vec.norm();

  // pt is in circle
  if (dist_pt_to_center < circle.radius - kRadiusCalcTolerance) {
    res = 0;

  } else if (dist_pt_to_center < circle.radius + kRadiusCalcTolerance) {
    res = 1;

    tangent_pts.emplace_back(pt);

  } else {
    res = 2;

    Eigen::Vector2d pt_to_center_unit = pt_to_center_vec.normalized();

    const double dist_pt_to_tangent_pt = std::sqrt(
        dist_pt_to_center * dist_pt_to_center - circle.radius * circle.radius);

    double sin_theta = circle.radius / dist_pt_to_center;
    double cos_theta = dist_pt_to_tangent_pt / dist_pt_to_center;

    rotm << cos_theta, -sin_theta, sin_theta, cos_theta;

    tangent_pts.emplace_back(rotm * pt_to_center_unit * dist_pt_to_tangent_pt);

    tangent_pts.emplace_back(rotm.transpose() * pt_to_center_unit *
                             dist_pt_to_tangent_pt);
  }

  return res;
}

const size_t CalcCrossPtsOfTwoCircle(const Circle &circle1,
                                     const Circle &circle2,
                                     std::vector<Eigen::Vector2d> &cross_pts) {
  size_t cross_pt_nums = 0;
  cross_pts.clear();
  cross_pts.reserve(2);

  const Eigen::Vector2d o1o2_vec = circle2.center - circle1.center;
  const double dist_o1o2 = o1o2_vec.norm();
  const Eigen::Vector2d o1o2_unit = o1o2_vec.normalized();

  if (dist_o1o2 > circle1.radius + circle2.radius + kRadiusCalcTolerance) {
    // detached
    cross_pt_nums = 0;

  } else if (dist_o1o2 >=
             circle1.radius + circle2.radius - kRadiusCalcTolerance) {
    // outted tangent
    cross_pt_nums = 1;
    const auto pt = circle1.center + o1o2_unit * dist_o1o2 * circle1.radius /
                                         (circle1.radius + circle2.radius);

    cross_pts.emplace_back(pt);

  } else if (dist_o1o2 > std::fabs(circle1.radius - circle2.radius) +
                             kRadiusCalcTolerance) {
    // crossed
    cross_pt_nums = 2;
    // theta  (0,pi)
    const double cos_theta =
        (dist_o1o2 * dist_o1o2 + circle1.radius * circle1.radius -
         circle2.radius * circle2.radius) /
        (2.0 * circle1.radius * circle2.radius);

    const double sin_theta = std::sqrt(1.0 - cos_theta * cos_theta);

    Eigen::Matrix2d rot_m;
    rot_m << cos_theta, -sin_theta, sin_theta, cos_theta;

    const Eigen::Vector2d multiplier = o1o2_unit * circle1.radius;

    cross_pts.emplace_back(rot_m * multiplier + circle1.center);
    cross_pts.emplace_back(rot_m.transpose() * multiplier + circle1.center);

  } else if (dist_o1o2 > std::fabs(circle1.radius - circle2.radius) -
                             kRadiusCalcTolerance) {
    // inner tangent
    cross_pt_nums = 1;
    Eigen::Vector2d o1_to_corss_pt_unit =
        circle1.radius > circle2.radius ? -o1o2_unit : o1o2_unit;

    cross_pts.emplace_back(circle1.center +
                           circle1.radius * o1_to_corss_pt_unit);

  } else {
    // inner contained
    cross_pt_nums = 0;
  }

  return cross_pt_nums;
}

const bool CalTangentCirclesOfTwoLines(
    const LineSegment &line1, const LineSegment &line2, const double radius,
    std::vector<Eigen::Vector2d> &centers,
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> &tangent_pts) {
  centers.clear();
  tangent_pts.clear();

  centers.reserve(4);
  tangent_pts.reserve(4);
  Eigen::Vector2d line_corss_pt = Eigen::Vector2d::Zero();

  if (CalCrossPtOfTwoLines(line1, line2, line_corss_pt)) {
    Eigen::Vector2d line_1t_vec = (line1.pB - line1.pA).normalized();
    Eigen::Vector2d line_2t_vec = (line2.pB - line2.pA).normalized();

    for (size_t i = 0; i < 2; i++) {
      line_2t_vec *= -1.0;
      const Eigen::Vector2d n21_vec = line_1t_vec - line_2t_vec;

      Eigen::Vector2d middle_vec(-n21_vec.y(), n21_vec.x());
      middle_vec.normalize();

      const double sin_theta =
          std::fabs(GetCrossFromTwoVec2d(middle_vec, line_1t_vec));

      Eigen::Vector2d cross_to_center_vec = radius / sin_theta * middle_vec;
      centers.emplace_back(line_corss_pt + cross_to_center_vec);
      centers.emplace_back(line_corss_pt - cross_to_center_vec);

      const double cos_theta = middle_vec.dot(line_1t_vec);
      const double dist_cross_pt = cross_to_center_vec.norm() * cos_theta;

      tangent_pts.emplace_back(
          std::make_pair(line_corss_pt + line_1t_vec * dist_cross_pt,
                         line_corss_pt + line_2t_vec * dist_cross_pt));

      tangent_pts.emplace_back(
          std::make_pair(line_corss_pt - line_1t_vec * dist_cross_pt,
                         line_corss_pt - line_2t_vec * dist_cross_pt));
    }
    return true;
  } else {
    return false;
  }
}

const bool CalCrossPtOfTwoLines(const LineSegment &line1,
                                const LineSegment &line2,
                                Eigen::Vector2d &cross_pt) {
  const Eigen::Vector2d line1_t_vec = (line1.pB - line1.pA).normalized();
  const Eigen::Vector2d line2_t_vec = (line2.pB - line2.pA).normalized();

  const Eigen::Vector2d line1_n_vec(-line1_t_vec.y(), line1_t_vec.x());
  const Eigen::Vector2d line2_n_vec(-line2_t_vec.y(), line2_t_vec.x());

  // ax + by = c
  const double a1 = line1_n_vec.x();
  const double b1 = line1_n_vec.y();
  const double c1 = a1 * line1.pA.x() + b1 * line1.pA.y();

  const double a2 = line2_n_vec.x();
  const double b2 = line2_n_vec.y();
  const double c2 = a2 * line2.pA.x() + b2 * line2.pA.y();

  const double D = a1 * b2 - a2 * b1;
  const double Dx = c1 * b2 - c2 * b1;
  const double Dy = a1 * c2 - a2 * c1;

  if (D == 0.0) {
    return false;
  } else {
    const double multiplier = 1.0 / D;
    cross_pt << Dx * multiplier, Dy * multiplier;
    return true;
  }
}

const bool CalProjFromSplineByBisection(
    const double &s_start, const double &s_end, double &s_proj,
    const Eigen::Vector2d &current_pos, const pnc::mathlib::spline &x_s_spline,
    const pnc::mathlib::spline &y_s_spline) {
  static const size_t max_iter = 20;
  static const double min_err = 1e-3;

  if (s_end - s_start < 0.05) {
    return false;
  }

  double s_left = s_start + min_err;
  double s_right = s_end - min_err;
  double s_mid = 0.0;

  const auto &x0 = current_pos.x();
  const auto &y0 = current_pos.y();

  for (size_t i = 0; i < max_iter; ++i) {
    s_mid = (s_left + s_right) * 0.5;

    if (fabs(s_right - s_left) < min_err) {
      break;
    }

    double g_mid = (x_s_spline(s_mid) - x0) * x_s_spline.deriv(1, s_mid) +
                   (y_s_spline(s_mid) - y0) * y_s_spline.deriv(1, s_mid);

    double g_left = (x_s_spline(s_left) - x0) * x_s_spline.deriv(1, s_left) +
                    (y_s_spline(s_left) - y0) * y_s_spline.deriv(1, s_left);

    double g_right = (x_s_spline(s_right) - x0) * x_s_spline.deriv(1, s_right) +
                     (y_s_spline(s_right) - y0) * y_s_spline.deriv(1, s_right);

    if (g_left * g_mid <= 0.0) {
      s_right = s_mid;
    } else if (g_right * g_mid <= 0.0) {
      s_left = s_mid;
    } else {
      if (g_left > 0.0) {
        s_mid = s_left;
      } else {
        s_mid = s_right;
      }
      break;
    }
  }
  s_proj = s_mid;
  return true;
}

const bool CalExtendedPointByTwoPoints(const Eigen::Vector2d &start_point,
                                       const Eigen::Vector2d &end_point,
                                       Eigen::Vector2d &extended_point,
                                       const double extended_distance) {
  // calculate a extended point using vector
  const Eigen::Vector2d line = end_point - start_point;
  const double distance = line.norm();

  if (pnc::mathlib::IsDoubleEqual(distance, 0.0)) {
    return false;
  }

  Eigen::Vector2d unit_line = line / distance;
  Eigen::Vector2d new_line = line + extended_distance * unit_line;

  extended_point = new_line + start_point;

  return true;
}

const bool IsPointOnLeftSideOfLineSeg(const Eigen::Vector2d &point,
                                      const LineSegment &line_seg) {
  const Eigen::Vector2d line_seg_vec = line_seg.pB - line_seg.pA;
  const Eigen::Vector2d line_to_pt_vec = point - line_seg.pA;
  const double cross = GetCrossFromTwoVec2d(line_seg_vec, line_to_pt_vec);
  return cross > 0.0;
}

const bool OneStepArcTargetLineByGear(
    Arc &arc, const Eigen::Vector2d &start_pos, const double start_heading,
    const bool is_advanve, const pnc::geometry_lib::LineSegment &target_line) {
  Arc tmp_arc;

  const bool res =
      OneStepArcTargetLine(tmp_arc, start_pos, start_heading, target_line);

  if (!res) {
    return false;
  }
  if (IsArcAdvance(tmp_arc) != is_advanve) {
    std::cout << "gear different, cal failed!" << std::endl;
    return false;
  }
  arc = tmp_arc;
  return true;
}

const bool OneStepArcTargetLine(
    Arc &arc, const Eigen::Vector2d &start_pos, const double start_heading,
    const pnc::geometry_lib::LineSegment &target_line) {
  bool success = false;
  bool is_left = false;
  for (size_t i = 0; i < 2; i++) {
    Arc tmp_arc;
    const bool res = OneStepArcTargetLine(tmp_arc, start_pos, start_heading,
                                          is_left, target_line);
    // std::cout << "No." << i << " calc res: " << res << std::endl;
    // std::cout << "tmp_arc.headingB =" << tmp_arc.headingB * kRad2Deg <<
    // std::endl;
    if (res &&
        std::fabs(NormalizeAngle(tmp_arc.headingB - target_line.heading)) <=
            kSameHeadingEps) {
      success = true;
      arc = tmp_arc;
      break;
    }
    is_left = !is_left;
  }
  return success;
}

const bool OneStepArcTargetLine(
    Arc &arc, const Eigen::Vector2d &start_pos, const double start_heading,
    bool is_left, const pnc::geometry_lib::LineSegment &target_line) {
  const double dist_start_to_target_line =
      CalPoint2LineDist(start_pos, target_line);

  if (mathlib::IsDoubleEqual(dist_start_to_target_line, 0.0)) {
    std::cout << "point is on the target line, cal one arc failed!"
              << std::endl;
    return false;
  }

  const Eigen::Vector2d v_start_heading(std::cos(start_heading),
                                        std::sin(start_heading));

  const Eigen::Vector2d v_target_heading =
      (target_line.pB - target_line.pA).normalized();

  const double sin_heading_diff =
      GetCrossFromTwoVec2d(v_start_heading, v_target_heading);

  if (mathlib::IsDoubleEqual(sin_heading_diff, 0.0)) {
    std::cout << "heading of start pos and target line are parallel, cal one "
                 "arc failed!"
              << std::endl;
    return false;
  }

  // cal start n vec
  Eigen::Vector2d v_start_heading_n(-v_start_heading.y(), v_start_heading.x());
  if (!is_left) {
    v_start_heading_n *= -1.0;
  }

  Eigen::Vector2d v_target_n_to_start = Eigen::Vector2d::Zero();

  const bool cal_res = CalNormalVecOfLineTowardsGivenPt(v_target_n_to_start,
                                                        target_line, start_pos);
  if (!cal_res) {
    std::cout << "cal normal vec of target line failed!" << std::endl;
    return false;
  }

  const Eigen::Vector2d v_target_to_start = start_pos - target_line.pA;
  const Eigen::Vector2d v_n_start_to_target =
      v_start_heading_n - v_target_n_to_start;

  // notice: denominator != 0, parallel condition is excluded.
  const double radius =
      GetCrossFromTwoVec2d(v_target_to_start, v_target_heading) /
      GetCrossFromTwoVec2d(v_target_heading, v_n_start_to_target);
  if (radius < 0.0) {
    std::cout << "cal radius failed" << std::endl;
    return false;
  }

  arc.is_ignored = false;
  arc.pA = start_pos;
  arc.headingA = start_heading;

  arc.circle_info.radius = radius;
  arc.circle_info.center = start_pos + v_start_heading_n * radius;
  arc.pB = arc.circle_info.center - radius * v_target_n_to_start;

  const auto theta = geometry_lib::GetAngleFromTwoVec(
      arc.pA - arc.circle_info.center, arc.pB - arc.circle_info.center);

  arc.headingB = theta + arc.headingA;
  arc.is_anti_clockwise = (theta > 0.0);
  arc.length = std::fabs(theta) * arc.circle_info.radius;

  return true;
}

const bool CalNormalVecOfLineTowardsGivenPt(Eigen::Vector2d &v_line_n,
                                            const LineSegment &line,
                                            const Eigen::Vector2d &pt) {
  const double dist_pt_to_line = CalPoint2LineDist(pt, line);

  if (mathlib::IsDoubleEqual(dist_pt_to_line, 0.0)) {
    std::cout << "point is on the target line, cal line normal vector failed!"
              << std::endl;
    return false;
  }

  const Eigen::Vector2d v_line_to_pt = pt - line.pA;
  const Eigen::Vector2d v_line_heading = (line.pB - line.pA).normalized();
  Eigen::Vector2d v_line_heading_n(-v_line_heading.y(), v_line_heading.x());

  if (v_line_heading_n.dot(v_line_to_pt) < 0.0) {
    v_line_heading_n *= -1.0;
  }

  v_line_n = v_line_heading_n;
  return true;
}

void OneStepArcTargetHeading(Arc &arc, const Eigen::Vector2d &start_pos,
                             const double start_heading,
                             const double target_heading, const double radius,
                             const bool is_advance) {
  const Eigen::Vector2d v_start_heading(std::cos(start_heading),
                                        std::sin(start_heading));

  Eigen::Vector2d v_start_heading_n(-v_start_heading.y(), v_start_heading.x());

  const double head_diff = NormalizeAngle(target_heading - start_heading);

  const bool is_left =
      (head_diff > 0.0 && is_advance) || (head_diff < 0.0 && !is_advance);

  if (!is_left) {
    v_start_heading_n *= -1.0;
  }

  arc.circle_info.center = start_pos + v_start_heading_n * radius;
  arc.circle_info.radius = radius;
  arc.pA = start_pos;
  arc.headingA = start_heading;
  arc.headingB = target_heading;

  const double theta_diff = NormalizeAngle(target_heading - start_heading);

  const Eigen::Vector2d v_oa = start_pos - arc.circle_info.center;
  const Eigen::Matrix2d rot_m = GetRotm2dFromTheta(theta_diff);
  arc.pB = rot_m * v_oa + arc.circle_info.center;
  arc.length = std::fabs(theta_diff) * radius;
  arc.is_anti_clockwise = (theta_diff > 0.0);
  arc.is_ignored = false;

  // std::cout << "start_pos: " << arc.pA.transpose() << std::endl;
  // std::cout << "start_heading: " << arc.headingA * kRad2Deg << std::endl;
  // std::cout << "end_pos: " << arc.pB.transpose() << std::endl;
  // std::cout << "end_heading: " << arc.headingB * kRad2Deg << std::endl;
  // std::cout << "arc.length: " << arc.length << std::endl;
  // std::cout << "is_anti_clockwise: " << arc.is_anti_clockwise << std::endl;
}

const bool OneStepParallelShift(
    std::pair<Arc, Arc> &arc_pair, const Eigen::Vector2d &start_pos,
    const double start_heading,
    const pnc::geometry_lib::LineSegment &target_line, const double radius,
    const bool is_advance) {
  const Eigen::Vector2d v_start_heading =
      Eigen::Vector2d(std::cos(start_heading), std::sin(start_heading));

  Eigen::Vector2d v_start_n(-v_start_heading.y(), v_start_heading.x());
  const Eigen::Vector2d start_to_target_vec = target_line.pA - start_pos;

  if (start_to_target_vec.dot(v_start_n) < 0.0) {
    v_start_n *= -1.0;
  }

  Arc first_arc;
  first_arc.circle_info.center = start_pos + v_start_n * radius;
  first_arc.circle_info.radius = radius;

  // std::cout << "first_arc.circle_info.center = " <<
  // first_arc.circle_info.center
  //           << std::endl;

  first_arc.pA = start_pos;
  first_arc.headingA = start_heading;

  const double lat_dist = CalPoint2LineDist(start_pos, target_line);

  // std::cout << "lat_dist = " << lat_dist << std::endl;
  // std::cout << "start_pos = " << start_pos.transpose() << std::endl;
  // std::cout << "target_line_x = " << target_line.pA.transpose() << std::endl;
  // std::cout << "target_line_y = " << target_line.pB.transpose() << std::endl;

  const double cos_theta = 1.0 - lat_dist / (2.0 * radius);

  const double sin_theta =
      std::sqrt(1.0 - std::min(cos_theta * cos_theta, 1.0));

  const double theta = std::atan2(sin_theta, cos_theta);
  const double start_to_tan_lon_dist = radius * sin_theta;
  // std::cout << "start_to_tan_lon_dist = " << start_to_tan_lon_dist <<
  // std::endl;

  const double advance_sgn = (is_advance ? 1.0 : -1.0);
  first_arc.pB = first_arc.pA + lat_dist * 0.5 * v_start_n +
                 advance_sgn * start_to_tan_lon_dist * v_start_heading;

  const bool is_turn_right = IsPointOnLeftSideOfLineSeg(start_pos, target_line);

  if ((is_turn_right && is_advance) || (!is_turn_right && !is_advance)) {
    first_arc.is_anti_clockwise = false;
    first_arc.headingB = first_arc.headingA - theta;
  } else {
    first_arc.is_anti_clockwise = true;
    first_arc.headingB = first_arc.headingA + theta;
  }

  first_arc.length = theta * radius;
  first_arc.is_ignored = false;

  Arc second_arc;
  second_arc.pA = first_arc.pB;
  second_arc.headingA = first_arc.headingB;
  second_arc.headingB = target_line.heading;

  second_arc.pB = start_pos + lat_dist * v_start_n +
                  2.0 * advance_sgn * start_to_tan_lon_dist * v_start_heading;

  second_arc.circle_info.center = second_arc.pB - v_start_n * radius;
  second_arc.circle_info.radius = radius;
  second_arc.is_anti_clockwise = (!first_arc.is_anti_clockwise);
  second_arc.length = first_arc.length;
  second_arc.is_ignored = false;

  arc_pair.first = first_arc;
  arc_pair.second = second_arc;

  return true;
}

const bool IsArcAdvance(const pnc::geometry_lib::Arc &arc) {
  const Eigen::Vector2d v_ab = arc.pB - arc.pA;

  const Eigen::Vector2d v_heading_a(std::cos(arc.headingA),
                                    std::sin(arc.headingA));

  return (v_ab.dot(v_heading_a) > 0.0);
}

const bool IsArcTurnLeft(const pnc::geometry_lib::Arc &arc) {
  const Eigen::Vector2d v_heading_a(std::cos(arc.headingA),
                                    std::sin(arc.headingA));

  const Eigen::Vector2d v_oa = arc.pA - arc.circle_info.center;

  return (pnc::geometry_lib::GetCrossFromTwoVec2d(v_oa, v_heading_a) > 0.0);
}

const bool IsLineAdvance(const LineSegment &line_seg) {
  const Eigen::Vector2d v_line = line_seg.pB - line_seg.pA;
  const Eigen::Vector2d v_line_heading = GenHeadingVec(line_seg.heading);
  return (v_line.dot(v_line_heading) > 0.0);
}

const bool SamplePointSetInLineSeg(std::vector<Eigen::Vector2d> &point_set,
                                   const LineSegment &line, const double ds) {
  if (!IsDoublePositive(line.length) || !IsDoublePositive(ds)) {
    return LogErr(__func__, 0);
  }

  point_set.clear();
  point_set.reserve(100);

  Eigen::Vector2d pn;
  // get first point
  pn = line.pA;
  point_set.emplace_back(pn);
  const Eigen::Vector2d unit_line_vec = (line.pB - line.pA).normalized();
  double s = ds;
  while (s < line.length) {
    pn = line.pA + s * unit_line_vec;
    point_set.emplace_back(pn);
    s += ds;
  }
  // get last point
  pn = line.pB;
  point_set.emplace_back(pn);
  return true;
}

const bool SamplePointSetInArc(std::vector<Eigen::Vector2d> &point_set,
                               const Arc &arc, const double ds) {
  if (!IsDoublePositive(arc.length) || !IsDoublePositive(ds) ||
      !IsDoublePositive(arc.circle_info.radius)) {
    return LogErr(__func__, 0);
  }

  point_set.clear();
  point_set.reserve(50);

  Eigen::Vector2d pn;
  // get first point
  pn = arc.pA;
  point_set.emplace_back(pn);

  const auto &pO = arc.circle_info.center;
  Eigen::Vector2d v_n = arc.pA - pO;
  if (!IsDoublePositive(v_n.norm())) {
    return LogErr(__func__, 1);
  }
  const double dheading =
      ds / arc.circle_info.radius * (arc.is_anti_clockwise ? 1.0 : -1.0);

  const auto rot_m = pnc::geometry_lib::GetRotm2dFromTheta(dheading);
  double s = ds;
  while (s < arc.length) {
    v_n = rot_m * v_n;
    pn = pO + v_n;
    point_set.emplace_back(pn);
    s += ds;
  }
  // get last point
  pn = arc.pB;
  point_set.emplace_back(pn);

  return true;
}

const bool SamplePointSetInPathSeg(std::vector<Eigen::Vector2d> &point_set,
                                   const PathSegment &path_seg,
                                   const double ds) {
  if (path_seg.seg_type == SEG_TYPE_LINE) {
    return SamplePointSetInLineSeg(point_set, path_seg.line_seg, ds);
  } else if (path_seg.seg_type == SEG_TYPE_ARC) {
    return SamplePointSetInArc(point_set, path_seg.arc_seg, ds);
  } else {
    return false;
  }
}

const bool SamplePointSetInLineSeg(std::vector<PathPoint> &point_set,
                                   const LineSegment &line, const double ds) {
  if (!IsDoublePositive(line.length) || !IsDoublePositive(ds)) {
    return LogErr(__func__, 0);
  }

  point_set.clear();
  point_set.reserve(50);

  PathPoint pn;
  // get first point
  pn.Set(line.pA, NormalizeAngle(line.heading));
  point_set.emplace_back(pn);
  const Eigen::Vector2d unit_line_vec = (line.pB - line.pA).normalized();
  double s = ds;
  while (s < line.length) {
    pn.pos = line.pA + s * unit_line_vec;
    pn.heading = line.heading;
    point_set.emplace_back(pn);
    s += ds;
  }
  // get last point
  pn.Set(line.pB, NormalizeAngle(line.heading));
  point_set.emplace_back(pn);
  return true;
}

const bool SamplePointSetInArc(std::vector<PathPoint> &point_set,
                               const Arc &arc, const double ds) {
  if (!IsDoublePositive(arc.length) || !IsDoublePositive(ds) ||
      !IsDoublePositive(arc.circle_info.radius)) {
    return LogErr(__func__, 0);
  }
  point_set.clear();
  point_set.reserve(50);

  PathPoint pn;
  // get first point
  pn.Set(arc.pA, NormalizeAngle(arc.headingA));
  point_set.emplace_back(pn);

  const auto &pO = arc.circle_info.center;
  Eigen::Vector2d v_n = arc.pA - pO;
  if (!IsDoublePositive(v_n.norm())) {
    return LogErr(__func__, 1);
  }
  double heading = arc.headingA;
  const double dheading =
      ds / arc.circle_info.radius * (arc.is_anti_clockwise ? 1.0 : -1.0);

  const auto rot_m = pnc::geometry_lib::GetRotm2dFromTheta(dheading);
  double s = ds;
  while (s < arc.length) {
    v_n = rot_m * v_n;
    pn.pos = pO + v_n;
    heading += dheading;
    pn.heading = NormalizeAngle(heading);
    point_set.emplace_back(pn);
    s += ds;
  }
  // get last point
  pn.Set(arc.pB, NormalizeAngle(arc.headingB));
  point_set.emplace_back(pn);

  return true;
}

const bool SamplePointSetInPathSeg(std::vector<PathPoint> &point_set,
                                   const PathSegment &path_seg,
                                   const double ds) {
  if (path_seg.seg_type == SEG_TYPE_LINE) {
    return SamplePointSetInLineSeg(point_set, path_seg.line_seg, ds);
  } else if (path_seg.seg_type == SEG_TYPE_ARC) {
    return SamplePointSetInArc(point_set, path_seg.arc_seg, ds);
  } else {
    return false;
  }
}

const bool IsPointInPolygon(const std::vector<Eigen::Vector2d> &polygon,
                            const Eigen::Vector2d &point) {
  // int crossProductSign = 0;

  // for (size_t i = 0; i < polygon.size(); i++) {
  //   Eigen::Vector2d vertex1 = polygon[i];
  //   Eigen::Vector2d vertex2 = polygon[(i + 1) % polygon.size()];

  //   const double crossProduct =
  //       (vertex2.x() - vertex1.x()) * (point.y() - vertex1.y()) -
  //       (point.x() - vertex1.x()) * (vertex2.y() - vertex1.y());

  //   if (std::fabs(crossProduct) < 1.0e-4) {
  //     return true;
  //   } else if (crossProductSign == 0) {
  //     crossProductSign = (crossProduct > 0) ? 1 : -1;
  //   } else if (crossProductSign != ((crossProduct > 0.0) ? 1 : -1)) {
  //     return false;
  //   }
  // }

  // return true;

  int numVertices = polygon.size();
  if (numVertices < 3) {
    return false;
  }
  bool inside = false;

  // traverse every edge of the polygon
  for (int i = 0, j = numVertices - 1; i < numVertices; j = i++) {
    // Check the intersection of rays and polygon edges
    if (((polygon[i](1) > point(1)) != (polygon[j](1) > point(1))) &&
        (point(0) < (polygon[j](0) - polygon[i](0)) *
                            (point(1) - polygon[i](1)) /
                            (polygon[j](1) - polygon[i](1)) +
                        polygon[i](0))) {
      inside = !inside;
    }
  }

  return inside;
}

const bool CalOneArcWithLine(Arc &arc, LineSegment &line, double r_err) {
  // arc and line are tangent
  // the start point and center info of the arc are known, cal the
  // end point info of the arc, the end point is tangent
  // known: arc: pA, headingA, pO, radius   line:pA, pB, heading, length
  // unknown: arc:pB, headingB, length, is_anti_clockwise
  // check input
  using namespace mathlib;
  if (IsDoubleEqual((line.pB - line.pA).norm(), 0.0) ||
      IsDoubleEqual((arc.pB - arc.pA).norm(), 0.0) ||
      IsDoubleEqual((arc.circle_info.center - arc.pA).norm(), 0.0)) {
    return LogErr(__func__, 0);
  }

  if (IsDoubleEqual(line.length, 0.0) ||
      IsDoubleEqual(arc.circle_info.radius, 0.0)) {
    line.length = (line.pB - line.pA).norm();
    arc.circle_info.radius = (arc.circle_info.center - arc.pA).norm();
  }

  // line(A->B) arc(D->E, O)
  r_err = (r_err > 0.0) ? r_err : -r_err;
  // arc must be tangent with the line
  const auto dist = CalPoint2LineDist(arc.circle_info.center, line);
  if (!(dist <= arc.circle_info.radius + r_err &&
        dist >= arc.circle_info.radius - r_err)) {
    return LogErr(__func__, 1);
  }

  // the direction from center to line
  Eigen::Vector2d line_norm_vec;
  if (!CalLineUnitNormVecByPos(arc.circle_info.center, line, line_norm_vec)) {
    return LogErr(__func__, 2);
  };
  // note C is the tangent, also the arc end point
  // AC = AO + OC --> OC = line_norm_vec * radius
  const auto AO = arc.circle_info.center - line.pA;
  arc.pB = AO + line_norm_vec * arc.circle_info.radius + line.pA;
  if (!CompleteArcInfo(arc)) {
    return LogErr(__func__, 3);
  };
  if (!mathlib::IsDoubleEqual(arc.headingB, line.heading)) {
    return LogErr(__func__, 3);
  }
  return true;
}

const bool CalTwoArcWithLine(Arc &arc1, Arc &arc2, LineSegment &line,
                             bool is_shifted) {
  // arc1 and line are intersected
  // the start point and center info of the arc1 are known, cal the
  // end point info of the arc1, the end point is tangent
  // arc2 is tangent with arc1, also line
  // known: arc1: pA, headingA, pO, radius  line:pA, pB, heading, length
  // unknown: arc1:pB, headingB, length, is_anti_clockwise
  // unknown:arc2:pA,headingA,pO,radius,pB,headingB,length,is_anti_clockwise
  using namespace mathlib;
  if (IsDoubleEqual((line.pB - line.pA).norm(), 0.0) ||
      IsDoubleEqual((arc1.pB - arc1.pA).norm(), 0.0) ||
      IsDoubleEqual((arc1.circle_info.center - arc1.pA).norm(), 0.0)) {
    return LogErr(__func__, 0);
  }

  if (IsDoubleEqual(line.length, 0.0) ||
      IsDoubleEqual(arc1.circle_info.radius, 0.0)) {
    line.length = (line.pB - line.pA).norm();
    arc1.circle_info.radius = (arc1.circle_info.center - arc1.pA).norm();
  }

  // arc1 must be intersected with the line
  const auto dist = CalPoint2LineDist(arc1.circle_info.center, line);
  if (dist > arc1.circle_info.radius - 1e-6) {
    return LogErr(__func__, 1);
  }

  // line(A->B) arc1(D1->E1, O1)  arc2(E2->F2 O2)
  // the direction from O1 to line
  Eigen::Vector2d line_norm_vec;
  if (!CalLineUnitNormVecByPos(arc1.circle_info.center, line, line_norm_vec)) {
    return LogErr(__func__, 2);
  }

  // O1 and O2 are on different sides of the line
  // the O2 is on arc2_center_line
  LineSegment arc2_center_line(
      line.pA + line_norm_vec * arc1.circle_info.radius,
      line.pB + line_norm_vec * arc1.circle_info.radius);

  // the O2 is on tmp_circle, arc1 is tangent with arc2, than the dist of O1
  // and O2 is 2*r
  Circle tmp_circle;
  tmp_circle.center = arc1.circle_info.center;
  tmp_circle.radius = arc1.circle_info.radius * 2.0;

  std::vector<Eigen::Vector2d> arc2_centers;
  // must 2 cross points
  if (CalcCrossPointsOfLineAndCircle(arc2_center_line, tmp_circle,
                                     arc2_centers) < 2) {
    return LogErr(__func__, 3);
  }

  const auto O1D1 = arc1.pA - arc1.circle_info.center;
  const auto O1O2_1 = arc2_centers.front() - arc1.circle_info.center;
  const auto O1O2_2 = arc2_centers.back() - arc1.circle_info.center;

  const auto angle_O1O2_1 = std::fabs(GetAngleFromTwoVec(O1D1, O1O2_1));
  const auto angle_O1O2_2 = std::fabs(GetAngleFromTwoVec(O1D1, O1O2_2));

  if ((angle_O1O2_1 < angle_O1O2_2 && is_shifted) ||
      (angle_O1O2_1 > angle_O1O2_2 && !is_shifted)) {
    arc2.circle_info.center = arc2_centers.front();
  } else {
    arc2.circle_info.center = arc2_centers.back();
  }

  // the mid of two center is tangent, also pB
  arc1.pB = (arc1.circle_info.center + arc2.circle_info.center) * 0.5;
  if (!CompleteArcInfo(arc1)) {
    return LogErr(__func__, 4);
  }

  // give arc2 info
  arc2.circle_info.radius = arc1.circle_info.radius;
  arc2.pA = arc1.pB;
  arc2.headingA = arc1.headingB;

  // note C2 is the tangent, also the arc2 end point
  // AC2 = AO2 + O2C2 --> O2C2 = -line_norm_vec * radius
  arc2.pB = arc2.circle_info.center - line.pA -
            line_norm_vec * arc2.circle_info.radius + line.pA;

  if (!CompleteArcInfo(arc2)) {
    return LogErr(__func__, 5);
  }
  if (!mathlib::IsDoubleEqual(arc2.headingB, line.heading)) {
    std::cout << "arc2.headingB = " << arc2.headingB * kRad2Deg
              << "  line.heading = " << line.heading * kRad2Deg << std::endl;
    return LogErr(__func__, 6);
  }
  return true;
}

const bool CalTwoSameGearArcWithLine(Arc &arc1, Arc &arc2, LineSegment &line,
                                     const uint8_t gear) {
  // arc1 and line are intersected
  // the start point and center info of the arc1 are known, cal the
  // end point info of the arc1, the end point is tangent
  // arc2 is tangent with arc1, also line
  // known: arc1: pA, headingA, pO, radius  line:pA, pB, heading, length, gear
  // unknown: arc1:pB, headingB, length, is_anti_clockwise
  // unknown:arc2:pA,headingA,pO,radius,pB,headingB,length,is_anti_clockwise
  using namespace mathlib;
  if (IsDoubleEqual((line.pB - line.pA).norm(), 0.0) ||
      IsDoubleEqual((arc1.pB - arc1.pA).norm(), 0.0) ||
      IsDoubleEqual((arc1.circle_info.center - arc1.pA).norm(), 0.0)) {
    return LogErr(__func__, 0);
  }

  if (IsDoubleEqual(line.length, 0.0) ||
      IsDoubleEqual(arc1.circle_info.radius, 0.0)) {
    line.length = (line.pB - line.pA).norm();
    arc1.circle_info.radius = (arc1.circle_info.center - arc1.pA).norm();
  }

  // arc1 must be intersected with the line
  const auto dist = CalPoint2LineDist(arc1.circle_info.center, line);
  if (dist > arc1.circle_info.radius - 1e-6) {
    // std::cout << "dist = " << dist << std::endl;
    // std::cout << "center = " << arc1.circle_info.center.transpose()
    //           << std::endl;
    // std::cout << "line pa = " << line.pA.transpose()
    //           << ", pb = " << line.pB.transpose() << std::endl;

    return LogErr(__func__, 1);
  }

  if (gear != SEG_GEAR_DRIVE && gear != SEG_GEAR_REVERSE) {
    return LogErr(__func__, 6);
  }

  // line(A->B) arc1(D1->E1, O1)  arc2(E2->F2 O2)
  // the direction from ego to line
  Eigen::Vector2d line_norm_vec;
  if (!CalLineUnitNormVecByPos(arc1.pA, line, line_norm_vec)) {
    return LogErr(__func__, 2);
  }

  // O2 is on ego car sides of the line
  // the O2 is on arc2_center_line
  LineSegment arc2_center_line(
      line.pA - line_norm_vec * arc1.circle_info.radius,
      line.pB - line_norm_vec * arc1.circle_info.radius);

  // the O2 is on tmp_circle, arc1 is tangent with arc2, than the dist of O1
  // and O2 is 2*r
  Circle tmp_circle;
  tmp_circle.center = arc1.circle_info.center;
  tmp_circle.radius = arc1.circle_info.radius * 2.0;

  std::vector<Eigen::Vector2d> arc2_centers;
  // must 2 cross points
  if (CalcCrossPointsOfLineAndCircle(arc2_center_line, tmp_circle,
                                     arc2_centers) < 2) {
    return LogErr(__func__, 3);
  }

  if (gear == SEG_GEAR_DRIVE) {
    arc2.circle_info.center = arc2_centers.back();
  } else if (gear == SEG_GEAR_REVERSE) {
    arc2.circle_info.center = arc2_centers.front();
  }

  // the mid of two center is tangent, also pB
  arc1.pB = (arc1.circle_info.center + arc2.circle_info.center) * 0.5;
  if (!CompleteArcInfo(arc1)) {
    return LogErr(__func__, 4);
  }

  // give arc2 info
  arc2.circle_info.radius = arc1.circle_info.radius;
  arc2.pA = arc1.pB;
  arc2.headingA = arc1.headingB;

  // note C2 is the tangent, also the arc2 end point
  // AC2 = AO2 + O2C2 --> O2C2 = -line_norm_vec * radius
  arc2.pB = arc2.circle_info.center + line_norm_vec * arc2.circle_info.radius;

  if (!CompleteArcInfo(arc2)) {
    return LogErr(__func__, 5);
  }
  if (!mathlib::IsDoubleEqual(arc2.headingB, line.heading)) {
    return LogErr(__func__, 3);
  }
  return true;
}

const bool IsPoseOnLine(const PathPoint &pose, LineSegment &line,
                        const double lat_err, const double heading_err) {
  using namespace mathlib;
  if (lat_err < 0.0 || heading_err < 0.0 ||
      IsDoubleEqual((line.pB - line.pA).norm(), 0.0)) {
    return false;
  }

  if (IsDoubleEqual(line.length, 0.0)) {
    line.length = (line.pB - line.pA).norm();
  }

  const auto dist = CalPoint2LineDist(pose.pos, line);
  // std::cout << "pos = " << pose.pos.transpose()
  //           << "  heading = " << pose.heading << "  dist = " << dist
  //           << "   line.heading = " << line.heading << std::endl;
  if (dist < lat_err &&
      std::fabs(NormalizeAngle(pose.heading - line.heading)) < heading_err) {
    return true;
  }
  // std::cout << "pos = " << pose.pos.transpose()
  //           << "  heading = " << pose.heading << "  dist = " << dist
  //           << "   line.heading = " << line.heading << std::endl;
  return false;
}

const bool CalCommonTangentCircleOfTwoLine(
    LineSegment &line1, LineSegment &line2, const double &radius,
    std::vector<Eigen::Vector2d> &centers,
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> &tangent_ptss) {
  // the circle is tangent with line1, also with line2
  // known:line1:pA, pB, heading  line2:pA, pB, heading
  // unknown:circle:center, tangent
  using namespace mathlib;
  if (IsDoubleEqual((line1.pB - line1.pA).norm(), 0.0) ||
      IsDoubleEqual((line2.pB - line2.pA).norm(), 0.0)) {
    return LogErr(__func__, 0);
  }

  if (IsDoubleEqual(line1.length, 0.0) || IsDoubleEqual(line2.length, 0.0)) {
    line1.length = (line1.pB - line1.pA).norm();
    line2.length = (line2.pB - line2.pA).norm();
  }

  Eigen::Vector2d intersection = Eigen::Vector2d::Zero();
  if (GetIntersectionFromTwoLine(intersection, line1, line2)) {
    const auto unit_line1_vec = (line1.pB - line1.pA).normalized();
    const auto unit_line2_vec = (line2.pB - line2.pA).normalized();
    // every line have two direction, so two lines have four combination
    std::vector<Eigen::Vector2d> combination{
        Eigen::Vector2d(1.0, 1.0), Eigen::Vector2d(1.0, -1.0),
        Eigen::Vector2d(-1.0, 1.0), Eigen::Vector2d(-1.0, -1.0)};
    centers.clear();
    centers.reserve(combination.size());
    tangent_ptss.clear();
    tangent_ptss.reserve(combination.size());
    for (size_t i = 0; i < combination.size(); i++) {
      auto actual_unit_line1_vec = unit_line1_vec * combination[i].x();
      auto actual_unit_line2_vec = unit_line2_vec * combination[i].y();

      const auto unit_angular_bisector_vec =
          (actual_unit_line1_vec + actual_unit_line2_vec).normalized();

      // the angle between two lines is 2*theta, sin_theta is no impossible 0
      const auto sin_theta = GetCrossFromTwoVec2d(actual_unit_line1_vec,
                                                  unit_angular_bisector_vec);

      const double dist_intersection_center = radius / sin_theta;

      // A is intersection, O is center
      const auto AO = dist_intersection_center * unit_angular_bisector_vec;
      const auto center = AO + intersection;

      centers.emplace_back(center);

      std::pair<Eigen::Vector2d, Eigen::Vector2d> tang_pts;

      const double cos_theta =
          actual_unit_line1_vec.dot(unit_angular_bisector_vec);

      const double dist_intersection_tangpt =
          dist_intersection_center * cos_theta;

      // P1, P2 is tang pt
      const auto AP1 = dist_intersection_tangpt * actual_unit_line1_vec;
      const auto AP2 = dist_intersection_tangpt * actual_unit_line2_vec;
      tang_pts.first = AP1 + intersection;
      tang_pts.second = AP2 + intersection;
      tangent_ptss.emplace_back(tang_pts);
    }
  } else {
    std::cout << "two lines have no intersection, no common tangent circle\n";
    return LogErr(__func__, 1);
  }

  return true;
}

const bool MinimumBoundingBox(
    std::vector<Eigen::Vector2d> &target_boundingbox,
    const std::vector<Eigen::Vector2d> &original_vertices) {
  Eigen::MatrixXd vertices_matrix(2, original_vertices.size());

  for (size_t i = 0; i < original_vertices.size(); i++) {
    vertices_matrix.col(i) = original_vertices[i];
  }
  // compute center
  const auto center = vertices_matrix.rowwise().mean();
  // compute covariance
  const auto covaraince_matrix =
      (vertices_matrix.colwise() - center) *
      ((vertices_matrix.colwise() - center).transpose()) /
      (vertices_matrix.cols() - 1);

  // value and vector
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigen_solver(
      covaraince_matrix);
  const auto eigen_values = eigen_solver.eigenvalues();
  const Eigen::Matrix2d eigen_vectors = eigen_solver.eigenvectors();

  // compute main direction(length, x)
  // std::cout << "eigen_values:" << eigen_values(0) << "\n";
  // std::cout << "eigen_values:" << eigen_values(1) << "\n";
  // std::cout << "eigen_vectors:" << eigen_vectors.col(0).transpose() << "\n";
  // std::cout << "eigen_vectors:" << eigen_vectors.col(1).transpose() << "\n";
  if (std::fabs(eigen_values(0)) < 1e-4) {
    return false;
  }
  const auto direction = eigen_vectors.col(0);
  // compute norm direction
  const Eigen::Vector2d direction_t(-direction.y(), direction.x());

  // length
  const double half_length =
      0.25 * ((original_vertices[0] - original_vertices[2]).norm() +
              (original_vertices[1] - original_vertices[3]).norm());
  const double half_width =
      0.25 * ((original_vertices[1] - original_vertices[0]).norm() +
              (original_vertices[3] - original_vertices[2]).norm());

  // std::cout << "direction:" << direction.transpose() << "\n";
  // std::cout << "half_length:" << half_length << "\n";
  // std::cout << "half_width:" << half_width << "\n";

  target_boundingbox.emplace_back(center + direction * half_length -
                                  direction_t * half_width);

  target_boundingbox.emplace_back(center + direction * half_length +
                                  direction_t * half_width);

  target_boundingbox.emplace_back(center - direction * half_length -
                                  direction_t * half_width);

  target_boundingbox.emplace_back(center - direction * half_length +
                                  direction_t * half_width);
  return true;
}

const Eigen::Vector2d GetUnitTangVecByHeading(const double &heading) {
  return Eigen::Vector2d(std::cos(heading), std::sin(heading));
}

const LineSegment BuildLineSegByPose(const Eigen::Vector2d &current_pos,
                                     const double &current_heading) {
  const Eigen::Vector2d next_pos =
      current_pos + GetUnitTangVecByHeading(current_heading);

  return LineSegment(current_pos, next_pos, current_heading);
}

const bool CheckTwoVecCollinear(const Eigen::Vector2d &v1,
                                const Eigen::Vector2d &v2) {
  const auto unit_v1 = v1.normalized();
  const auto unit_v2 = v2.normalized();
  const double sin_theta = GetCrossFromTwoVec2d(unit_v1, unit_v2);
  if (std::fabs(sin_theta) < 0.015) {
    return true;
  }
  std::cout << "sin_theta = " << sin_theta << std::endl;
  return false;
}

const bool CheckTwoVecVertical(const Eigen::Vector2d &v1,
                               const Eigen::Vector2d &v2) {
  const auto unit_v1 = v1.normalized();
  const auto unit_v2 = v2.normalized();

  const double cos_theta = unit_v1.dot(unit_v2);

  if (std::fabs(cos_theta) < 0.015) {
    return true;
  }
  std::cout << "cos_theta = " << cos_theta << std::endl;
  return false;
}

const bool CheckTwoVecSameOrOppositeDirection(const Eigen::Vector2d &v1,
                                              const Eigen::Vector2d &v2) {
  const auto unit_v1 = v1.normalized();
  const auto unit_v2 = v2.normalized();
  const double cos_theta = unit_v1.dot(unit_v2);
  if (cos_theta > 1 - 0.015) {
    return true;
  }
  return false;
}

const bool CompleteArcInfo(Arc &arc) {
  using namespace mathlib;
  // arc(A-B) known:pA, headingA, pB, pO(turn center)
  // unknown: headingB, radius, length, is_anti_clockwise
  const auto OA = arc.pA - arc.circle_info.center;
  const auto OB = arc.pB - arc.circle_info.center;
  const auto OA_norm = OA.norm();
  const auto OB_norm = OB.norm();
  // const auto AB_norm = (arc.pB - arc.pA).norm();
  if (IsDoubleEqual(OA_norm, 0.0) || IsDoubleEqual(OB_norm, 0.0) ||
      !IsDoubleEqual(OA_norm, OB_norm)) {
    std::cout << "OA_norm = " << OA_norm << "  OB_norm = " << OB_norm
              << std::endl;
    return LogErr(__func__, 0);
  }

  arc.circle_info.radius = OA_norm;

  const auto rot_angle = GetAngleFromTwoVec(OA, OB);
  if (rot_angle > 0.0) {
    arc.is_anti_clockwise = true;
  } else if (rot_angle < 0.0) {
    arc.is_anti_clockwise = false;
  } else {
    // return LogErr(__func__, 1);
  }

  arc.headingB = NormalizeAngle(arc.headingA + rot_angle);

  arc.length = std::fabs(rot_angle) * OA_norm;

  return true;
}

const bool CompleteArcInfo(Arc &arc, const double rot_angle) {
  using namespace mathlib;
  // arc(A->B  O:turn center)
  // known: pA, headingA, pO, rot_angle
  // unknown: pB, headingB, radius, length, is_anti_clockwise
  const auto OA = arc.pA - arc.circle_info.center;
  const auto OA_norm = OA.norm();
  if (IsDoubleEqual(OA_norm, 0.0)) {
    return LogErr(__func__, 0);
  }

  arc.circle_info.radius = OA_norm;

  const double tmp_rot_angle = NormalizeAngle(rot_angle);
  const auto rot_m = GetRotm2dFromTheta(tmp_rot_angle);
  const auto OB = rot_m * OA;
  arc.pB = arc.circle_info.center + OB;
  return CompleteArcInfo(arc);
}

const bool CompleteArcInfo(Arc &arc, const uint8_t arc_steer) {
  using namespace mathlib;
  // arc(A->B  O:turn center)
  // known: pA, headingA, headingB, radius
  // unknown: pB, pO, length, is_anti_clockwise
  if (IsDoubleEqual(arc.headingA, arc.headingB) ||
      IsDoubleEqual(arc.circle_info.radius, 0.0) ||
      (arc_steer != SEG_STEER_LEFT && arc_steer != SEG_STEER_RIGHT)) {
    return LogErr(__func__, 0);
  }
  const auto A_tang_vec = GetUnitTangVecByHeading(arc.headingA);
  Eigen::Vector2d A_norm_vec;
  if (arc_steer == SEG_STEER_LEFT) {
    A_norm_vec << -A_tang_vec.y(), A_tang_vec.x();
  } else {
    A_norm_vec << A_tang_vec.y(), -A_tang_vec.x();
  }
  arc.circle_info.center = arc.pA + A_norm_vec * arc.circle_info.radius;

  const double rot_angle = NormalizeAngle(arc.headingB - arc.headingA);

  return CompleteArcInfo(arc, rot_angle);
}

const bool CompleteArcInfo(Arc &arc, const double length,
                           const bool is_anti_clockwise) {
  // arc(A->B  O:turn center)
  // known: pA, headingA, radius, pO, length, is_anti_clockwise
  // unknown: pB, headingB
  double rot_angle = length / arc.circle_info.radius;
  rot_angle = is_anti_clockwise ? rot_angle : -rot_angle;
  arc.length = length;
  arc.is_anti_clockwise = is_anti_clockwise;
  return CompleteArcInfo(arc, rot_angle);
}

const bool CompleteArcInfo(Arc &arc, const double length,
                           const bool is_anti_clockwise,
                           const bool save_start_pt) {
  arc.length = length;
  double rot_angle = length / arc.circle_info.radius;
  if (save_start_pt) {
    rot_angle = is_anti_clockwise ? rot_angle : -rot_angle;
    const Eigen::Vector2d OA = arc.pA - arc.circle_info.center;
    const double tmp_rot_angle = NormalizeAngle(rot_angle);
    const auto rot_m = GetRotm2dFromTheta(tmp_rot_angle);
    const auto OB = rot_m * OA;
    arc.pB = arc.circle_info.center + OB;
    arc.headingB = NormalizeAngle(arc.headingA + rot_angle);
  } else {
    rot_angle = is_anti_clockwise ? -rot_angle : rot_angle;
    const Eigen::Vector2d OB = arc.pB - arc.circle_info.center;
    const double tmp_rot_angle = NormalizeAngle(rot_angle);
    const auto rot_m = GetRotm2dFromTheta(tmp_rot_angle);
    const auto OA = rot_m * OB;
    arc.pA = arc.circle_info.center + OA;
    arc.headingA = NormalizeAngle(arc.headingB + rot_angle);
  }
  return true;
}

const bool CompleteLineInfo(LineSegment &line, const double length) {
  // known:pA, pB, heading
  const auto AB = line.pB - line.pA;
  if (mathlib::IsDoubleEqual(AB.norm(), 0.0)) {
    return LogErr(__func__, 0);
  }
  const auto unit_line_vec = AB.normalized();
  line.pB = length * unit_line_vec + line.pA;
  line.length = length;
  return true;
}

const bool CompleteLineInfo(LineSegment &line, const double length,
                            const double heading) {
  // known:pA, heading, length
  if (!IsDoublePositive(length)) {
    return LogErr(__func__, 0);
  }
  const auto unit_line_vec = GetUnitTangVecByHeading(heading);
  line.pB = length * unit_line_vec + line.pA;
  line.length = length;
  return true;
}

const bool CompleteLineInfo(LineSegment &line, const double length,
                            const bool save_start_pt) {
  line.length = length;
  const Eigen::Vector2d unit_vec = (line.pB - line.pA).normalized();
  if (save_start_pt) {
    line.pB = line.pA + length * unit_vec;
  } else {
    line.pA = line.pB - length * unit_vec;
  }
  return true;
}

const bool CompletePathSeg(PathSegment &path_seg, const double length,
                           const bool save_start_pt) {
  if (length < 0.0168) {
    return true;
  }
  if (path_seg.seg_type == SEG_TYPE_LINE) {
    CompleteLineInfo(path_seg.line_seg, length, save_start_pt);
  } else if (path_seg.seg_type == SEG_TYPE_ARC) {
    CompleteArcInfo(path_seg.arc_seg, length,
                    path_seg.arc_seg.is_anti_clockwise, save_start_pt);
  } else {
    return false;
  }
  return true;
}

const uint8_t CalArcGear(const Arc &arc) {
  const Eigen::Vector2d v_ab = arc.pB - arc.pA;
  if (mathlib::IsDoubleEqual(v_ab.norm(), 0.0)) {
    std::cout << "arc input err, no gear\n";
    return SEG_GEAR_INVALID;
  }

  const Eigen::Vector2d v_heading_a(std::cos(arc.headingA),
                                    std::sin(arc.headingA));

  return (v_ab.dot(v_heading_a) > 0.0) ? SEG_GEAR_DRIVE : SEG_GEAR_REVERSE;
}

const uint8_t CalArcSteer(const Arc &arc) {
  const Eigen::Vector2d v_oa = arc.pA - arc.circle_info.center;
  if (mathlib::IsDoubleEqual(v_oa.norm(), 0.0)) {
    std::cout << "arc input err, no steer\n";
    return SEG_STEER_INVALID;
  }
  const Eigen::Vector2d v_heading_a(std::cos(arc.headingA),
                                    std::sin(arc.headingA));

  return (GetCrossFromTwoVec2d(v_oa, v_heading_a) > 0.0) ? SEG_STEER_LEFT
                                                         : SEG_STEER_RIGHT;
}

const uint8_t CalLineSegGear(const LineSegment &line_seg) {
  const auto &v1 = line_seg.pB - line_seg.pA;
  if (mathlib::IsDoubleEqual(v1.norm(), 0.0)) {
    std::cout << "line input err, no gear\n";
    return SEG_GEAR_INVALID;
  }
  const auto &v2 = GetUnitTangVecByHeading(line_seg.heading);

  if (!CheckTwoVecCollinear(v1, v2)) {
    return SEG_GEAR_INVALID;
  }
  if (CheckTwoVecSameOrOppositeDirection(v1, v2)) {
    return SEG_GEAR_DRIVE;
  }
  return SEG_GEAR_REVERSE;
}

const uint8_t ReverseGear(const uint8_t gear) {
  uint8_t desired_gear = SEG_GEAR_INVALID;
  if (gear == SEG_GEAR_REVERSE) {
    desired_gear = SEG_GEAR_DRIVE;
  } else if (gear == SEG_GEAR_DRIVE) {
    desired_gear = SEG_GEAR_REVERSE;
  }
  return desired_gear;
}

const uint8_t ReverseSteer(const uint8_t steer) {
  uint8_t desired_steer = SEG_STEER_INVALID;
  if (steer == SEG_STEER_RIGHT) {
    desired_steer = SEG_STEER_LEFT;
  } else if (steer == SEG_STEER_LEFT) {
    desired_steer = SEG_STEER_RIGHT;
  }
  return desired_steer;
}

const bool CalcArcDirection(bool &is_anti_clockwise, const uint8_t gear,
                            const uint8_t steer) {
  if (gear != pnc::geometry_lib::SEG_GEAR_DRIVE &&
      gear != pnc::geometry_lib::SEG_GEAR_REVERSE) {
    std::cout << "arc fault gear type!" << std::endl;
    return false;
  }

  if (steer != pnc::geometry_lib::SEG_STEER_RIGHT &&
      steer != pnc::geometry_lib::SEG_STEER_LEFT) {
    std::cout << "arc fault steer type!" << std::endl;
    return false;
  }

  is_anti_clockwise = (((steer == pnc::geometry_lib::SEG_STEER_LEFT) &&
                        (gear == pnc::geometry_lib::SEG_GEAR_DRIVE)) ||
                       ((steer == pnc::geometry_lib::SEG_STEER_RIGHT) &&
                        (gear == pnc::geometry_lib::SEG_GEAR_REVERSE)));

  return true;
}

const bool IsValidGear(const uint8_t gear) {
  return (gear == SEG_GEAR_DRIVE || gear == SEG_GEAR_REVERSE);
}

const bool IsValidLineSteer(const uint8_t steer) {
  return (steer == SEG_STEER_STRAIGHT);
}

const bool IsValidArcSteer(const uint8_t steer) {
  return (steer == SEG_STEER_LEFT || steer == SEG_STEER_RIGHT);
}

const bool ReversePathSegInfo(PathSegment &path_seg) {
  if (path_seg.seg_type == SEG_TYPE_LINE) {
    return ReverseLineSegInfo(path_seg);
  } else if (path_seg.seg_type == SEG_TYPE_ARC) {
    return ReverseArcSegInfo(path_seg);
  } else {
    return false;
  }
}

const bool ReverseArcSegInfo(PathSegment &path_seg) {
  if (path_seg.seg_type != SEG_TYPE_ARC) {
    std::cout << "input is not arc!" << std::endl;
    return false;
  }

  if (path_seg.seg_steer != SEG_STEER_LEFT &&
      path_seg.seg_steer != SEG_STEER_RIGHT) {
    std::cout << "arc input steer error!" << std::endl;
    return false;
  }
  path_seg.seg_gear = ReverseGear(path_seg.seg_gear);

  auto &arc_seg = path_seg.arc_seg;
  arc_seg.pA.swap(arc_seg.pB);
  std::swap(arc_seg.headingA, arc_seg.headingB);
  arc_seg.is_anti_clockwise = !arc_seg.is_anti_clockwise;
  return true;
}

const bool ReverseLineSegInfo(PathSegment &path_seg) {
  if (path_seg.seg_type != SEG_TYPE_LINE) {
    std::cout << "input is not line!" << std::endl;
    return false;
  }

  if (path_seg.seg_steer != SEG_STEER_STRAIGHT) {
    std::cout << "line input steer error!" << std::endl;
    return false;
  }
  path_seg.seg_gear = ReverseGear(path_seg.seg_gear);
  path_seg.line_seg.pA.swap(path_seg.line_seg.pB);
  return true;
}

const bool CalLineUnitNormVecByPos(const Eigen::Vector2d &pos,
                                   const LineSegment &line,
                                   Eigen::Vector2d &line_norm_vec) {
  using namespace mathlib;
  if (IsDoubleEqual((line.pB - line.pA).norm(), 0.0)) {
    return LogErr(__func__, 0);
  }
  const double cross = GetCrossFromTwoVec2d(line.pB - line.pA, pos - line.pA);
  const auto line_tang_vec = (line.pB - line.pA).normalized();
  // line_norm_vec is from pos to line
  if (cross > 1e-6) {
    // pos is on left side of the line
    line_norm_vec << line_tang_vec.y(), -line_tang_vec.x();
  } else if (cross < -1e-6) {
    // pos is on right side of the line
    line_norm_vec << -line_tang_vec.y(), line_tang_vec.x();
  } else {
    std::cout << "pos = " << pos.transpose()
              << "  line.pA = " << line.pA.transpose()
              << "  line.pB = " << line.pB.transpose()
              << "  line.heading = " << line.heading * kRad2Deg << std::endl;
    return LogErr(__func__, 1);
  }
  return true;
}

const bool CalOneArcWithLineAndGear(Arc &arc, const LineSegment &line,
                                    const uint8_t current_seg_gear) {
  // the arc which is tangent with the line
  // arc(C->D), the D is tangent, headingD must be equal with line
  // the C and the O shoule be one side of the line
  // known: arc: pA, headingA, line:pA, pB, heading
  // unknown: arc:pB, headingB, pO, length, radius, is_anti_clockwise
  // the line_norm_vec should from pose to line
  Eigen::Vector2d line_norm_vec;
  if (!CalLineUnitNormVecByPos(arc.pA, line, line_norm_vec) ||
      mathlib::IsDoubleEqual(arc.headingA, line.heading)) {
    return LogErr(__func__, 0);
  }

  // sure arc steer
  const double heading_diff = NormalizeAngle(arc.headingA - line.heading);
  uint8_t arc_steer = 0;
  if ((heading_diff < 0 && current_seg_gear == SEG_GEAR_DRIVE) ||
      (heading_diff > 0 && current_seg_gear == SEG_GEAR_REVERSE)) {
    arc_steer = SEG_STEER_LEFT;
  } else {
    arc_steer = SEG_STEER_RIGHT;
  }

  // pose(C, heading) arc(C->D)
  const auto pose_tang_vec = GetUnitTangVecByHeading(arc.headingA);
  Eigen::Vector2d pose_norm_vec;
  if (arc_steer == SEG_STEER_RIGHT) {
    pose_norm_vec << pose_tang_vec.y(), -pose_tang_vec.x();
  } else if (arc_steer == SEG_STEER_LEFT) {
    pose_norm_vec << -pose_tang_vec.y(), pose_tang_vec.x();
  }

  // pose turn center (O), first assume turn radius is r, then assume the
  // tangent of arc and line is D, and D shoule be on the line,
  // so AD × AB = 0.0, and the r can be calculated
  const auto AC = arc.pA - line.pA;
  const auto line_tang_vec = (line.pB - line.pA).normalized();
  arc.circle_info.radius =
      -GetCrossFromTwoVec2d(AC, line_tang_vec) /
      GetCrossFromTwoVec2d(pose_norm_vec + line_norm_vec, line_tang_vec);

  // cal arc other info
  arc.circle_info.center = arc.pA + arc.circle_info.radius * pose_norm_vec;
  arc.pB = arc.circle_info.center + arc.circle_info.radius * line_norm_vec;
  if (!CompleteArcInfo(arc)) {
    return LogErr(__func__, 1);
  }
  if (!mathlib::IsDoubleEqual(arc.headingB, line.heading)) {
    std::cout << "arc.headingB = " << arc.headingB * kRad2Deg
              << "  line.heading = " << line.heading << std::endl;
    return LogErr(__func__, 2);
  }
  return true;
}

const bool LogErr(const std::string &func_name, uint8_t index,
                  const uint8_t type) {
  std::string err_type(" input ");
  if (type == 0) {
    err_type = " input ";
  } else if (type == 1) {
    err_type = " fail ";
  }

  std::cout << func_name + err_type + " err " + std::to_string(index)
            << std::endl;

  return false;
}

const bool CalOneArcWithTargetHeading(Arc &arc, const uint8_t &current_seg_gear,
                                      const double &target_heading) {
  using namespace mathlib;
  // check input
  if ((current_seg_gear != SEG_GEAR_DRIVE &&
       current_seg_gear != SEG_GEAR_REVERSE) ||
      !IsDoublePositive(arc.circle_info.radius) ||
      IsDoubleEqual(arc.headingA, target_heading)) {
    return LogErr(__func__, 0);
  }

  // arc (A->B) known: pA, headingA, radius, gear
  // unknown: pB, headingB, pO(turn center), length, is_anti_clockwise

  // sure arc steer
  arc.headingB = target_heading;
  const double heading_diff = NormalizeAngle(arc.headingA - arc.headingB);
  uint8_t arc_steer = 0;
  if ((heading_diff < 0 && current_seg_gear == SEG_GEAR_DRIVE) ||
      (heading_diff > 0 && current_seg_gear == SEG_GEAR_REVERSE)) {
    arc_steer = SEG_STEER_LEFT;
  } else {
    arc_steer = SEG_STEER_RIGHT;
  }

  if (!CompleteArcInfo(arc, arc_steer)) {
    return LogErr(__func__, 1);
  }

  if (!mathlib::IsDoubleEqual(arc.headingB, target_heading)) {
    std::cout << "arc.headingB =" << arc.headingB * kRad2Deg << " , "
              << "target_heading =" << target_heading * kRad2Deg << std::endl;
    return LogErr(__func__, 2);
  }

  return true;
}

const bool CalTwoArcWithSameHeading(Arc &arc1, Arc &arc2,
                                    const uint8_t seg_gear) {
  // known the pA pos and headingA of arc1
  // known the headingB of arc2
  // known the radius of arc1 and arc2, and must be equal
  // the headingA must be equal to headingB
  // unknown: arc1: pB, headingB, pO, length, is_anti_clockwise
  // unknown: arc2: pA, headingA, pB, pO, length, is_anti_clockwise
  // the arc2.pB init value is dummy, just indicating which line arc2.pB is on
  // check input
  using namespace mathlib;
  if (arc1.circle_info.radius <= 1e-8 || arc2.circle_info.radius <= 1e-8 ||
      !IsDoubleEqual(arc1.circle_info.radius, arc2.circle_info.radius) ||
      !IsDoubleEqual(arc1.headingA, arc2.headingB) ||
      (seg_gear != SEG_GEAR_DRIVE && seg_gear != SEG_GEAR_REVERSE)) {
    return LogErr(__func__, 0);
  }

  const auto line2 = BuildLineSegByPose(arc2.pB, arc2.headingB);
  Eigen::Vector2d line1_norm_vec;
  if (!CalLineUnitNormVecByPos(arc1.pA, line2, line1_norm_vec)) {
    return LogErr(__func__, 1);
  }
  // line_norm_vec is from arc.pA to line2
  arc1.circle_info.center = arc1.pA + arc1.circle_info.radius * line1_norm_vec;

  const double lat_dist = CalPoint2LineDist(arc1.pA, line2);
  // if radius is too small, arc1 and arc2 cannot be tangent
  if (arc1.circle_info.radius * 4.0 < lat_dist + 1e-8 || lat_dist < 1e-5) {
    std::cout << "radius = " << arc1.circle_info.radius
              << "  lat_dist = " << lat_dist << std::endl;
    return LogErr(__func__, 2);
  }
  // the C is the tangent of arc1 and arc2
  // the dist from C to line2 is the half of lat_dist because of the same
  // radius of arc1 and arc2 the theta is rot_angle of arc1
  const double cos_theta =
      (arc1.circle_info.radius - lat_dist * 0.5) / arc1.circle_info.radius;

  const double sin_theta = std::sqrt(1 - std::min(cos_theta * cos_theta, 1.0));

  // lon_dist is the proj length of AC in line2
  const double lon_dist = arc1.circle_info.radius * sin_theta;

  auto line1_tang_vec = GetUnitTangVecByHeading(arc1.headingA);
  auto tmp_tang_vec = line1_tang_vec;
  if (seg_gear == SEG_GEAR_REVERSE) {
    tmp_tang_vec = -line1_tang_vec;
  }

  arc1.pB = arc1.pA + lat_dist * 0.5 * line1_norm_vec + lon_dist * tmp_tang_vec;

  if (!CompleteArcInfo(arc1)) {
    return LogErr(__func__, 3);
  }

  arc2.pA = arc1.pB;
  arc2.headingA = arc1.headingB;
  arc2.pB = arc2.pA + lat_dist * 0.5 * line1_norm_vec + lon_dist * tmp_tang_vec;
  arc2.circle_info.center = arc2.pB - line1_norm_vec * arc2.circle_info.radius;
  arc2.is_anti_clockwise = !arc1.is_anti_clockwise;
  arc2.length = arc1.length;

  return true;
}

const bool IsDoublePositive(const double x) { return x > 1e-8; }

const double CalPoint2LineSegDist(const Eigen::Vector2d &pO,
                                  const LineSegment &line) {
  double dist;
  Eigen::Vector2d unit_AB = (line.pB - line.pA).normalized();
  Eigen::Vector2d unit_BA = (line.pA - line.pB).normalized();

  Eigen::Vector2d unit_AO = (pO - line.pA).normalized();
  Eigen::Vector2d unit_BO = (pO - line.pB).normalized();

  double cos_OAB = unit_AB.dot(unit_AO);
  double cos_OBA = unit_BA.dot(unit_BO);
  if (cos_OAB > 1e-6 && cos_OAB < 1 - 1e-6 && cos_OBA > 1e-6 &&
      cos_OBA < 1 - 1e-6) {
    dist = CalPoint2LineDist(pO, line);
  } else if (mathlib::IsDoubleEqual(cos_OAB, -1.0)) {
    // pO is on line seg
    dist = 0.0;
  } else {
    dist = std::min((line.pB - pO).norm(), (line.pA - pO).norm());
  }
  return dist;
}

const bool CheckTwoPoseIsSame(const PathPoint &pose1, const PathPoint &pose2,
                              const double pos_err, const double heading_err) {
  if ((pose1.pos - pose2.pos).norm() < pos_err &&
      std::fabs(pnc::geometry_lib::NormalizeAngle(
          pose1.heading - pose2.heading)) < heading_err) {
    return true;
  }
  return false;
}

std::vector<double> Linspace(const double start, const double stop,
                             const double ds) {
  std::vector<double> res;
  res.clear();
  if (mathlib::IsDoubleEqual(start, stop)) {
    return res;
  }

  const double diff = stop - start;
  const double sgn = diff > 0.0 ? 1.0 : -1.0;
  const double deta_s = ds * sgn;

  double value = start;
  while (value * sgn < stop * sgn) {
    res.emplace_back(value);
    value += deta_s;
  }
  res.emplace_back(stop);
  return res;
}

std::vector<Eigen::Vector2d> LinSpace(const Eigen::Vector2d &start_pos,
                                      const Eigen::Vector2d &stop_pos,
                                      const double ds) {
  std::vector<Eigen::Vector2d> res;
  res.clear();

  const Eigen::Vector2d pos_diff = stop_pos - start_pos;
  if (mathlib::IsDoubleEqual(pos_diff.norm(), 0.0)) {
    return res;
  }

  const Eigen::Vector2d v_deta = ds * pos_diff.normalized();

  auto pos = start_pos;
  while ((stop_pos - pos).dot(v_deta) > 0.0) {
    res.emplace_back(pos);
    pos += v_deta;
  }
  res.emplace_back(stop_pos);
  return res;
}

void PrintPose(const pnc::geometry_lib::PathPoint &pose) {
  std::cout << " " << pose.pos.x() << ", " << pose.pos.y()
            << ", heading(deg): " << pose.heading * kRad2Deg << std::endl;
}

void PrintPose(const std::string &str,
               const pnc::geometry_lib::PathPoint &pose) {
  std::cout << str << "= " << pose.pos.x() << ", " << pose.pos.y()
            << ", heading(deg): " << pose.heading * kRad2Deg << std::endl;
}

void PrintPose(const Eigen::Vector2d &pos, const double heading) {
  std::cout << " " << pos.x() << ", " << pos.y()
            << ", heading(deg): " << heading * kRad2Deg << std::endl;
}

void PrintPose(const std::string &str, const Eigen::Vector2d &pos,
               const double heading) {
  std::cout << str << "= " << pos.x() << ", " << pos.y()
            << ", heading(deg): " << heading * kRad2Deg << std::endl;
}

void PrintSegmentInfo(const pnc::geometry_lib::PathSegment &seg) {
  std::cout << "----" << std::endl;
  std::cout << "seg_gear: " << static_cast<int>(seg.seg_gear) << std::endl;

  std::cout << "seg_steer: " << static_cast<int>(seg.seg_steer) << std::endl;
  std::cout << "seg_type: " << static_cast<int>(seg.seg_type) << std::endl;
  std::cout << "length: " << seg.Getlength() << std::endl;

  if (seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
    std::cout << "start_pos: " << seg.GetLineSeg().pA.transpose() << std::endl;
    std::cout << "start_heading deg: " << seg.GetLineSeg().heading * kRad2Deg
              << std::endl;
    std::cout << "end_pos: " << seg.GetLineSeg().pB.transpose() << std::endl;
    std::cout << "end_heading deg: " << seg.GetLineSeg().heading * kRad2Deg
              << std::endl;
  } else {
    std::cout << "start_pos: " << seg.GetArcSeg().pA.transpose() << std::endl;
    std::cout << "start_heading deg: " << seg.GetArcSeg().headingA * kRad2Deg
              << std::endl;
    std::cout << "end_pos: " << seg.GetArcSeg().pB.transpose() << std::endl;
    std::cout << "end_heading deg: " << seg.GetArcSeg().headingB * kRad2Deg
              << std::endl;
  }
}

void PrintSegmentsVecInfo(
    const std::vector<pnc::geometry_lib::PathSegment> &path_segment_vec) {
  std::cout << "-------------- OutputSegmentsInfo --------------" << std::endl;
  for (size_t i = 0; i < path_segment_vec.size(); i++) {
    const auto &current_seg = path_segment_vec[i];

    if (current_seg.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
      const auto &line_seg = current_seg.line_seg;

      std::cout << "Segment [" << i << "] "
                << " LINE_SEGMENT "
                << " length= " << line_seg.length << std::endl;

      std::cout << "seg_gear: " << static_cast<int>(current_seg.seg_gear)
                << std::endl;

      std::cout << "seg_steer: " << static_cast<int>(current_seg.seg_steer)
                << std::endl;

      std::cout << "start_pos: " << line_seg.pA.transpose() << std::endl;
      std::cout << "start_heading deg: " << line_seg.heading * kRad2Deg
                << std::endl;
      std::cout << "end_pos: " << line_seg.pB.transpose() << std::endl;
    } else {
      const auto &arc_seg = current_seg.arc_seg;

      std::cout << "Segment [" << i << "] "
                << "ARC_SEGMENT "
                << "length= " << arc_seg.length << std::endl;

      std::cout << "seg_gear: " << static_cast<int>(current_seg.seg_gear)
                << std::endl;

      std::cout << "seg_steer: " << static_cast<int>(current_seg.seg_steer)
                << std::endl;

      std::cout << "start_pos: " << arc_seg.pA.transpose() << std::endl;
      std::cout << "start_heading deg: " << arc_seg.headingA * kRad2Deg
                << std::endl;
      std::cout << "end_pos: " << arc_seg.pB.transpose() << std::endl;
      std::cout << "end_heading deg: " << arc_seg.headingB * kRad2Deg << std::endl;
      std::cout << "center: " << arc_seg.circle_info.center.transpose()
                << "radius = " << arc_seg.circle_info.radius << std::endl;
    }
  }
}

}  // namespace geometry_lib
}  // namespace pnc