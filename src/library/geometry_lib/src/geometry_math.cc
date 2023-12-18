#include "geometry_math.h"

#include <cmath>
#include <cstddef>
#include <iostream>
#include <utility>
#include <vector>

#include "Eigen/Geometry"
#include "math_lib.h"

namespace pnc {
namespace geometry_lib {

static const double kRadiusCalcTolerance = 1e-6;
static const double kSameHeadingEps = 0.5 / 57.3;

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

const bool GetLineLineIntersection(
    Eigen::Vector2d &intersection,
    const pnc::geometry_lib::LineSegment &line_seg1,
    const pnc::geometry_lib::LineSegment &line_seg2) {
  Eigen::Vector2d AB = line_seg1.pB - line_seg1.pA;
  Eigen::Vector2d CD = line_seg2.pB - line_seg2.pA;

  const double cross_AB_CD = AB.x() * CD.y() - AB.y() * CD.x();

  if (mathlib::IsDoubleEqual(cross_AB_CD, 0.0)) {
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

  if (pnc::mathlib::IsInBound(t1, 0.0, 1.0) &&
      pnc::mathlib::IsInBound(t2, 0.0, 1.0)) {
    intersection = line_seg1.pA + t1 * AB;
    return true;
  } else {
    return false;
  }

  return false;
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
    return false;
  }
  return false;
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
    s_mid = (s_left + s_right) / 2.0;

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
    // std::cout << "tmp_arc.headingB =" << tmp_arc.headingB * 57.3 <<
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
  // std::cout << "start_heading: " << arc.headingA * 57.3 << std::endl;
  // std::cout << "end_pos: " << arc.pB.transpose() << std::endl;
  // std::cout << "end_heading: " << arc.headingB * 57.3 << std::endl;
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

  std::cout << "first_arc.circle_info.center = " << first_arc.circle_info.center
            << std::endl;

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
  std::cout << "start_to_tan_lon_dist = " << start_to_tan_lon_dist << std::endl;

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
                                   const LineSegment &line, const double &ds) {
  if (mathlib::IsDoubleEqual(line.length, 0.0)) {
    std::cout << "the length of line can not be 0\n";
    return false;
  }
  if (ds <= 0.0) {
    std::cout << "sample ds can not be smaller than 0\n";
    return false;
  }

  point_set.clear();
  point_set.reserve(50);

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
                               const Arc &arc, const double &ds) {
  if (mathlib::IsDoubleEqual(arc.length, 0.0) ||
      mathlib::IsDoubleEqual(arc.circle_info.radius, 0.0)) {
    std::cout << "the length or radius of arc can not be 0\n";
    return false;
  }
  if (ds <= 0.0) {
    std::cout << "sample ds can not be smaller than 0\n";
    return false;
  }
  point_set.clear();
  point_set.reserve(50);

  Eigen::Vector2d pn;
  // get first point
  pn = arc.pA;
  point_set.emplace_back(pn);

  const auto &pO = arc.circle_info.center;
  Eigen::Vector2d v_n = arc.pA - pO;
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

const bool SamplePointSetInLineSeg(std::vector<PathPoint> &point_set,
                                   const LineSegment &line, const double &ds) {
  if (mathlib::IsDoubleEqual(line.length, 0.0)) {
    std::cout << "the length of line can not be 0\n";
    return false;
  }
  if (ds <= 0.0) {
    std::cout << "sample ds can not be smaller than 0\n";
    return false;
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
                               const Arc &arc, const double &ds) {
  if (mathlib::IsDoubleEqual(arc.length, 0.0) ||
      mathlib::IsDoubleEqual(arc.circle_info.radius, 0.0)) {
    std::cout << "the length or radius of arc can not be 0\n";
    return false;
  }
  if (ds <= 0.0) {
    std::cout << "sample ds can not be smaller than 0\n";
    return false;
  }
  point_set.clear();
  point_set.reserve(50);

  PathPoint pn;
  // get first point
  pn.Set(arc.pA, NormalizeAngle(arc.headingA));
  point_set.emplace_back(pn);

  const auto &pO = arc.circle_info.center;
  Eigen::Vector2d v_n = arc.pA - pO;
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

}  // namespace geometry_lib
}  // namespace pnc