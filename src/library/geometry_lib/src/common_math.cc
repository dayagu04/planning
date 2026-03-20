#include "common_math.h"

#include <cstdint>

#include "log_glog.h"

namespace planning {
namespace common_math {

const float kDeg2RadF = M_PIf32 / 180.0f;
const float kRad2DegF = 180.0f / M_PIf32;

static const double kEqualHeadingEpsF = 1e-2 * kDeg2RadF;

template <typename T>
const T UnifyAngle(const T angle) {
  if (angle < T(M_PIf32) - 0.001f && angle > -T(M_PIf32) + 0.001f) {
    return angle;
  }
  T a = std::fmod(angle + T(M_PIf32), T(2.0f) * T(M_PIf32));
  if (a < T(0.0f)) {
    a += T(2.0f) * T(M_PIf32);
  }
  return a - T(M_PIf32);
}

template <typename T>
const T UnifyAngleDiff(const T angle1, const T angle2) {
  return UnifyAngle(angle1 - angle2);
}

template <typename T>
const T UnifyAngleSum(const T angle1, const T angle2) {
  return UnifyAngle(angle1 + angle2);
}

template <typename T>
const Eigen::Matrix<T, 2, 2> CalRotMatFromTheta(const T theta) {
  const T cos_theta = std::cos(theta);
  const T sin_theta = std::sin(theta);
  Eigen::Matrix<T, 2, 2> mat;
  mat << cos_theta, -sin_theta, sin_theta, cos_theta;
  return mat;
}

template <typename T>
const T CalPt2LineDistSquare(const Pos<T>& pt, const LineSeg<T>& line) {
  const Pos<T> v_AB = line.pB - line.pA;
  const Pos<T> v_AO = pt - line.pA;

  if (v_AB.dot(v_AB) < T(1e-3f)) {
    return T(0.0f);
  }

  const T v_AO_dot_v_AB = v_AO.dot(v_AB);

  return v_AO.dot(v_AO) - v_AO_dot_v_AB * v_AO_dot_v_AB / v_AB.dot(v_AB);
}

template <typename T>
const T CalPt2LineDist(const Pos<T>& pt, const LineSeg<T>& line) {
  const T dist_square = CalPt2LineDistSquare(pt, line);
  if (dist_square < T(1e-4f)) {
    return T(0.0f);
  }
  return std::sqrt(dist_square);
}

template <typename T>
const bool CalLineSegByGearAndPose(const AstarPathGear gear, const T length,
                                   const PathPt<T>& pose,
                                   PathSeg<T>& line_seg) {
  if (length < T(1e-3f)) {
    return false;
  }

  LineSeg<T>& line = line_seg.line_seg;

  line.pA = pose.pos;
  line.length = std::fabs(length);
  line.dir = pose.dir;
  line.theta = pose.theta;

  const T sign = (gear == AstarPathGear::DRIVE) ? 1.0 : -1.0;
  line.pB = pose.pos + sign * line.length * pose.dir;

  line_seg.steer = AstarPathSteer::STRAIGHT;
  line_seg.gear = gear;
  line_seg.kappa = T(0.0f);

  return true;
}

template <typename T>
const bool CalLineUnitNormVecByPt(const Pos<T>& pt, const LineSeg<T>& line,
                                  Pos<T>& line_norm_vec) {
  const Pos<T> v_AO = pt - line.pA;
  const T cross = CalCrossFromTwoVec(line.dir, v_AO);
  const Pos<T> line_tang_vec = line.dir;
  // line_norm_vec is from pos to line
  if (cross > T(1e-6f)) {
    // pos is on left side of the line
    line_norm_vec = Pos<T>(line_tang_vec.y(), -line_tang_vec.x());
  } else if (cross < T(-1e-6f)) {
    // pos is on right side of the line
    line_norm_vec = Pos<T>(-line_tang_vec.y(), line_tang_vec.x());
  } else {
    return false;
  }
  return true;
}

template <typename T>
const bool SamplePathSeg(std::vector<PathPt<T>>& pts, const PathSeg<T>& seg,
                         const T ds, const T kappa) {
  pts.clear();
  pts.reserve(50);

  PathPt<T> pn;
  // get first point
  pn.pos = seg.GetStartPos();
  pn.theta = seg.GetStartTheta();
  pn.kappa = kappa;
  pts.emplace_back(pn);

  // get mid point
  if (seg.steer == AstarPathSteer::STRAIGHT) {
    const Pos<T> unit_line_vec =
        (seg.GetEndPos() - seg.GetStartPos()).normalized();
    T s = ds;
    while (s < seg.GetLength()) {
      pn.pos = seg.GetStartPos() + s * unit_line_vec;
      pn.s = s;
      pts.emplace_back(pn);
      s += ds;
    }
  } else {
    const Pos<T>& pO = seg.GetCenter();
    Pos<T> v_n = seg.GetStartPos() - pO;
    T theta = seg.GetStartTheta();
    const T dtheta =
        ds / seg.GetRadius() * (seg.GetIsAnticlockwise() ? 1.0 : -1.0);

    const Eigen::Matrix<T, 2, 2> rot_m = CalRotMatFromTheta(dtheta);
    T s = ds;
    while (s < seg.GetLength()) {
      v_n = rot_m * v_n;
      pn.pos = pO + v_n;
      theta += dtheta;
      pn.theta = UnifyAngle(theta);
      pn.s = s;
      pts.emplace_back(pn);
      s += ds;
    }
  }

  // get last point
  pn.pos = seg.GetEndPos();
  pn.theta = seg.GetEndTheta();
  pn.s = seg.GetLength();
  pts.emplace_back(pn);

  return true;
}

template <typename T>
const bool CompleteArcSeg(ArcSeg<T>& arc) {
  // known: pA, pB, thetaA, center, radius
  // unknown: thetaB, dirB, length, is_anticlockwise
  const Pos<T> OA = arc.pA - arc.center;
  const Pos<T> OB = arc.pB - arc.center;
  const T rot_angle = FastSignedAngle(OA, OB);

  arc.is_anticlockwise = rot_angle > T(0.0f);

  const T OA_norm = arc.radius;

  arc.thetaB = UnifyAngle(arc.thetaA + rot_angle);

  arc.dirB << std::cos(arc.thetaB), std::sin(arc.thetaB);

  arc.length = std::fabs(rot_angle) * OA_norm;

  return true;
}

template <typename T>
const bool CompleteArcSeg(ArcSeg<T>& arc, AstarPathSteer steer) {
  const Pos<T> t = arc.dirA;
  const Pos<T> n = (steer == AstarPathSteer::LEFT) ? Pos<T>(-t.y(), t.x())
                                                   : Pos<T>(t.y(), -t.x());

  arc.center = arc.pA + arc.radius * n;

  const T rot_angle = UnifyAngleDiff(arc.thetaB, arc.thetaA);

  if (rot_angle > T(0.0f)) {
    arc.is_anticlockwise = true;
  } else if (rot_angle < T(0.0f)) {
    arc.is_anticlockwise = false;
  } else {
    return false;
  }

  const Pos<T> OA = arc.pA - arc.center;
  const T cos_rot = std::cos(rot_angle);
  const T sin_rot = std::sin(rot_angle);
  const Pos<T> OB(cos_rot * OA.x() - sin_rot * OA.y(),
                  sin_rot * OA.x() + cos_rot * OA.y());

  arc.pB = arc.center + OB;
  arc.length = std::fabs(rot_angle) * arc.radius;
  arc.dirB << std::cos(arc.thetaB), std::sin(arc.thetaB);

  return true;
}

template <typename T>
const uint8_t CalIntersectionOfLineAndCircle(const LineSeg<T>& line,
                                             const ArcSeg<T>& circle,
                                             std::array<Pos<T>, 2>& pts) {
  const Pos<T> v_AB = line.pB - line.pA;
  const Pos<T> v_AO = circle.center - line.pA;
  const T v_AB_sq = v_AB.squaredNorm();

  if (v_AB_sq < T(1e-4f)) {
    return 0;
  }

  const T v_AO_dot_v_AB = v_AO.dot(v_AB);
  const T dist_sq =
      v_AO.squaredNorm() - v_AO_dot_v_AB * v_AO_dot_v_AB / v_AB_sq;

  if (dist_sq > Square(circle.radius + T(1e-6f))) {
    return 0;  // no intersection, avoid sqrt
  }

  const T dist = (dist_sq < T(1e-4f)) ? T(0.0f) : std::sqrt(dist_sq);
  const Pos<T> line_tang_vec = v_AB / std::sqrt(v_AB_sq);
  const T cross = CalCrossFromTwoVec(v_AB, v_AO);
  const Pos<T> line_norm_vec =
      (cross > T(1e-6f)) ? Pos<T>(line_tang_vec.y(), -line_tang_vec.x())
                         : Pos<T>(-line_tang_vec.y(), line_tang_vec.x());

  if (dist < circle.radius - T(1e-6f)) {
    const T d1 = std::sqrt(circle.radius * circle.radius - dist_sq);
    pts[0] = circle.center + dist * line_norm_vec + d1 * line_tang_vec;
    pts[1] = circle.center + dist * line_norm_vec - d1 * line_tang_vec;
    return 2;
  }
  pts[0] = circle.center + dist * line_norm_vec;
  return 1;
}

template <typename T>
const bool CalIntersectionOfTwoLines(Pos<T>& intersection,
                                     const LineSeg<T>& line1,
                                     const LineSeg<T>& line2) {
  // two line, no two line seg

  // use determinant and Cramer's rule
  // a*x + b*y = e
  // c*x + d*y = f
  // D = |a b|
  //     |c d| = a*d - b*c
  // x = (e*d - b*f) / D
  // y = (a*f - e*c) / D

  // P is the intersection of line segment AB and CD
  // P = A + t1 * AB;  P = C + t2 * CD
  // t1 * AB - t2 * CD = C - A
  // t1 * AB.x - t2 * CD.x = AC.x
  // t1 * AB.y - t2 * CD.y = AC.y
  // D = | AB.x  -CD.x |
  //     | AB.y  -CD.y | = AB.x() * (-CD.y()) - (-CD.x()) * AB.y()

  const Pos<T> AB = line1.pB - line1.pA;
  const Pos<T> CD = line2.pB - line2.pA;

  const T det = -CalCrossFromTwoVec(AB, CD);
  if (IsTwoNumerEqual(det, T(0.0f))) {
    return false;
  }

  const Pos<T> AC = line2.pA - line1.pA;
  const T t1 = -CalCrossFromTwoVec(AC, CD) / det;
  // const T t2 = CalCrossFromTwoVec(AB, AC) / det;

  intersection = line1.pA + t1 * AB;
  return true;
}

template <typename T>
const bool CalOneArcWithTargetThetaAndGear(ArcSeg<T>& arc,
                                           const AstarPathGear gear,
                                           const T target_theta) {
  arc.thetaB = target_theta;
  const T theta_diff = UnifyAngleDiff(arc.thetaA, target_theta);
  if (IsTwoNumerEqual(theta_diff, T(0.0f), T(1e-4f))) {
    return false;
  }
  const AstarPathSteer arc_steer =
      ((theta_diff < T(0.0f) && gear == AstarPathGear::DRIVE) ||
       (theta_diff > T(0.0f) && gear == AstarPathGear::REVERSE))
          ? AstarPathSteer::LEFT
          : AstarPathSteer::RIGHT;

  return CompleteArcSeg(arc, arc_steer);
}

template <typename T>
const bool CalOneArcWithLineAndGear(ArcSeg<T>& arc, const LineSeg<T>& line,
                                    const AstarPathGear gear,
                                    const T min_radius) {
  const T heading_diff = UnifyAngleDiff(arc.thetaA, line.theta);
  if (std::fabs(heading_diff) < kEqualHeadingEpsF) {
    return false;
  }

  Pos<T> line_norm_vec;
  if (!CalLineUnitNormVecByPt(arc.pA, line, line_norm_vec)) {
    return false;
  }

  const bool is_left =
      (heading_diff < T(0.0f) && gear == AstarPathGear::DRIVE) ||
      (heading_diff > T(0.0f) && gear == AstarPathGear::REVERSE);

  const Pos<T> pose_norm_vec = is_left ? Pos<T>(-arc.dirA.y(), arc.dirA.x())
                                       : Pos<T>(arc.dirA.y(), -arc.dirA.x());

  const Pos<T> AC = arc.pA - line.pA;
  const Pos<T> line_tang_vec = line.dir;
  const Pos<T> norm_vec = pose_norm_vec + line_norm_vec;
  const T a = CalCrossFromTwoVec(norm_vec, line_tang_vec);
  if (IsTwoNumerEqual(a, T(0.0f))) {
    return false;
  }

  arc.radius = -CalCrossFromTwoVec(AC, line_tang_vec) / a;

  if (arc.radius < min_radius) {
    return false;
  }

  arc.center = arc.pA + arc.radius * pose_norm_vec;
  arc.pB = arc.center + arc.radius * line_norm_vec;

  return CompleteArcSeg(arc);
}

template <typename T>
const uint8_t CalTwoArcWithLine(
    const PathPt<T>& pose, const LineSeg<T>& line, const T radius1,
    const T radius2,
    std::array<std::pair<ArcSeg<T>, ArcSeg<T>>, 8>& arc_pairs) {
  uint8_t count = 0;

  std::array<Pos<T>, 2> center1s = {
      pose.pos + radius1 * Pos<T>(pose.dir.y(), -pose.dir.x()),
      pose.pos + radius1 * Pos<T>(-pose.dir.y(), pose.dir.x())};

  std::array<Pos<T>, 2> line_norm_dirs = {Pos<T>(line.dir.y(), -line.dir.x()),
                                          Pos<T>(-line.dir.y(), line.dir.x())};

  for (const Pos<T>& center1 : center1s) {
    for (const Pos<T>& line_norm_dir : line_norm_dirs) {
      if (count >= 8) {
        return count;
      }
      // the O2 is on arc2_center_line
      LineSeg<T> arc2_center_line(line.pA + line_norm_dir * radius2,
                                  line.pB + line_norm_dir * radius2);
      // the O2 is on virtual circle, arc1 is tangent with arc2, than the dist
      // of O1 and O2 is r1+r2
      ArcSeg<T> virtual_circle(center1, radius1 + radius2);
      // the intersection of arc2_center_line and virtual arc is arc2 center
      std::array<Pos<T>, 2> center2s;
      const uint8_t center2_count = CalIntersectionOfLineAndCircle(
          arc2_center_line, virtual_circle, center2s);
      if (center2_count < 1) {
        continue;
      }

      for (uint8_t j = 0; j < center2_count; ++j) {
        const Pos<T>& center2 = center2s[j];
        if (count >= 8) {
          return count;
        }
        ArcSeg<T> arc1(center1, radius1);
        arc1.pA = pose.pos;
        arc1.thetaA = pose.theta;
        arc1.dirA = pose.dir;

        // calc two arc tangent
        const Pos<T> arc1_arc2_tangent =
            center1 + radius1 * (center2 - center1).normalized();

        arc1.pB = arc1_arc2_tangent;

        if (!CompleteArcSeg(arc1)) {
          continue;
        }

        ArcSeg<T> arc2(center2, radius2);
        arc2.pA = arc1.pB;
        arc2.thetaA = arc1.thetaB;
        arc2.dirA = arc1.dirB;

        // assume C2 is the tangent of arc2 and line
        // C2O2 = O2 - C2 -> C2 = O2
        const Pos<T> C2O2 = radius2 * line_norm_dir;
        arc2.pB = center2 - C2O2;

        if (!CompleteArcSeg(arc2)) {
          continue;
        }

        arc_pairs[count++] = std::make_pair(arc1, arc2);
      }
    }
  }

  return count;
}

template <typename T>
const uint8_t CalCommonTangentCircleOfTwoLine(
    const LineSeg<T>& line1, const LineSeg<T>& line2, const T radius,
    std::array<Pos<T>, 4>& centers,
    std::array<std::pair<Pos<T>, Pos<T>>, 4>& tangent_ptss) {
  Pos<T> intersection = Pos<T>(0.0f, 0.0f);

  if (!CalIntersectionOfTwoLines(intersection, line1, line2)) {
    return 0;
  }

  const Pos<T> unit_line1_vec = (line1.pB - line1.pA).normalized();
  const Pos<T> unit_line2_vec = (line2.pB - line2.pA).normalized();
  // every line have two direction, so two lines have four combination
  const std::array<Pos<T>, 4> combination = {
      Pos<T>(1.0f, 1.0f), Pos<T>(1.0f, -1.0f), Pos<T>(-1.0f, 1.0f),
      Pos<T>(-1.0f, -1.0f)};

  for (uint8_t i = 0; i < combination.size(); ++i) {
    const Pos<T> actual_unit_line1_vec = unit_line1_vec * combination[i].x();
    const Pos<T> actual_unit_line2_vec = unit_line2_vec * combination[i].y();

    const Pos<T> unit_angular_bisector_vec =
        (actual_unit_line1_vec + actual_unit_line2_vec).normalized();

    // the angle between two lines is 2*theta, sin_theta is no impossible 0
    const T sin_theta =
        CalCrossFromTwoVec(actual_unit_line1_vec, unit_angular_bisector_vec);
    if (IsTwoNumerEqual(sin_theta, T(0.0f))) {
      return 0;
    }

    const T dist_intersection_center = radius / sin_theta;

    // A is intersection, O is center
    const Pos<T> AO = dist_intersection_center * unit_angular_bisector_vec;
    const T cos_theta = actual_unit_line1_vec.dot(unit_angular_bisector_vec);
    const T dist_intersection_tangpt = dist_intersection_center * cos_theta;

    centers[i] = AO + intersection;

    // P1, P2 is tang pt
    const Pos<T> AP1 = dist_intersection_tangpt * actual_unit_line1_vec;
    const Pos<T> AP2 = dist_intersection_tangpt * actual_unit_line2_vec;

    tangent_ptss[i] = std::make_pair(AP1 + intersection, AP2 + intersection);
  }

  return static_cast<uint8_t>(combination.size());
}

template <typename T>
const bool CalTwoArcWithSameThetaAndGear(ArcSeg<T>& arc1, ArcSeg<T>& arc2,
                                         const AstarPathGear gear) {
  const LineSeg<T> line(arc2.pB, arc2.dirB, arc2.thetaB, 1.0);

  Pos<T> line_norm_vec;
  if (!CalLineUnitNormVecByPt(arc1.pA, line, line_norm_vec)) {
    return false;
  }

  arc1.center = arc1.pA + arc1.radius * line_norm_vec;

  const T lat_dist = CalPt2LineDist(arc1.pA, line);

  // if lat_dist is too big, arc1 and arc2 cannot be tangent
  if (lat_dist > arc1.radius * 2.0 + 1e-2) {
    return false;
  }

  // the C is the tangent of arc1 and arc2
  // the dist from C to line2 is the half of lat_dist because of the same
  // radius of arc1 and arc2 the theta is rot_angle of arc1
  const T cos_theta = (arc1.radius - lat_dist * 0.5) / arc1.radius;

  const T sin_theta =
      std::sqrt(T(1.0f) - std::min(cos_theta * cos_theta, T(1.0f)));

  // lon_dist is the proj length of AC in line2
  const T lon_dist = arc1.radius * sin_theta;

  const Pos<T> tmp_tang_vec =
      (gear == AstarPathGear::REVERSE) ? -arc1.dirA : arc1.dirA;

  arc1.pB =
      arc1.pA + lat_dist * T(0.5f) * line_norm_vec + lon_dist * tmp_tang_vec;

  if (!CompleteArcSeg(arc1)) {
    return false;
  }

  arc2.pA = arc1.pB;
  arc2.thetaA = arc1.thetaB;
  arc2.dirA = arc1.dirA;
  arc2.pB =
      arc2.pA + lat_dist * T(0.5f) * line_norm_vec + lon_dist * tmp_tang_vec;
  arc2.center = arc2.pB - line_norm_vec * arc2.radius;
  arc2.is_anticlockwise = !arc1.is_anticlockwise;
  arc2.length = arc1.length;

  return true;
}

template const float UnifyAngle(const float);
template const double UnifyAngle(const double);

template const float UnifyAngleDiff(const float, const float);
template const double UnifyAngleDiff(const double, const double);

template const float UnifyAngleSum(const float, const float);
template const double UnifyAngleSum(const double, const double);

template const Eigen::Matrix<float, 2, 2> CalRotMatFromTheta(const float);
template const Eigen::Matrix<double, 2, 2> CalRotMatFromTheta(const double);

template const float CalPt2LineDistSquare<float>(const Pos<float>&,
                                                 const LineSeg<float>&);
template const double CalPt2LineDistSquare<double>(const Pos<double>&,
                                                   const LineSeg<double>&);

template const float CalPt2LineDist(const Pos<float>&, const LineSeg<float>&);
template const double CalPt2LineDist(const Pos<double>&,
                                     const LineSeg<double>&);

template const bool CalLineSegByGearAndPose(const AstarPathGear, const float,
                                            const PathPt<float>&,
                                            PathSeg<float>&);
template const bool CalLineSegByGearAndPose(const AstarPathGear, const double,
                                            const PathPt<double>&,
                                            PathSeg<double>&);

template const bool CalLineUnitNormVecByPt(const Pos<float>&,
                                           const LineSeg<float>&, Pos<float>&);
template const bool CalLineUnitNormVecByPt(const Pos<double>&,
                                           const LineSeg<double>&,
                                           Pos<double>&);

template const bool SamplePathSeg(std::vector<PathPt<float>>&,
                                  const PathSeg<float>&, const float,
                                  const float);
template const bool SamplePathSeg(std::vector<PathPt<double>>&,
                                  const PathSeg<double>&, const double,
                                  const double);

template const bool CompleteArcSeg(ArcSeg<float>&);
template const bool CompleteArcSeg(ArcSeg<double>&);

template const bool CompleteArcSeg(ArcSeg<float>&, AstarPathSteer);
template const bool CompleteArcSeg(ArcSeg<double>&, AstarPathSteer);

template const uint8_t CalIntersectionOfLineAndCircle(
    const LineSeg<float>&, const ArcSeg<float>&, std::array<Pos<float>, 2>&);
template const uint8_t CalIntersectionOfLineAndCircle(
    const LineSeg<double>&, const ArcSeg<double>&, std::array<Pos<double>, 2>&);

template const bool CalIntersectionOfTwoLines(Pos<float>&,
                                              const LineSeg<float>&,
                                              const LineSeg<float>&);
template const bool CalIntersectionOfTwoLines(Pos<double>&,
                                              const LineSeg<double>&,
                                              const LineSeg<double>&);

template const bool CalOneArcWithTargetThetaAndGear(ArcSeg<float>&,
                                                    const AstarPathGear,
                                                    const float);
template const bool CalOneArcWithTargetThetaAndGear(ArcSeg<double>&,
                                                    const AstarPathGear,
                                                    const double);

template const bool CalOneArcWithLineAndGear(ArcSeg<float>&,
                                             const LineSeg<float>&,
                                             const AstarPathGear, const float);
template const bool CalOneArcWithLineAndGear(ArcSeg<double>&,
                                             const LineSeg<double>&,
                                             const AstarPathGear, const double);

template const uint8_t CalTwoArcWithLine<float>(
    const PathPt<float>&, const LineSeg<float>&, const float, const float,
    std::array<std::pair<ArcSeg<float>, ArcSeg<float>>, 8>&);
template const uint8_t CalTwoArcWithLine<double>(
    const PathPt<double>&, const LineSeg<double>&, const double, const double,
    std::array<std::pair<ArcSeg<double>, ArcSeg<double>>, 8>&);

template const uint8_t CalCommonTangentCircleOfTwoLine(
    const LineSeg<float>&, const LineSeg<float>&, const float,
    std::array<Pos<float>, 4>&,
    std::array<std::pair<Pos<float>, Pos<float>>, 4>&);
template const uint8_t CalCommonTangentCircleOfTwoLine(
    const LineSeg<double>&, const LineSeg<double>&, const double,
    std::array<Pos<double>, 4>&,
    std::array<std::pair<Pos<double>, Pos<double>>, 4>&);

template const bool CalTwoArcWithSameThetaAndGear(ArcSeg<float>&,
                                                  ArcSeg<float>&,
                                                  const AstarPathGear);
template const bool CalTwoArcWithSameThetaAndGear(ArcSeg<double>&,
                                                  ArcSeg<double>&,
                                                  const AstarPathGear);

}  // namespace common_math
}  // namespace planning
