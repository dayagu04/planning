#ifndef __GEOMETERY_MATH_H__
#define __GEOMETERY_MATH_H__

#include <math.h>

#include <cmath>
#include <utility>

#include "Eigen/Core"
#include "spline.h"

namespace pnc {

namespace geometry_lib {

struct PathPoint {
  PathPoint() = default;
  PathPoint(const Eigen::Vector2d &pos_in, const double heading_in) {
    pos = pos_in;
    heading = heading_in;
  }

  void Set(const Eigen::Vector2d &pos_in, const double heading_in) {
    pos = pos_in, heading = heading_in;
  }

  Eigen::Vector2d pos = Eigen::Vector2d::Zero();
  double heading = 0.0;
};

struct LineSegment {
  LineSegment() = default;
  LineSegment(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2) {
    SetPoints(p1, p2);
  }

  LineSegment(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2,
              const double heading_r) {
    SetPoints(p1, p2);
    heading = heading_r;
  }

  void SetPoints(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2) {
    pA = p1;
    pB = p2;
    length = (p1 - p2).norm();
  }

  Eigen::Vector2d pA = Eigen::Vector2d::Zero();
  Eigen::Vector2d pB = Eigen::Vector2d::Zero();
  double heading = 0.0;
  double length = 0.0;
  bool is_ignored = false;
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
  bool is_ignored = false;
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

  GlobalToLocalTf() {}

  GlobalToLocalTf(const Eigen::Vector2d &p_n_ori, const double theta_ori) {
    Init(p_n_ori, theta_ori);
  }

  void Init(const Eigen::Vector2d &p_n_ori, const double theta_ori) {
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

  LocalToGlobalTf() = default;

  LocalToGlobalTf(const Eigen::Vector2d &p_n_ori, const double theta_ori) {
    Init(p_n_ori, theta_ori);
  }

  void Init(const Eigen::Vector2d &p_n_ori, const double theta_ori) {
    pos_n_ori = p_n_ori;
    heading_ori = theta_ori;
    const double cos_theta = std::cos(theta_ori);
    const double sin_theta = std::sin(theta_ori);
    rot_m << cos_theta, -sin_theta, sin_theta, cos_theta;
  }

  const Eigen::Vector2d GetPos(const Eigen::Vector2d &p_n) const {
    return (rot_m * p_n + pos_n_ori);
  }

  const double GetHeading(const double heading) const {
    return NormalizeAngle(heading + heading_ori);
  }
};

const Eigen::Vector2d GenHeadingVec(const double heading);

const double NormSquareOfTwoVector2d(const Eigen::Vector2d &p1,
                                     const Eigen::Vector2d &p2);

const double NormSquareOfVector2d(const Eigen::Vector2d &p1);

double GetAngleFromTwoVec(const Eigen::Vector2d &a, const Eigen::Vector2d &b);

const Eigen::Matrix2d GetRotm2dFromTheta(const double theta);

const double CalPoint2LineDist(const Eigen::Vector2d &pO,
                               const LineSegment &line);

const double CalPoint2LineDistSquare(const Eigen::Vector2d &pO,
                                     const LineSegment &line);

const bool CheckLineSegmentInCircle(const LineSegment &line, const Circle &c);

const bool CalTangentPointsOfEqualCircles(TangentOutput &output,
                                          const Circle &c1, const Circle &c2);

const Eigen::Matrix2d GetRotm2dFromTwoVec(const Eigen::Vector2d &a,
                                          const Eigen::Vector2d &b);

const LineSegment GetEgoHeadingLine(const Eigen::Vector2d &ego_pos,
                                    const double ego_heading);

const bool GetLineLineIntersection(
    Eigen::Vector2d &intersection,
    const pnc::geometry_lib::LineSegment &line_seg1,
    const pnc::geometry_lib::LineSegment &line_seg2);

const bool CheckPointLiesOnArc(const pnc::geometry_lib::Arc &arc,
                               const Eigen::Vector2d &pC);

const bool GetArcLineIntersection(
    Eigen::Vector2d &intersection, const pnc::geometry_lib::Arc &arc,
    const pnc::geometry_lib::LineSegment &line_seg);

const bool CheckTwoCircleIntersection(const Circle &c1, const Circle &c2);

const std::pair<Eigen::Vector2d, Eigen::Vector2d> GetTwoCircleIntersection(
    const Circle &c1, const Circle &c2);

const bool GetTwoArcIntersection(Eigen::Vector2d &intersection, const Arc &arc1,
                                 const Arc &arc2);
const double GetCrossFromTwoVec2d(const Eigen::Vector2d &vec0,
                                  const Eigen::Vector2d &vec1);

const size_t CalcCrossPointsOfLineAndCircle(
    const LineSegment &line, const Circle &circle,
    std::vector<Eigen::Vector2d> &cross_points);

const size_t CalcCrossPointsOfLineSegAndCircle(
    const LineSegment &line_seg, const Circle &circle,
    std::vector<Eigen::Vector2d> &cross_points);

const size_t CalcTangentPtOfCircleAndLinePassingThroughAGivenPt(
    const Circle &circle, Eigen::Vector2d pt,
    std::vector<Eigen::Vector2d> &tangent_pts);

const size_t CalcCrossPtsOfTwoCircle(const Circle &circle1,
                                     const Circle &circle2,
                                     std::vector<Eigen::Vector2d> &cross_pts);

const bool CalTangentCirclesOfTwoLines(
    const LineSegment &line1, const LineSegment &line2, const double radius,
    std::vector<Eigen::Vector2d> &centers,
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> &tangent_ptss);

const bool CalCrossPtOfTwoLines(const LineSegment &line1,
                                const LineSegment &line2,
                                Eigen::Vector2d &cross_pt);

const bool CalProjFromSplineByBisection(const double &s_start,
                                        const double &s_end, double &s_proj,
                                        const Eigen::Vector2d &current_pos,
                                        const pnc::mathlib::spline &x_s_spline,
                                        const pnc::mathlib::spline &y_s_spline);

const bool CalExtendedPointByTwoPoints(const Eigen::Vector2d &start_point,
                                       const Eigen::Vector2d &end_point,
                                       Eigen::Vector2d &extended_point,
                                       const double extended_distance);

const bool IsPointOnLeftSideOfLineSeg(
    const Eigen::Vector2d &point,
    const LineSegment &line_seg);  // note that line is started from A

const bool OneStepArcTargetLineByGear(
    Arc &arc, const Eigen::Vector2d &start_pos, const double start_heading,
    const bool is_advanve,
    const pnc::geometry_lib::LineSegment
        &target_line);  // note that target_line has heading

const bool OneStepArcTargetLine(
    Arc &arc, const Eigen::Vector2d &start_pos, const double start_heading,
    const pnc::geometry_lib::LineSegment &target_line);

const bool OneStepArcTargetLine(
    Arc &arc, const Eigen::Vector2d &start_pos, const double start_heading,
    bool is_left, const pnc::geometry_lib::LineSegment &target_line);

const bool CalNormalVecOfLineTowardsGivenPt(Eigen::Vector2d &v_line_n,
                                            const LineSegment &line,
                                            const Eigen::Vector2d &pt);

void OneStepArcTargetHeading(Arc &arc, const Eigen::Vector2d &start_pos,
                             const double start_heading,
                             const double target_heading, const double radius,
                             const bool is_advance);

const bool OneStepParallelShift(
    std::pair<Arc, Arc> &arc_pair, const Eigen::Vector2d &start_pos,
    const double start_heading,
    const pnc::geometry_lib::LineSegment &target_line, const double radius,
    const bool is_advance);

const bool CalInverseTwoArcGeometry(
    std::pair<Arc, Arc> &arc_pair, const Eigen::Vector2d &start_pos,
    const double start_heading,
    const pnc::geometry_lib::LineSegment &target_line, const bool is_advance);

const bool IsArcAdvance(const pnc::geometry_lib::Arc &arc);
const bool IsArcTurnLeft(const pnc::geometry_lib::Arc &arc);
const bool IsLineAdvance(const LineSegment &line_seg);

const bool SamplePointSetInLineSeg(std::vector<Eigen::Vector2d> &point_set,
                                   const LineSegment &line, const double &ds);

const bool SamplePointSetInArc(std::vector<Eigen::Vector2d> &point_set,
                               const Arc &arc, const double &ds);

const bool SamplePointSetInLineSeg(std::vector<PathPoint> &point_set,
                                   const LineSegment &line, const double &ds);

const bool SamplePointSetInArc(std::vector<PathPoint> &point_set,
                               const Arc &arc, const double &ds);

}  // namespace geometry_lib
}  // namespace pnc

#endif