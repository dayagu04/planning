#ifndef __GEOMETERY_MATH_H__
#define __GEOMETERY_MATH_H__

#include <bits/stdint-uintn.h>
#include <math.h>

#include <cmath>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "spline.h"

extern double kDeg2Rad;
extern double kRad2Deg;

namespace pnc {

namespace geometry_lib {

enum SlotSide {
  SLOT_SIDE_INVALID,
  SLOT_SIDE_LEFT,
  SLOT_SIDE_RIGHT,
};

enum PathPlanType {
  PLAN_TYPE_INVALID,
  PLAN_TYPE_ONE_ARC,
  PLAN_TYPE_TWO_ARC,
  PLAN_TYPE_LINE_ARC,
  PLAN_TYPE_ONE_LINE,
  PLAN_TYPE_S_TURN,
  PLAN_TYPE_ALIGN_BODY,
  PLAN_TYPE_COUNT,
};

enum PathSegType {
  SEG_TYPE_INVALID,
  SEG_TYPE_LINE,
  SEG_TYPE_ARC,
  SEG_TYPE_COUNT,
};

enum PathSegSteer {
  SEG_STEER_INVALID,
  SEG_STEER_STRAIGHT,
  SEG_STEER_LEFT,
  SEG_STEER_RIGHT,
  SEG_STEER_COUNT,
};

enum PathSegGear {
  SEG_GEAR_INVALID,
  SEG_GEAR_DRIVE,
  SEG_GEAR_REVERSE,
  SEG_GEAR_COUNT,
};

enum class RotateDirection {
  NONE,
  COUNTER_CLOCKWISE,
  CLOCKWISE,
  SAME_DIRECTION,
  ROTATE_DIRECTION_MAX_NUM,
};

struct PlanSegState {
  uint8_t cur_seg_type = SEG_TYPE_LINE;
  uint8_t cur_seg_steer = SEG_STEER_STRAIGHT;
  uint8_t cur_seg_gear = SEG_GEAR_DRIVE;
};

struct PathPoint {
  PathPoint() = default;
  PathPoint(const Eigen::Vector2d &pos_in, const double heading_in) {
    pos = pos_in;
    heading = heading_in;
  }

  PathPoint(const Eigen::Vector2d &pos_in, const double heading_in,
            const double kappa_in) {
    pos = pos_in;
    heading = heading_in;
    kappa = kappa_in;
  }

  void Set(const Eigen::Vector2d &pos_in, const double heading_in) {
    pos = pos_in, heading = heading_in;
  }

  Eigen::Vector2d pos = Eigen::Vector2d::Zero();
  double heading = 0.0;
  // todo: path point related codes is too much, unify them.
  double kappa = 0.0;

  void Reset() {
    pos.setZero();
    heading = 0.0;
    kappa = 0.0;
  }
};

struct LineSegment {
  LineSegment() = default;
  LineSegment(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2) {
    SetPoints(p1, p2);
  }

  LineSegment(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2,
              const double heading_in) {
    SetPoints(p1, p2);
    heading = heading_in;
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

  void Reset() {
    pA.setZero();
    pB.setZero();
    heading = 0.0;
    length = 0.0;
    is_ignored = false;
  }
};

struct Circle {
  Eigen::Vector2d center = Eigen::Vector2d::Zero();
  double radius = 0.0;

  void Reset() {
    center.setZero();
    radius = 0.0;
  }
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

  void Reset() {
    circle_info.Reset();
    pA.setZero();
    pB.setZero();
    length = 0.0;
    headingA = 0.0;
    headingB = 0.0;
    is_anti_clockwise = true;
    is_ignored = false;
  }
};

struct PathSegment {
  uint8_t seg_type = SEG_TYPE_LINE;
  uint8_t seg_steer = SEG_STEER_STRAIGHT;
  uint8_t seg_gear = SEG_GEAR_DRIVE;
  uint8_t plan_type = PLAN_TYPE_INVALID;

  bool collision_flag = false;

  LineSegment line_seg;
  Arc arc_seg;

  PathSegment() = default;

  const double Getlength() const {
    if (seg_type == SEG_TYPE_LINE) {
      return line_seg.length;
    } else if (seg_type == SEG_TYPE_ARC) {
      return arc_seg.length;
    } else {
      return 0.0;
    }
  }

  const Eigen::Vector2d GetStartPos() const {
    if (seg_type == SEG_TYPE_LINE) {
      return line_seg.pA;
    } else if (seg_type == SEG_TYPE_ARC) {
      return arc_seg.pA;
    } else {
      return Eigen::Vector2d::Zero();
    }
  }

  const Eigen::Vector2d GetEndPos() const {
    if (seg_type == SEG_TYPE_LINE) {
      return line_seg.pB;
    } else if (seg_type == SEG_TYPE_ARC) {
      return arc_seg.pB;
    } else {
      return Eigen::Vector2d::Zero();
    }
  }

  const double GetStartHeading() const {
    if (seg_type == SEG_TYPE_LINE) {
      return line_seg.heading;
    } else if (seg_type == SEG_TYPE_ARC) {
      return arc_seg.headingA;
    } else {
      return 0.0;
    }
  }

  const double GetEndHeading() const {
    if (seg_type == SEG_TYPE_LINE) {
      return line_seg.heading;
    } else if (seg_type == SEG_TYPE_ARC) {
      return arc_seg.headingB;
    } else {
      return 0.0;
    }
  }

  const PathPoint GetStartPose() const {
    return PathPoint(GetStartPos(), GetStartHeading());
  }
  const PathPoint GetEndPose() const {
    return PathPoint(GetEndPos(), GetEndHeading());
  }

  PathSegment(const uint8_t seg_type_in, const uint8_t seg_steer_in,
              const uint8_t seg_gear_in, const LineSegment &line_seg_in,
              const Arc &arc_seg_in) {
    seg_type = seg_type_in;
    seg_steer = seg_steer_in;
    seg_gear = seg_gear_in;
    line_seg = line_seg_in;
    arc_seg = arc_seg_in;
  }

  // construct line segment
  PathSegment(const uint8_t seg_gear_in, const LineSegment &line_seg_in) {
    seg_type = SEG_TYPE_LINE;

    seg_steer = SEG_STEER_STRAIGHT;
    seg_gear = seg_gear_in;
    line_seg = line_seg_in;
  }

  // construct arc segment
  PathSegment(const uint8_t seg_steer_in, const uint8_t seg_gear_in,
              const Arc &arc_seg_in) {
    seg_type = SEG_TYPE_ARC;
    seg_steer = seg_steer_in;
    seg_gear = seg_gear_in;
    arc_seg = arc_seg_in;
  }

  const LineSegment &GetLineSeg() const { return line_seg; }
  const Arc &GetArcSeg() const { return arc_seg; }
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

struct Compare {
  size_t type = 0;
  Compare(const size_t _type) : type(_type) {}

  const bool operator()(const Eigen::Vector2d &a,
                        const Eigen::Vector2d &b) const {
    if (type == 0) {
      return a.x() < b.x();  // big -> small
    } else if (type == 1) {
      return a.x() > b.x();  // small -> big
    } else if (type == 2) {
      return a.y() < b.y();  // big -> small
    } else {
      return a.y() > b.y();  // small -> big
    }
  }
};

const Eigen::Vector2d GenHeadingVec(const double heading);

const double NormSquareOfTwoVector2d(const Eigen::Vector2d &p1,
                                     const Eigen::Vector2d &p2);

const double NormSquareOfVector2d(const Eigen::Vector2d &p1);

/**
 * return value range [-PI, PI)
 */
double GetAngleFromTwoVec(const Eigen::Vector2d &a, const Eigen::Vector2d &b);

const Eigen::Matrix2d GetRotm2dFromTheta(const double theta);

const double CalTwoPointDistSquare(const Eigen::Vector2d &p0,
                                   const Eigen::Vector2d &p1);

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

const bool GetIntersectionFromTwoLineSeg(Eigen::Vector2d &intersection,
                                         LineSegment &line_seg1,
                                         LineSegment &line_seg2);

const bool GetIntersectionFromTwoLine(Eigen::Vector2d &intersection,
                                      LineSegment &line1, LineSegment &line2);

const bool CheckPointLiesOnArc(const pnc::geometry_lib::Arc &arc,
                               const Eigen::Vector2d &pC);

const bool GetArcLineIntersection(
    Eigen::Vector2d &intersection, const pnc::geometry_lib::Arc &arc,
    const pnc::geometry_lib::LineSegment &line_seg);

const size_t GetArcLineIntersection(
    std::pair<Eigen::Vector2d, Eigen::Vector2d> &intersections,
    const pnc::geometry_lib::Arc &arc,
    const pnc::geometry_lib::LineSegment &line_seg);

const bool CheckTwoCircleIntersection(const Circle &c1, const Circle &c2);

const std::pair<Eigen::Vector2d, Eigen::Vector2d> GetTwoCircleIntersection(
    const Circle &c1, const Circle &c2);

const bool GetTwoArcIntersection(Eigen::Vector2d &intersection, const Arc &arc1,
                                 const Arc &arc2);

/**
 * if return value > 0, then vec1 is counter clockwise
 * if return value < 0, then vec1 is clockwise
 */
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

const bool IsArcAdvance(const pnc::geometry_lib::Arc &arc);
const bool IsArcTurnLeft(const pnc::geometry_lib::Arc &arc);
const bool IsLineAdvance(const LineSegment &line_seg);

const bool SamplePointSetInLineSeg(std::vector<Eigen::Vector2d> &point_set,
                                   const LineSegment &line, const double ds);

const bool SamplePointSetInArc(std::vector<Eigen::Vector2d> &point_set,
                               const Arc &arc, const double ds);

const bool SamplePointSetInPathSeg(std::vector<Eigen::Vector2d> &point_set,
                                   const PathSegment &path_seg,
                                   const double ds);

const bool SamplePointSetInLineSeg(std::vector<PathPoint> &point_set,
                                   const LineSegment &line, const double ds);

const bool SamplePointSetInArc(std::vector<PathPoint> &point_set,
                               const Arc &arc, const double ds);

const bool SamplePointSetInPathSeg(std::vector<PathPoint> &point_set,
                                   const PathSegment &path_seg,
                                   const double ds);

const bool IsPointInPolygon(const std::vector<Eigen::Vector2d> &polygon,
                            const Eigen::Vector2d &point);

// The eigenvalues of the symmetry matrix must be real numbers ; so there is no
// need to use bool
const bool MinimumBoundingBox(
    std::vector<Eigen::Vector2d> &target_boundingbox,
    const std::vector<Eigen::Vector2d> &original_vertices);

const bool CalOneArcWithLine(Arc &arc, LineSegment &line, double r_err = 0.001);

const bool CalTwoArcWithLine(Arc &arc1, Arc &arc2, LineSegment &line,
                             bool is_shifted = true);

const bool CalTwoSameGearArcWithLine(Arc &arc1, Arc &arc2, LineSegment &line,
                                     const uint8_t gear);

const bool IsPoseOnLine(const PathPoint &pose, LineSegment &line,
                        const double lat_err, const double heading_err);

const Eigen::Vector2d GetUnitTangVecByHeading(const double &heading);

const LineSegment BuildLineSegByPose(const Eigen::Vector2d &current_pos,
                                     const double &current_heading);

const bool CalCommonTangentCircleOfTwoLine(
    LineSegment &line1, LineSegment &line2, const double radius,
    std::vector<Eigen::Vector2d> &centers,
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> &tangent_ptss);

const bool CalLineArcOfTwoLine(LineSegment &line1, LineSegment &line2,
                               LineSegment &line, Arc &arc, const double radius,
                               const bool is_shifted);

const bool CheckTwoVecCollinear(const Eigen::Vector2d &v1,
                                const Eigen::Vector2d &v2);

const bool CheckTwoVecVertical(const Eigen::Vector2d &v1,
                               const Eigen::Vector2d &v2);

const bool CheckTwoVecSameOrOppositeDirection(const Eigen::Vector2d &v1,
                                              const Eigen::Vector2d &v2);

const bool CompleteArcInfo(Arc &arc);

const bool CompleteArcInfo(Arc &arc, const double rot_angle);

const bool CompleteArcInfo(Arc &arc, const uint8_t arc_steer);

const bool CompleteArcInfo(Arc &arc, const double length,
                           const bool is_anti_clockwise);

const bool CompleteArcInfo(Arc &arc, const double length,
                           const bool is_anti_clockwise,
                           const bool save_start_pt);

const bool CompleteArcInfo(Arc &arc, const double length, const uint8_t steer);

const bool CompleteLineInfo(LineSegment &line, const double length);

const bool CompleteLineInfo(LineSegment &line, const double length,
                            const double heading);

const bool CompleteLineInfo(LineSegment &line, const double length,
                            const bool save_start_pt);

const bool CompletePathSeg(PathSegment &path_seg, const double length,
                           const bool save_start_pt = true);

const bool CompletePathSegInfo(PathSegment &path_seg, const double length);

const uint8_t CalArcGear(const Arc &arc);

const uint8_t CalArcSteer(const Arc &arc);

const uint8_t CalLineSegGear(const LineSegment &line_seg);

const uint8_t ReverseGear(const uint8_t gear);

const uint8_t ReverseSteer(const uint8_t steer);

const bool CalcArcDirection(bool &is_anti_clockwise, const uint8_t gear,
                            const uint8_t steer);

const bool IsValidGear(const uint8_t gear);

const bool IsValidLineSteer(const uint8_t steer);

const bool IsValidArcSteer(const uint8_t steer);

const bool ReversePathSegInfo(PathSegment &path_seg);

const bool ReverseArcSegInfo(PathSegment &path_seg);

const bool ReverseLineSegInfo(PathSegment &path_seg);

const bool CalOneArcWithLineAndGear(Arc &arc, const LineSegment &line,
                                    const uint8_t current_arc_steer);

const bool CalOneArcWithTargetHeading(Arc &arc, const uint8_t &current_seg_gear,
                                      const double &target_heading);

const bool LogErr(const std::string &func_name, uint8_t index,
                  const uint8_t type = 0);

const bool CalLineUnitNormVecByPos(const Eigen::Vector2d &pos,
                                   const LineSegment &line,
                                   Eigen::Vector2d &line_norm_vec);

const bool CalTwoArcWithSameHeading(Arc &arc1, Arc &arc2,
                                    const uint8_t seg_gear);

const bool IsDoublePositive(const double x);

const double CalPoint2LineSegDist(const Eigen::Vector2d &pO,
                                  const LineSegment &line);

const bool CheckTwoPoseIsSame(const PathPoint &pose1, const PathPoint &pose2,
                              const double pos_err = 0.01,
                              const double heading_err = 0.8 / 57.3);
std::vector<double> Linspace(const double start, const double stop,
                             const double ds);

std::vector<Eigen::Vector2d> LinSpace(const Eigen::Vector2d &start_pos,
                                      const Eigen::Vector2d &stop_pos,
                                      const double ds);

void PrintPose(const pnc::geometry_lib::PathPoint &pose);
void PrintPose(const std::string &str,
               const pnc::geometry_lib::PathPoint &pose);
void PrintPose(const Eigen::Vector2d &pos, const double heading);
void PrintPose(const std::string &str, const Eigen::Vector2d &pos,
               const double heading);
void PrintSegmentInfo(const pnc::geometry_lib::PathSegment &seg);
void PrintSegmentsVecInfo(
    const std::vector<pnc::geometry_lib::PathSegment> &path_segment_vec);

const double GetTwoPointDist(const PathPoint &start, const PathPoint &end);
const bool CalArcFromPt(const uint8_t gear, const uint8_t steer,
                        const double length, const double radius,
                        const PathPoint pose, PathSegment &arc_seg);

const bool CalPtFromPathSeg(PathPoint &pose, const PathSegment &path_seg,
                            const double length);

const bool IsSameDirGear(const uint8_t gear1, const uint8_t gear2);

const bool IsOppositeDirGear(const uint8_t gear1, const uint8_t gear2);

const bool IsSameSteer(const uint8_t steer1, const uint8_t steer2);

const bool IsOppositeSteer(const uint8_t steer1, const uint8_t steer2);

}  // namespace geometry_lib
}  // namespace pnc

#endif