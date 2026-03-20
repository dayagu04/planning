#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <vector>

#include "src/modules/apa_function/parking_scenario/parking_scenario.h"
#include "geometry_math.h"
#include "math_lib.h"
#include "perpendicular_tail_in_path_generator.h"
#include "config_context.h"

namespace py = pybind11;

int Init() {
  // const std::string flag_file_path =
  //     "/asw/planning/res/conf/planning_gflags.conf";
  // google::SetCommandLineOption("flagfile", flag_file_path.c_str());

  // FilePath::SetName("slant_simulation_pybind");
  // InitGlog(FilePath::GetName().c_str());
  (void)planning::common::ConfigurationContext::Instance();

  return 0;
}

static std::vector<double> res;
int UpdateOneStepArcTargetLineByGear(double ego_x, double ego_y,
                                     double ego_heading, double target_x,
                                     double target_y, double target_heading,
                                     bool is_advance) {
  Eigen::Vector2d start_pos(ego_x, ego_y);
  Eigen::Vector2d target_pos(target_x, target_y);

  Eigen::Vector2d target_heading_vec(std::cos(target_heading),
                                     std::sin(target_heading));

  pnc::geometry_lib::Arc arc;

  OneStepArcTargetLineByGear(
      arc, start_pos, ego_heading, is_advance,
      pnc::geometry_lib::LineSegment(
          target_pos, target_pos + target_heading_vec, target_heading));

  res.clear();

  std::cout << "arc.pA = " << arc.pA.transpose() << std::endl;
  std::cout << "arc.pB = " << arc.pB.transpose() << std::endl;

  res.emplace_back(arc.circle_info.center.x());
  res.emplace_back(arc.circle_info.center.y());
  res.emplace_back(arc.circle_info.radius);
  res.emplace_back(arc.pB.x());
  res.emplace_back(arc.pB.y());

  return 0;
}

int UpdateOneStepArcTargetLine(double ego_x, double ego_y, double ego_heading,
                               double target_x, double target_y,
                               double target_heading) {
  Eigen::Vector2d start_pos(ego_x, ego_y);
  Eigen::Vector2d target_pos(target_x, target_y);

  Eigen::Vector2d target_heading_vec(std::cos(target_heading),
                                     std::sin(target_heading));

  pnc::geometry_lib::Arc arc;

  OneStepArcTargetLine(
      arc, start_pos, ego_heading,
      pnc::geometry_lib::LineSegment(
          target_pos, target_pos + target_heading_vec, target_heading));

  res.clear();

  std::cout << "arc.pA = " << arc.pA.transpose() << std::endl;
  std::cout << "arc.pB = " << arc.pB.transpose() << std::endl;

  res.emplace_back(arc.circle_info.center.x());
  res.emplace_back(arc.circle_info.center.y());
  res.emplace_back(arc.circle_info.radius);
  res.emplace_back(arc.pB.x());
  res.emplace_back(arc.pB.y());

  return 0;
}

int UpdateOneStepArcTargetLineGivenTurn(double ego_x, double ego_y,
                                        double ego_heading, double target_x,
                                        double target_y, double target_heading,
                                        bool is_left) {
  Eigen::Vector2d start_pos(ego_x, ego_y);
  Eigen::Vector2d target_pos(target_x, target_y);

  Eigen::Vector2d target_heading_vec(std::cos(target_heading),
                                     std::sin(target_heading));

  pnc::geometry_lib::Arc arc;

  OneStepArcTargetLine(arc, start_pos, ego_heading, is_left,
                       pnc::geometry_lib::LineSegment(
                           target_pos, target_pos + target_heading_vec));

  res.clear();

  std::cout << "arc.pA = " << arc.pA.transpose() << std::endl;
  std::cout << "arc.pB = " << arc.pB.transpose() << std::endl;

  res.emplace_back(arc.circle_info.center.x());
  res.emplace_back(arc.circle_info.center.y());
  res.emplace_back(arc.circle_info.radius);
  res.emplace_back(arc.pB.x());
  res.emplace_back(arc.pB.y());

  return 0;
}

int UpdateOneStepArcTargetHeading(double ego_x, double ego_y,
                                  double ego_heading, double target_x,
                                  double target_y, double target_heading,
                                  bool is_advance) {
  Eigen::Vector2d start_pos(ego_x, ego_y);
  Eigen::Vector2d target_pos(target_x, target_y);

  Eigen::Vector2d target_heading_vec(std::cos(target_heading),
                                     std::sin(target_heading));

  pnc::geometry_lib::Arc arc;

  OneStepArcTargetHeading(arc, start_pos, ego_heading, target_heading, 5.5,
                          is_advance);

  res.clear();

  std::cout << "arc.pA = " << arc.pA.transpose() << std::endl;
  std::cout << "arc.pB = " << arc.pB.transpose() << std::endl;

  res.emplace_back(arc.circle_info.center.x());
  res.emplace_back(arc.circle_info.center.y());
  res.emplace_back(arc.circle_info.radius);
  res.emplace_back(arc.pB.x());
  res.emplace_back(arc.pB.y());

  return 0;
}

std::vector<double> GetOutput() { return res; }

int UpdateOneStepParallelShift(double ego_x, double ego_y, double ego_heading,
                               double target_x, double target_y,
                               bool is_advance, double radius) {
  Eigen::Vector2d start_pos(ego_x, ego_y);
  Eigen::Vector2d target_pos(target_x, target_y);

  Eigen::Vector2d target_pos2 =
      target_pos +
      Eigen::Vector2d(std::cos(ego_heading), std::sin(ego_heading));

  pnc::geometry_lib::LineSegment target_line(target_pos, target_pos2);

  std::pair<pnc::geometry_lib::Arc, pnc::geometry_lib::Arc> arc_pair;

  OneStepParallelShift(arc_pair, start_pos, ego_heading, target_line, radius,
                       is_advance);
  res.clear();

  res.emplace_back(arc_pair.first.circle_info.center.x());
  res.emplace_back(arc_pair.first.circle_info.center.y());
  res.emplace_back(arc_pair.first.circle_info.radius);
  res.emplace_back(arc_pair.first.pB.x());
  res.emplace_back(arc_pair.first.pB.y());

  res.emplace_back(arc_pair.second.circle_info.center.x());
  res.emplace_back(arc_pair.second.circle_info.center.y());
  res.emplace_back(arc_pair.second.circle_info.radius);
  res.emplace_back(arc_pair.second.pB.x());
  res.emplace_back(arc_pair.second.pB.y());

  return 0;
}

const std::vector<Eigen::Vector2d> UpdateSamplePointSet(
    const Eigen::Vector2d point_start, const double heading_start,
    const Eigen::Vector2d point_end, const double heading_end,
    const double radius, const double ds, const double traj_length,
    const int is_anticlockwise) {
  // std::vector<Eigen::Vector2d> point_set;
  // point_set.clear();
  // point_set.reserve(50);
  // pnc::geometry_lib::LineSegment line;
  // line.SetPoints(point_start, point_end);
  // line.heading = heading_start;
  // pnc::geometry_lib::SamplePointSetInLineSeg(point_set, line, ds);
  // return point_set;

  std::vector<pnc::geometry_lib::PathPoint> path_point_set;
  if (pnc::mathlib::IsDoubleEqual(heading_start, heading_end)) {
    pnc::geometry_lib::LineSegment line;
    line.SetPoints(point_start, point_end);
    line.heading = pnc::geometry_lib::NormalizeAngle(heading_start);
    pnc::geometry_lib::SamplePointSetInLineSeg(path_point_set, line, ds);
  } else {
    pnc::geometry_lib::Arc arc;
    double rot_angle = traj_length / radius;
    Eigen::Vector2d unit_tang_vec =
        pnc::geometry_lib::GenHeadingVec(heading_start);
    Eigen::Vector2d unit_norm_vec(-unit_tang_vec.y(), unit_tang_vec.x());
    if (is_anticlockwise == 0) {
      unit_norm_vec = -unit_norm_vec;
      rot_angle = -rot_angle;
      arc.is_anti_clockwise = false;
    }
    Eigen::Vector2d pO = point_start + radius * unit_norm_vec;
    arc.pA = point_start;
    arc.headingA = pnc::geometry_lib::NormalizeAngle(heading_start);
    auto rot_m = pnc::geometry_lib::GetRotm2dFromTheta(rot_angle);
    Eigen::Vector2d OA = arc.pA - pO;
    arc.pB = rot_m * OA + pO;
    arc.headingB = pnc::geometry_lib::NormalizeAngle(arc.headingA + rot_angle);
    arc.circle_info.center = pO;
    arc.circle_info.radius = radius;
    arc.length = traj_length;
    std::cout << "angle" << rot_angle << "  pO = " << pO.transpose()
              << "  pB = " << arc.pB.transpose() << std::endl;
    pnc::geometry_lib::SamplePointSetInArc(path_point_set, arc, ds);
  }
  std::vector<Eigen::Vector2d> point_set;
  point_set.clear();
  point_set.reserve(50);

  for (const auto& path_point : path_point_set) {
    point_set.emplace_back(path_point.pos);
  }
  return point_set;
}

int UpdateLineArcLine(double ego_x, double ego_y, double ego_heading,
                      double target_x, double target_y, double target_heading,
                      bool is_advance, bool is_turn_left, double radius) {
  std::vector<pnc::geometry_lib::PathSegment> line_arc_line_segments;

  planning::apa_planner::PerpendicularTailInPathGenerator::DebugInfo debug_info;

  const pnc::geometry_lib::PathPoint start_pose(Eigen::Vector2d(ego_x, ego_y),
                                                ego_heading);

  const pnc::geometry_lib::PathPoint target_pose(
      Eigen::Vector2d(target_x, target_y), target_heading);

  const uint8_t direction = (is_advance ? pnc::geometry_lib::SEG_GEAR_DRIVE
                                        : pnc::geometry_lib::SEG_GEAR_REVERSE);
  const uint8_t steer = (is_turn_left ? pnc::geometry_lib::SEG_STEER_LEFT
                                      : pnc::geometry_lib::SEG_STEER_RIGHT);

  // planning::apa_planner::PerpendicularTailInPathGenerator perpen_planner;

  // perpen_planner.LineArcLinePlan(line_arc_line_segments, debug_info,
  // start_pose,
  //                                target_pose, direction, steer, radius);
  res.clear();

  for (size_t i = 0; i < debug_info.center_vec.size(); i++) {
    res.emplace_back(debug_info.center_vec[i].x());
    res.emplace_back(debug_info.center_vec[i].y());
    res.emplace_back(radius);
    res.emplace_back(debug_info.tangent_pt_vec[i].first.x());
    res.emplace_back(debug_info.tangent_pt_vec[i].first.y());
    res.emplace_back(debug_info.tangent_pt_vec[i].second.x());
    res.emplace_back(debug_info.tangent_pt_vec[i].second.y());
  }

  return true;
}

const std::vector<Eigen::Vector2d> CalTangCirOfTwoLine(const Eigen::Vector2d p1,
                                                       const double heading1,
                                                       const Eigen::Vector2d p2,
                                                       const double heading2,
                                                       double radius) {
  std::vector<Eigen::Vector2d> possible_centers;
  std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> possible_tangent_pts;

  pnc::geometry_lib::LineSegment line1 =
      pnc::geometry_lib::BuildLineSegByPose(p1, heading1);

  pnc::geometry_lib::LineSegment line2 =
      pnc::geometry_lib::BuildLineSegByPose(p2, heading2);

  pnc::geometry_lib::CalCommonTangentCircleOfTwoLine(
      line1, line2, radius, possible_centers, possible_tangent_pts);

  return possible_centers;
}

const std::vector<Eigen::Vector2d> CalSetTangCirOfTwoLine(
    const Eigen::Vector2d p1, const double heading1, const Eigen::Vector2d p2,
    const double heading2, double radius, bool is_advance, bool is_turn_left) {
  using namespace pnc::geometry_lib;
  using namespace pnc::mathlib;
  std::vector<Eigen::Vector2d> possible_centers;
  std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> possible_tangent_pts;

  LineSegment line1 = BuildLineSegByPose(p1, heading1);

  LineSegment line2 = BuildLineSegByPose(p2, heading2);

  CalCommonTangentCircleOfTwoLine(line1, line2, radius, possible_centers,
                                  possible_tangent_pts);
  std::vector<Eigen::Vector2d> actual_center_info;
  actual_center_info.clear();
  printf("\n\n\n");
  for (size_t i = 0; i < possible_centers.size(); ++i) {
    Arc tmp_arc;
    tmp_arc.pA = possible_tangent_pts[i].first;
    tmp_arc.headingA = line1.heading;
    tmp_arc.pB = possible_tangent_pts[i].second;
    tmp_arc.headingB = line2.heading;
    tmp_arc.circle_info.center = possible_centers[i];
    tmp_arc.circle_info.radius = radius;

    const uint8_t tmp_gear = pnc::geometry_lib::CalArcGear(tmp_arc);
    const uint8_t tmp_steer = pnc::geometry_lib::CalArcSteer(tmp_arc);
    std::cout << "tmp_arc info: pA = " << tmp_arc.pA.transpose()
              << "  headingA = " << tmp_arc.headingA
              << "  pB = " << tmp_arc.pB.transpose()
              << "  headingB = " << tmp_arc.headingB << "  center"
              << tmp_arc.circle_info.center.transpose()
              << "  tmp_gear = " << tmp_gear << "  tmp_steer = " << tmp_steer
              << std::endl;

    if ((is_advance && tmp_gear != pnc::geometry_lib::SEG_GEAR_DRIVE) ||
        (!is_advance && tmp_gear != pnc::geometry_lib::SEG_GEAR_REVERSE) ||
        (is_turn_left && tmp_steer != pnc::geometry_lib::SEG_STEER_LEFT) ||
        (!is_turn_left && tmp_steer != pnc::geometry_lib::SEG_STEER_RIGHT)) {
      std::cout << i << "th arc is not ok\n";
      continue;
    }

    std::cout << i << "th arc is ok\n";

    LineSegment line(line1.pA, possible_tangent_pts[i].first, line1.heading);

    const uint8_t line_gear = pnc::geometry_lib::CalLineSegGear(line);
    if ((is_advance && tmp_gear != pnc::geometry_lib::SEG_GEAR_DRIVE) ||
        (!is_advance && tmp_gear != pnc::geometry_lib::SEG_GEAR_REVERSE)) {
      std::cout << i << "th line is not ok\n";
      continue;
    }

    std::cout << i << "th line is ok\n";

    actual_center_info.emplace_back(possible_centers[i]);
    actual_center_info.emplace_back(possible_tangent_pts[i].first);
    actual_center_info.emplace_back(possible_tangent_pts[i].second);
  }

  return actual_center_info;
}

const std::vector<double> CalTangCircleByPoseAndLine(
    const Eigen::Vector2d p1, const double heading1, const Eigen::Vector2d p2,
    const double heading2, const int is_advance, const int is_turn_left) {
  using namespace pnc::geometry_lib;
  const uint8_t current_gear =
      (is_advance == 1) ? SEG_GEAR_DRIVE : SEG_GEAR_REVERSE;

  const uint8_t current_arc_steer =
      (is_turn_left == 1) ? SEG_STEER_LEFT : SEG_STEER_RIGHT;

  LineSegment line = BuildLineSegByPose(p2, heading2);
  double radius = 0.0;
  Arc arc;
  arc.pA = p1;
  arc.headingA = heading1;
  CalOneArcWithLineAndGear(arc, line, current_gear);
  std::cout << "arc.headingB = " << arc.headingB * 57.3
            << "   line.heading  = " << line.heading * 57.3
            << "  arc.headingA = " << arc.headingA * 57.3
            << "  arc.pA = " << arc.pA.transpose()
            << "  arc.pB = " << arc.pB.transpose()
            << "  arc.center = " << arc.circle_info.center.transpose()
            << "  arc.r = " << arc.circle_info.radius << std::endl;
  std::vector<double> circle_info;
  circle_info.clear();
  if (arc.circle_info.radius > 20.0) {
    return circle_info;
  }
  circle_info.emplace_back(arc.pB[0]);
  circle_info.emplace_back(arc.pB[1]);
  circle_info.emplace_back(arc.circle_info.center[0]);
  circle_info.emplace_back(arc.circle_info.center[1]);
  circle_info.emplace_back(arc.circle_info.radius);
  return circle_info;
}

const Eigen::Vector4d CalOneArcByTargetHeading(
    const Eigen::Vector2d p1, const double heading1, const Eigen::Vector2d p2,
    const double heading2, const double radius, const int is_advance) {
  using namespace pnc::geometry_lib;
  Arc arc;
  arc.pA = p1;
  arc.headingA = heading1;
  arc.circle_info.radius = radius;
  const uint8_t current_gear =
      (is_advance == 1) ? SEG_GEAR_DRIVE : SEG_GEAR_REVERSE;
  CalOneArcWithTargetHeading(arc, current_gear, heading2);

  return Eigen::Vector4d(arc.circle_info.center.x(), arc.circle_info.center.y(),
                         arc.pB.x(), arc.pB.y());
}

const std::vector<double> CalTwoArcBySameHeading(
    const Eigen::Vector2d p1, const double heading1, const Eigen::Vector2d p2,
    const double heading2, const double radius, const int is_advance) {
  using namespace pnc::geometry_lib;
  Arc arc1;
  arc1.pA = p1;
  arc1.headingA = heading1;
  arc1.circle_info.radius = radius;

  Arc arc2;
  arc2.pB = p2;
  arc2.headingB = heading2;
  arc2.circle_info.radius = radius;

  const uint8_t current_gear =
      (is_advance == 1) ? SEG_GEAR_DRIVE : SEG_GEAR_REVERSE;

  std::vector<double> circle_info;
  circle_info.clear();
  if (CalTwoArcWithSameHeading(arc1, arc2, current_gear)) {
    const uint8_t tmp_gear1 = CalArcGear(arc1);
    const uint8_t tmp_gear2 = CalArcGear(arc2);
    const uint8_t tmp_steer1 = CalArcSteer(arc1);
    const uint8_t tmp_steer2 = CalArcSteer(arc2);
    if (tmp_gear1 == current_gear && tmp_gear2 == current_gear &&
        tmp_steer1 != tmp_steer2 &&
        std::fabs(pnc::geometry_lib::NormalizeAngle(
            arc1.headingA - arc1.headingB)) < 0.8 * 3.14) {
      circle_info.emplace_back(arc1.circle_info.center.x());
      circle_info.emplace_back(arc1.circle_info.center.y());
      circle_info.emplace_back(arc2.circle_info.center.x());
      circle_info.emplace_back(arc2.circle_info.center.y());
      circle_info.emplace_back(arc1.pB.x());
      circle_info.emplace_back(arc1.pB.y());
      circle_info.emplace_back(arc1.headingB);
      return circle_info;
    }
  } else {
    std:: cout<< "OneArcPlan fail" << std::endl;
  }
  return circle_info;
}

const std::vector<double> CalTwoArcByLine(
    const Eigen::Vector2d p1, const double heading1, const Eigen::Vector2d p2,
    const double heading2, const double radius, const int is_advance,
    const int is_turn_left) {
  using namespace pnc::geometry_lib;
  const uint8_t current_gear =
      (is_advance == 1) ? SEG_GEAR_DRIVE : SEG_GEAR_REVERSE;

  const uint8_t current_arc_steer =
      (is_turn_left == 1) ? SEG_STEER_LEFT : SEG_STEER_RIGHT;

  const double current_turn_radius = radius;
  const Eigen::Vector2d current_tang_vec =
      pnc::geometry_lib::GetUnitTangVecByHeading(heading1);

  Eigen::Vector2d current_norm_vec;
  if (current_arc_steer == pnc::geometry_lib::SEG_STEER_RIGHT) {
    current_norm_vec << current_tang_vec.y(), -current_tang_vec.x();
  } else if (current_arc_steer == pnc::geometry_lib::SEG_STEER_LEFT) {
    current_norm_vec << -current_tang_vec.y(), current_tang_vec.x();
  }

  const Eigen::Vector2d current_turn_center =
      p1 + current_norm_vec * current_turn_radius;

  Arc arc1;
  arc1.circle_info.center = current_turn_center;
  arc1.circle_info.radius = current_turn_radius;
  arc1.pA = p1;
  arc1.headingA = pnc::geometry_lib::NormalizeAngle(heading1);

  Arc arc2;
  std::vector<double> circle_info;
  circle_info.clear();
  LineSegment line = BuildLineSegByPose(p2, heading2);
  if (CalTwoArcWithLine(arc1, arc2, line)) {
    const uint8_t tmp_gear1 = CalArcGear(arc1);
    const uint8_t tmp_gear2 = CalArcGear(arc2);
    const uint8_t tmp_steer1 = CalArcSteer(arc1);
    const uint8_t tmp_steer2 = CalArcSteer(arc2);
    circle_info.emplace_back(arc1.circle_info.center.x());
    circle_info.emplace_back(arc1.circle_info.center.y());
    circle_info.emplace_back(arc2.circle_info.center.x());
    circle_info.emplace_back(arc2.circle_info.center.y());
    circle_info.emplace_back(arc1.pB.x());
    circle_info.emplace_back(arc1.pB.y());
    circle_info.emplace_back(arc1.headingB);
    circle_info.emplace_back(arc2.pB.x());
    circle_info.emplace_back(arc2.pB.y());
  } else {
    std:: cout<< "OneArcPlan fail" << std::endl;
  }
  return circle_info;
}

const std::vector<double> CalOneArcByLine(
    const Eigen::Vector2d p1, const double heading1, const Eigen::Vector2d p2,
    const double heading2, const double radius, const int is_advance,
    const int is_turn_left) {
  using namespace pnc::geometry_lib;
  const uint8_t current_gear =
      (is_advance == 1) ? SEG_GEAR_DRIVE : SEG_GEAR_REVERSE;

  const uint8_t current_arc_steer =
      (is_turn_left == 1) ? SEG_STEER_LEFT : SEG_STEER_RIGHT;

  const double current_turn_radius = radius;
  const Eigen::Vector2d current_tang_vec =
      pnc::geometry_lib::GetUnitTangVecByHeading(heading1);

  Eigen::Vector2d current_norm_vec;
  if (current_arc_steer == pnc::geometry_lib::SEG_STEER_RIGHT) {
    current_norm_vec << current_tang_vec.y(), -current_tang_vec.x();
  } else if (current_arc_steer == pnc::geometry_lib::SEG_STEER_LEFT) {
    current_norm_vec << -current_tang_vec.y(), current_tang_vec.x();
  }

  const Eigen::Vector2d current_turn_center =
      p1 + current_norm_vec * current_turn_radius;

  Arc arc1;
  arc1.circle_info.center = current_turn_center;
  arc1.circle_info.radius = current_turn_radius;
  arc1.pA = p1;
  arc1.headingA = pnc::geometry_lib::NormalizeAngle(heading1);

  std::vector<double> circle_info;
  circle_info.clear();
  LineSegment line = BuildLineSegByPose(p2, heading2);
  double r_err = 0.036;
  if (CalOneArcWithLine(arc1, line, r_err)) {
    const uint8_t tmp_gear1 = CalArcGear(arc1);
    const uint8_t tmp_steer1 = CalArcSteer(arc1);
    circle_info.emplace_back(arc1.circle_info.center.x());
    circle_info.emplace_back(arc1.circle_info.center.y());
    circle_info.emplace_back(arc1.circle_info.radius);
    circle_info.emplace_back(arc1.pB.x());
    circle_info.emplace_back(arc1.pB.y());
    circle_info.emplace_back(arc1.headingB);
  } else {
    std:: cout<< "OneArcPlan fail" << std::endl;
  }
  return circle_info;
}

const std::vector<double> CalPoint2LineSegDistPb(const Eigen::Vector2d pO,
                                                 const Eigen::Vector2d p1,
                                                 const Eigen::Vector2d p2) {
  pnc::geometry_lib::LineSegment line;
  line.SetPoints(p1, p2);
  std::vector<double> dis_info;
  dis_info.clear();
  const double dis = pnc::geometry_lib::CalPoint2LineSegDist(pO, line);
  dis_info.emplace_back(line.pA.x());
  dis_info.emplace_back(line.pA.y());
  dis_info.emplace_back(line.pB.x());
  dis_info.emplace_back(line.pB.y());
  dis_info.emplace_back(dis);
  return dis_info;
}

PYBIND11_MODULE(geometry_math_validation_py, m) {
  m.doc() = "m";

  m.def("UpdateOneStepArcTargetLine", &UpdateOneStepArcTargetLine)
      .def("UpdateOneStepArcTargetLineGivenTurn",
           &UpdateOneStepArcTargetLineGivenTurn)
      .def("UpdateOneStepArcTargetLineByGear",
           &UpdateOneStepArcTargetLineByGear)
      .def("UpdateOneStepArcTargetHeading", &UpdateOneStepArcTargetHeading)
      .def("UpdateOneStepParallelShift", &UpdateOneStepParallelShift)
      .def("UpdateLineArcLine", &UpdateLineArcLine)
      .def("UpdateSamplePointSet", &UpdateSamplePointSet)
      .def("GetOutput", &GetOutput)
      .def("CalTangCirOfTwoLine", &CalTangCirOfTwoLine)
      .def("CalSetTangCirOfTwoLine", &CalSetTangCirOfTwoLine)
      .def("CalTangCircleByPoseAndLine", &CalTangCircleByPoseAndLine)
      .def("CalOneArcByTargetHeading", &CalOneArcByTargetHeading)
      .def("CalTwoArcBySameHeading", &CalTwoArcBySameHeading)
      .def("CalPoint2LineSegDistPb", &CalPoint2LineSegDistPb)
      .def("CalTwoArcByLine", &CalTwoArcByLine)
      .def("CalOneArcByLine", &CalOneArcByLine);
}