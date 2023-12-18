#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <vector>

#include "apa_plan_base.h"
#include "geometry_math.h"
#include "math_lib.h"
#include "perpendicular_path_planner.h"

namespace py = pybind11;

template <class T>
inline T BytesToProto(py::bytes& bytes) {
  T proto_obj;
  py::buffer buf(bytes);
  py::buffer_info input_info = buf.request();
  char* input_ptr = static_cast<char*>(input_info.ptr);
  std::string input_s(input_ptr, input_info.size);

  T input;
  input.ParseFromString(input_s);
  return input;
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
  std::vector<planning::apa_planner::PerpendicularPathPlanner::PathSegment>
      line_arc_line_segments;

  planning::apa_planner::PerpendicularPathPlanner::DebugInfo debug_info;

  const pnc::geometry_lib::PathPoint start_pose(Eigen::Vector2d(ego_x, ego_y),
                                                ego_heading);

  const pnc::geometry_lib::PathPoint target_pose(
      Eigen::Vector2d(target_x, target_y), target_heading);

  const uint8_t direction =
      (is_advance ? planning::apa_planner::ApaPlannerBase::DRIVE
                  : planning::apa_planner::ApaPlannerBase::REVERSE);
  const uint8_t steer =
      (is_turn_left ? planning::apa_planner::ApaPlannerBase::LEFT
                    : planning::apa_planner::ApaPlannerBase::RIGHT);

  // planning::apa_planner::PerpendicularPathPlanner perpen_planner;

  // perpen_planner.LineArcLinePlan(line_arc_line_segments, debug_info, start_pose,
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
      .def("GetOutput", &GetOutput);
}