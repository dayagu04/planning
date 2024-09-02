#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstdint>
#include <iostream>
#include <vector>

#include "apa_param_setting.h"
#include "geometry_math.h"

namespace py = pybind11;
using namespace planning::apa_planner;

const double turn_radius = 5.5;
double slot_side_sign = 1.0;

Eigen::Vector2d pt_0 = Eigen::Vector2d::Zero();
Eigen::Vector2d pt_1 = Eigen::Vector2d::Zero();
Eigen::Vector2d line_tangent_vec;
Eigen::Vector2d line_normal_vec;
pnc::geometry_lib::LineSegment prepare_line;
pnc::geometry_lib::Circle mono_circle;
Eigen::Vector2d tangent_point;
double slot_x_base = 0.0;
void InitTargetInfo() {
  pt_0 << 5.0, 1.4;
  pt_1 << 5.0, -1.4;
  line_normal_vec.setZero();
  line_tangent_vec.setZero();
  prepare_line.Reset();
  mono_circle.Reset();
  tangent_point.setZero();
  slot_x_base = (0.5 * (pt_0 + pt_1)).x();
}

void Update(const double x_offset, const double heading_offset,
            const double length, const double slot_sign) {

  slot_side_sign = slot_sign;
  std::cout << "slot_side_sign = " << slot_side_sign << std::endl;
  if (slot_side_sign > 0.0) {
    pt_0 = pt_1;
  }
  const double start_heading =
      - slot_side_sign * (90.0 - heading_offset) * kDeg2Rad;

  pnc::geometry_lib::PathPoint start_pose;
  start_pose.Set(Eigen::Vector2d(x_offset + slot_x_base, 0.0), start_heading);
  // cal pre line tangent vec and normal vec
  line_tangent_vec = pnc::geometry_lib::GenHeadingVec(start_pose.heading);

  line_normal_vec(line_tangent_vec.y(), -line_tangent_vec.x());

  // sure line_normal_vec towards downward along the x axis.
  if (line_normal_vec.x() > 0.0) {
    line_normal_vec = -1.0 * line_normal_vec;
  }
  // gen prepare line
  prepare_line =
      pnc::geometry_lib::BuildLineSegByPose(start_pose.pos, start_pose.heading);

  prepare_line.pB =
      prepare_line.pA + length * (prepare_line.pB - prepare_line.pA);

  pnc::geometry_lib::PathPoint target_pose;
  if (start_heading >= 0) {
    target_pose.heading = start_heading - 180.0 * kDeg2Rad;
  } else {
    target_pose.heading = start_heading + 180.0 * kDeg2Rad;
  }

  // Calculate Mono Circle
  mono_circle.center.y() = -turn_radius * slot_side_sign;
  const double delta_x = std::sqrt(
      std::pow((turn_radius - apa_param.GetParam().car_width * 0.5 -
                apa_param.GetParam().car_lat_inflation_normal - 0.0268),
               2) -
      std::pow((mono_circle.center.y() - pt_0.y()), 2));

  mono_circle.center.x() = pt_0.x() - delta_x;
  mono_circle.radius = turn_radius;

  // check mono circle if feasible
  const double dist =
      pnc::geometry_lib::CalPoint2LineDist(mono_circle.center, prepare_line);

  std::cout << "dist = " << dist << std::endl;
  if (dist < 0.0) {
    std::cout << "mono_circle = " << mono_circle.center.transpose()
              << std::endl;
    std::cout << "prepare_line = " << prepare_line.pA.transpose()
              << " heading = " << prepare_line.heading * kRad2Deg << std::endl;
    return;
  }
  if (dist < turn_radius) {
    std::cout << "Calculate Failure" << std::endl;
  } else {
    std::cout << "Calculate Success" << std::endl;
    // the length from point_start to point_tangent
    const double length =
        (mono_circle.center.y() - turn_radius * line_normal_vec.y() -
         prepare_line.pA.y()) /
        line_tangent_vec.y();
    tangent_point = prepare_line.pA + length * line_tangent_vec;
  }
}

Eigen::Vector2d GetPt_0() { return pt_0; }
Eigen::Vector2d GetMonoCircleCenter() { return mono_circle.center; }
double GetMonoCircleRadius() { return mono_circle.radius; }
Eigen::Vector2d GetPreLinePointA() { return prepare_line.pA; }
Eigen::Vector2d GetPreLinePointB() { return prepare_line.pB; }
Eigen::Vector2d GetTangentPoint() { return tangent_point; }

PYBIND11_MODULE(prepareplan_py, m) {
  m.doc() = "m";
  m.def("InitTargetInfo", &InitTargetInfo)
      .def("Update", &Update)
      .def("GetPt_0", &GetPt_0)
      .def("GetMonoCircleCenter", &GetMonoCircleCenter)
      .def("GetMonoCircleRadius", &GetMonoCircleRadius)
      .def("GetPreLinePointA", &GetPreLinePointA)
      .def("GetPreLinePointB", &GetPreLinePointB)
      .def("GetTangentPoint", &GetTangentPoint);
}