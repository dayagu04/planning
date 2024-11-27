#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstddef>
#include <cstdint>

#include "apa_function/parking_task/optimizers/lateral_path_optimizer/src/lateral_path_optimizer_problem.h"
#include "geometry_math.h"
#include "lateral_path_optimizer.pb.h"
#include "math_lib.h"

#include "serialize_utils.h"

namespace py = pybind11;
using namespace planning::apa_planner;

static LateralPathOptimizerProblem *pABase = nullptr;
static LateralPathOptimizerProblem *pBBase = nullptr;
static bool c_ilqr_enable = false;
//  create init data
static std::vector<double> x_vec;
static std::vector<double> y_vec;
static std::vector<double> heading_vec;
static std::vector<double> curvature_vec;
static double length = 0.0;

int SamplingOneSeg(const double start_x, const double start_y,
                   const double start_heading, const double end_x,
                   const double end_y, const double end_heading,
                   const double center_x, const double center_y,
                   const double radius, const double ds) {
  pnc::geometry_lib::PathPoint start_pt(Eigen::Vector2d(start_x, start_y),
                                        start_heading);

  pnc::geometry_lib::PathPoint end_pt(Eigen::Vector2d(end_x, end_y),
                                      end_heading);
  Eigen::Vector2d center(center_x, center_y);
  // sampling
  pnc::geometry_lib::PathPoint path_point;
  // get first point
  path_point.Set(start_pt.pos,
                 pnc::geometry_lib::NormalizeAngle(start_pt.heading));

  const double kappa = 1.0 / radius;
  x_vec.emplace_back(path_point.pos.x());
  y_vec.emplace_back(path_point.pos.y());
  heading_vec.emplace_back(path_point.heading);
  curvature_vec.emplace_back(kappa);

  const auto theta_arc = pnc::geometry_lib::GetAngleFromTwoVec(
      start_pt.pos - center, end_pt.pos - center);

  bool is_anti_clockwise = (theta_arc > 0.0);
  const double seg_length = std::fabs(theta_arc) * radius;

  const auto &pO = Eigen::Vector2d(center_x, center_y);
  double theta = start_heading;

  const double dtheta = ds / radius * (is_anti_clockwise ? 1.0 : -1.0);

  const auto rot_m = pnc::geometry_lib::GetRotm2dFromTheta(dtheta);
  Eigen::Vector2d pn = start_pt.pos;
  Eigen::Vector2d v_n = start_pt.pos - center;

  double s = ds;
  while (s < seg_length) {
    v_n = rot_m * v_n;
    pn = pO + v_n;
    s += ds;
    theta += dtheta;

    path_point.Set(pn, pnc::geometry_lib::NormalizeAngle(theta));
    x_vec.emplace_back(path_point.pos.x());
    y_vec.emplace_back(path_point.pos.y());
    heading_vec.emplace_back(path_point.heading);
    curvature_vec.emplace_back(kappa);
  }
  return 0;
}

int GenPathPoints(double ds) {
  const double radius = 5.5;
  const bool is_advance = false;
  const pnc::geometry_lib::PathPoint start_pos(Eigen::Vector2d(0.0, 0.0), 0.0);
  const pnc::geometry_lib::PathPoint target_pos(Eigen::Vector2d(0.0, 0.5), 0.0);

  const Eigen::Vector2d target_pos2 =
      target_pos.pos + Eigen::Vector2d(std::cos(target_pos.heading),
                                       std::sin(target_pos.heading));

  const pnc::geometry_lib::LineSegment target_line(target_pos.pos, target_pos2);
  std::pair<pnc::geometry_lib::Arc, pnc::geometry_lib::Arc> arc_pair;

  OneStepParallelShift(arc_pair, start_pos.pos, start_pos.heading, target_line,
                       radius, is_advance);
  x_vec.clear();
  y_vec.clear();
  heading_vec.clear();
  curvature_vec.clear();

  length = arc_pair.first.length + arc_pair.second.length;

  SamplingOneSeg(arc_pair.first.pA.x(), arc_pair.first.pA.y(),
                 arc_pair.first.headingA, arc_pair.first.pB.x(),
                 arc_pair.first.pB.y(), arc_pair.first.headingB,
                 arc_pair.first.circle_info.center.x(),
                 arc_pair.first.circle_info.center.y(), radius, ds);

  SamplingOneSeg(arc_pair.second.pA.x(), arc_pair.second.pA.y(),
                 arc_pair.second.headingA, arc_pair.second.pB.x(),
                 arc_pair.second.pB.y(), arc_pair.second.headingB,
                 arc_pair.second.circle_info.center.x(),
                 arc_pair.second.circle_info.center.y(), radius, ds);

  x_vec.emplace_back(arc_pair.second.pB.x());
  y_vec.emplace_back(arc_pair.second.pB.y());
  heading_vec.emplace_back(arc_pair.second.headingB);
  curvature_vec.emplace_back(1.0 / arc_pair.second.circle_info.radius);

  return x_vec.size();
}

std::vector<double> GetXVec() { return x_vec; }
std::vector<double> GetYVec() { return y_vec; }
std::vector<double> GetHeadingVec() { return heading_vec; }
std::vector<double> GetCurvatureVec() { return curvature_vec; }
double GetLength() { return length; }

// ilqr optimizer
int Init() {
  pABase = new LateralPathOptimizerProblem();
  pABase->Init(true);

  pBBase = new LateralPathOptimizerProblem();
  pBBase->Init(false);
  return 0;
}

int UpdateBytes(py::bytes &planning_input_bytes) {
  planning::common::LateralPathOptimizerInput planning_input =
      BytesToProto<planning::common::LateralPathOptimizerInput>(
          planning_input_bytes);

  uint8_t gear_cmd = 0;
  if (c_ilqr_enable) {
    pABase->Update(planning_input, gear_cmd);
  } else {
    pBBase->Update(planning_input, gear_cmd);
  }
  return 0;
}

py::bytes GetOutputBytes() {
  planning::common::LateralPathOptimizerOutput res;
  if (c_ilqr_enable) {
    res = pABase->GetOutput();
  } else {
    res = pBBase->GetOutput();
  }
  std::string serialized_message;
  res.SerializeToString(&serialized_message);

  return serialized_message;
}

int UpdateByParams(py::bytes &planning_input_bytes, double q_ref_x,
                   double q_ref_y, double q_ref_theta, double q_terminal_theta,
                   double q_terminal_xy, double q_k, double q_u,
                   double q_k_bound, double q_u_bound) {
  planning::common::LateralPathOptimizerInput planning_input =
      BytesToProto<planning::common::LateralPathOptimizerInput>(
          planning_input_bytes);

  planning_input.set_q_ref_x(q_ref_x);
  planning_input.set_q_ref_y(q_ref_y);
  planning_input.set_q_ref_theta(q_ref_theta);
  planning_input.set_q_terminal_theta(q_terminal_theta);
  planning_input.set_q_terminal_x(q_terminal_xy);
  planning_input.set_q_terminal_y(q_terminal_xy);
  planning_input.set_q_k(q_k);
  planning_input.set_q_u(q_u);
  planning_input.set_q_k_bound(q_k_bound);
  planning_input.set_q_u_bound(q_u_bound);
  uint8_t gear_cmd = 0;
  bool is_cilqr_enable = false;
  c_ilqr_enable = is_cilqr_enable;
  if (is_cilqr_enable) {
    pABase->Update(planning_input, gear_cmd);
  } else {
    pBBase->Update(planning_input, gear_cmd);
  }
  return 0;
}

PYBIND11_MODULE(lateral_path_optimizer_py, m) {
  m.doc() = "m";

  m.def("Init", &Init)
      .def("SamplingOneSeg", &SamplingOneSeg)
      .def("GenPathPoints", &GenPathPoints)
      .def("GetXVec", &GetXVec)
      .def("GetYVec", &GetYVec)
      .def("GetHeadingVec", &GetHeadingVec)
      .def("GetCurvatureVec", &GetCurvatureVec)
      .def("GetLength", &GetLength)
      .def("UpdateByParams", &UpdateByParams)
      .def("GetOutputBytes", &GetOutputBytes);
}