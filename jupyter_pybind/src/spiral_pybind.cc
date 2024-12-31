#include <gflags/gflags.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstdint>
#include <iostream>
#include <vector>

#include "ifly_time.h"
#include "log_glog.h"
#include "src/library/spiral/spiral_path.h"

using namespace planning;

void Init() {
  planning::FilePath::SetName("spiral_py");
  planning::InitGlog(planning::FilePath::GetName().c_str());
}

std::vector<Eigen::Vector4d> CubicSpiralStrictSolvePybind(
    std::vector<double> input) {
  bool ret = false;
  solution_cubic_t sol;
  spiral_path_point_t start;
  start.x = input[0];
  start.y = input[1];
  start.theta = input[2] / 180 * M_PI;
  start.kappa = input[3];
  start.dir = VEHICLE_MOVE_DIR_FORWARD;
  spiral_path_point_t goal;
  goal.x = input[4];
  goal.y = input[5];
  goal.theta = input[6] / 180 * M_PI;
  goal.kappa = input[7];
  goal.dir = VEHICLE_MOVE_DIR_FORWARD;

  ret = CubicSpiralStrictSolve(&sol, &start, &goal);
  double step_length = 0.1;
  bool solution_usable = (bool)(sol.solve_status);
  std::vector<spiral_path_point_t> states;

  if (solution_usable) /* usable */
  {
    ret = SampleCubicSpiralStatesBySol(states, &sol, step_length);

    std::cout << "ret " << static_cast<int>(ret) << std::endl;
  }

  std::vector<Eigen::Vector4d> path_point_vec;

  for (size_t i = 0; i < states.size(); i++) {
    spiral_path_point_t state = states[i];
    Eigen::Vector4d tmep_point;
    tmep_point << state.x, state.y, state.theta, state.kappa;
    path_point_vec.emplace_back(tmep_point);
  }
  return path_point_vec;
}

std::vector<Eigen::Vector4d> CubicSpiralStartkFreeSolvePybind(
    std::vector<double> input) {
  bool ret = false;
  solution_cubic_t sol;
  spiral_path_point_t start;
  start.x = input[0];
  start.y = input[1];
  start.theta = input[2] / 180 * M_PI;
  start.kappa = input[3];
  start.dir = VEHICLE_MOVE_DIR_FORWARD;
  spiral_path_point_t goal;
  goal.x = input[4];
  goal.y = input[5];
  goal.theta = input[6] / 180 * M_PI;
  goal.kappa = input[7];
  goal.dir = VEHICLE_MOVE_DIR_FORWARD;

  ret = CubicSpiralStartkFreeSolve(&sol, &start, &goal);
  double step_length = 0.1;
  bool solution_usable = (bool)(sol.solve_status);
  std::vector<spiral_path_point_t> states;

  if (solution_usable) /* usable */
  {
    ret = SampleCubicSpiralStatesBySol(states, &sol, step_length);

    std::cout << "Cubic Spiral Start k Free Solve ret " << static_cast<int>(ret)
              << std::endl;
  }

  std::vector<Eigen::Vector4d> path_point_vec;

  for (size_t i = 0; i < states.size(); i++) {
    spiral_path_point_t state = states[i];
    Eigen::Vector4d tmep_point;
    tmep_point << state.x, state.y, state.theta, state.kappa;
    path_point_vec.emplace_back(tmep_point);
  }
  return path_point_vec;
}

std::vector<Eigen::Vector4d> CubicSpiralEndkFreeSolvePybind(
    std::vector<double> input) {
  bool ret = false;
  solution_cubic_t sol;
  spiral_path_point_t start;
  start.x = input[0];
  start.y = input[1];
  start.theta = input[2] / 180 * M_PI;
  start.kappa = input[3];
  start.dir = VEHICLE_MOVE_DIR_FORWARD;
  spiral_path_point_t goal;
  goal.x = input[4];
  goal.y = input[5];
  goal.theta = input[6] / 180 * M_PI;
  goal.kappa = input[7];
  goal.dir = VEHICLE_MOVE_DIR_FORWARD;

  ret = CubicSpiralEndkFreeSolve(&sol, &start, &goal);
  double step_length = 0.1;
  bool solution_usable = (bool)(sol.solve_status);
  std::vector<spiral_path_point_t> states;

  if (solution_usable) /* usable */
  {
    ret = SampleCubicSpiralStatesBySol(states, &sol, step_length);

    std::cout << "Cubic Spiral End k Free Solve ret " << static_cast<int>(ret)
              << std::endl;
  }

  std::vector<Eigen::Vector4d> path_point_vec;

  for (size_t i = 0; i < states.size(); i++) {
    spiral_path_point_t state = states[i];
    Eigen::Vector4d tmep_point;
    tmep_point << state.x, state.y, state.theta, state.kappa;
    path_point_vec.emplace_back(tmep_point);
  }
  return path_point_vec;
}

std::vector<Eigen::Vector4d> CubicSpiralBothkFreeSolvePybind(
    std::vector<double> input) {
  bool ret = false;
  solution_cubic_t sol;
  spiral_path_point_t start;
  start.x = input[0];
  start.y = input[1];
  start.theta = input[2] / 180 * M_PI;
  start.kappa = input[3];
  start.dir = VEHICLE_MOVE_DIR_FORWARD;
  spiral_path_point_t goal;
  goal.x = input[4];
  goal.y = input[5];
  goal.theta = input[6] / 180 * M_PI;
  goal.kappa = input[7];
  goal.dir = VEHICLE_MOVE_DIR_FORWARD;

  ret = CubicSpiralBothkFreeSolve(&sol, &start, &goal);
  double step_length = 0.1;
  bool solution_usable = (bool)(sol.solve_status);
  std::vector<spiral_path_point_t> states;

  if (solution_usable) /* usable */
  {
    ret = SampleCubicSpiralStatesBySol(states, &sol, step_length);

    std::cout << "Cubic Spiral Both k Free Solve ret " << static_cast<int>(ret)
              << std::endl;
  }

  std::vector<Eigen::Vector4d> path_point_vec;

  for (size_t i = 0; i < states.size(); i++) {
    spiral_path_point_t state = states[i];
    Eigen::Vector4d tmep_point;
    tmep_point << state.x, state.y, state.theta, state.kappa;
    path_point_vec.emplace_back(tmep_point);
  }
  return path_point_vec;
}

PYBIND11_MODULE(spiral_py, m) {
  m.doc() = "m";

  m.def("Init", Init)
      .def("CubicSpiralStrictSolvePybind", &CubicSpiralStrictSolvePybind)
      .def("CubicSpiralStartkFreeSolvePybind", CubicSpiralStartkFreeSolvePybind)
      .def("CubicSpiralEndkFreeSolvePybind", CubicSpiralEndkFreeSolvePybind)
      .def("CubicSpiralBothkFreeSolvePybind", CubicSpiralBothkFreeSolvePybind);
}