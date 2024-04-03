#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstddef>
#include <vector>
#include "jerk_limited_trajectory.h"
#include "jerk_limited_trajectory_define.h"

namespace py = pybind11;
using namespace planning;

static planning::jlt::JerkLimitedTrajectory *pBase = nullptr;

int Init() {
  pBase = new jlt::JerkLimitedTrajectory();
  return 0;
}

int UpdateByParams(double s0, double v0, double a0, double s_des, double v_des,
                   double v_min, double v_max, double a_min, double a_max,
                   double j_min, double j_max) {
  planning::jlt::StateLimitParam state_limit;
  planning::jlt::PointState init_point_state;
  state_limit.v_max = v_max;
  state_limit.v_min = v_min;
  state_limit.a_max = a_max;
  state_limit.a_min = a_min;
  state_limit.j_max = j_max;
  state_limit.j_min = j_min;
  state_limit.p_desire = s_des;
  state_limit.v_desire = v_des;

  init_point_state.p = s0;
  init_point_state.v = v0;
  init_point_state.a = a0;

  pBase->Update(init_point_state, state_limit, planning::jlt::SOLVE_VEL, 0.1);

  return 0;
}

double GetTotalTime() {
  auto time = pBase->ParamLength();
  return time;
}

std::vector<double> GetSOutput() {
  auto p = pBase->GetSCurve();
  return p;
}

std::vector<double> GetVelOutput() {
  auto v = pBase->GetVelCurve();
  return v;
}

std::vector<double> GetAccOutput() {
  auto a = pBase->GetAccCurve();
  return a;
}

std::vector<double> GetJerkOutput() {
  auto j = pBase->GetJerkCurve();
  return j;
}

PYBIND11_MODULE(jerk_limited_trajectory_py, m) {
  m.doc() = "m";

  m.def("Init", Init)
      .def("UpdateByParams", &UpdateByParams)
      .def("GetTotalTime", &GetTotalTime)
      .def("GetSOutput", &GetSOutput)
      .def("GetVelOutput", &GetVelOutput)
      .def("GetAccOutput", &GetAccOutput)
      .def("GetJerkOutput", &GetJerkOutput);
}