#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "longitudinal_motion_planner.pb.h"
#include "motion_planners/realtime_longitudinal_motion_planner/src/realtime_longitudinal_motion_planning_problem.h"
#include "planning_debug_info.pb.h"

namespace py = pybind11;
using namespace pnc::realtime_longitudinal_planning;

static RealtimeLongitudinalMotionPlanningProblem *pBase = nullptr;

int Init() {
  pBase = new RealtimeLongitudinalMotionPlanningProblem();
  pBase->Init();
  return 0;
}

template <class T>
inline T BytesToProto(py::bytes &bytes) {
  T proto_obj;
  py::buffer buf(bytes);
  py::buffer_info input_info = buf.request();
  char *input_ptr = static_cast<char *>(input_info.ptr);
  std::string input_s(input_ptr, input_info.size);

  T input;
  input.ParseFromString(input_s);
  return input;
}

int UpdateBytes(py::bytes &planning_input_bytes) {
  planning::common::LongitudinalPlanningInput planning_input =
      BytesToProto<planning::common::LongitudinalPlanningInput>(planning_input_bytes);

  pBase->Update(planning_input);

  return 0;
}

py::bytes GetOutputBytes() {
  auto res = pBase->GetOutput();
  std::string serialized_message;
  res.SerializeToString(&serialized_message);

  return serialized_message;
}

int UpdateByParams(py::bytes &planning_input_bytes, double q_ref_pos, double q_ref_vel, double q_acc, double q_jerk,
                   double q_soft_pos_bound, double q_hard_pos_bound, double q_vel_bound, double q_acc_bound, double q_jerk_bound,
                   double q_sv_bound, double q_stop_s) {
  planning::common::LongitudinalPlanningInput planning_input =
      BytesToProto<planning::common::LongitudinalPlanningInput>(planning_input_bytes);
  planning_input.set_q_ref_pos(q_ref_pos);
  planning_input.set_q_ref_vel(q_ref_vel);

  planning_input.set_q_acc(q_acc);
  planning_input.set_q_jerk(q_jerk);

  planning_input.set_q_soft_pos_bound(q_soft_pos_bound);
  planning_input.set_q_hard_pos_bound(q_hard_pos_bound);
  planning_input.set_q_vel_bound(q_vel_bound);
  planning_input.set_q_acc_bound(q_acc_bound);
  planning_input.set_q_jerk_bound(q_jerk_bound);
  planning_input.set_q_sv_bound(q_sv_bound);

  planning_input.set_q_stop_s(q_stop_s);

  pBase->Update(planning_input);

  return 0;
}

PYBIND11_MODULE(realtime_longitudinal_motion_planning_py, m) {
  m.doc() = "m";

  m.def("Init", &Init)
      .def("Update", &UpdateBytes)
      .def("UpdateByParams", &UpdateByParams)
      .def("GetOutputBytes", &GetOutputBytes);
}
