#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstddef>

#include "lateral_motion_planner.pb.h"
#include "motion_planners/lateral_motion_planner/src/lateral_motion_planning_problem.h"
#include "planning_debug_info.pb.h"

namespace py = pybind11;
using namespace pnc::lateral_planning;

static LateralMotionPlanningProblem *pBase = nullptr;

int Init() {
  pBase = new LateralMotionPlanningProblem();
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
  planning::common::LateralPlanningInput planning_input =
      BytesToProto<planning::common::LateralPlanningInput>(planning_input_bytes);

  pBase->Update(planning_input);

  return 0;
}

py::bytes GetOutputBytes() {
  auto res = pBase->GetOutput();
  std::string serialized_message;
  res.SerializeToString(&serialized_message);

  return serialized_message;
}

int UpdateByParams(py::bytes &planning_input_bytes, double q_ref_xy, double q_ref_theta, double q_acc, double q_jerk,
                   double q_acc_bound, double q_jerk_bound, double acc_bound, double jerk_bound, double q_safe_bound,
                   double q_hard_bound, double upper_safe_bound, double lower_safe_bound) {
  planning::common::LateralPlanningInput planning_input =
      BytesToProto<planning::common::LateralPlanningInput>(planning_input_bytes);
  planning_input.set_acc_bound(acc_bound);
  planning_input.set_jerk_bound(jerk_bound);

  planning_input.set_q_ref_x(q_ref_xy);
  planning_input.set_q_ref_y(q_ref_xy);
  planning_input.set_q_ref_theta(q_ref_theta);
  planning_input.set_q_acc(q_acc);
  planning_input.set_q_jerk(q_jerk);

  planning_input.set_q_acc_bound(q_acc_bound);
  planning_input.set_q_jerk_bound(q_jerk_bound);
  planning_input.set_q_soft_corridor(q_safe_bound);
  planning_input.set_q_hard_corridor(q_hard_bound);

  // tune the path bound and soft bound
  auto N = planning_input.ref_x_vec().size();
  for (size_t i = 0; i < N; i++) {
    Eigen::Vector2d upper_unit_vector(
        planning_input.soft_upper_bound_x0_vec(i) - planning_input.soft_lower_bound_x0_vec(i),
        planning_input.soft_upper_bound_y0_vec(i) - planning_input.soft_lower_bound_y0_vec(i));
    Eigen::Vector2d lower_unit_vector(
        planning_input.soft_lower_bound_x0_vec(i) - planning_input.soft_upper_bound_x0_vec(i),
        planning_input.soft_lower_bound_y0_vec(i) - planning_input.soft_upper_bound_y0_vec(i));

    upper_unit_vector.normalize();
    lower_unit_vector.normalize();
    planning_input.mutable_soft_upper_bound_x0_vec()->Set(
        i, planning_input.soft_upper_bound_x0_vec(i) + upper_unit_vector.x() * upper_safe_bound);
    planning_input.mutable_soft_upper_bound_y0_vec()->Set(
        i, planning_input.soft_upper_bound_y0_vec(i) + upper_unit_vector.y() * upper_safe_bound);
    planning_input.mutable_soft_lower_bound_x0_vec()->Set(
        i, planning_input.soft_lower_bound_x0_vec(i) + lower_unit_vector.x() * lower_safe_bound);
    planning_input.mutable_soft_lower_bound_y0_vec()->Set(
        i, planning_input.soft_lower_bound_y0_vec(i) + lower_unit_vector.y() * lower_safe_bound);
  }

  pBase->Update(planning_input);

  return 0;
}

// planning::common::LateralPlanningOutput GetOutput() {
//   auto res = pBase->GetOutput();

//   return res;
// }

PYBIND11_MODULE(lateral_motion_planning_py, m) {
  m.doc() = "m";

  m.def("Init", &Init)
      .def("Update", &UpdateBytes)
      .def("UpdateByParams", &UpdateByParams)
      .def("GetOutputBytes", &GetOutputBytes);
}
