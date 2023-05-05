#include "ilqr_core.h"
#include "lateral_motion_planning_problem.h"
#include "lateral_motion_planning_pybind_func.hpp"
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
namespace py = pybind11;
using namespace pnc::lateral_planning;
using namespace ilqr_solver;

PYBIND11_MODULE(lateral_motion_planning_py, m) {

  m.doc() = "m";
  // LateralMotionPlanningProblem
  py::class_<LateralMotionPlanningProblem>(m, "LateralMotionPlanningProblem")
      .def(py::init<>())
      .def("Init", &LateralMotionPlanningProblem::Init)
      .def("Update", &LateralMotionPlanningProblem::Update)
      .def("SetWarmStart", &LateralMotionPlanningProblem::SetWarmStart)
      .def("SetMaxIter", &LateralMotionPlanningProblem::SetMaxIter)
      .def("GetOutput", &LateralMotionPlanningProblem::GetOutput);


  // LateralMotionPlanningInput
  py::class_<LateralMotionPlanningInput>(m, "LateralMotionPlanningInput")
      .def(py::init<>())
      .def_readwrite("init_state", &LateralMotionPlanningInput::init_state)
      .def_readwrite("ref_x_vec", &LateralMotionPlanningInput::ref_x_vec)
      .def_readwrite("ref_y_vec", &LateralMotionPlanningInput::ref_y_vec)
      .def_readwrite("ref_theta_vec",
                     &LateralMotionPlanningInput::ref_theta_vec)
      .def_readwrite("last_x_vec", &LateralMotionPlanningInput::last_x_vec)
      .def_readwrite("last_y_vec", &LateralMotionPlanningInput::last_y_vec)
      .def_readwrite("last_theta_vec",
                     &LateralMotionPlanningInput::last_theta_vec)
      .def_readwrite("ref_vel", &LateralMotionPlanningInput::ref_vel)
      .def_readwrite("curv_factor", &LateralMotionPlanningInput::curv_factor)
      .def_readwrite("soft_upper_bound_x0_vec",
                     &LateralMotionPlanningInput::soft_upper_bound_x0_vec)
      .def_readwrite("soft_upper_bound_y0_vec",
                     &LateralMotionPlanningInput::soft_upper_bound_y0_vec)
      .def_readwrite("soft_upper_bound_x1_vec",
                     &LateralMotionPlanningInput::soft_upper_bound_x1_vec)
      .def_readwrite("soft_lower_bound_x0_vec",
                     &LateralMotionPlanningInput::soft_lower_bound_x0_vec)
      .def_readwrite("soft_lower_bound_y0_vec",
                     &LateralMotionPlanningInput::soft_lower_bound_y0_vec)
      .def_readwrite("soft_lower_bound_x1_vec",
                     &LateralMotionPlanningInput::soft_lower_bound_x1_vec)
      .def_readwrite("soft_lower_bound_y1_vec",
                     &LateralMotionPlanningInput::soft_lower_bound_y1_vec)
      .def_readwrite("acc_bound", &LateralMotionPlanningInput::acc_bound)
      .def_readwrite("jerk_bound", &LateralMotionPlanningInput::jerk_bound)
      .def_readwrite("q_ref_x", &LateralMotionPlanningInput::q_ref_x)
      .def_readwrite("q_ref_y", &LateralMotionPlanningInput::q_ref_y)
      .def_readwrite("q_ref_theta", &LateralMotionPlanningInput::q_ref_theta)
      .def_readwrite("q_continuity", &LateralMotionPlanningInput::q_continuity)
      .def_readwrite("q_acc", &LateralMotionPlanningInput::q_acc)
      .def_readwrite("q_jerk", &LateralMotionPlanningInput::q_jerk)
      .def_readwrite("q_acc_bound",
                     &LateralMotionPlanningInput::q_acc_bound)
      .def_readwrite("q_jerk_bound",
                     &LateralMotionPlanningInput::q_jerk_bound)
      .def_readwrite("q_soft_corridor",
                     &LateralMotionPlanningInput::q_soft_corridor)
      .def_readwrite("q_hard_corridor",
                     &LateralMotionPlanningInput::q_hard_corridor);

  // LateralMotionPlanningOutput
  py::class_<LateralMotionPlanningOutput>(m, "LateralMotionPlanningOutput")
      .def(py::init<>())
      .def_readwrite("time_vec", &LateralMotionPlanningOutput::time_vec)
      .def_readwrite("x_vec", &LateralMotionPlanningOutput::x_vec)
      .def_readwrite("y_vec", &LateralMotionPlanningOutput::y_vec)
      .def_readwrite("theta_vec", &LateralMotionPlanningOutput::theta_vec)
      .def_readwrite("delta_vec", &LateralMotionPlanningOutput::delta_vec)
      .def_readwrite("omega_vec", &LateralMotionPlanningOutput::omega_vec)
      .def_readwrite("acc_vec", &LateralMotionPlanningOutput::acc_vec)
      .def_readwrite("jerk_vec", &LateralMotionPlanningOutput::jerk_vec)
      .def_readwrite("solver_info", &LateralMotionPlanningOutput::solver_info);

  // PybindFuncParam for planning_input hack
  py::class_<PybindFuncParam>(m, "PybindFuncParam")
      .def(py::init<>())
      .def_readwrite("pos_x", &PybindFuncParam::pos_x)
      .def_readwrite("pos_y", &PybindFuncParam::pos_y)
      .def_readwrite("theta", &PybindFuncParam::theta)
      .def_readwrite("delta", &PybindFuncParam::delta)
      .def_readwrite("set_vel_gain", &PybindFuncParam::set_vel_gain)
      .def_readwrite("ref_offset", &PybindFuncParam::ref_offset)
      .def_readwrite("last_offset", &PybindFuncParam::last_offset)
      .def_readwrite("corridor_upper_offset",
                     &PybindFuncParam::corridor_upper_offset)
      .def_readwrite("corridor_lower_offset",
                     &PybindFuncParam::corridor_lower_offset)
      .def_readwrite("acc_bound", &PybindFuncParam::acc_bound)
      .def_readwrite("jerk_bound", &PybindFuncParam::jerk_bound)
      .def_readwrite("q_ref_x", &PybindFuncParam::q_ref_x)
      .def_readwrite("q_ref_y", &PybindFuncParam::q_ref_y)
      .def_readwrite("q_ref_theta", &PybindFuncParam::q_ref_theta)
      .def_readwrite("q_continuity", &PybindFuncParam::q_continuity)
      .def_readwrite("q_acc", &PybindFuncParam::q_acc)
      .def_readwrite("q_jerk", &PybindFuncParam::q_jerk)
      .def_readwrite("q_snap", &PybindFuncParam::q_snap)
      .def_readwrite("q_acc_bound", &PybindFuncParam::q_acc_bound)
      .def_readwrite("q_jerk_bound", &PybindFuncParam::q_jerk_bound)
      .def_readwrite("q_soft_corridor", &PybindFuncParam::q_soft_corridor)
      .def_readwrite("q_hard_corridor", &PybindFuncParam::q_hard_corridor);

  // LateralMotionPlanningPybindFunc
  py::class_<LateralMotionPlanningPybindFunc>(m,
                                              "LateralMotionPlanningPybindFunc")
      .def(py::init<>())
      .def("Update", &LateralMotionPlanningPybindFunc::Update)
      .def("GetOutput", &LateralMotionPlanningPybindFunc::GetOutput);

  py::class_<iLqr::iLqrSolverInfo>(m, "iLqrSolverInfo")
      .def(py::init<>())
      .def_readwrite("solver_condition", &iLqr::iLqrSolverInfo::solver_condition)
      .def_readwrite("cost_size", &iLqr::iLqrSolverInfo::cost_size)
      .def_readwrite("iter_count", &iLqr::iLqrSolverInfo::iter_count)
      .def_readwrite("init_cost", &iLqr::iLqrSolverInfo::init_cost)
      .def_readwrite("cost_iter_vec", &iLqr::iLqrSolverInfo::cost_iter_vec);
}