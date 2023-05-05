#include "ilqr_core.h"
#include "longitudinal_motion_planning_problem.h"
#include "longitudinal_motion_planning_pybind_func.hpp"
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
namespace py = pybind11;
using namespace pnc::longitudinal_planning;
using namespace ilqr_solver;

PYBIND11_MODULE(longitudinal_motion_planning_py, m) {

  m.doc() = "m";
  // LongitudinalMotionPlanningProblem
  py::class_<LongitudinalMotionPlanningProblem>(m, "LongitudinalMotionPlanningProblem")
      .def(py::init<>())
      .def("Init", &LongitudinalMotionPlanningProblem::Init)
      .def("Update", &LongitudinalMotionPlanningProblem::Update)
      .def("SetWarmStart", &LongitudinalMotionPlanningProblem::SetWarmStart)
      .def("SetMaxIter", &LongitudinalMotionPlanningProblem::SetMaxIter)
      .def("GetOutput", &LongitudinalMotionPlanningProblem::GetOutput);


  // LongitudinalMotionPlanningInput
  py::class_<LongitudinalMotionPlanningInput>(m, "LongitudinalMotionPlanningInput")
      .def(py::init<>())
      .def_readwrite("init_state", &LongitudinalMotionPlanningInput::init_state)
      .def_readwrite("ref_pos_vec", &LongitudinalMotionPlanningInput::ref_pos_vec)
      .def_readwrite("ref_vel_vec", &LongitudinalMotionPlanningInput::ref_vel_vec)
      .def_readwrite("pos_max_vec", &LongitudinalMotionPlanningInput::pos_max_vec)
      .def_readwrite("pos_min_vec", &LongitudinalMotionPlanningInput::pos_min_vec)
      .def_readwrite("vel_max_vec", &LongitudinalMotionPlanningInput::vel_max_vec)
      .def_readwrite("vel_min_vec", &LongitudinalMotionPlanningInput::vel_min_vec)
      .def_readwrite("acc_max_vec", &LongitudinalMotionPlanningInput::acc_max_vec)
      .def_readwrite("acc_min_vec", &LongitudinalMotionPlanningInput::acc_min_vec)
      .def_readwrite("jerk_max_vec", &LongitudinalMotionPlanningInput::jerk_max_vec)
      .def_readwrite("jerk_min_vec", &LongitudinalMotionPlanningInput::jerk_min_vec)
      .def_readwrite("s_stop", &LongitudinalMotionPlanningInput::s_stop)
      .def_readwrite("q_ref_vel", &LongitudinalMotionPlanningInput::q_ref_vel)
      .def_readwrite("q_acc", &LongitudinalMotionPlanningInput::q_acc)
      .def_readwrite("q_jerk", &LongitudinalMotionPlanningInput::q_jerk)
      .def_readwrite("q_snap", &LongitudinalMotionPlanningInput::q_snap)
      .def_readwrite("q_pos_bound", &LongitudinalMotionPlanningInput::q_pos_bound)
      .def_readwrite("q_vel_bound", &LongitudinalMotionPlanningInput::q_vel_bound)
      .def_readwrite("q_acc_bound", &LongitudinalMotionPlanningInput::q_acc_bound)
      .def_readwrite("q_jerk_bound", &LongitudinalMotionPlanningInput::q_jerk_bound)
      .def_readwrite("q_stop_s", &LongitudinalMotionPlanningInput::q_stop_s);

  // LongitudinalMotionPlanningOutput
  py::class_<LongitudinalMotionPlanningOutput>(m, "LongitudinalMotionPlanningOutput")
      .def(py::init<>())
      .def_readwrite("time_vec", &LongitudinalMotionPlanningOutput::time_vec)
      .def_readwrite("pos_vec", &LongitudinalMotionPlanningOutput::pos_vec)
      .def_readwrite("vel_vec", &LongitudinalMotionPlanningOutput::vel_vec)
      .def_readwrite("acc_vec", &LongitudinalMotionPlanningOutput::acc_vec)
      .def_readwrite("jerk_vec", &LongitudinalMotionPlanningOutput::jerk_vec)
      .def_readwrite("solver_info", &LongitudinalMotionPlanningOutput::solver_info);

  // PybindFuncParam for planning_input hack
  py::class_<PybindFuncParam>(m, "PybindFuncParam")
      .def(py::init<>())
      .def_readwrite("pos", &PybindFuncParam::pos)
      .def_readwrite("vel", &PybindFuncParam::vel)
      .def_readwrite("acc", &PybindFuncParam::acc)
      .def_readwrite("jerk", &PybindFuncParam::jerk)
      .def_readwrite("set_vel", &PybindFuncParam::set_vel)
      .def_readwrite("cipv_vel", &PybindFuncParam::cipv_vel)
      .def_readwrite("cipv_dist", &PybindFuncParam::cipv_dist)
      .def_readwrite("stop_s", &PybindFuncParam::stop_s)
      .def_readwrite("ref_acc_inc", &PybindFuncParam::ref_acc_inc)
      .def_readwrite("ref_acc_dec", &PybindFuncParam::ref_acc_dec)
      .def_readwrite("vel_max", &PybindFuncParam::vel_max)
      .def_readwrite("acc_max", &PybindFuncParam::acc_max)
      .def_readwrite("acc_min", &PybindFuncParam::acc_min)
      .def_readwrite("jerk_max", &PybindFuncParam::jerk_max)
      .def_readwrite("jerk_min", &PybindFuncParam::jerk_min)
      .def_readwrite("stop_enable", &PybindFuncParam::stop_enable)
      .def_readwrite("q_ref_pos", &PybindFuncParam::q_ref_pos)
      .def_readwrite("q_ref_vel", &PybindFuncParam::q_ref_vel)
      .def_readwrite("q_acc", &PybindFuncParam::q_acc)
      .def_readwrite("q_jerk", &PybindFuncParam::q_jerk)
      .def_readwrite("q_snap", &PybindFuncParam::q_snap)
      .def_readwrite("q_pos_bound", &PybindFuncParam::q_pos_bound)
      .def_readwrite("q_vel_bound", &PybindFuncParam::q_vel_bound)
      .def_readwrite("q_acc_bound", &PybindFuncParam::q_acc_bound)
      .def_readwrite("q_jerk_bound", &PybindFuncParam::q_jerk_bound)
      .def_readwrite("q_stop_s", &PybindFuncParam::q_stop_s);

  // LongitudinalMotionPlanningPybindFunc
  py::class_<LongitudinalMotionPlanningPybindFunc>(m, "LongitudinalMotionPlanningPybindFunc")
      .def(py::init<>())
      .def("Update", &LongitudinalMotionPlanningPybindFunc::Update)
      .def("GetOutput", &LongitudinalMotionPlanningPybindFunc::GetOutput);

  py::class_<iLqr::iLqrSolverInfo>(m, "iLqrSolverInfo")
      .def(py::init<>())
      .def_readwrite("solver_condition", &iLqr::iLqrSolverInfo::solver_condition)
      .def_readwrite("cost_size", &iLqr::iLqrSolverInfo::cost_size)
      .def_readwrite("iter_count", &iLqr::iLqrSolverInfo::iter_count)
      .def_readwrite("init_cost", &iLqr::iLqrSolverInfo::init_cost)
      .def_readwrite("cost_iter_vec", &iLqr::iLqrSolverInfo::cost_iter_vec);
}