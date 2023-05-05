#include "acado_lat_mpc.h"
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
namespace py = pybind11;
using namespace pnc::control;


PYBIND11_MODULE(mpc_common, m) {

  m.doc() = "mpc common";
  py::class_<LatMpcState>(m, "LatMpcState")
      .def(py::init<>())
      .def_readwrite("dx_", &LatMpcState::dx_)
      .def_readwrite("dy_", &LatMpcState::dy_)
      .def_readwrite("dphi_", &LatMpcState::dphi_)
      .def_readwrite("delta_", &LatMpcState::delta_);

  py::enum_<LatMpcStatus>(m, "LatMpcStatus")
      .value("MPC_SOLVE_SUCCESS", MPC_SOLVE_SUCCESS)
      .value("MPC_FAULT_SIZE", MPC_FAULT_SIZE)
      .value("MPC_SOLVE_FAIL", MPC_SOLVE_FAIL)
      .export_values();

  py::class_<LatMpcInput>(m, "LatMpcInput")
      .def(py::init<>())
      .def_readwrite("q_y_", &LatMpcInput::q_y_)
      .def_readwrite("q_phi_", &LatMpcInput::q_phi_)
      .def_readwrite("q_omega_", &LatMpcInput::q_omega_)
      .def_readwrite("q_phi_WN_", &LatMpcInput::q_phi_WN_)
      .def_readwrite("omega_limit_", &LatMpcInput::omega_limit_)
      .def_readwrite("delta_limit_", &LatMpcInput::delta_limit_)
      .def_readwrite("curv_factor_", &LatMpcInput::curv_factor_)
      .def_readwrite("curv_ref_factor_", &LatMpcInput::curv_ref_factor_)
      .def_readwrite("init_state_", &LatMpcInput::init_state_)
      .def_readwrite("dx_ref_mpc_vec_", &LatMpcInput::dx_ref_mpc_vec_)
      .def_readwrite("dy_ref_mpc_vec_", &LatMpcInput::dy_ref_mpc_vec_)
      .def_readwrite("dphi_ref_mpc_vec_", &LatMpcInput::dphi_ref_mpc_vec_)
      .def_readwrite("vel_lat_ctrl_mpc_vec_", &LatMpcInput::vel_lat_ctrl_mpc_vec_);

  py::class_<LatMpcOutput>(m, "LatMpcOutput")
      .def(py::init<>())
      .def_readwrite("time_vec_mpc_", &LatMpcOutput::time_vec_mpc_)
      .def_readwrite("dx_vec_mpc_", &LatMpcOutput::dx_vec_mpc_)
      .def_readwrite("dy_vec_mpc_", &LatMpcOutput::dy_vec_mpc_)
      .def_readwrite("dphi_vec_mpc_", &LatMpcOutput::dphi_vec_mpc_)
      .def_readwrite("delta_vec_mpc_", &LatMpcOutput::delta_vec_mpc_)
      .def_readwrite("omega_vec_mpc_", &LatMpcOutput::omega_vec_mpc_)
      .def_readwrite("status_", &LatMpcOutput::status_)
      .def_readwrite("iteration_", &LatMpcOutput::iteration_);
}

PYBIND11_MODULE(acado_lat_mpc_py, m) {

  m.doc() = "bind acado_lat_mpc";
  py::class_<LatMpc>(m, "LatMpc")
      .def(py::init<>())
      .def("Init", &LatMpc::Init)
      .def("Update", &LatMpc::Update)
      .def("GetMpcOutput", &LatMpc::GetMpcOutput);
}