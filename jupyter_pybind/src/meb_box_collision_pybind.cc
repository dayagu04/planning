#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "meb_function/meb_box_collision_lib.h"

namespace py = pybind11;
using namespace adas_function;

PYBIND11_MODULE(meb_box_collision_py, m) {
  m.doc() = "MEB Box Collision Library Pybind Wrapper";

  py::class_<CalculateETTCInputStr>(m, "CalculateETTCInputStr")
      .def(py::init<>())
      .def_readwrite("ego_width", &CalculateETTCInputStr::ego_width)
      .def_readwrite("ego_length", &CalculateETTCInputStr::ego_length)
      .def_readwrite("ego_backshaft_2_fbumper",
                     &CalculateETTCInputStr::ego_backshaft_2_fbumper)
      .def_readwrite("ego_v_x", &CalculateETTCInputStr::ego_v_x)
      .def_readwrite("ego_a_x", &CalculateETTCInputStr::ego_a_x)
      .def_readwrite("ego_radius", &CalculateETTCInputStr::ego_radius)
      .def_readwrite("obj_x", &CalculateETTCInputStr::obj_x)
      .def_readwrite("obj_y", &CalculateETTCInputStr::obj_y)
      .def_readwrite("obj_v_x", &CalculateETTCInputStr::obj_v_x)
      .def_readwrite("obj_v_y", &CalculateETTCInputStr::obj_v_y)
      .def_readwrite("obj_a_x", &CalculateETTCInputStr::obj_a_x)
      .def_readwrite("obj_a_y", &CalculateETTCInputStr::obj_a_y)
      .def_readwrite("obj_width", &CalculateETTCInputStr::obj_width)
      .def_readwrite("obj_length", &CalculateETTCInputStr::obj_length)
      .def_readwrite("obj_heading_angle",
                     &CalculateETTCInputStr::obj_heading_angle);

  py::class_<DebugFrame>(m, "DebugFrame")
      .def(py::init<>())
      .def_readwrite("time", &DebugFrame::time)
      .def_readwrite("ego_x", &DebugFrame::ego_x)
      .def_readwrite("ego_y", &DebugFrame::ego_y)
      .def_readwrite("ego_heading", &DebugFrame::ego_heading)
      .def_readwrite("ego_v", &DebugFrame::ego_v)
      .def_readwrite("ego_a", &DebugFrame::ego_a)
      .def_readwrite("obj_x", &DebugFrame::obj_x)
      .def_readwrite("obj_y", &DebugFrame::obj_y)
      .def_readwrite("obj_heading", &DebugFrame::obj_heading)
      .def_readwrite("obj_vx", &DebugFrame::obj_vx)
      .def_readwrite("obj_vy", &DebugFrame::obj_vy)
      .def_readwrite("collision", &DebugFrame::collision);

  py::class_<BoxCollisonLib>(m, "BoxCollisonLib")
      .def(py::init<>())
      .def_readwrite("boxs_info_", &BoxCollisonLib::boxs_info_)
      .def_readwrite("t_start", &BoxCollisonLib::t_start)
      .def_readwrite("t_end", &BoxCollisonLib::t_end)
      .def_readwrite("dec_request", &BoxCollisonLib::dec_request)
      .def_readwrite("time_dealy", &BoxCollisonLib::time_dealy)
      .def_readwrite("time_step", &BoxCollisonLib::time_step)
      .def_readwrite("debug_trace_", &BoxCollisonLib::debug_trace_)
      .def("GetCollisionResultBySimEgoDec",
           &BoxCollisonLib::GetCollisionResultBySimEgoDec)
      .def("GetCollisionResultBySimEgoSteerAngleSpeed",
           &BoxCollisonLib::GetCollisionResultBySimEgoSteerAngleSpeed);
}
