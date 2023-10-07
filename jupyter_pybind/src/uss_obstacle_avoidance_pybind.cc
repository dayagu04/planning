#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "apa_planner/uss_obstacle_avoidance/uss_obstacle_avoidance.h"
#include "planning_debug_info.pb.h"
#include "planning_plan.pb.h"
#include "uss_wave_info.pb.h"

namespace py = pybind11;
using namespace planning;

static UssObstacleAvoidance *pBase = nullptr;

int Init() {
  pBase = new UssObstacleAvoidance();
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

int UpdateBytes(py::bytes &func_statemachine_bytes,
                py::bytes &localization_info_bytes,
                py::bytes &vehicle_service_output_info_bytes,
                py::bytes &uss_wave_info_bytes,
                py::bytes &planning_output_bytes) {
  auto func_statemachine =
      BytesToProto<FuncStateMachine::FuncStateMachine>(func_statemachine_bytes);

  auto localization_info =
      BytesToProto<LocalizationOutput::LocalizationEstimate>(
          localization_info_bytes);

  auto vehicle_service_output_info =
      BytesToProto<VehicleService::VehicleServiceOutputInfo>(
          vehicle_service_output_info_bytes);

  auto uss_wave_info =
      BytesToProto<UssWaveInfo::UssWaveInfo>(uss_wave_info_bytes);

  auto planning_output =
      BytesToProto<PlanningOutput::PlanningOutput>(planning_output_bytes);

  static planning::LocalView local_view;

  local_view.localization_estimate = localization_info;
  local_view.vehicle_service_output_info = vehicle_service_output_info;
  local_view.function_state_machine_info = func_statemachine;
  local_view.uss_wave_info = uss_wave_info;

  pBase->SetLocalView(&local_view);

  pBase->Update(&planning_output);

  return 0;
}

const size_t GetMinDistVehicleArcIndex() {
  return pBase->GetMinDistVehicleArcIndex();
}

const size_t GetMinDistUssArcIndex() { return pBase->GetMinDistUssArcIndex(); }

const double GetRemainDist() { return pBase->GetRemainDist(); }

const std::vector<Eigen::Vector2d> GetVehicleVertex() {
  return pBase->GetVehicleVertex();
}

const std::vector<Eigen::Vector2d> GetUssVertex() {
  return pBase->GetUssVertex();
}

const std::vector<double> GetUssRawDist() { return pBase->GetUssRawDist(); }

PYBIND11_MODULE(uss_obstacle_avoidance_py, m) {
  m.doc() = "m";

  m.def("Init", &Init)
      .def("UpdateBytes", &UpdateBytes)
      .def("GetMinDistVehicleArcIndex", &GetMinDistVehicleArcIndex)
      .def("GetMinDistUssArcIndex", &GetMinDistUssArcIndex)
      .def("GetRemainDist", &GetRemainDist)
      .def("GetVehicleVertex", &GetVehicleVertex)
      .def("GetUssVertex", &GetUssVertex)
      .def("GetUssRawDist", &GetUssRawDist);
}
