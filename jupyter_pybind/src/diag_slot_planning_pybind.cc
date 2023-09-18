#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "apa_planner/diagonal/diagonal_in_trajectory_generator.h"
#include "local_view.h"

namespace py = pybind11;
using namespace planning::apa_planner;

static DiagonalInTrajectoryGenerator *pBase = nullptr;

int Init() {
  pBase = new DiagonalInTrajectoryGenerator();
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
                py::bytes &parking_slot_info_bytes,
                py::bytes &localization_info_bytes,
                py::bytes &vehicle_service_output_info_bytes) {
  auto func_statemachine =
      BytesToProto<FuncStateMachine::FuncStateMachine>(func_statemachine_bytes);

  auto parking_slot_info =
      BytesToProto<ParkingFusion::ParkingFusionInfo>(parking_slot_info_bytes);

  auto localization_info =
      BytesToProto<LocalizationOutput::LocalizationEstimate>(
          localization_info_bytes);

  auto vehicle_service_output_info =
      BytesToProto<VehicleService::VehicleServiceOutputInfo>(
          vehicle_service_output_info_bytes);

  static planning::LocalView local_view;

  local_view.localization_estimate = localization_info;
  local_view.vehicle_service_output_info = vehicle_service_output_info;
  local_view.parking_fusion_info = parking_slot_info;
  local_view.function_state_machine_info = func_statemachine;

  pBase->SetLocalView(&local_view);

  pBase->SingleSlotPlanSimulation();

  return 0;
}

int UpdateBytesByParam(py::bytes &func_statemachine_bytes,
                       py::bytes &parking_slot_info_bytes,
                       py::bytes &localization_info_bytes,
                       py::bytes &vehicle_service_output_info_bytes,
                       int selected_id, bool force_planning,
                       bool clear_seg_name) {
  auto func_statemachine =
      BytesToProto<FuncStateMachine::FuncStateMachine>(func_statemachine_bytes);

  auto parking_slot_info =
      BytesToProto<ParkingFusion::ParkingFusionInfo>(parking_slot_info_bytes);

  auto localization_info =
      BytesToProto<LocalizationOutput::LocalizationEstimate>(
          localization_info_bytes);

  auto vehicle_service_output_info =
      BytesToProto<VehicleService::VehicleServiceOutputInfo>(
          vehicle_service_output_info_bytes);

  static planning::LocalView local_view;

  local_view.localization_estimate = localization_info;
  local_view.vehicle_service_output_info = vehicle_service_output_info;
  local_view.parking_fusion_info = parking_slot_info;
  local_view.function_state_machine_info = func_statemachine;

  pBase->SetLocalView(&local_view);

  DiagonalInTrajectoryGenerator::SimulationParam param;
  param.force_planning_ = force_planning;
  param.selected_id_ = selected_id;
  param.clear_seg_name_ = clear_seg_name;

  pBase->SetSimulationParam(param);

  pBase->SingleSlotPlanSimulation();

  return 0;
}

py::bytes GetOutputBytes() {
  auto res = pBase->GetOutput();
  std::string serialized_message;
  res.SerializeToString(&serialized_message);

  return serialized_message;
}

PYBIND11_MODULE(diag_slot_planning_py, m) {
  m.doc() = "m";

  m.def("Init", &Init)
      .def("UpdateBytes", &UpdateBytes)
      .def("UpdateBytesByParam", &UpdateBytesByParam)
      .def("GetOutputBytes", &GetOutputBytes);
}
