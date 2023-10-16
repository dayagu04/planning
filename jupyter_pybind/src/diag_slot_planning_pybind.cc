#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "apa_planner/diagonal/diagonal_in_trajectory_generator.h"
#include "local_view.h"
#include "slot_management_info.pb.h"
#include "uss_wave_info.pb.h"

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
                py::bytes &vehicle_service_output_info_bytes,
                py::bytes &slot_management_info_bytes, int selected_id) {
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

  auto slot_management_info =
      BytesToProto<planning::common::SlotManagementInfo>(
          slot_management_info_bytes);

  static planning::LocalView local_view;

  local_view.localization_estimate = localization_info;
  local_view.vehicle_service_output_info = vehicle_service_output_info;
  local_view.parking_fusion_info = parking_slot_info;
  local_view.function_state_machine_info = func_statemachine;

  pBase->SetLocalView(&local_view);

  pBase->PathPlanOnceSimulation(slot_management_info);

  return 0;
}

int UpdateBytesByParam(py::bytes &func_statemachine_bytes,
                       py::bytes &parking_slot_info_bytes,
                       py::bytes &localization_info_bytes,
                       py::bytes &vehicle_service_output_info_bytes,
                       py::bytes &uss_wave_info_bytes,
                       py::bytes &slot_management_info_bytes, int selected_id,
                       bool force_planning, uint8_t force_plan_stm,
                       bool is_complete_path, double sublane_left_length,
                       double sublane_right_length, double sublane_width) {
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

  auto uss_wave_info =
      BytesToProto<UssWaveInfo::UssWaveInfo>(uss_wave_info_bytes);

  auto slot_management_info =
      BytesToProto<planning::common::SlotManagementInfo>(
          slot_management_info_bytes);

  static planning::LocalView local_view;

  local_view.localization_estimate = localization_info;
  local_view.vehicle_service_output_info = vehicle_service_output_info;
  local_view.parking_fusion_info = parking_slot_info;
  local_view.function_state_machine_info = func_statemachine;
  local_view.uss_wave_info = uss_wave_info;

  pBase->SetLocalView(&local_view);

  DiagonalInTrajectoryGenerator::SimulationParam param;
  param.force_planning_ = force_planning;
  param.selected_id_ = selected_id;
  param.force_plan_stm = force_plan_stm;
  param.is_complete_path = is_complete_path;
  param.sublane_left_length = sublane_left_length;
  param.sublane_right_length = sublane_right_length;
  param.sublane_width = sublane_width;

  pBase->SetSimulationParam(param);
  pBase->PathPlanOnceSimulation(slot_management_info);

  return 0;
}

py::bytes GetOutputBytes() {
  auto res = pBase->GetOutput();
  std::string serialized_message;
  res.SerializeToString(&serialized_message);

  return serialized_message;
}

py::bytes GetSlotManagementOutputBytes() {
  auto res = pBase->GetSlotManagementOutput();
  std::string serialized_message;
  res.SerializeToString(&serialized_message);

  return serialized_message;
}

std::vector<double> GetPathEle(size_t index) {
  return pBase->GetDubinsPlanner().GetPathEle(index);
}

std::vector<double> GetObstaclesX() {
  const auto &obstacles = pBase->GetObstacles();
  std::vector<double> x_vec;
  x_vec.reserve(obstacles.size());

  for (const auto &obs : obstacles) {
    x_vec.emplace_back(obs.pA.x());
    x_vec.emplace_back(obs.pB.x());
  }

  return x_vec;
}

std::vector<double> GetObstaclesY() {
  const auto &obstacles = pBase->GetObstacles();
  std::vector<double> y_vec;
  y_vec.reserve(obstacles.size());

  for (const auto &obs : obstacles) {
    y_vec.emplace_back(obs.pA.y());
    y_vec.emplace_back(obs.pB.y());
  }

  return y_vec;
}

PYBIND11_MODULE(diag_slot_planning_py, m) {
  m.doc() = "m";

  m.def("Init", &Init)
      .def("UpdateBytes", &UpdateBytes)
      .def("UpdateBytesByParam", &UpdateBytesByParam)
      .def("GetSlotManagementOutputBytes", &GetSlotManagementOutputBytes)
      .def("GetOutputBytes", &GetOutputBytes)
      .def("GetObstaclesX", &GetObstaclesX)
      .def("GetObstaclesY", &GetObstaclesY)
      .def("GetPathEle", &GetPathEle);
}
