#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>
#include <pybind11/stl.h>

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <vector>

#include "apa_plan_base.h"
#include "apa_plan_interface.h"
#include "func_state_machine.pb.h"
#include "perfect_control.h"
#include "planning_debug_info.pb.h"
#include "planning_plan.pb.h"
#include "spline.h"

namespace py = pybind11;
using namespace planning;

static apa_planner::ApaPlanInterface *apa_interface_ptr = nullptr;
static PerfectControl *perfect_control_ptr;

static planning::LocalView local_view;

int Init() {
  apa_interface_ptr = new apa_planner::ApaPlanInterface();
  apa_interface_ptr->Init();

  perfect_control_ptr = new PerfectControl();
  perfect_control_ptr->Init();

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

template <class T>
py::bytes ProtoToBytes(T proto) {
  std::string serialized_message;
  proto.SerializeToString(&serialized_message);
  return serialized_message;
}

const bool InterfaceUpdate(py::bytes &func_statemachine_bytes,
                           py::bytes &parking_slot_info_bytes,
                           py::bytes &localization_info_bytes,
                           py::bytes &vehicle_service_output_info_bytes,
                           py::bytes &uss_wave_info_bytes) {
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

  local_view.localization_estimate = localization_info;
  local_view.vehicle_service_output_info = vehicle_service_output_info;
  local_view.parking_fusion_info = parking_slot_info;
  local_view.function_state_machine_info = func_statemachine;
  local_view.uss_wave_info = uss_wave_info;

  const bool result = apa_interface_ptr->Update(&local_view);
  apa_interface_ptr->UpdateDebugInfo();

  return result;
}

const bool InterfaceUpdateClosedLoop(
    py::bytes &func_statemachine_bytes, py::bytes &parking_slot_info_bytes,
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

  local_view.localization_estimate = localization_info;
  local_view.vehicle_service_output_info = vehicle_service_output_info;
  local_view.parking_fusion_info = parking_slot_info;
  local_view.function_state_machine_info = func_statemachine;

  const bool result = apa_interface_ptr->Update(&local_view);
  apa_interface_ptr->UpdateDebugInfo();

  return result;
}

const bool InterfaceUpdateParam(py::bytes &func_statemachine_bytes,
                                py::bytes &parking_slot_info_bytes,
                                py::bytes &localization_info_bytes,
                                py::bytes &vehicle_service_output_info_bytes,
                                py::bytes &uss_wave_info_bytes, int select_id,
                                bool force_plan, bool is_path_optimization, bool is_reset,
                                bool is_complete_path, double sample_ds,
                                std::vector<double> target_managed_slot_x_vec,
                                std::vector<double> target_managed_slot_y_vec,
                                std::vector<double> target_managed_limiter_x_vec,
                                std::vector<double> target_managed_limiter_y_vec)
                                 {
  apa_planner::ApaPlannerBase::SimulationParam param;
  param.is_complete_path = is_complete_path;
  param.force_plan = force_plan;
  param.is_path_optimization = is_path_optimization;
  param.sample_ds = sample_ds;
  param.is_reset = is_reset;
  param.target_managed_slot_x_vec = target_managed_slot_x_vec;
  param.target_managed_slot_y_vec = target_managed_slot_y_vec;
  param.target_managed_limiter_x_vec = target_managed_limiter_x_vec;
  param.target_managed_limiter_y_vec = target_managed_limiter_y_vec;

  const auto &apa_planner_stack = apa_interface_ptr->GetPlannerStack();

  for (const auto &planner : apa_planner_stack) {
    planner->SetSimuParam(param);
  }

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

  local_view.localization_estimate = localization_info;
  local_view.vehicle_service_output_info = vehicle_service_output_info;
  local_view.parking_fusion_info = parking_slot_info;
  local_view.uss_wave_info = uss_wave_info;
  local_view.function_state_machine_info = func_statemachine;

  if (force_plan) {
    local_view.function_state_machine_info.set_current_state(
        FuncStateMachine::FunctionalState::PARK_IN_ACTIVATE_WAIT);
    if (select_id > 0) {
      local_view.parking_fusion_info.set_select_slot_id(select_id);
    }
  }

  const bool result = apa_interface_ptr->Update(&local_view);
  apa_interface_ptr->UpdateDebugInfo();

  return result;
}

py::bytes GetPlanningOutput() {
  return ProtoToBytes(apa_interface_ptr->GetPlaningOutput());
}

py::bytes GetPlanningDebugInfo() {
  return ProtoToBytes(apa_interface_ptr->GetPlanningDebugInfo());
}

py::bytes PlanningOutputTransfer(py::bytes &new_planning_bytes,
                                 py::bytes &old_planning_bytes) {
  auto new_planning =
      BytesToProto<PlanningOutput::PlanningOutput>(new_planning_bytes);
  auto old_planning =
      BytesToProto<PlanningOutput::PlanningOutput>(old_planning_bytes);

  new_planning.clear_meta();
  new_planning.mutable_meta()->CopyFrom(old_planning.meta());

  return ProtoToBytes(new_planning);
}

py::bytes PlanningDebugInforansfer(py::bytes &new_planning_bytes,
                                   py::bytes &old_planning_bytes) {
  auto new_planning =
      BytesToProto<common::PlanningDebugInfo>(new_planning_bytes);
  auto old_planning =
      BytesToProto<common::PlanningDebugInfo>(old_planning_bytes);

  old_planning.set_data_json(new_planning.data_json());

  return ProtoToBytes(old_planning);
}

void DynamicsUpdate(py::bytes &planning_output_bytes, double dt) {
  PlanningOutput::PlanningOutput planning_output =
      BytesToProto<PlanningOutput::PlanningOutput>(planning_output_bytes);

  perfect_control_ptr->Update(planning_output, dt);
}

std::vector<double> GetDynamicState() {
  const auto &state = perfect_control_ptr->GetState();

  std::vector<double> res = {state.pos.x(), state.pos.y(), state.heading,
                             state.vel};

  return res;
}

void DynamicsSwitchBuf(double x, double y, double heading) {
  perfect_control_ptr->SetState(
      PerfectControl::DynamicState(Eigen::Vector2d(x, y), heading));
}

PYBIND11_MODULE(apa_simulation_py, m) {
  m.doc() = "m";

  m.def("Init", &Init)
      .def("InterfaceUpdate", &InterfaceUpdate)
      .def("InterfaceUpdateClosedLoop", &InterfaceUpdateClosedLoop)
      .def("InterfaceUpdateParam", &InterfaceUpdateParam)
      .def("GetPlanningOutput", &GetPlanningOutput)
      .def("GetPlanningDebugInfo", &GetPlanningDebugInfo)
      .def("PlanningOutputTransfer", &PlanningOutputTransfer)
      .def("DynamicsUpdate", &DynamicsUpdate)
      .def("DynamicsSwitchBuf", &DynamicsSwitchBuf)
      .def("GetDynamicState", &GetDynamicState);
}
