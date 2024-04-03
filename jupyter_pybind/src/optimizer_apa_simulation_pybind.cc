#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>
#include <pybind11/stl.h>

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <memory>
#include <vector>

#include "apa_plan_base.h"
#include "apa_plan_interface.h"
#include "func_state_machine_c.h"
#include "perfect_control.h"
#include "planning_debug_info.pb.h"
#include "planning_plan_c.h"
#include "spline.h"

#include "serialize_utils.h"
#include "struct_convert/common_c.h"
#include "struct_convert/func_state_machine_c.h"
#include "struct_convert/fusion_parking_slot_c.h"
#include "struct_convert/localization_c.h"
#include "struct_convert/planning_plan_c.h"
#include "struct_convert/uss_perception_info_c.h"
#include "struct_convert/uss_wave_info_c.h"
#include "struct_convert/vehicle_service_c.h"
#include "struct_msgs/FuncStateMachine.h"
#include "struct_msgs/LocalizationEstimate.h"
#include "struct_msgs/ParkingFusionInfo.h"
#include "struct_msgs/PlanningOutput.h"
#include "struct_msgs/UssPerceptInfo.h"
#include "struct_msgs/UssWaveInfo.h"
#include "struct_msgs/VehicleServiceOutputInfo.h"

namespace py = pybind11;
using namespace planning;

static apa_planner::ApaPlanInterface *apa_interface_ptr = nullptr;
static PerfectControl *perfect_control_ptr;

static planning::LocalView local_view;

static bool last_cilqr_optimization_enable = false;

int Init() {
  const auto plan_data_ptr = std::make_shared<plan_interface::PlanData>();
  apa_interface_ptr->Init();

  perfect_control_ptr = new PerfectControl();
  perfect_control_ptr->Init();

  return 0;
}

const bool InterfaceUpdate(py::bytes &func_statemachine_bytes,
                           py::bytes &parking_slot_info_bytes,
                           py::bytes &localization_info_bytes,
                           py::bytes &vehicle_service_output_info_bytes,
                           py::bytes &uss_wave_info_bytes) {
  iflyauto::FuncStateMachine func_statemachine =
      BytesToStruct<iflyauto::FuncStateMachine, struct_msgs::FuncStateMachine>(
          func_statemachine_bytes);

  iflyauto::ParkingFusionInfo parking_slot_info =
      BytesToStruct<iflyauto::ParkingFusionInfo,
                    struct_msgs::ParkingFusionInfo>(parking_slot_info_bytes);

  iflyauto::LocalizationEstimate localization_info =
      BytesToStruct<iflyauto::LocalizationEstimate,
                    struct_msgs::LocalizationEstimate>(localization_info_bytes);

  iflyauto::VehicleServiceOutputInfo vehicle_service_output_info =
      BytesToStruct<iflyauto::VehicleServiceOutputInfo,
                    struct_msgs::VehicleServiceOutputInfo>(
          vehicle_service_output_info_bytes);

  iflyauto::UssWaveInfo uss_wave_info =
      BytesToStruct<iflyauto::UssWaveInfo, struct_msgs::UssWaveInfo>(
          uss_wave_info_bytes);

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
  iflyauto::FuncStateMachine func_statemachine =
      BytesToStruct<iflyauto::FuncStateMachine, struct_msgs::FuncStateMachine>(
          func_statemachine_bytes);

  iflyauto::ParkingFusionInfo parking_slot_info =
      BytesToStruct<iflyauto::ParkingFusionInfo,
                    struct_msgs::ParkingFusionInfo>(parking_slot_info_bytes);

  iflyauto::LocalizationEstimate localization_info =
      BytesToStruct<iflyauto::LocalizationEstimate,
                    struct_msgs::LocalizationEstimate>(localization_info_bytes);

  iflyauto::VehicleServiceOutputInfo vehicle_service_output_info =
      BytesToStruct<iflyauto::VehicleServiceOutputInfo,
                    struct_msgs::VehicleServiceOutputInfo>(
          vehicle_service_output_info_bytes);

  local_view.localization_estimate = localization_info;
  local_view.vehicle_service_output_info = vehicle_service_output_info;
  local_view.parking_fusion_info = parking_slot_info;
  local_view.function_state_machine_info = func_statemachine;

  const bool result = apa_interface_ptr->Update(&local_view);
  apa_interface_ptr->UpdateDebugInfo();

  return result;
}

const bool InterfaceUpdateParam(
    py::bytes &func_statemachine_bytes, py::bytes &parking_slot_info_bytes,
    py::bytes &localization_info_bytes,
    py::bytes &vehicle_service_output_info_bytes,
    py::bytes &uss_wave_info_bytes, int select_id, bool force_plan,
    bool is_path_optimization, bool is_cilqr_optimization, bool is_reset,
    bool is_complete_path, double sample_ds, double q_ref_xy,
    double q_ref_theta, double q_terminal_xy, double q_terminal_theta,
    double q_k, double q_u, double q_k_bound, double q_u_bound) {
  apa_planner::ApaPlannerBase::SimulationParam param;
  param.is_complete_path = is_complete_path;
  param.force_plan = force_plan;
  param.is_path_optimization = is_path_optimization;
  param.is_cilqr_optimization = is_cilqr_optimization;
  param.last_cilqr_optimization_enable = last_cilqr_optimization_enable;
  param.q_ref_xy = q_ref_xy;
  param.q_terminal_xy = q_terminal_xy;
  param.q_terminal_theta = q_terminal_theta;
  param.q_k = q_k;
  param.q_u = q_u;
  param.q_k_bound = q_k_bound;
  param.q_u_bound = q_u_bound;

  const auto &apa_planner_stack = apa_interface_ptr->GetPlannerStack();

  for (const auto &planner : apa_planner_stack) {
    planner->SetSimuParam(param);
  }
  last_cilqr_optimization_enable = is_cilqr_optimization;

  iflyauto::FuncStateMachine func_statemachine =
      BytesToStruct<iflyauto::FuncStateMachine, struct_msgs::FuncStateMachine>(
          func_statemachine_bytes);

  iflyauto::ParkingFusionInfo parking_slot_info =
      BytesToStruct<iflyauto::ParkingFusionInfo,
                    struct_msgs::ParkingFusionInfo>(parking_slot_info_bytes);

  iflyauto::LocalizationEstimate localization_info =
      BytesToStruct<iflyauto::LocalizationEstimate,
                    struct_msgs::LocalizationEstimate>(localization_info_bytes);

  iflyauto::VehicleServiceOutputInfo vehicle_service_output_info =
      BytesToStruct<iflyauto::VehicleServiceOutputInfo,
                    struct_msgs::VehicleServiceOutputInfo>(
          vehicle_service_output_info_bytes);

  iflyauto::UssWaveInfo uss_wave_info =
      BytesToStruct<iflyauto::UssWaveInfo, struct_msgs::UssWaveInfo>(
          uss_wave_info_bytes);

  local_view.localization_estimate = localization_info;
  local_view.vehicle_service_output_info = vehicle_service_output_info;
  local_view.parking_fusion_info = parking_slot_info;
  local_view.uss_wave_info = uss_wave_info;
  local_view.function_state_machine_info = func_statemachine;

  if (force_plan) {
    local_view.function_state_machine_info.current_state =
        iflyauto::FunctionalState_PARK_IN_ACTIVATE_WAIT;
    if (select_id > 0) {
      local_view.parking_fusion_info.select_slot_id = select_id;
    }
  }

  const bool result = apa_interface_ptr->Update(&local_view);
  apa_interface_ptr->UpdateDebugInfo();

  return result;
}

py::bytes GetPlanningOutput() {
  iflyauto::PlanningOutput planning_output =
      apa_interface_ptr->GetPlaningOutput();
  return StructToBytes<iflyauto::PlanningOutput, struct_msgs::PlanningOutput>(
      planning_output);
}

py::bytes GetPlanningOutputDebugInfo() {
  auto res = apa_interface_ptr->GetOutputDebugInfo();
  std::string serialized_message;
  res.SerializeToString(&serialized_message);
  return serialized_message;
}

py::bytes GetPlanningInputDebugInfo() {
  auto res = apa_interface_ptr->GetInputDebugInfo();
  std::string serialized_message;
  res.SerializeToString(&serialized_message);
  return serialized_message;
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
  iflyauto::PlanningOutput planning_output =
      BytesToStruct<iflyauto::PlanningOutput, struct_msgs::PlanningOutput>(
          planning_output_bytes);
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

PYBIND11_MODULE(optimizer_apa_simulation_py, m) {
  m.doc() = "m";

  m.def("Init", &Init)
      .def("InterfaceUpdate", &InterfaceUpdate)
      .def("InterfaceUpdateClosedLoop", &InterfaceUpdateClosedLoop)
      .def("InterfaceUpdateParam", &InterfaceUpdateParam)
      .def("GetPlanningOutput", &GetPlanningOutput)
      .def("GetPlanningOutputDebugInfo", &GetPlanningOutputDebugInfo)
      .def("GetPlanningInputDebugInfo", &GetPlanningInputDebugInfo)
      .def("DynamicsUpdate", &DynamicsUpdate)
      .def("DynamicsSwitchBuf", &DynamicsSwitchBuf)
      .def("GetDynamicState", &GetDynamicState);
}
