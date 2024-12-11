#include <gflags/gflags.h>
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

#include "src/modules/apa_function/parking_scenario/parking_scenario.h"
#include "apa_plan_interface.h"
#include "camera_preception_groundline_c.h"
#include "config_context.h"
#include "control_command_c.h"
#include "debug_info_log.h"
#include "func_state_machine_c.h"
#include "fusion_objects_c.h"
#include "fusion_occupancy_objects_c.h"
#include "fusion_parking_slot_c.h"
#include "ifly_localization_c.h"
#include "perfect_control.h"
#include "planning_debug_info.pb.h"
#include "planning_plan_c.h"
#include "serialize_utils.h"
#include "spline.h"
#include "struct_convert/camera_preception_groundline_c.h"
#include "struct_convert/common_c.h"
#include "struct_convert/control_command_c.h"
#include "struct_convert/func_state_machine_c.h"
#include "struct_convert/fusion_objects_c.h"
#include "struct_convert/fusion_occupancy_objects_c.h"
#include "struct_convert/fusion_parking_slot_c.h"
#include "struct_convert/hmi_inner_c.h"
#include "struct_convert/ifly_localization_c.h"
#include "struct_convert/planning_plan_c.h"
#include "struct_convert/uss_perception_info_c.h"
#include "struct_convert/uss_wave_info_c.h"
#include "struct_convert/vehicle_service_c.h"
#include "struct_msgs/ControlOutput.h"
#include "struct_msgs/FuncStateMachine.h"
#include "struct_msgs/FusionObjectsInfo.h"
#include "struct_msgs/FusionOccupancyObjectsInfo.h"
#include "struct_msgs/GroundLinePerceptionInfo.h"
#include "struct_msgs/IFLYLocalization.h"
#include "struct_msgs/ParkingFusionInfo.h"
#include "struct_msgs/PlanningOutput.h"
#include "struct_msgs/UssPerceptInfo.h"
#include "struct_msgs/UssWaveInfo.h"
#include "struct_msgs/VehicleServiceOutputInfo.h"
#include "vehicle_service_c.h"

namespace py = pybind11;
using namespace planning;
using namespace planning::apa_planner;
static apa_planner::ApaPlanInterface *apa_interface_ptr = nullptr;
static PerfectControl *perfect_control_ptr;

static planning::LocalView local_view;

int Init() {
  // const std::string flag_file_path =
  //     "/asw/planning/res/conf/planning_gflags.conf";
  // google::SetCommandLineOption("flagfile", flag_file_path.c_str());
  (void)common::ConfigurationContext::Instance();

  apa_interface_ptr = new apa_planner::ApaPlanInterface();

  FilePath::SetName("apa_simulation_py");
  InitGlog(FilePath::GetName().c_str());

  apa_interface_ptr->Init(true);

  perfect_control_ptr = new PerfectControl();
  perfect_control_ptr->Init();

  return 0;
}

const bool InterfaceUpdate(py::bytes &func_statemachine_bytes,
                           py::bytes &parking_slot_info_bytes,
                           py::bytes &localization_info_bytes,
                           py::bytes &vehicle_service_output_info_bytes,
                           py::bytes &uss_wave_info_bytes) {
  // iflyauto::FuncStateMachine func_statemachine =
  //     BytesToStruct<iflyauto::FuncStateMachine,
  //     struct_msgs::FuncStateMachine>(
  //         func_statemachine_bytes);

  iflyauto::FuncStateMachine func_statemachine;

  iflyauto::ParkingFusionInfo parking_slot_info =
      BytesToStruct<iflyauto::ParkingFusionInfo,
                    struct_msgs::ParkingFusionInfo>(parking_slot_info_bytes);

  iflyauto::IFLYLocalization localization_info =
      BytesToStruct<iflyauto::IFLYLocalization, struct_msgs::IFLYLocalization>(
          localization_info_bytes);

  // iflyauto::VehicleServiceOutputInfo vehicle_service_output_info =
  //     BytesToStruct<iflyauto::VehicleServiceOutputInfo,
  //                   struct_msgs::VehicleServiceOutputInfo>(
  //         vehicle_service_output_info_bytes);

  iflyauto::VehicleServiceOutputInfo vehicle_service_output_info;

  iflyauto::UssWaveInfo uss_wave_info =
      BytesToStruct<iflyauto::UssWaveInfo, struct_msgs::UssWaveInfo>(
          uss_wave_info_bytes);

  local_view.localization = localization_info;
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
  // iflyauto::FuncStateMachine func_statemachine =
  //     BytesToStruct<iflyauto::FuncStateMachine,
  //     struct_msgs::FuncStateMachine>(
  //         func_statemachine_bytes);
  iflyauto::FuncStateMachine func_statemachine;

  iflyauto::ParkingFusionInfo parking_slot_info =
      BytesToStruct<iflyauto::ParkingFusionInfo,
                    struct_msgs::ParkingFusionInfo>(parking_slot_info_bytes);

  iflyauto::IFLYLocalization localization_info =
      BytesToStruct<iflyauto::IFLYLocalization, struct_msgs::IFLYLocalization>(
          localization_info_bytes);

  // iflyauto::VehicleServiceOutputInfo vehicle_service_output_info =
  //     BytesToStruct<iflyauto::VehicleServiceOutputInfo,
  //                   struct_msgs::VehicleServiceOutputInfo>(
  //         vehicle_service_output_info_bytes);
  iflyauto::VehicleServiceOutputInfo vehicle_service_output_info;

  local_view.localization = localization_info;
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
    py::bytes &uss_wave_info_bytes, py::bytes &uss_perception_info_bytes,
    py::bytes &ground_line_info_bytes, py::bytes &fus_obj_info_bytes,
    py::bytes &fus_occ_obj_info_bytes, py::bytes &control_output_bytes,
    int select_id, bool force_plan, bool is_path_optimization,
    bool is_cilqr_optimization, bool is_reset, bool is_complete_path,
    bool sim_to_target, bool use_slot_in_bag, bool use_obs_in_bag,
    double sample_ds, std::vector<double> target_managed_slot_x_vec,
    std::vector<double> target_managed_slot_y_vec,
    std::vector<double> target_managed_limiter_x_vec,
    std::vector<double> target_managed_limiter_y_vec,
    std::vector<double> obs_x_vec, std::vector<double> obs_y_vec, std::vector<double> lat_path_optimizier_params) {
  SimulationParam param;
  param.is_simulation = true;
  param.is_complete_path = is_complete_path;
  param.force_plan = force_plan;
  param.is_path_optimization = is_path_optimization;
  param.is_cilqr_optimization = is_cilqr_optimization;
  param.q_ref_xy = lat_path_optimizier_params[0];
  param.q_ref_theta = lat_path_optimizier_params[1];
  param.q_terminal_xy = lat_path_optimizier_params[2];
  param.q_terminal_theta = lat_path_optimizier_params[3];
  param.q_k = lat_path_optimizier_params[4];
  param.q_u = lat_path_optimizier_params[5];
  param.q_k_bound = lat_path_optimizier_params[6];
  param.q_u_bound = lat_path_optimizier_params[7];
  param.sample_ds = sample_ds;
  param.is_reset = is_reset;
  param.sim_to_target = sim_to_target;
  param.use_slot_in_bag = use_slot_in_bag;
  param.use_obs_in_bag = use_obs_in_bag;
  param.target_managed_slot_x_vec = target_managed_slot_x_vec;
  param.target_managed_slot_y_vec = target_managed_slot_y_vec;
  param.target_managed_limiter_x_vec = target_managed_limiter_x_vec;
  param.target_managed_limiter_y_vec = target_managed_limiter_y_vec;
  param.obs_x_vec = obs_x_vec;
  param.obs_y_vec = obs_y_vec;

  iflyauto::FuncStateMachine func_statemachine =
      BytesToStruct<iflyauto::FuncStateMachine, struct_msgs::FuncStateMachine>(
          func_statemachine_bytes);

  iflyauto::ParkingFusionInfo parking_slot_info =
      BytesToStruct<iflyauto::ParkingFusionInfo,
                    struct_msgs::ParkingFusionInfo>(parking_slot_info_bytes);

  iflyauto::IFLYLocalization localization_info =
      BytesToStruct<iflyauto::IFLYLocalization, struct_msgs::IFLYLocalization>(
          localization_info_bytes);

  iflyauto::GroundLinePerceptionInfo ground_line_info =
      BytesToStruct<iflyauto::GroundLinePerceptionInfo,
                    struct_msgs::GroundLinePerceptionInfo>(
          ground_line_info_bytes);

  iflyauto::FusionObjectsInfo fus_obj_info =
      BytesToStruct<iflyauto::FusionObjectsInfo,
                    struct_msgs::FusionObjectsInfo>(fus_obj_info_bytes);

  iflyauto::FusionOccupancyObjectsInfo fus_occ_obj_info =
      BytesToStruct<iflyauto::FusionOccupancyObjectsInfo,
                    struct_msgs::FusionOccupancyObjectsInfo>(
          fus_occ_obj_info_bytes);

  iflyauto::ControlOutput control_output_info =
      BytesToStruct<iflyauto::ControlOutput, struct_msgs::ControlOutput>(
          control_output_bytes);

  iflyauto::VehicleServiceOutputInfo vehicle_service_output_info =
      BytesToStruct<iflyauto::VehicleServiceOutputInfo,
                    struct_msgs::VehicleServiceOutputInfo>(
          vehicle_service_output_info_bytes);

  iflyauto::UssWaveInfo uss_wave_info =
      BytesToStruct<iflyauto::UssWaveInfo, struct_msgs::UssWaveInfo>(
          uss_wave_info_bytes);

  iflyauto::UssPerceptInfo uss_perception_info =
      BytesToStruct<iflyauto::UssPerceptInfo, struct_msgs::UssPerceptInfo>(
          uss_perception_info_bytes);

  local_view.localization = localization_info;
  local_view.vehicle_service_output_info = vehicle_service_output_info;
  local_view.parking_fusion_info = parking_slot_info;
  local_view.uss_wave_info = uss_wave_info;
  local_view.function_state_machine_info = func_statemachine;
  local_view.uss_percept_info = uss_perception_info;
  local_view.ground_line_perception = ground_line_info;
  local_view.fusion_objects_info = fus_obj_info;
  local_view.fusion_occupancy_objects_info = fus_occ_obj_info;
  local_view.control_output = control_output_info;

  // DEBUG_PRINT(
  //     "c++ gl size = " << static_cast<int>(ground_line_info.ground_lines_size));

  // DEBUG_PRINT("c++ fus_obj_num = "
  //             << static_cast<int>(fus_obj_info.fusion_object_size));

  // DEBUG_PRINT("c++ fus_occ_obj_num = "
  //             << static_cast<int>(fus_occ_obj_info.fusion_object_size));

  // Note: currently plan once when slot selected in searching state
  if (force_plan) {
    local_view.function_state_machine_info.current_state =
        iflyauto::FunctionalState_PARK_GUIDANCE;
  }
  if (select_id > 0) {
    local_view.parking_fusion_info.select_slot_id = select_id;
    std::cout << "pybind select slot id = "
              << local_view.parking_fusion_info.select_slot_id << std::endl;
  }

  apa_interface_ptr->SetSimuParam(param);

  const bool result = apa_interface_ptr->Update(&local_view);
  if (apa_interface_ptr->GetPlaningOutput()
          .planning_status.apa_planning_status != iflyauto::APA_IN_PROGRESS) {
    return false;
  }
  apa_interface_ptr->UpdateDebugInfo();

  return result;
}

py::bytes GetPlanningOutput() {
  iflyauto::PlanningOutput planning_output =
      apa_interface_ptr->GetPlaningOutput();
  return StructToBytes<iflyauto::PlanningOutput, struct_msgs::PlanningOutput>(
      planning_output);
}

py::bytes GetPlanningDebugInfo() {
  return ProtoToBytes(apa_interface_ptr->GetPlanningDebugInfo());
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

PYBIND11_MODULE(apa_simulation_py, m) {
  m.doc() = "m";

  m.def("Init", &Init)
      .def("InterfaceUpdate", &InterfaceUpdate)
      .def("InterfaceUpdateClosedLoop", &InterfaceUpdateClosedLoop)
      .def("InterfaceUpdateParam", &InterfaceUpdateParam)
      .def("GetPlanningOutput", &GetPlanningOutput)
      .def("GetPlanningDebugInfo", &GetPlanningDebugInfo)
      .def("DynamicsUpdate", &DynamicsUpdate)
      .def("DynamicsSwitchBuf", &DynamicsSwitchBuf)
      .def("GetDynamicState", &GetDynamicState);
}
