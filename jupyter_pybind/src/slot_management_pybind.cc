#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <memory>

#include "camera_preception_groundline_c.h"
#include "fusion_objects_c.h"
#include "fusion_occupancy_objects_c.h"
#include "planning_debug_info.pb.h"
#include "serialize_utils.h"
#include "slot_manager.h"
#include "struct_convert/camera_preception_groundline_c.h"
#include "struct_convert/common_c.h"
#include "struct_convert/func_state_machine_c.h"
#include "struct_convert/fusion_objects_c.h"
#include "struct_convert/fusion_occupancy_objects_c.h"
#include "struct_convert/fusion_parking_slot_c.h"
#include "struct_convert/ifly_localization_c.h"
#include "struct_convert/planning_plan_c.h"
#include "struct_convert/uss_perception_info_c.h"
#include "struct_convert/uss_wave_info_c.h"
#include "struct_convert/vehicle_service_c.h"
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

namespace py = pybind11;
using namespace planning;
using namespace planning::apa_planner;
static SlotManager *pBase = nullptr;

int Init() {
  pBase = new SlotManager();
  pBase->Reset();
  return 0;
}

int UpdateBytes(py::bytes &func_statemachine_bytes,
                py::bytes &parking_slot_info_bytes,
                py::bytes &localization_info_bytes,
                py::bytes &uss_wave_info_bytes,
                py::bytes &uss_perception_info_bytes,
                py::bytes &ground_line_perception_info_bytes,
                py::bytes &fusion_objects_info_bytes,
                py::bytes &fusion_occupancy_objects_info_bytes) {
  //   iflyauto::FuncStateMachine func_statemachine =
  //       BytesToStruct<iflyauto::FuncStateMachine,
  //       struct_msgs::FuncStateMachine>(
  //           func_statemachine_bytes);
  iflyauto::FuncStateMachine func_statemachine;

  iflyauto::ParkingFusionInfo parking_slot_info =
      BytesToStruct<iflyauto::ParkingFusionInfo,
                    struct_msgs::ParkingFusionInfo>(parking_slot_info_bytes);

  iflyauto::IFLYLocalization localization_info =
      BytesToStruct<iflyauto::IFLYLocalization, struct_msgs::IFLYLocalization>(
          localization_info_bytes);

  iflyauto::UssWaveInfo uss_wave_info =
      BytesToStruct<iflyauto::UssWaveInfo, struct_msgs::UssWaveInfo>(
          uss_wave_info_bytes);

  iflyauto::UssPerceptInfo uss_perception_info =
      BytesToStruct<iflyauto::UssPerceptInfo, struct_msgs::UssPerceptInfo>(
          uss_perception_info_bytes);

  iflyauto::GroundLinePerceptionInfo ground_line_perception_info =
      BytesToStruct<iflyauto::GroundLinePerceptionInfo,
                    struct_msgs::GroundLinePerceptionInfo>(
          ground_line_perception_info_bytes);

  iflyauto::FusionObjectsInfo fusion_objects_info =
      BytesToStruct<iflyauto::FusionObjectsInfo,
                    struct_msgs::FusionObjectsInfo>(fusion_objects_info_bytes);

  iflyauto::FusionOccupancyObjectsInfo fusion_occupancy_objects_info =
      BytesToStruct<iflyauto::FusionOccupancyObjectsInfo,
                    struct_msgs::FusionOccupancyObjectsInfo>(
          fusion_occupancy_objects_info_bytes);

  std::shared_ptr<ApaData> apa_data_ptr = std::make_shared<ApaData>();
//   pBase->Update(apa_data_ptr);

  return 0;
}

int UpdateBytesByParam(py::bytes &func_statemachine_bytes,
                       py::bytes &parking_slot_info_bytes,
                       py::bytes &localization_info_bytes,
                       py::bytes &uss_wave_info_bytes,
                       py::bytes &uss_perception_info_bytes,
                       py::bytes &ground_line_perception_info_bytes,
                       py::bytes &fusion_objects_info_bytes,
                       py::bytes &fusion_occupancy_objects_info_bytes,
                       bool force_apa_on, bool force_clear,
                       double max_slots_update_angle_dis_limit_deg,
                       double max_slot_boundary_line_angle_dif_deg,
                       double outside_lon_dist_max_slot2mirror,
                       double outside_lon_dist_min_slot2mirror) {
  iflyauto::FuncStateMachine func_statemachine;
  //   iflyauto::FuncStateMachine func_statemachine =
  //       BytesToStruct<iflyauto::FuncStateMachine,
  //       struct_msgs::FuncStateMachine>(
  //           func_statemachine_bytes);

  iflyauto::ParkingFusionInfo parking_slot_info =
      BytesToStruct<iflyauto::ParkingFusionInfo,
                    struct_msgs::ParkingFusionInfo>(parking_slot_info_bytes);

  iflyauto::IFLYLocalization localization_info =
      BytesToStruct<iflyauto::IFLYLocalization, struct_msgs::IFLYLocalization>(
          localization_info_bytes);

  iflyauto::UssWaveInfo uss_wave_info =
      BytesToStruct<iflyauto::UssWaveInfo, struct_msgs::UssWaveInfo>(
          uss_wave_info_bytes);

  iflyauto::UssPerceptInfo uss_perception_info =
      BytesToStruct<iflyauto::UssPerceptInfo, struct_msgs::UssPerceptInfo>(
          uss_perception_info_bytes);

  iflyauto::GroundLinePerceptionInfo ground_line_perception_info =
      BytesToStruct<iflyauto::GroundLinePerceptionInfo,
                    struct_msgs::GroundLinePerceptionInfo>(
          ground_line_perception_info_bytes);

  iflyauto::FusionObjectsInfo fusion_objects_info =
      BytesToStruct<iflyauto::FusionObjectsInfo,
                    struct_msgs::FusionObjectsInfo>(fusion_objects_info_bytes);

  iflyauto::FusionOccupancyObjectsInfo fusion_occupancy_objects_info =
      BytesToStruct<iflyauto::FusionOccupancyObjectsInfo,
                    struct_msgs::FusionOccupancyObjectsInfo>(
          fusion_occupancy_objects_info_bytes);

  SlotManager::Param param;
  param.force_apa_on = force_apa_on;
  param.force_clear = force_clear;
  param.max_slots_update_angle_dis_limit_deg =
      max_slots_update_angle_dis_limit_deg;
  param.max_slot_boundary_line_angle_dif_deg =
      max_slot_boundary_line_angle_dif_deg;
  param.outside_lon_dist_max_slot2mirror = outside_lon_dist_max_slot2mirror;
  param.outside_lon_dist_min_slot2mirror = outside_lon_dist_min_slot2mirror;

  pBase->SetParam(param);
  std::shared_ptr<ApaData> apa_data_ptr = std::make_shared<ApaData>();
//   pBase->Update(apa_data_ptr);

  return 0;
}

py::bytes GetOutputBytes() {
  auto res = pBase->GetOutput();
  std::string serialized_message;
  res.SerializeToString(&serialized_message);

  return serialized_message;
}

PYBIND11_MODULE(slot_management_py, m) {
  m.doc() = "m";

  m.def("Init", &Init)
      .def("Update", &UpdateBytes)
      .def("UpdateBytes", &UpdateBytes)
      .def("UpdateBytesByParam", &UpdateBytesByParam)
      .def("GetOutputBytes", &GetOutputBytes);
}
