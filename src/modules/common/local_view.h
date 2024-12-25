#pragma once

#include <memory>

#include "planning_intf.h"
#include "ehr.pb.h"
#include "interface/src/legacy/interface2.4.5/hmi_mcu_inner_c.h"
#include "interface/src/legacy/interface2.4.6/localization_c.h"

namespace planning {
/**
 * @struct local_view
 * @brief LocalView contains all necessary data as planning input
 */

struct LocalView {
  iflyauto::PredictionResult prediction_result;
  double prediction_result_recv_time = 0.0;

  iflyauto::RoadInfo road_info;
  double road_info_recv_time = 0.0;

  iflyauto::IFLYLocalization localization;
  double localization_recv_time = 0.0;

  iflyauto::interface_2_4_6::LocalizationEstimate localization_estimate;
  double localization_estimate_recv_time = 0.0;

  iflyauto::FusionObjectsInfo fusion_objects_info;
  double fusion_objects_info_recv_time = 0.0;

  iflyauto::FusionOccupancyObjectsInfo fusion_occupancy_objects_info;
  double fusion_occupancy_objects_info_recv_time = 0.0;

  iflyauto::VehicleServiceOutputInfo vehicle_service_output_info;
  double vehicle_service_output_info_recv_time = 0.0;

  iflyauto::ControlOutput control_output;
  double control_output_recv_time = 0.0;

  iflyauto::HmiInner hmi_inner_info;
  double hmi_inner_info_recv_time = 0.0;

  iflyauto::interface_2_4_5::HmiMcuInner hmi_mcu_inner_info;
  double hmi_mcu_inner_info_recv_time = 0.0;

  iflyauto::ParkingFusionInfo parking_fusion_info;
  double parking_fusion_info_recv_time = 0.0;

  iflyauto::FuncStateMachine function_state_machine_info;
  double function_state_machine_info_recv_time = 0.0;

  iflyauto::UssWaveInfo uss_wave_info;
  double uss_wave_info_recv_time = 0.0;

  iflyauto::UssPerceptInfo uss_percept_info;
  double uss_percept_info_recv_time = 0.0;

  Map::StaticMap static_map_info;
  double static_map_info_recv_time = 0.0;

  SdMapSwtx::SdMap sd_map_info;
  double sd_map_info_recv_time = 0.0;
  iflyauto::CameraPerceptionTsrInfo perception_tsr_info;
  double perception_tsr_info_recv_time = 0.0;

  // iflyauto::ParkingInfo parking_map_info;
  // double parking_map_info_recv_time = 0.0;

  iflyauto::GroundLinePerceptionInfo ground_line_perception;
  double ground_line_perception_recv_time = 0.0;
};

}  // namespace planning
