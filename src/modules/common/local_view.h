#pragma once

#include <memory>

#include "control_command.pb.h"
#include "ehr.pb.h"
#include "func_state_machine.pb.h"
#include "fusion_objects.pb.h"
#include "fusion_road.pb.h"
#include "hmi_mcu_inner.pb.h"
#include "localization.pb.h"
#include "parking_fusion.pb.h"
#include "parking_slot_list.pb.h"
#include "planning_plan.pb.h"
#include "prediction.pb.h"
#include "uss_wave_info.pb.h"
#include "vehicle_service.pb.h"

namespace planning {
/**
 * @struct local_view
 * @brief LocalView contains all necessary data as planning input
 */

struct LocalView {
  Prediction::PredictionResult prediction_result;
  double prediction_result_recv_time = 0.0;

  FusionRoad::RoadInfo road_info;
  double road_info_recv_time = 0.0;

  LocalizationOutput::LocalizationEstimate localization_estimate;
  double localization_estimate_recv_time = 0.0;

  FusionObjects::FusionObjectsInfo fusion_objects_info;
  double fusion_objects_info_recv_time = 0.0;

  VehicleService::VehicleServiceOutputInfo vehicle_service_output_info;
  double vehicle_service_output_info_recv_time = 0.0;

  ControlCommand::ControlOutput control_output;
  double control_output_recv_time = 0.0;

  HmiMcuInner::HmiMcuInner hmi_mcu_inner_info;
  double hmi_mcu_inner_info_recv_time = 0.0;

  ParkingFusion::ParkingFusionInfo parking_fusion_info;
  double parking_fusion_info_recv_time = 0.0;

  FuncStateMachine::FuncStateMachine function_state_machine_info;
  double function_state_machine_info_recv_time = 0.0;

  UssWaveInfo::UssWaveInfo uss_wave_info;
  double uss_wave_info_recv_time = 0.0;

  Map::StaticMap static_map_info;
  double static_map_info_recv_time = 0.0;

  double hdmap_time = 0.0;
};

}  // namespace planning
