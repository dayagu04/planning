#pragma once

#include <memory>

#include "vehicle_service.pb.h"
#include "fusion_objects.pb.h"
#include "localization.pb.h"
#include "planning_plan.pb.h"
#include "control_command.pb.h"
#include "parking_fusion.pb.h"
#include "parking_slot_list.pb.h"
#include "parking_slot_select.pb.h"
#include "prediction.pb.h"
#include "fusion_road.pb.h"
#include "radar_perception_objects.pb.h"
#include "hmi_mcu_inner.pb.h"

namespace planning {
/**
 * @struct local_view
 * @brief LocalView contains all necessary data as planning input
 */

struct LocalView {

  Prediction::PredictionResult prediction_result;
  FusionRoad::RoadInfo road_info;
  LocalizationOutput::LocalizationEstimate localization_estimate;
  FusionObjects::FusionObjectsInfo fusion_objects_info;
  VehicleService::VehicleServiceOutputInfo vehicel_service_output_info;
  RadarPerceptionObjects::RadarPerceptionObjectsInfo radar_perception_objects_info;
  ControlCommand::ControlOutput control_output;
  HmiMcuInner::HmiMcuInner hmi_mcu_inner_info;
  ParkingFusion::ParkingFusionInfo parking_fusion_info;
};

}  // namespace planning
