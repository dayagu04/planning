#pragma once

#include <memory>

#include "../res/include/proto/vehicle_service.pb.h"
#include "../res/include/proto/fusion_objects.pb.h"
#include "../res/include/proto/localization.pb.h"
#include "../res/include/proto/planning_plan.pb.h"
#include "../res/include/proto/control_command.pb.h"
#include "../res/include/proto/parking_fusion.pb.h"
#include "../res/include/proto/parking_slot_list.pb.h"
#include "../res/include/proto/parking_slot_select.pb.h"
#include "../res/include/proto/prediction.pb.h"
#include "../res/include/proto/fusion_road.pb.h"
#include "../res/include/proto/radar_perception_objects.pb.h"
#include "../res/include/proto/hmi_mcu_inner.pb.h"

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
  HimMcuInner::HmiMcuInner hmi_mcu_inner_info;
  ParkingFusion::ParkingFusionInfo parking_fusion_info;
};

}  // namespace planning
