#pragma once

#include <memory>

#include "camera_preception_groundline_c.h"
#include "control_command_c.h"
#include "ehr.pb.h"
#include "ehr_sdmap.pb.h"
#include "func_state_machine_c.h"
#include "fusion_objects_c.h"
#include "fusion_occupancy_objects_c.h"
#include "fusion_parking_slot_c.h"
#include "fusion_road_c.h"
#include "hmi_inner_c.h"
#include "ifly_localization_c.h"
#include "ifly_parking_map_c.h"
#include "planning_plan_c.h"
#include "prediction_c.h"
#include "uss_perception_info_c.h"
#include "uss_wave_info_c.h"
#include "vehicle_service_c.h"

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

  // iflyauto::ParkingInfo parking_map_info;
  // double parking_map_info_recv_time = 0.0;

  iflyauto::GroundLinePerceptionInfo ground_line_perception;
  double ground_line_perception_recv_time = 0.0;
};

}  // namespace planning
