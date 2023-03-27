#pragma once

#include <memory>

#include "../res/include/proto/bsw_proto_vehicle_service.pb.h"
#include "../res/include/proto/bsw_proto_fusion_objects.pb.h"
#include "../res/include/proto/asw_proto_localization.pb.h"
#include "../res/include/proto/bsw_proto_planning.pb.h"
#include "../res/include/proto/bsw_proto_control.pb.h"
#include "../res/include/proto/asw_proto_parking_fusion.pb.h"
#include "../res/include/proto/asw_proto_parking_slot_list.pb.h"
#include "../res/include/proto/asw_proto_parking_slot_select.pb.h"
#include "../res/include/proto/asw_proto_prediction.pb.h"
#include "../res/include/proto/asw_proto_road_fusion.pb.h"
#include "../res/include/proto/bsw_proto_radar_perception_objects.pb.h"


namespace planning {
/**
 * @struct local_view
 * @brief LocalView contains all necessary data as planning input
 */

struct LocalView {

  Asw::Prediction::PredictionResult prediction_result;
  Asw::RoadFusion::RoadInfo road_info;
  Asw::LocalizationOutput::LocalizationEstimate localization_estimate;
  Bsw::ObjectFusion::FusionObjectsInfo fusion_objects_info;
  Bsw::VehicleService::VehicleServiceOutputInfo vehicel_service_output_info;
  Bsw::RadarPerceptionObjects::RadarPerceptionObjectsInfo radar_perception_objects_info;
  Bsw::ControlOutput::ControlOutput control_output;

};

}  // namespace planning
