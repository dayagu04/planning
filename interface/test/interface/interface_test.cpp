
#pragma once
#include "common_c.h"
#include "common_platform_type_soc.h"
#include "struct_container.hpp"

#include "adas_function_c.h"
#include "adas_function_debug_c.h"
#include "calib_info_c.h"
#include "camera_perception_deceler_c.h"
#include "camera_perception_feature_c.h"
#include "camera_perception_groundline_c.h"
#include "camera_perception_lane_lines_c.h"
#include "camera_perception_objects_c.h"
#include "camera_perception_occupancy_objects_c.h"
#include "camera_perception_occupancy_grid_c.h"
#include "camera_perception_parking_slot_c.h"
#include "camera_perception_scene_c.h"
#include "camera_perception_tsr_c.h"
#include "control_command_c.h"
#include "dtc_code_c.h"
#include "ehr_c.h"
#include "factory_calib_c.h"
#include "fdl_upload_state_c.h"
#include "fm_info_c.h"
#include "fsm_mcu_out_c.h"
#include "func_state_machine_c.h"
#include "fusion_deceler_c.h"
#include "fusion_groundline_c.h"
#include "fusion_objects_c.h"
#include "fusion_objects_compress_c.h"
#include "fusion_occupancy_objects_c.h"
#include "fusion_parking_slot_c.h"
#include "fusion_road_c.h"
#include "fusion_road_compress_c.h"
#include "fusion_spatial_parking_slot_c.h"
#include "hmi_inner_c.h"
#include "hmi_soc_outer_c.h"
#include "ifly_localization_c.h"
#include "ifly_mapping_status_c.h"
#include "ifly_parking_map_c.h"
#include "planning_hmi_c.h"
#include "planning_plan_c.h"
#include "post_sale_calibration_c.h"
#include "prediction_c.h"
#include "radar_perception_objects_c.h"
#include "sensor_gnss_c.h"
#include "sensor_imu_c.h"
#include "sensor_radar_c.h"
#include "system_version_c.h"
#include "uss_perception_debug_info_c.h"
#include "uss_perception_info_c.h"
#include "uss_wave_info_c.h"
#include "vehicle_service_c.h"
#include "sentinel_mode_c.h"
#include "degraded_driving_function_c.h"
#include "engineer_mode_c.h"
#include "fdl_trigger_c.h"

#include "avm_reload_result_c.h"
#include "can_raw_eth_data_c.h"
#include "hmi_mcu_out_c.h"
#include "hmi_service_c.h"
#include "lidar_objects_c.h"
#include "sensor_image_c.h"
#include "sensor_lidar_c.h"
#include "uss_model_info_c.h"
#include "uss_wave_debug_info_c.h"
#include "time_synchronization_c.h"
#include "ifly_phm_c.h"
#include "camera_hot_swap_c.h"
#include "disk_info_c.h"
#include "fdl_net_stat_check_c.h"
#include "ota_log_upload_c.h"
#include "sensor_camera_status_c.h"
#include "icc_c.h"
#include "system_monitor_c.h"
// legacy_c_interface
#include "interface2.4.5/hmi_hpp_inner_c.h"
#include "interface2.4.5/hmi_hpp_outer_c.h"
#include "interface2.4.5/hmi_mcu_inner_c.h"
#include "interface2.4.5/hmi_soc_inner_c.h"
#include "interface2.4.5/hmi_soc_outer_c.h"
#include "interface2.4.5/func_state_machine_c.h"
#include "interface2.4.6/localization_c.h"
#include "system_monitor_c.h"
#include "ota_log_upload_c.h"
#include "icc_c.h"
#include "sensor_camera_status_c.h"

using namespace iflyauto;
int main(int argc, char const *argv[])
{

    printf("iflyauto::ADASFunctionDebugOutputInfo\t%d\n",sizeof(iflyauto::ADASFunctionDebugOutputInfo));
    printf("iflyauto::ADASFunctionOutputInfo\t%d\n",sizeof(iflyauto::ADASFunctionOutputInfo));
    printf("iflyauto::CalibInfo\t%d\n",sizeof(iflyauto::CalibInfo));
    printf("iflyauto::CameraPerceptionFeatureInfo\t%d\n",sizeof(iflyauto::CameraPerceptionFeatureInfo));
    printf("iflyauto::CameraPerceptionObjectsInfo\t%d\n",sizeof(iflyauto::CameraPerceptionObjectsInfo));
    printf("iflyauto::CameraPerceptionOccObjectsInfo\t%d\n",sizeof(iflyauto::CameraPerceptionOccObjectsInfo));
    printf("iflyauto::CameraPerceptionScene\t%d\n",sizeof(iflyauto::CameraPerceptionScene));
    printf("iflyauto::CameraPerceptionTsrInfo\t%d\n",sizeof(iflyauto::CameraPerceptionTsrInfo));
    printf("iflyauto::ControlOutput\t%d\n",sizeof(iflyauto::ControlOutput));
    printf("iflyauto::DecelerPerceptionInfo\t%d\n",sizeof(iflyauto::DecelerPerceptionInfo));
    printf("iflyauto::EhpOutput\t%d\n",sizeof(iflyauto::EhpOutput));
    printf("iflyauto::FactoryCalibActiveInfo\t%d\n",sizeof(iflyauto::FactoryCalibActiveInfo));
    printf("iflyauto::FactoryCalibActiveInfoResult\t%d\n",sizeof(iflyauto::FactoryCalibActiveInfoResult));
    printf("iflyauto::FactoryCalibGetCalibResReqInfo\t%d\n",sizeof(iflyauto::FactoryCalibGetCalibResReqInfo));
    printf("iflyauto::FactoryCalibResults\t%d\n",sizeof(iflyauto::FactoryCalibResults));
    printf("iflyauto::FdlUploadState\t%d\n",sizeof(iflyauto::FdlUploadState));
    printf("iflyauto::FeaturePoint\t%d\n",sizeof(iflyauto::FeaturePoint));
    printf("iflyauto::FmInfo\t%d\n",sizeof(iflyauto::FmInfo));
    printf("iflyauto::FsmMcuOut\t%d\n",sizeof(iflyauto::FsmMcuOut));
    printf("iflyauto::FuncStateMachine\t%d\n",sizeof(iflyauto::FuncStateMachine));
    printf("iflyauto::FusionDecelerInfo\t%d\n",sizeof(iflyauto::FusionDecelerInfo));
    printf("iflyauto::FusionGroundLineInfo\t%d\n",sizeof(iflyauto::FusionGroundLineInfo));
    printf("iflyauto::FusionObjectsInfo\t%d\n",sizeof(iflyauto::FusionObjectsInfo));
    printf("iflyauto::FusionObjectsInfoCompress\t%d\n",sizeof(iflyauto::FusionObjectsInfoCompress));
    printf("iflyauto::FusionOccupancyObjectsInfo\t%d\n",sizeof(iflyauto::FusionOccupancyObjectsInfo));
    printf("iflyauto::FusionSpatialParkingSlot\t%d\n",sizeof(iflyauto::FusionSpatialParkingSlot));
    printf("iflyauto::GroundLinePerceptionInfo\t%d\n",sizeof(iflyauto::GroundLinePerceptionInfo));
    printf("iflyauto::HmiInner\t%d\n",sizeof(iflyauto::HmiInner));
    printf("iflyauto::HmiSocOuter\t%d\n",sizeof(iflyauto::HmiSocOuter));
    printf("iflyauto::IFLYAllDtcState\t%d\n",sizeof(iflyauto::IFLYAllDtcState));
    printf("iflyauto::IFLYGnss\t%d\n",sizeof(iflyauto::IFLYGnss));
    printf("iflyauto::IFLYIMU\t%d\n",sizeof(iflyauto::IFLYIMU));
    printf("iflyauto::IFLYLocalization\t%d\n",sizeof(iflyauto::IFLYLocalization));
    printf("iflyauto::LaneLineSet\t%d\n",sizeof(iflyauto::LaneLineSet));
    printf("iflyauto::MappingStatusInfo\t%d\n",sizeof(iflyauto::MappingStatusInfo));
    printf("iflyauto::ParkingFusionInfo\t%d\n",sizeof(iflyauto::ParkingFusionInfo));
    printf("iflyauto::ParkingInfo\t%d\n",sizeof(iflyauto::ParkingInfo));
    printf("iflyauto::ParkingMapFileInfo\t%d\n",sizeof(iflyauto::ParkingMapFileInfo));
    printf("iflyauto::ParkingSlotSelectInfo\t%d\n",sizeof(iflyauto::ParkingSlotSelectInfo));
    printf("iflyauto::PlanningHMIOutputInfoStr\t%d\n",sizeof(iflyauto::PlanningHMIOutputInfoStr));
    printf("iflyauto::PlanningOutput\t%d\n",sizeof(iflyauto::PlanningOutput));
    printf("iflyauto::Position\t%d\n",sizeof(iflyauto::Position));
    printf("iflyauto::PostSaleCalibActiveInfo\t%d\n",sizeof(iflyauto::PostSaleCalibActiveInfo));
    printf("iflyauto::PostSaleCalibActiveInfoResult\t%d\n",sizeof(iflyauto::PostSaleCalibActiveInfoResult));
    printf("iflyauto::PostSaleCalibGetCalibResReqInfo\t%d\n",sizeof(iflyauto::PostSaleCalibGetCalibResReqInfo));
    printf("iflyauto::PostSaleCalibResults\t%d\n",sizeof(iflyauto::PostSaleCalibResults));
    printf("iflyauto::PostSaleCalibStopReqInfo\t%d\n",sizeof(iflyauto::PostSaleCalibStopReqInfo));
    printf("iflyauto::PostSaleCalibStopResult\t%d\n",sizeof(iflyauto::PostSaleCalibStopResult));
    printf("iflyauto::PredictionResult\t%d\n",sizeof(iflyauto::PredictionResult));
    printf("iflyauto::RadarPerceptionObjectsInfo\t%d\n",sizeof(iflyauto::RadarPerceptionObjectsInfo));
    printf("iflyauto::RadarPointCloud\t%d\n",sizeof(iflyauto::RadarPointCloud));
    printf("iflyauto::RoadInfo\t%d\n",sizeof(iflyauto::RoadInfo));
    printf("iflyauto::RoadInfoCompress\t%d\n",sizeof(iflyauto::RoadInfoCompress));
    printf("iflyauto::SemanticInfo\t%d\n",sizeof(iflyauto::SemanticInfo));
    printf("iflyauto::SystemVersion\t%d\n",sizeof(iflyauto::SystemVersion));
    printf("iflyauto::UssPdcIccSendDataType\t%d\n",sizeof(iflyauto::UssPdcIccSendDataType));
    printf("iflyauto::UssPerceptDebugInfo\t%d\n",sizeof(iflyauto::UssPerceptDebugInfo));
    printf("iflyauto::UssPerceptInfo\t%d\n",sizeof(iflyauto::UssPerceptInfo));
    printf("iflyauto::VehicleServiceOutputInfo\t%d\n",sizeof(iflyauto::VehicleServiceOutputInfo));
    return 0;
}
