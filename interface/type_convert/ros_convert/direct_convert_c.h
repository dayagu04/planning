#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

REG_CONVERT_DIRECT(_iflytek_ace_debug_info_converter, "/iflytek/ace/debug_info");
REG_CONVERT_DIRECT(_iflytek_control_debug_info_converter, "/iflytek/control/debug_info");
REG_CONVERT_DIRECT(_iflytek_planning_debug_info_converter, "/iflytek/planning/debug_info");
REG_CONVERT_DIRECT(_iflytek_prediction_prediction_debug_info_converter, "/iflytek/prediction/prediction_debug_info");
REG_CONVERT_DIRECT(_iflytek_ehr_debuginfo_converter, "/iflytek/ehr/debuginfo");
REG_CONVERT_DIRECT(_iflytek_ehr_static_map_converter, "/iflytek/ehr/static_map");
REG_CONVERT_DIRECT(_iflytek_ehr_origin_data_converter, "/iflytek/ehr/origin_data");
REG_CONVERT_DIRECT(_iflytek_ehr_sdmap_info_converter, "/iflytek/ehr/sdmap_info");
REG_CONVERT_DIRECT(_iflytek_ehp_parking_map_converter, "/iflytek/ehp/parking_map");
REG_CONVERT_DIRECT(_iflytek_camera_perception_3d_occupancy_objects_converter, "/iflytek/camera_perception/3d_occupancy_objects");
REG_CONVERT_DIRECT(_iflytek_sdmap_sdmap_info_converter, "/iflytek/sdmap/sdmap_info");
REG_CONVERT_DIRECT(_iflytek_hmi_hpp_inner_converter, "/iflytek/hmi/hpp_inner");
REG_CONVERT_DIRECT(_iflytek_hmi_hpp_outer_converter, "/iflytek/hmi/hpp_outer");
REG_CONVERT_DIRECT(_iflytek_hmi_mcu_inner_converter, "/iflytek/hmi/mcu_inner");
REG_CONVERT_DIRECT(_iflytek_hmi_soc_inner_converter, "/iflytek/hmi/soc_inner");
REG_CONVERT_DIRECT(_iflytek_hmi_soc_outer_converter, "/iflytek/hmi/soc_outer");
REG_CONVERT_DIRECT(_iflytek_localization_ego_pose_converter, "/iflytek/localization/ego_pose");
