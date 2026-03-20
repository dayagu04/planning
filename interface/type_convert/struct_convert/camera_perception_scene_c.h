#pragma once

#include "base_convert.h"
#include "c/camera_perception_scene_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::CameraPerceptionScene &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.isp_timestamp, ros_v.isp_timestamp, type);
  convert(struct_v.road_type, ros_v.road_type, type);
  convert(struct_v.weather_condition, ros_v.weather_condition, type);
  convert(struct_v.traffic_condition, ros_v.traffic_condition, type);
  convert(struct_v.lighting_condition, ros_v.lighting_condition, type);
}

