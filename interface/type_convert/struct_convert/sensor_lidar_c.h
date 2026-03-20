#pragma once

#include "base_convert.h"
#include "c/sensor_lidar_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::LidarMsgInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.sensor_meta, ros_v.sensor_meta, type);
  for (size_t i0 = 0; i0 < ros_v.frame_id.size(); i0++) {
	  convert(struct_v.frame_id[i0], ros_v.frame_id[i0], type);
  }
  convert(struct_v.height, ros_v.height, type);
  convert(struct_v.width, ros_v.width, type);
}

