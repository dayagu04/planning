#pragma once

#include "base_convert.h"
#include "c/calib_info_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::CalibInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.calib_type, ros_v.calib_type, type);
  convert(struct_v.calib_status, ros_v.calib_status, type);
  convert(struct_v.action, ros_v.action, type);
  convert(struct_v.time, ros_v.time, type);
  for (size_t i0 = 0; i0 < ros_v.calib_sensors.size(); i0++) {
	  convert(struct_v.calib_sensors[i0], ros_v.calib_sensors[i0], type);
  }
  for (size_t i1 = 0; i1 < ros_v.desc.size(); i1++) {
	  convert(struct_v.desc[i1], ros_v.desc[i1], type);
  }
}

