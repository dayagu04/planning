#pragma once

#include "base_convert.h"
#include "c/hmi_mcu_out_c.h"
#include "struct_convert/common_c.h"
#include "struct_convert/adas_function_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::PdcRadarInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.obj_dis, ros_v.obj_dis, type);
  convert(struct_v.radar_pos, ros_v.radar_pos, type);
  convert(struct_v.warning_level, ros_v.warning_level, type);
}

template <typename T2>
void convert(iflyauto::HmiPdcInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.pdc_sys_sts, ros_v.pdc_sys_sts, type);
  for (size_t i0 = 0; i0 < ros_v.pdc_info.size(); i0++) {
	  convert(struct_v.pdc_info[i0], ros_v.pdc_info[i0], type);
  }
}

template <typename T2>
void convert(iflyauto::HmiMcuOut &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.adas_out_info, ros_v.adas_out_info, type);
  convert(struct_v.hmi_pdc_info, ros_v.hmi_pdc_info, type);
}

