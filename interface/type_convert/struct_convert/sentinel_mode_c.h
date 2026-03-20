#pragma once

#include "base_convert.h"
#include "c/sentinel_mode_c.h"
#include "struct_convert/common_c.h"
#include "struct_convert/adas_function_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::SENTINEL_Mcu2Soc &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.Mode_State, ros_v.Mode_State, type);
  convert(struct_v.Request, ros_v.Request, type);
  convert(struct_v.VCU_HVReady, ros_v.VCU_HVReady, type);
}

template <typename T2>
void convert(iflyauto::SENTINEL_Soc2Mcu &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.CameraStreamSts, ros_v.CameraStreamSts, type);
  convert(struct_v.CameraErrorCode, ros_v.CameraErrorCode, type);
}

