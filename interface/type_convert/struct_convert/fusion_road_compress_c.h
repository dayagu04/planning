#pragma once

#include "base_convert.h"
#include "c/fusion_road_compress_c.h"
#include "struct_convert/common_c.h"
#include "struct_convert/fusion_road_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::LaneBoundaryCompress &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.existence, ros_v.existence, type);
  convert(struct_v.type, ros_v.type, type);
  for (size_t i0 = 0; i0 < ros_v.poly_coefficient.size(); i0++) {
	  convert(struct_v.poly_coefficient[i0], ros_v.poly_coefficient[i0], type);
  }
  convert(struct_v.begin, ros_v.begin, type);
  convert(struct_v.end, ros_v.end, type);
  convert(struct_v.line_type, ros_v.line_type, type);
}

template <typename T2>
void convert(iflyauto::RoadInfoCompress &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.isp_timestamp, ros_v.isp_timestamp, type);
  for (size_t i0 = 0; i0 < ros_v.line_info.size(); i0++) {
	  convert(struct_v.line_info[i0], ros_v.line_info[i0], type);
  }
}

