#pragma once

#include "base_convert.h"
#include "c/uss_model_info_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::UssModelCoordinateDataType &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  for (size_t i0 = 0; i0 < ros_v.obj_pt_global.size(); i0++) {
	  convert(struct_v.obj_pt_global[i0], ros_v.obj_pt_global[i0], type);
  }
  for (size_t i1 = 0; i1 < ros_v.obj_pt_local.size(); i1++) {
	  convert(struct_v.obj_pt_local[i1], ros_v.obj_pt_local[i1], type);
  }
  for (size_t i2 = 0; i2 < ros_v.obj_type0.size(); i2++) {
	  convert(struct_v.obj_type0[i2], ros_v.obj_type0[i2], type);
  }
  convert(struct_v.wr_index, ros_v.wr_index, type);
  convert(struct_v.obj_pt_cnt, ros_v.obj_pt_cnt, type);
}

template <typename T2>
void convert(iflyauto::UssModelPosition &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.position, ros_v.position, type);
  convert(struct_v.yaw, ros_v.yaw, type);
}

template <typename T2>
void convert(iflyauto::UssModelInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  for (size_t i0 = 0; i0 < ros_v.uss_wave_distance.size(); i0++) {
	  convert(struct_v.uss_wave_distance[i0], ros_v.uss_wave_distance[i0], type);
  }
  convert(struct_v.uss_wave_position, ros_v.uss_wave_position, type);
  for (size_t i1 = 0; i1 < ros_v.dis_from_car_to_obj.size(); i1++) {
	  convert(struct_v.dis_from_car_to_obj[i1], ros_v.dis_from_car_to_obj[i1], type);
  }
  for (size_t i2 = 0; i2 < ros_v.out_line_dataori.size(); i2++) {
	  convert(struct_v.out_line_dataori[i2], ros_v.out_line_dataori[i2], type);
  }
}

