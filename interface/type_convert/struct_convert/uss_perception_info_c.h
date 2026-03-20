#pragma once

#include "base_convert.h"
#include "c/uss_perception_info_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::UssObjPointInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.obj_point_type, ros_v.obj_point_type, type);
  convert(struct_v.obj_point_xpos, ros_v.obj_point_xpos, type);
  convert(struct_v.obj_point_ypos, ros_v.obj_point_ypos, type);
}

template <typename T2>
void convert(iflyauto::ApaSlotOutlineCoordinateDataType &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  for (size_t i0 = 0; i0 < ros_v.obj_pt_global.size(); i0++) {
	  convert(struct_v.obj_pt_global[i0], ros_v.obj_pt_global[i0], type);
  }
  for (size_t i1 = 0; i1 < ros_v.obj_pt_local.size(); i1++) {
	  convert(struct_v.obj_pt_local[i1], ros_v.obj_pt_local[i1], type);
  }
  for (size_t i2 = 0; i2 < ros_v.point_high.size(); i2++) {
	  convert(struct_v.point_high[i2], ros_v.point_high[i2], type);
  }
  for (size_t i3 = 0; i3 < ros_v.obj_type.size(); i3++) {
	  convert(struct_v.obj_type[i3], ros_v.obj_type[i3], type);
  }
  convert(struct_v.wr_index, ros_v.wr_index, type);
  convert(struct_v.obj_pt_cnt, ros_v.obj_pt_cnt, type);
}

template <typename T2>
void convert(iflyauto::UssPerceptInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  for (size_t i0 = 0; i0 < ros_v.dis_from_car_to_obj.size(); i0++) {
	  convert(struct_v.dis_from_car_to_obj[i0], ros_v.dis_from_car_to_obj[i0], type);
  }
  for (size_t i1 = 0; i1 < ros_v.out_line_dataori.size(); i1++) {
	  convert(struct_v.out_line_dataori[i1], ros_v.out_line_dataori[i1], type);
  }
}

