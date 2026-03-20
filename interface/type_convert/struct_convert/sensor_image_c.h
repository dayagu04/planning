#pragma once

#include "base_convert.h"
#include "c/sensor_image_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::Camera_Image_Info &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.sensor_meta, ros_v.sensor_meta, type);
  for (size_t i0 = 0; i0 < ros_v.frame_id.size(); i0++) {
	  convert(struct_v.frame_id[i0], ros_v.frame_id[i0], type);
  }
  for (size_t i1 = 0; i1 < ros_v.format.size(); i1++) {
	  convert(struct_v.format[i1], ros_v.format[i1], type);
  }
  for (size_t i2 = 0; i2 < ros_v.camera_view_angle.size(); i2++) {
	  convert(struct_v.camera_view_angle[i2], ros_v.camera_view_angle[i2], type);
  }
  for (size_t i3 = 0; i3 < ros_v.camera_resolution.size(); i3++) {
	  convert(struct_v.camera_resolution[i3], ros_v.camera_resolution[i3], type);
  }
  convert(struct_v.height, ros_v.height, type);
  convert(struct_v.width, ros_v.width, type);
  convert(struct_v.dmabuf_fd_index, ros_v.dmabuf_fd_index, type);
}

template <typename T2>
void convert(iflyauto::Front_Camera_Image_Info &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.front_camera_data, ros_v.front_camera_data, type);
}

template <typename T2>
void convert(iflyauto::Surround_Camera_Image_Info &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.right_camera_data, ros_v.right_camera_data, type);
  convert(struct_v.front_camera_data, ros_v.front_camera_data, type);
  convert(struct_v.left_camera_data, ros_v.left_camera_data, type);
  convert(struct_v.rear_camera_data, ros_v.rear_camera_data, type);
}

template <typename T2>
void convert(iflyauto::Panorama_Camera_Image_Info &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.front120_camera_data, ros_v.front120_camera_data, type);
  convert(struct_v.rear_camera_data, ros_v.rear_camera_data, type);
  convert(struct_v.side_fl_camera_data, ros_v.side_fl_camera_data, type);
  convert(struct_v.side_fr_camera_data, ros_v.side_fr_camera_data, type);
  convert(struct_v.side_rl_camera_data, ros_v.side_rl_camera_data, type);
  convert(struct_v.side_rr_camera_data, ros_v.side_rr_camera_data, type);
}

