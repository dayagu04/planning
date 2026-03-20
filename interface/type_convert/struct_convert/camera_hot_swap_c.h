#pragma once

#include "base_convert.h"
#include "c/camera_hot_swap_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::CameraRestartRequest &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.reqCnt, ros_v.reqCnt, type);
  convert(struct_v.type, ros_v.type, type);
}

template <typename T2>
void convert(iflyauto::CameraBootReasonMsg &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.reason, ros_v.reason, type);
}

template <typename T2>
void convert(iflyauto::CameraLinkStatusMsg &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.camera_fault_mask, ros_v.camera_fault_mask, type);
  convert(struct_v.camera_link_mask, ros_v.camera_link_mask, type);
}

template <typename T2>
void convert(iflyauto::CameraHotSwapStatus &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.is_camera_restarting, ros_v.is_camera_restarting, type);
}

