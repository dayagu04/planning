#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/FdlUploadState.h"
#include "struct_msgs_v2_10/FdlUploadState.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::FdlUploadState &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.upload_state, ros_v.upload_state, type);
}

REG_CONVERT_SINGLE(_iflytek_fdl_upload_state_converter, "/iflytek/fdl/upload_state", FdlUploadState);
