#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/LidarMsgInfo.h"
#include "struct_msgs_v2_10/LidarMsgInfo.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::LidarMsgInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.sensor_meta, ros_v.sensor_meta, type);
	for (int i = 0; i < 8; i++) {
	    convert(old_ros_v.frame_id[i], ros_v.frame_id[i], type);
	}
	convert(old_ros_v.height, ros_v.height, type);
	convert(old_ros_v.width, ros_v.width, type);
}

REG_CONVERT_SINGLE(_iflytek_sensor_lidar_rsp128_converter, "/iflytek/sensor/lidar/rsp128", LidarMsgInfo);
