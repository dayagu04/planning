#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/LidarAdditional.h"
#include "struct_msgs_v2_10/LidarAdditional.h"
#include "struct_msgs/LidarObject.h"
#include "struct_msgs_v2_10/LidarObject.h"
#include "struct_msgs/LidarObjectsInfo.h"
#include "struct_msgs_v2_10/LidarObjectsInfo.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::LidarAdditional &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.track_age, ros_v.track_age, type);
	convert(old_ros_v.confidence, ros_v.confidence, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::LidarObject &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.common_info, ros_v.common_info, type);
	convert(old_ros_v.additional_info, ros_v.additional_info, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::LidarObjectsInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.lidar_object_size, ros_v.lidar_object_size, type);
	ros_v.lidar_object.resize(old_ros_v.lidar_object.size());
	for (int i = 0; i < ros_v.lidar_object.size(); i++) {
	    convert(old_ros_v.lidar_object[i], ros_v.lidar_object[i], type);
	}
}

