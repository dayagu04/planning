#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/CameraPerceptionScene.h"
#include "struct_msgs_v2_10/CameraPerceptionScene.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::CameraPerceptionScene &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.isp_timestamp, ros_v.isp_timestamp, type);
	convert(old_ros_v.road_type, ros_v.road_type, type);
	convert(old_ros_v.weather_condition, ros_v.weather_condition, type);
	convert(old_ros_v.traffic_condition, ros_v.traffic_condition, type);
	convert(old_ros_v.lighting_condition, ros_v.lighting_condition, type);
}

REG_CONVERT_SINGLE(_iflytek_camera_perception_scene_converter, "/iflytek/camera_perception/scene", CameraPerceptionScene);
