#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/CameraPerceptionSuppSign.h"
#include "struct_msgs_v2_10/CameraPerceptionSuppSign.h"
#include "struct_msgs/CameraPerceptionTrafficLight.h"
#include "struct_msgs_v2_10/CameraPerceptionTrafficLight.h"
#include "struct_msgs/CameraPerceptionTrafficStatus.h"
#include "struct_msgs_v2_10/CameraPerceptionTrafficStatus.h"
#include "struct_msgs/CameraPerceptionTsrInfo.h"
#include "struct_msgs_v2_10/CameraPerceptionTsrInfo.h"
#include "struct_msgs/TrafficSignBoundingbox.h"
#include "struct_msgs_v2_10/TrafficSignBoundingbox.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::CameraPerceptionSuppSign &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.id, ros_v.id, type);
	convert(old_ros_v.supp_sign_type, ros_v.supp_sign_type, type);
	convert(old_ros_v.supp_sign_x, ros_v.supp_sign_x, type);
	convert(old_ros_v.supp_sign_y, ros_v.supp_sign_y, type);
	convert(old_ros_v.supp_sign_z, ros_v.supp_sign_z, type);
	convert(old_ros_v.speed_limit, ros_v.speed_limit, type);
	convert(old_ros_v.bbox, ros_v.bbox, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::CameraPerceptionTrafficLight &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.id, ros_v.id, type);
	convert(old_ros_v.traffic_light_type, ros_v.traffic_light_type, type);
	convert(old_ros_v.traffic_light_color, ros_v.traffic_light_color, type);
	convert(old_ros_v.traffic_light_arrange_type, ros_v.traffic_light_arrange_type, type);
	convert(old_ros_v.traffic_light_x, ros_v.traffic_light_x, type);
	convert(old_ros_v.traffic_light_y, ros_v.traffic_light_y, type);
	convert(old_ros_v.traffic_light_z, ros_v.traffic_light_z, type);
	convert(old_ros_v.bbox, ros_v.bbox, type);
	convert(old_ros_v.countdown_number, ros_v.countdown_number, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::CameraPerceptionTrafficStatus &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.go_left, ros_v.go_left, type);
	convert(old_ros_v.go_straight, ros_v.go_straight, type);
	convert(old_ros_v.go_right, ros_v.go_right, type);
	convert(old_ros_v.go_uturn, ros_v.go_uturn, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::CameraPerceptionTsrInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.isp_timestamp, ros_v.isp_timestamp, type);
	convert(old_ros_v.supp_signs_size, ros_v.supp_signs_size, type);
	ros_v.supp_signs.resize(old_ros_v.supp_signs.size());
	for (int i = 0; i < ros_v.supp_signs.size(); i++) {
	    convert(old_ros_v.supp_signs[i], ros_v.supp_signs[i], type);
	}
	convert(old_ros_v.traffic_lights_size, ros_v.traffic_lights_size, type);
	ros_v.traffic_lights.resize(old_ros_v.traffic_lights.size());
	for (int i = 0; i < ros_v.traffic_lights.size(); i++) {
	    convert(old_ros_v.traffic_lights[i], ros_v.traffic_lights[i], type);
	}
	convert(old_ros_v.traffic_status, ros_v.traffic_status, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::TrafficSignBoundingbox &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.x, ros_v.x, type);
	convert(old_ros_v.y, ros_v.y, type);
	convert(old_ros_v.width, ros_v.width, type);
	convert(old_ros_v.height, ros_v.height, type);
}

REG_CONVERT_SINGLE(_iflytek_camera_perception_traffic_sign_recognition_converter, "/iflytek/camera_perception/traffic_sign_recognition", CameraPerceptionTsrInfo);
