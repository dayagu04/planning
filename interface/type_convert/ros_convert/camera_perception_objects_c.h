#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/CameraPerceptionAdditional.h"
#include "struct_msgs_v2_10/CameraPerceptionAdditional.h"
#include "struct_msgs/CameraPerceptionObject.h"
#include "struct_msgs_v2_10/CameraPerceptionObject.h"
#include "struct_msgs/CameraPerceptionObjectsInfo.h"
#include "struct_msgs_v2_10/CameraPerceptionObjectsInfo.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::CameraPerceptionAdditional &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.sensor_type, ros_v.sensor_type, type);
	convert(old_ros_v.bounding_box_points_size, ros_v.bounding_box_points_size, type);
	ros_v.bounding_box_points.resize(old_ros_v.bounding_box_points.size());
	for (int i = 0; i < ros_v.bounding_box_points.size(); i++) {
	    convert(old_ros_v.bounding_box_points[i], ros_v.bounding_box_points[i], type);
	}
	convert(old_ros_v.contour_points_size, ros_v.contour_points_size, type);
	ros_v.contour_points.resize(old_ros_v.contour_points.size());
	for (int i = 0; i < ros_v.contour_points.size(); i++) {
	    convert(old_ros_v.contour_points[i], ros_v.contour_points[i], type);
	}
	convert(old_ros_v.life_time, ros_v.life_time, type);
	convert(old_ros_v.age, ros_v.age, type);
	convert(old_ros_v.conf, ros_v.conf, type);
	convert(old_ros_v.orientation_angle_conf, ros_v.orientation_angle_conf, type);
	convert(old_ros_v.vel_conf, ros_v.vel_conf, type);
	convert(old_ros_v.accel_conf, ros_v.accel_conf, type);
	convert(old_ros_v.is_fusion, ros_v.is_fusion, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::CameraPerceptionObject &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.common_info, ros_v.common_info, type);
	convert(old_ros_v.additional_info, ros_v.additional_info, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::CameraPerceptionObjectsInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.isp_timestamp, ros_v.isp_timestamp, type);
	convert(old_ros_v.camera_perception_objects_size, ros_v.camera_perception_objects_size, type);
	ros_v.camera_perception_objects.resize(old_ros_v.camera_perception_objects.size());
	for (int i = 0; i < ros_v.camera_perception_objects.size(); i++) {
	    convert(old_ros_v.camera_perception_objects[i], ros_v.camera_perception_objects[i], type);
	}
	convert(old_ros_v.camera_perception_input_timestamp, ros_v.camera_perception_input_timestamp, type);
}

REG_CONVERT_SINGLE(_iflytek_camera_perception_3d_general_objects_converter, "/iflytek/camera_perception/3d_general_objects", CameraPerceptionObjectsInfo);
REG_CONVERT_SINGLE(_iflytek_camera_perception_objects_converter, "/iflytek/camera_perception/objects", CameraPerceptionObjectsInfo);
REG_CONVERT_SINGLE(_mobileye_camera_perception_objects_converter, "/mobileye/camera_perception/objects", CameraPerceptionObjectsInfo);
