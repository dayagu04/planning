#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/CameraPerceptionOccObject.h"
#include "struct_msgs_v2_10/CameraPerceptionOccObject.h"
#include "struct_msgs/CameraPerceptionOccObjectsInfo.h"
#include "struct_msgs_v2_10/CameraPerceptionOccObjectsInfo.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::CameraPerceptionOccObject &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.id, ros_v.id, type);
	convert(old_ros_v.type, ros_v.type, type);
	convert(old_ros_v.visable_seg_num, ros_v.visable_seg_num, type);
	convert(old_ros_v.contour_points_size, ros_v.contour_points_size, type);
	ros_v.contour_points.resize(old_ros_v.contour_points.size());
	for (int i = 0; i < ros_v.contour_points.size(); i++) {
	    convert(old_ros_v.contour_points[i], ros_v.contour_points[i], type);
	}
	convert(old_ros_v.life_time, ros_v.life_time, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::CameraPerceptionOccObjectsInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.isp_timestamp, ros_v.isp_timestamp, type);
	convert(old_ros_v.camera_perception_objects_size, ros_v.camera_perception_objects_size, type);
	ros_v.camera_perception_objects.resize(old_ros_v.camera_perception_objects.size());
	for (int i = 0; i < ros_v.camera_perception_objects.size(); i++) {
	    convert(old_ros_v.camera_perception_objects[i], ros_v.camera_perception_objects[i], type);
	}
}

REG_CONVERT_SINGLE(_iflytek_camera_perception_occupancy_objects_converter, "/iflytek/camera_perception/occupancy_objects", CameraPerceptionOccObjectsInfo);
