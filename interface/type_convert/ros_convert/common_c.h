#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/CommonCameraPerceptionInputTimestamp.h"
#include "struct_msgs_v2_10/CommonCameraPerceptionInputTimestamp.h"
#include "struct_msgs/FloorId.h"
#include "struct_msgs_v2_10/FloorId.h"
#include "struct_msgs/InputHistoryInfo.h"
#include "struct_msgs_v2_10/InputHistoryInfo.h"
#include "struct_msgs/LaneMarkMsg.h"
#include "struct_msgs_v2_10/LaneMarkMsg.h"
#include "struct_msgs/LaneMergeSplitPointData.h"
#include "struct_msgs_v2_10/LaneMergeSplitPointData.h"
#include "struct_msgs/LaneTypeMsg.h"
#include "struct_msgs_v2_10/LaneTypeMsg.h"
#include "struct_msgs/MsgHeader.h"
#include "struct_msgs_v2_10/MsgHeader.h"
#include "struct_msgs/MsgMeta.h"
#include "struct_msgs_v2_10/MsgMeta.h"
#include "struct_msgs/Obstacle.h"
#include "struct_msgs_v2_10/Obstacle.h"
#include "struct_msgs/Point2d.h"
#include "struct_msgs_v2_10/Point2d.h"
#include "struct_msgs/Point2f.h"
#include "struct_msgs_v2_10/Point2f.h"
#include "struct_msgs/Point2si.h"
#include "struct_msgs_v2_10/Point2si.h"
#include "struct_msgs/Point3d.h"
#include "struct_msgs_v2_10/Point3d.h"
#include "struct_msgs/Point3f.h"
#include "struct_msgs_v2_10/Point3f.h"
#include "struct_msgs/Point3si.h"
#include "struct_msgs_v2_10/Point3si.h"
#include "struct_msgs/PointENU.h"
#include "struct_msgs_v2_10/PointENU.h"
#include "struct_msgs/PointLLH.h"
#include "struct_msgs_v2_10/PointLLH.h"
#include "struct_msgs/Quaternion.h"
#include "struct_msgs_v2_10/Quaternion.h"
#include "struct_msgs/SensorMeta.h"
#include "struct_msgs_v2_10/SensorMeta.h"
#include "struct_msgs/Shape3d.h"
#include "struct_msgs_v2_10/Shape3d.h"
#include "struct_msgs/Shape3f.h"
#include "struct_msgs_v2_10/Shape3f.h"
#include "struct_msgs/TileId.h"
#include "struct_msgs_v2_10/TileId.h"
#include "struct_msgs/UrId.h"
#include "struct_msgs_v2_10/UrId.h"
#include "struct_msgs/UtcTime.h"
#include "struct_msgs_v2_10/UtcTime.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::CommonCameraPerceptionInputTimestamp &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.front_isp_timestamp, ros_v.front_isp_timestamp, type);
	convert(old_ros_v.rear_isp_timestamp, ros_v.rear_isp_timestamp, type);
	convert(old_ros_v.fl_isp_timestamp, ros_v.fl_isp_timestamp, type);
	convert(old_ros_v.rl_isp_timestamp, ros_v.rl_isp_timestamp, type);
	convert(old_ros_v.fr_isp_timestamp, ros_v.fr_isp_timestamp, type);
	convert(old_ros_v.rr_isp_timestamp, ros_v.rr_isp_timestamp, type);
	convert(old_ros_v.vehicle_service_timestamp, ros_v.vehicle_service_timestamp, type);
	convert(old_ros_v.pbox_imu_timestamp, ros_v.pbox_imu_timestamp, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::FloorId &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.id, ros_v.id, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::InputHistoryInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.input_type, ros_v.input_type, type);
	convert(old_ros_v.seq, ros_v.seq, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::LaneMarkMsg &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.begin, ros_v.begin, type);
	convert(old_ros_v.end, ros_v.end, type);
	convert(old_ros_v.lane_mark, ros_v.lane_mark, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::LaneMergeSplitPointData &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.distance, ros_v.distance, type);
	convert(old_ros_v.length, ros_v.length, type);
	convert(old_ros_v.is_split, ros_v.is_split, type);
	convert(old_ros_v.is_continue, ros_v.is_continue, type);
	convert(old_ros_v.orientation, ros_v.orientation, type);
	convert(old_ros_v.point, ros_v.point, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::LaneTypeMsg &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.begin, ros_v.begin, type);
	convert(old_ros_v.end, ros_v.end, type);
	convert(old_ros_v.type, ros_v.type, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::MsgHeader &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.seq, ros_v.seq, type);
	convert(old_ros_v.stamp, ros_v.stamp, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::MsgMeta &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.start_time, ros_v.start_time, type);
	for (int i = 0; i < 64; i++) {
	    convert(old_ros_v.version[i], ros_v.version[i], type);
	}
	convert(old_ros_v.input_list_size, ros_v.input_list_size, type);
	ros_v.input_list.resize(old_ros_v.input_list.size());
	for (int i = 0; i < ros_v.input_list.size(); i++) {
	    convert(old_ros_v.input_list[i], ros_v.input_list[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::Obstacle &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.id, ros_v.id, type);
	convert(old_ros_v.type, ros_v.type, type);
	convert(old_ros_v.light_type, ros_v.light_type, type);
	convert(old_ros_v.shape, ros_v.shape, type);
	convert(old_ros_v.relative_position, ros_v.relative_position, type);
	convert(old_ros_v.relative_velocity, ros_v.relative_velocity, type);
	convert(old_ros_v.relative_acceleration, ros_v.relative_acceleration, type);
	convert(old_ros_v.relative_center_position, ros_v.relative_center_position, type);
	convert(old_ros_v.relative_heading_angle, ros_v.relative_heading_angle, type);
	convert(old_ros_v.relative_heading_angle_rate, ros_v.relative_heading_angle_rate, type);
	convert(old_ros_v.position, ros_v.position, type);
	convert(old_ros_v.velocity, ros_v.velocity, type);
	convert(old_ros_v.acceleration, ros_v.acceleration, type);
	convert(old_ros_v.center_position, ros_v.center_position, type);
	convert(old_ros_v.heading_angle, ros_v.heading_angle, type);
	convert(old_ros_v.heading_angle_rate, ros_v.heading_angle_rate, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::Point2d &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.x, ros_v.x, type);
	convert(old_ros_v.y, ros_v.y, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::Point2f &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.x, ros_v.x, type);
	convert(old_ros_v.y, ros_v.y, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::Point2si &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.x, ros_v.x, type);
	convert(old_ros_v.y, ros_v.y, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::Point3d &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.x, ros_v.x, type);
	convert(old_ros_v.y, ros_v.y, type);
	convert(old_ros_v.z, ros_v.z, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::Point3f &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.x, ros_v.x, type);
	convert(old_ros_v.y, ros_v.y, type);
	convert(old_ros_v.z, ros_v.z, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::Point3si &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.x, ros_v.x, type);
	convert(old_ros_v.y, ros_v.y, type);
	convert(old_ros_v.z, ros_v.z, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::PointENU &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.x, ros_v.x, type);
	convert(old_ros_v.y, ros_v.y, type);
	convert(old_ros_v.z, ros_v.z, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::PointLLH &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.lon, ros_v.lon, type);
	convert(old_ros_v.lat, ros_v.lat, type);
	convert(old_ros_v.height, ros_v.height, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::Quaternion &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.w, ros_v.w, type);
	convert(old_ros_v.x, ros_v.x, type);
	convert(old_ros_v.y, ros_v.y, type);
	convert(old_ros_v.z, ros_v.z, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::SensorMeta &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.stamp, ros_v.stamp, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::Shape3d &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.length, ros_v.length, type);
	convert(old_ros_v.width, ros_v.width, type);
	convert(old_ros_v.height, ros_v.height, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::Shape3f &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.length, ros_v.length, type);
	convert(old_ros_v.width, ros_v.width, type);
	convert(old_ros_v.height, ros_v.height, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::TileId &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.id, ros_v.id, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::UrId &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.id, ros_v.id, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::UtcTime &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.year, ros_v.year, type);
	convert(old_ros_v.month, ros_v.month, type);
	convert(old_ros_v.day, ros_v.day, type);
	convert(old_ros_v.hour, ros_v.hour, type);
	convert(old_ros_v.minute, ros_v.minute, type);
	convert(old_ros_v.second, ros_v.second, type);
	convert(old_ros_v.milli_second, ros_v.milli_second, type);
	convert(old_ros_v.time_accuracy, ros_v.time_accuracy, type);
}

