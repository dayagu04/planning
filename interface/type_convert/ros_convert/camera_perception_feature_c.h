#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/CameraPerceptionFeatureInfo.h"
#include "struct_msgs_v2_10/CameraPerceptionFeatureInfo.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::CameraPerceptionFeatureInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.isp_timestamp, ros_v.isp_timestamp, type);
	convert(old_ros_v.index, ros_v.index, type);
	convert(old_ros_v.bev_feature_dim, ros_v.bev_feature_dim, type);
	convert(old_ros_v.bev_feature_num, ros_v.bev_feature_num, type);
	ros_v.bev_feature.resize(old_ros_v.bev_feature.size());
	for (int i = 0; i < ros_v.bev_feature.size(); i++) {
	    convert(old_ros_v.bev_feature[i], ros_v.bev_feature[i], type);
	}
	convert(old_ros_v.img_feature_dim, ros_v.img_feature_dim, type);
	convert(old_ros_v.img_feature_num, ros_v.img_feature_num, type);
	ros_v.img_feature.resize(old_ros_v.img_feature.size());
	for (int i = 0; i < ros_v.img_feature.size(); i++) {
	    convert(old_ros_v.img_feature[i], ros_v.img_feature[i], type);
	}
	convert(old_ros_v.reserved, ros_v.reserved, type);
}

REG_CONVERT_SINGLE(_iflytek_camera_perception_feature_converter, "/iflytek/camera_perception/feature", CameraPerceptionFeatureInfo);
