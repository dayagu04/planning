#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/RadarPointCloud.h"
#include "struct_msgs_v2_10/RadarPointCloud.h"
#include "struct_msgs/RadarPointCloudUnit.h"
#include "struct_msgs_v2_10/RadarPointCloudUnit.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::RadarPointCloud &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.sensor_meta, ros_v.sensor_meta, type);
	convert(old_ros_v.short_dpl_unambi_scale_mps, ros_v.short_dpl_unambi_scale_mps, type);
	convert(old_ros_v.long_dpl_unambi_scale_mps, ros_v.long_dpl_unambi_scale_mps, type);
	convert(old_ros_v.is_valid, ros_v.is_valid, type);
	convert(old_ros_v.pointcloud_size, ros_v.pointcloud_size, type);
	for (int i = 0; i < 2048; i++) {
	    convert(old_ros_v.pointcloud_unit[i], ros_v.pointcloud_unit[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::RadarPointCloudUnit &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.matched_obj_id, ros_v.matched_obj_id, type);
	convert(old_ros_v.rng_m, ros_v.rng_m, type);
	convert(old_ros_v.azi_deg, ros_v.azi_deg, type);
	convert(old_ros_v.ele_deg, ros_v.ele_deg, type);
	convert(old_ros_v.x, ros_v.x, type);
	convert(old_ros_v.y, ros_v.y, type);
	convert(old_ros_v.z, ros_v.z, type);
	convert(old_ros_v.motion_status, ros_v.motion_status, type);
	convert(old_ros_v.motion_speed, ros_v.motion_speed, type);
	convert(old_ros_v.dpl_raw_mps, ros_v.dpl_raw_mps, type);
	convert(old_ros_v.dpl_unambi_mps, ros_v.dpl_unambi_mps, type);
	convert(old_ros_v.bandwidth_mode, ros_v.bandwidth_mode, type);
	convert(old_ros_v.snr_db, ros_v.snr_db, type);
	convert(old_ros_v.rcs_dbsm, ros_v.rcs_dbsm, type);
	convert(old_ros_v.is_peek, ros_v.is_peek, type);
	convert(old_ros_v.is_ghost, ros_v.is_ghost, type);
	convert(old_ros_v.azimuth_order, ros_v.azimuth_order, type);
}

REG_CONVERT_SINGLE(_iflytek_radar_fm_perception_4d_point_cloud_converter, "/iflytek/radar_fm_perception/4d_point_cloud", RadarPointCloud);
