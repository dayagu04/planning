#pragma once

#include "base_convert.h"
#include "c/sensor_radar_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::RadarPointCloudUnit &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.matched_obj_id, ros_v.matched_obj_id, type);
  convert(struct_v.rng_m, ros_v.rng_m, type);
  convert(struct_v.azi_deg, ros_v.azi_deg, type);
  convert(struct_v.ele_deg, ros_v.ele_deg, type);
  convert(struct_v.x, ros_v.x, type);
  convert(struct_v.y, ros_v.y, type);
  convert(struct_v.z, ros_v.z, type);
  convert(struct_v.motion_status, ros_v.motion_status, type);
  convert(struct_v.motion_speed, ros_v.motion_speed, type);
  convert(struct_v.dpl_raw_mps, ros_v.dpl_raw_mps, type);
  convert(struct_v.dpl_unambi_mps, ros_v.dpl_unambi_mps, type);
  convert(struct_v.bandwidth_mode, ros_v.bandwidth_mode, type);
  convert(struct_v.snr_db, ros_v.snr_db, type);
  convert(struct_v.rcs_dbsm, ros_v.rcs_dbsm, type);
  convert(struct_v.is_peek, ros_v.is_peek, type);
  convert(struct_v.is_ghost, ros_v.is_ghost, type);
  convert(struct_v.azimuth_order, ros_v.azimuth_order, type);
}

template <typename T2>
void convert(iflyauto::RadarPointCloud &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.sensor_meta, ros_v.sensor_meta, type);
  convert(struct_v.short_dpl_unambi_scale_mps, ros_v.short_dpl_unambi_scale_mps, type);
  convert(struct_v.long_dpl_unambi_scale_mps, ros_v.long_dpl_unambi_scale_mps, type);
  convert(struct_v.is_valid, ros_v.is_valid, type);
  convert(struct_v.pointcloud_size, ros_v.pointcloud_size, type);
  for (size_t i0 = 0; i0 < ros_v.pointcloud_unit.size(); i0++) {
	  convert(struct_v.pointcloud_unit[i0], ros_v.pointcloud_unit[i0], type);
  }
}

