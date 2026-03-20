#pragma once

#include "base_convert.h"
#include "c/ifly_localization_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::IFLYReserved &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.available, ros_v.available, type);
  for (size_t i0 = 0; i0 < ros_v.reserved_data.size(); i0++) {
	  convert(struct_v.reserved_data[i0], ros_v.reserved_data[i0], type);
  }
}

template <typename T2>
void convert(iflyauto::IFLYLocalizationMeta &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.id, ros_v.id, type);
  convert(struct_v.timestamp, ros_v.timestamp, type);
}

template <typename T2>
void convert(iflyauto::IFLYAccGlobal &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.ae, ros_v.ae, type);
  convert(struct_v.an, ros_v.an, type);
  convert(struct_v.au, ros_v.au, type);
}

template <typename T2>
void convert(iflyauto::IFLYAccLocal &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.ax, ros_v.ax, type);
  convert(struct_v.ay, ros_v.ay, type);
  convert(struct_v.az, ros_v.az, type);
}

template <typename T2>
void convert(iflyauto::IFLYAcceleration &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.acceleration_enu, ros_v.acceleration_enu, type);
  convert(struct_v.acceleration_boot, ros_v.acceleration_boot, type);
  convert(struct_v.acceleration_body, ros_v.acceleration_body, type);
}

template <typename T2>
void convert(iflyauto::IFLYAngVelLocal &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.vx, ros_v.vx, type);
  convert(struct_v.vy, ros_v.vy, type);
  convert(struct_v.vz, ros_v.vz, type);
}

template <typename T2>
void convert(iflyauto::IFLYAngularVelocity &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.angvelocity_boot, ros_v.angvelocity_boot, type);
  convert(struct_v.angvelocity_body, ros_v.angvelocity_body, type);
}

template <typename T2>
void convert(iflyauto::IFLYVelGlobal &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.available, ros_v.available, type);
  convert(struct_v.ve, ros_v.ve, type);
  convert(struct_v.vn, ros_v.vn, type);
  convert(struct_v.vu, ros_v.vu, type);
}

template <typename T2>
void convert(iflyauto::IFLYVelLocal &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.vx, ros_v.vx, type);
  convert(struct_v.vy, ros_v.vy, type);
  convert(struct_v.vz, ros_v.vz, type);
}

template <typename T2>
void convert(iflyauto::IFLYVelocity &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.velocity_enu, ros_v.velocity_enu, type);
  convert(struct_v.velocity_boot, ros_v.velocity_boot, type);
  convert(struct_v.velocity_body, ros_v.velocity_body, type);
}

template <typename T2>
void convert(iflyauto::IFLYEuler &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.yaw, ros_v.yaw, type);
  convert(struct_v.pitch, ros_v.pitch, type);
  convert(struct_v.roll, ros_v.roll, type);
}

template <typename T2>
void convert(iflyauto::IFLYQuaternion &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.w, ros_v.w, type);
  convert(struct_v.x, ros_v.x, type);
  convert(struct_v.y, ros_v.y, type);
  convert(struct_v.z, ros_v.z, type);
}

template <typename T2>
void convert(iflyauto::IFLYOrientation &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.euler_enu, ros_v.euler_enu, type);
  convert(struct_v.quaternion_enu, ros_v.quaternion_enu, type);
  convert(struct_v.euler_boot, ros_v.euler_boot, type);
  convert(struct_v.quaternion_boot, ros_v.quaternion_boot, type);
}

template <typename T2>
void convert(iflyauto::IFLYPosGlobal &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.available, ros_v.available, type);
  convert(struct_v.type, ros_v.type, type);
  convert(struct_v.longitude, ros_v.longitude, type);
  convert(struct_v.latitude, ros_v.latitude, type);
  convert(struct_v.height, ros_v.height, type);
}

template <typename T2>
void convert(iflyauto::IFLYPosLocal &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.available, ros_v.available, type);
  convert(struct_v.x, ros_v.x, type);
  convert(struct_v.y, ros_v.y, type);
  convert(struct_v.z, ros_v.z, type);
}

template <typename T2>
void convert(iflyauto::IFLYPosEnu &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.available, ros_v.available, type);
  convert(struct_v.e, ros_v.e, type);
  convert(struct_v.n, ros_v.n, type);
  convert(struct_v.u, ros_v.u, type);
}

template <typename T2>
void convert(iflyauto::IFLYPosition &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.position_llh, ros_v.position_llh, type);
  convert(struct_v.position_boot, ros_v.position_boot, type);
  convert(struct_v.position_enu, ros_v.position_enu, type);
}

template <typename T2>
void convert(iflyauto::IFLYTransformInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.available, ros_v.available, type);
  convert(struct_v.transform_q, ros_v.transform_q, type);
  convert(struct_v.transform_t, ros_v.transform_t, type);
  convert(struct_v.transform_center, ros_v.transform_center, type);
}

template <typename T2>
void convert(iflyauto::IFLYTransform &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.transform_llh_to_boot, ros_v.transform_llh_to_boot, type);
  convert(struct_v.transform_ego_motion_to_boot, ros_v.transform_ego_motion_to_boot, type);
}

template <typename T2>
void convert(iflyauto::IFLYStatusInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.mode, ros_v.mode, type);
  convert(struct_v.common, ros_v.common, type);
  convert(struct_v.extended, ros_v.extended, type);
}

template <typename T2>
void convert(iflyauto::IFLYStatus &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.status_info, ros_v.status_info, type);
}

template <typename T2>
void convert(iflyauto::IFLYPoseDetailInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.type, ros_v.type, type);
  convert(struct_v.is_keyframe, ros_v.is_keyframe, type);
  convert(struct_v.has_scale, ros_v.has_scale, type);
  convert(struct_v.pose_type, ros_v.pose_type, type);
  convert(struct_v.map_floor_id, ros_v.map_floor_id, type);
}

template <typename T2>
void convert(iflyauto::IFLYPoseDetail &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.pose_detail_info, ros_v.pose_detail_info, type);
}

template <typename T2>
void convert(iflyauto::IFLYLocalizationException &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.type, ros_v.type, type);
  for (size_t i0 = 0; i0 < ros_v.reserved_data.size(); i0++) {
	  convert(struct_v.reserved_data[i0], ros_v.reserved_data[i0], type);
  }
}

template <typename T2>
void convert(iflyauto::IFLYLocalization &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.reserved, ros_v.reserved, type);
  convert(struct_v.meta, ros_v.meta, type);
  convert(struct_v.acceleration, ros_v.acceleration, type);
  convert(struct_v.angular_velocity, ros_v.angular_velocity, type);
  convert(struct_v.velocity, ros_v.velocity, type);
  convert(struct_v.orientation, ros_v.orientation, type);
  convert(struct_v.position, ros_v.position, type);
  convert(struct_v.transform, ros_v.transform, type);
  convert(struct_v.status, ros_v.status, type);
  convert(struct_v.pose_detail, ros_v.pose_detail, type);
  convert(struct_v.exception, ros_v.exception, type);
}

