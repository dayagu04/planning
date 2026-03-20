#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/IFLYAccGlobal.h"
#include "struct_msgs_v2_10/IFLYAccGlobal.h"
#include "struct_msgs/IFLYAccLocal.h"
#include "struct_msgs_v2_10/IFLYAccLocal.h"
#include "struct_msgs/IFLYAcceleration.h"
#include "struct_msgs_v2_10/IFLYAcceleration.h"
#include "struct_msgs/IFLYAngVelLocal.h"
#include "struct_msgs_v2_10/IFLYAngVelLocal.h"
#include "struct_msgs/IFLYAngularVelocity.h"
#include "struct_msgs_v2_10/IFLYAngularVelocity.h"
#include "struct_msgs/IFLYEuler.h"
#include "struct_msgs_v2_10/IFLYEuler.h"
#include "struct_msgs/IFLYLocalization.h"
#include "struct_msgs_v2_10/IFLYLocalization.h"
#include "struct_msgs/IFLYLocalizationException.h"
#include "struct_msgs_v2_10/IFLYLocalizationException.h"
#include "struct_msgs/IFLYLocalizationMeta.h"
#include "struct_msgs_v2_10/IFLYLocalizationMeta.h"
#include "struct_msgs/IFLYOrientation.h"
#include "struct_msgs_v2_10/IFLYOrientation.h"
#include "struct_msgs/IFLYPosEnu.h"
#include "struct_msgs_v2_10/IFLYPosEnu.h"
#include "struct_msgs/IFLYPosGlobal.h"
#include "struct_msgs_v2_10/IFLYPosGlobal.h"
#include "struct_msgs/IFLYPosLocal.h"
#include "struct_msgs_v2_10/IFLYPosLocal.h"
#include "struct_msgs/IFLYPoseDetail.h"
#include "struct_msgs_v2_10/IFLYPoseDetail.h"
#include "struct_msgs/IFLYPoseDetailInfo.h"
#include "struct_msgs_v2_10/IFLYPoseDetailInfo.h"
#include "struct_msgs/IFLYPosition.h"
#include "struct_msgs_v2_10/IFLYPosition.h"
#include "struct_msgs/IFLYQuaternion.h"
#include "struct_msgs_v2_10/IFLYQuaternion.h"
#include "struct_msgs/IFLYReserved.h"
#include "struct_msgs_v2_10/IFLYReserved.h"
#include "struct_msgs/IFLYStatus.h"
#include "struct_msgs_v2_10/IFLYStatus.h"
#include "struct_msgs/IFLYStatusInfo.h"
#include "struct_msgs_v2_10/IFLYStatusInfo.h"
#include "struct_msgs/IFLYTransform.h"
#include "struct_msgs_v2_10/IFLYTransform.h"
#include "struct_msgs/IFLYTransformInfo.h"
#include "struct_msgs_v2_10/IFLYTransformInfo.h"
#include "struct_msgs/IFLYVelGlobal.h"
#include "struct_msgs_v2_10/IFLYVelGlobal.h"
#include "struct_msgs/IFLYVelLocal.h"
#include "struct_msgs_v2_10/IFLYVelLocal.h"
#include "struct_msgs/IFLYVelocity.h"
#include "struct_msgs_v2_10/IFLYVelocity.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::IFLYAccGlobal &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.ae, ros_v.ae, type);
	convert(old_ros_v.an, ros_v.an, type);
	convert(old_ros_v.au, ros_v.au, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::IFLYAccLocal &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.ax, ros_v.ax, type);
	convert(old_ros_v.ay, ros_v.ay, type);
	convert(old_ros_v.az, ros_v.az, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::IFLYAcceleration &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.acceleration_enu, ros_v.acceleration_enu, type);
	convert(old_ros_v.acceleration_boot, ros_v.acceleration_boot, type);
	convert(old_ros_v.acceleration_body, ros_v.acceleration_body, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::IFLYAngVelLocal &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.vx, ros_v.vx, type);
	convert(old_ros_v.vy, ros_v.vy, type);
	convert(old_ros_v.vz, ros_v.vz, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::IFLYAngularVelocity &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.angvelocity_boot, ros_v.angvelocity_boot, type);
	convert(old_ros_v.angvelocity_body, ros_v.angvelocity_body, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::IFLYEuler &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.yaw, ros_v.yaw, type);
	convert(old_ros_v.pitch, ros_v.pitch, type);
	convert(old_ros_v.roll, ros_v.roll, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::IFLYLocalization &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.reserved, ros_v.reserved, type);
	convert(old_ros_v.meta, ros_v.meta, type);
	convert(old_ros_v.acceleration, ros_v.acceleration, type);
	convert(old_ros_v.angular_velocity, ros_v.angular_velocity, type);
	convert(old_ros_v.velocity, ros_v.velocity, type);
	convert(old_ros_v.orientation, ros_v.orientation, type);
	convert(old_ros_v.position, ros_v.position, type);
	convert(old_ros_v.transform, ros_v.transform, type);
	convert(old_ros_v.status, ros_v.status, type);
	convert(old_ros_v.pose_detail, ros_v.pose_detail, type);
	convert(old_ros_v.exception, ros_v.exception, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::IFLYLocalizationException &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.type, ros_v.type, type);
	for (int i = 0; i < 16; i++) {
	    convert(old_ros_v.reserved_data[i], ros_v.reserved_data[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::IFLYLocalizationMeta &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.id, ros_v.id, type);
	convert(old_ros_v.timestamp, ros_v.timestamp, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::IFLYOrientation &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.euler_enu, ros_v.euler_enu, type);
	convert(old_ros_v.quaternion_enu, ros_v.quaternion_enu, type);
	convert(old_ros_v.euler_boot, ros_v.euler_boot, type);
	convert(old_ros_v.quaternion_boot, ros_v.quaternion_boot, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::IFLYPosEnu &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.available, ros_v.available, type);
	convert(old_ros_v.e, ros_v.e, type);
	convert(old_ros_v.n, ros_v.n, type);
	convert(old_ros_v.u, ros_v.u, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::IFLYPosGlobal &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.available, ros_v.available, type);
	convert(old_ros_v.longitude, ros_v.longitude, type);
	convert(old_ros_v.latitude, ros_v.latitude, type);
	convert(old_ros_v.height, ros_v.height, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::IFLYPosLocal &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.available, ros_v.available, type);
	convert(old_ros_v.x, ros_v.x, type);
	convert(old_ros_v.y, ros_v.y, type);
	convert(old_ros_v.z, ros_v.z, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::IFLYPoseDetail &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.pose_detail_info, ros_v.pose_detail_info, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::IFLYPoseDetailInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.type, ros_v.type, type);
	convert(old_ros_v.is_keyframe, ros_v.is_keyframe, type);
	convert(old_ros_v.has_scale, ros_v.has_scale, type);
	convert(old_ros_v.pose_type, ros_v.pose_type, type);
	convert(old_ros_v.map_floor_id, ros_v.map_floor_id, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::IFLYPosition &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.position_llh, ros_v.position_llh, type);
	convert(old_ros_v.position_boot, ros_v.position_boot, type);
	convert(old_ros_v.position_enu, ros_v.position_enu, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::IFLYQuaternion &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.w, ros_v.w, type);
	convert(old_ros_v.x, ros_v.x, type);
	convert(old_ros_v.y, ros_v.y, type);
	convert(old_ros_v.z, ros_v.z, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::IFLYReserved &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.available, ros_v.available, type);
	for (int i = 0; i < 16; i++) {
	    convert(old_ros_v.reserved_data[i], ros_v.reserved_data[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::IFLYStatus &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.status_info, ros_v.status_info, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::IFLYStatusInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.mode, ros_v.mode, type);
	convert(old_ros_v.common, ros_v.common, type);
	convert(old_ros_v.extended, ros_v.extended, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::IFLYTransform &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.transform_llh_to_boot, ros_v.transform_llh_to_boot, type);
	convert(old_ros_v.transform_ego_motion_to_boot, ros_v.transform_ego_motion_to_boot, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::IFLYTransformInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.available, ros_v.available, type);
	convert(old_ros_v.transform_q, ros_v.transform_q, type);
	convert(old_ros_v.transform_t, ros_v.transform_t, type);
	convert(old_ros_v.transform_center, ros_v.transform_center, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::IFLYVelGlobal &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.available, ros_v.available, type);
	convert(old_ros_v.ve, ros_v.ve, type);
	convert(old_ros_v.vn, ros_v.vn, type);
	convert(old_ros_v.vu, ros_v.vu, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::IFLYVelLocal &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.vx, ros_v.vx, type);
	convert(old_ros_v.vy, ros_v.vy, type);
	convert(old_ros_v.vz, ros_v.vz, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::IFLYVelocity &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.velocity_enu, ros_v.velocity_enu, type);
	convert(old_ros_v.velocity_boot, ros_v.velocity_boot, type);
	convert(old_ros_v.velocity_body, ros_v.velocity_body, type);
}

REG_CONVERT_SINGLE(_iflytek_localization_egomotion_converter, "/iflytek/localization/egomotion", IFLYLocalization);
