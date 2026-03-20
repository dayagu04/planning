// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef _IFLYAUTO_LEGACY_INTERFACE2_4_6_LOCALIZATION_H_
#define _IFLYAUTO_LEGACY_INTERFACE2_4_6_LOCALIZATION_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "interface2.4.6/common_c.h"

#ifdef __cplusplus
namespace iflyauto {
  namespace interface_2_4_6 {
#endif

#pragma pack(4)

typedef enum {
  Pose_Type_NONE = 0,
  Pose_Type_LOCATION_ENU = 1,
  Pose_Type_LOCATION_LLH = 2,
  Pose_Type_LOCATION_LOCAL = 3
} _ENUM_PACKED_ LocationType;

typedef enum {
  MsfStatusType_NORMAL = 0,
  MsfStatusType_WARNING = 1,
  MsfStatusType_ERROR = 2
} _ENUM_PACKED_ MsfStatusType;

typedef enum {
  OddStatusType_OFF_ODD = 0,  //
  OddStatusType_IN_ODD = 1
} _ENUM_PACKED_ OddStatusType;

typedef struct {
  float64 yaw;
  float64 pitch;
  float64 roll;
} _STRUCT_ALIGNED_ EulerAngle;

typedef struct {
  LocationType type;      // 使用的定位方式
  PointENU enu_position;  // 东北天坐标           (米)
  PointLLH llh_position;  // 经纬高坐标           (米)

  /** 自定义绝对坐标系
   *  单位：米
   *  备注：上电时刻确定坐标原点
   **/
  Point3d local_position;

  /** 姿态四元数
   *  备注：IMU在绝对坐标系下四元数旋转矩阵
   *  A quaternion that represents the rotation from the IMU coordinate
   *  (Right/Forward/Up) to the world coordinate (East/North/Up).
   **/
  Quaternion orientation;

  /** 后轴中心线加速度
   *  单位：m/s^2
   *  备注：Linear acceleration of the vehicle in the map reference frame.
   **/
  Point3d linear_acceleration;

  /** 后轴中心角速度
   *  单位：rad/s
   *  备注：Angular velocity of the vehicle in the map reference frame.
   **/
  Point3d angular_velocity;

  /** 后轴中心速度的朝向
   *  单位：rad
   *  备注：绝对坐标系下的朝向角
   **/
  float64 heading;

  /** 车身姿态，欧拉角
   *  单位：rad
   *  备注：顺序yaw, pitch, roll。在绝对坐标系下，
   *  The roll, in (-pi/2, pi/2), corresponds to a rotation around the y-axis.
   *  The pitch, in [-pi, pi), corresponds to a rotation around the x-axis.
   *  The yaw, in [-pi, pi), corresponds to a rotation around the z-axis.
   *  The direction of rotation follows the right-hand rule.
   **/
  EulerAngle euler_angles;

  /** 后轴中心线速度
   *  单位：m/s
   *  备注：基于轮速计算得到
   **/
  float64 linear_velocity_from_wheel;
} _STRUCT_ALIGNED_ Pose;

typedef struct {
  boolean available;
  MsfStatusType msf_status;
} _STRUCT_ALIGNED_ MsfStatus;

typedef struct {
  boolean available;
  OddStatusType odd_status;
} _STRUCT_ALIGNED_ OddStatus;

typedef struct {
  float64 var_x;
  float64 var_y;
  float64 var_theta;
} _STRUCT_ALIGNED_ Uncertainty;

typedef struct {
  Header header;
  Pose pose;
  MsfStatus msf_status;
  Uncertainty uncertainty;
} _STRUCT_ALIGNED_ LocalizationEstimate;

#pragma pack()
#ifdef __cplusplus
  }  // namespace interface_2_4_6
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_LEGACY_INTERFACE2_4_6_LOCALIZATION_H_