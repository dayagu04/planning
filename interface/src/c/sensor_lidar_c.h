// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2024/01/14

#ifndef _IFLYAUTO_SENSOR_LIDAR_H_
#define _IFLYAUTO_SENSOR_LIDAR_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"

#ifdef __cplusplus
namespace iflyauto {
#endif

#pragma pack(4)

#define SENSOR_LIDAR_FRAME_ID_MAX_NUM 8

// 雷达消息结构
typedef struct {
  MsgHeader msg_header;                             // SOC消息发送信息
  SensorMeta sensor_meta;                           // lidar消息的时间戳
  uint8 frame_id[SENSOR_LIDAR_FRAME_ID_MAX_NUM];                     // 图片帧数
  uint32 height;                                    // 点云的高
  uint32 width;                                     // 点云的宽
} _STRUCT_ALIGNED_ LidarMsgInfo;

// 点云所有点信息存放mbuf，单点信息结构体如下
// mbuf size = sizeof(PointXYZIRT) * height * width
// 速腾RS128 mbuf size = 32 * 128 * 1800 = 7372800

/*****************************************************************
单点信息结构体
#pragma once
#include <pcl/io/io.h>
#include <pcl/point_types.h>
typedef pcl::PointXYZI PointXYZI;
struct PointXYZIRT
{
  PCL_ADD_POINT4D;
  float intensity;
  std::uint16_t channel;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT, 
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint16_t, channel, channel)
    (double, timestamp, timestamp))
*******************************************************************/

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_SENSOR_LIDAR_H_