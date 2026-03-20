// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef _IFLYAUTO_LIDAR_OBJECTS_H_
#define _IFLYAUTO_LIDAR_OBJECTS_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"

#ifdef __cplusplus
namespace iflyauto {
#endif

#define LIDAR_OBJECT_MAX_NUM 128

#pragma pack(4)

typedef struct {
  uint32 track_age;   // 融合障碍物存在周期   (帧数)
  uint32 confidence;  // 障碍物置信度         [0-100]
} _STRUCT_ALIGNED_ LidarAdditional;

typedef struct {
  Obstacle common_info;             // 障碍物通用信息
  LidarAdditional additional_info;  // 障碍物附加信息
} _STRUCT_ALIGNED_ LidarObject;

typedef struct {
  MsgHeader msg_header;
  MsgMeta msg_meta;
  uint8 lidar_object_size;                     // 激光雷达障碍物数量
  LidarObject lidar_object[LIDAR_OBJECT_MAX_NUM];  // 激光障碍物信息
} _STRUCT_ALIGNED_ LidarObjectsInfo;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_LIDAR_OBJECTS_H_