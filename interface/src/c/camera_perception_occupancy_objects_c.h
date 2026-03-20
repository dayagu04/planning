// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2024/07/03

#ifndef _IFLYAUTO_CAMERA_PERCEPTION_OCCUPANCY_OBJECTS_H_
#define _IFLYAUTO_CAMERA_PERCEPTION_OCCUPANCY_OBJECTS_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"

#define CAMERA_PERCEPTION_OCC_OBJECT_MAX_NUM 64  //最大输出障碍物个数
#define CAMERA_PERCEPTION_OCC_POINT_MAX_NUM 576  //障碍物的点理论值最大为（320+256）

#ifdef __cplusplus
namespace iflyauto {
#endif

#pragma pack(4)

// 通用障碍物信息
typedef struct {
  uint8 id;                                           // 障碍物id
  ObjectType type;                                    // 障碍物类型
  int32 visable_seg_num;                              // 值大于0点有序，数值表示前多少个可见点，其余点为预测点;值小于0点无序，点全为预测点
  uint32 contour_points_size;                         // 障碍物点集中点数量
  Point3f contour_points[CAMERA_PERCEPTION_OCC_POINT_MAX_NUM];   // 障碍物点集
  int32 life_time;                                    // 生命周期         (毫秒)
} _STRUCT_ALIGNED_ CameraPerceptionOccObject;

// 通用障碍物信息集合
typedef struct {
  MsgHeader msg_header;
  MsgMeta msg_meta;
  uint64 isp_timestamp;                                                            // ISP出图时间戳    (微秒)
  uint8 camera_perception_objects_size;                                            // 障碍物数量
  CameraPerceptionOccObject camera_perception_objects[CAMERA_PERCEPTION_OCC_OBJECT_MAX_NUM];  // 障碍物列表
} _STRUCT_ALIGNED_ CameraPerceptionOccObjectsInfo;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_CAMERA_PERCEPTION_OCCUPANCY_OBJECTS_H_