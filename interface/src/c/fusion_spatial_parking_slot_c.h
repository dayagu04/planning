// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2024/10/21

#ifndef _IFLYAUTO_FUSION_SPATIAL_PARKING_SLOT_H_
#define _IFLYAUTO_FUSION_SPATIAL_PARKING_SLOT_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"

#ifdef __cplusplus
namespace iflyauto {
#endif

#define FUSION_SPATIAL_PARKING_SLOT_BORDER_TYPE_MAX_NUM 4
#define FUSION_SPATIAL_PARKING_SLOT_CORNER_POINT_MAX_NUM 4
#define FUSION_SPATIAL_PARKING_SLOT_MAX_NUM 6
#define FUSION_SPATIAL_PARKING_SLOT_COLLISION_QUADRANT 6

#pragma pack(4)

typedef struct {
  uint32 id;                                  // 库位id
  ParkingSlotPositionType slot_side;          // 车位方位
  Point2d local_corner_point[FUSION_SPATIAL_PARKING_SLOT_BORDER_TYPE_MAX_NUM];     // 四个角点坐标(local坐标)
  Point2d global_corner_point[FUSION_SPATIAL_PARKING_SLOT_CORNER_POINT_MAX_NUM];   // 四个角点坐标(global坐标)
  float32 theta;                              // 车位角度(度)
  ParkingSlotType slot_type;                  // 车位类型
  boolean slot_update_flag;                   // 车位更新标志位(true: 更新/ false:未更新)
  boolean is_free_slot;                       // 是否激活自定义泊车功能(true: 激活/ false:未激活)
  boolean free_slot_collision_quadrant[FUSION_SPATIAL_PARKING_SLOT_COLLISION_QUADRANT];  // 自定义泊车功能的车位碰撞象限（true:碰撞/ false:未碰撞）
  boolean is_one_touch_park_side;             // 是否激活一键贴边功能(true: 激活/ false:未激活)
} _STRUCT_ALIGNED_ FusionSpatialParkingSlotInfo;

typedef struct {
  MsgHeader msg_header;                                                     // SOC消息发送信息
  MsgMeta msg_meta;                                                         // 用于记录模块各种信息
  uint8 spatial_slots_size;                                                             // 空间车位大小
  FusionSpatialParkingSlotInfo spatial_slots[FUSION_SPATIAL_PARKING_SLOT_MAX_NUM];      // 空间车位信息[6]
  boolean is_near_obstacle_park_side;         // 一键贴边功能中判断自车与障碍物距离是否过近
} _STRUCT_ALIGNED_ FusionSpatialParkingSlot;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_FUSION_SPATIAL_PARKING_SLOT_H_