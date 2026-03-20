// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef _IFLYAUTO_FUSION_PARKING_SLOT_H_
#define _IFLYAUTO_FUSION_PARKING_SLOT_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"

#define FUSION_PARKING_SLOT_CORNER_POINT_NUM 4
#define FUSION_PARKING_SLOT_MAX_NUM 48
#define FUSION_PARKING_SLOT_LIMITER_MAX_NUM 2
#define FUSION_PARKING_SLOT_LIMITER_POINT_NUM 2
#define FUSION_FREE_SLOT_REGION_NUM 6

#ifdef __cplusplus
namespace iflyauto {
#endif

#pragma pack(4)

typedef enum {
  NOT_ALLOW_PARKING = 0,  // 不允许泊入
  ALLOW_PARKING = 1,      // 允许泊入
} _ENUM_PACKED_ AllowParking;

// 单个限位器信息
typedef struct {
  uint32 id;                                                  // 属性标识id信息
  StaticFusionResourceType resource_type;                     // 数据来源
  Point2f end_points[FUSION_PARKING_SLOT_LIMITER_POINT_NUM];  // 车体限位器端点位置 （米） <固定2个>
} _STRUCT_ALIGNED_ ParkingFusionLimiter;

typedef struct {
    uint32 id;                                                           // 车位ID              [1-100]
    uint32 uss_id;                                                       // 车位ID对应的超声车位id
    StaticFusionResourceType resource_type;                              // 数据来源
    ParkingSlotType type;                                                // 车位类型
    Point2f corner_points[FUSION_PARKING_SLOT_CORNER_POINT_NUM];         // 单个车位角点信息     (米)    <固定4个>
    float line_a;                                                        // 车位入口A斜率
    float line_b;                                                        // 车位入口B斜率
    SlotSourceType fusion_source;                                        // 融合车位的数据来源
    uint32 confidence;                                                   // 位置置信度       [0-100]
    ParkingSlotPositionType slot_side;                                   // 车位相对自车位置
    uint8 limiters_size;                                                 // 此库位限位器个数
    ParkingFusionLimiter limiters[FUSION_PARKING_SLOT_LIMITER_MAX_NUM];  // 此库位限位器信息  <最大2个>
    AllowParking allow_parking;
    boolean is_turn_corner;  // 车位角点是否翻转
} _STRUCT_ALIGNED_ ParkingFusionSlot;

typedef struct {
  MsgHeader msg_header;
  MsgMeta msg_meta;
  uint64 isp_timestamp;  // 相机曝光时的时间戳    (微秒)
  uint8 parking_fusion_slot_lists_size;                                     // 车位数量         
  ParkingFusionSlot parking_fusion_slot_lists[FUSION_PARKING_SLOT_MAX_NUM]; // 车位信息列表     <最大16个>
  uint32 select_slot_id;                                                    // 用户选中的泊入车位ID号   <0为无效>
  uint32 select_local_map_slot_id;                                          // 用户选中的车位ID对应的mega局部地图的ID
  uint32 memorized_slot_id;                                                 // 用户记忆的泊入车位ID对应静态融合的ID
  boolean is_in_parking_slot;                                               // 车辆是否在车位中
  uint32 ego_slot_id;                                                       // 车辆所在车位id
} _STRUCT_ALIGNED_ ParkingFusionInfo;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_FUSION_PARKING_SLOT_H_
