// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef _IFLYAUTO_CAMERA_PERCEPTION_PARKING_SLOT_H_
#define _IFLYAUTO_CAMERA_PERCEPTION_PARKING_SLOT_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"

#define CAMERA_PERCEPTION_PARKING_SLOT_CORNER_POINTS_NUM 4
#define CAMERA_PERCEPTION_PARKING_SLOT_LIMITER_POINTS_NUM 2
#define CAMERA_PERCEPTION_PARKING_SLOT_SPECIFICATION_POINTS_NUM 2
#define CAMERA_PERCEPTION_PARKING_SLOTS_NUM 12

#ifdef __cplusplus
namespace iflyauto {
#endif

#pragma pack(4)

typedef struct {
  uint32 id;                                              // 车位id
  ParkingSlotType type;                                   // 车位类型
  ParkingSlotMaterial material;                           // 车位材质
  Point2f corner_points[CAMERA_PERCEPTION_PARKING_SLOT_CORNER_POINTS_NUM];  // 车位角点信息     (米)  <固定4个>
  float32 line_a;                                         // 车位入口A斜率
  float32 line_b;                                         // 车位入口B斜率
  float32 confidence;                                     // 感知置信度       [0.0-1.0]
  uint32 allow_parking;                                   // 车位是否可泊     (0:不可泊/1:可泊)
} _STRUCT_ALIGNED_ CameraSlotType;

typedef struct {
  uint32 id;                                                // 限位器id
  Point2f limiter_points[CAMERA_PERCEPTION_PARKING_SLOT_LIMITER_POINTS_NUM];  // 车位线点信息     (米)  <固定2个>
  float32 confidence;                                       // 识别置信度       [0.0-1.0]
} _STRUCT_ALIGNED_ VisionSlotLimiter;

typedef struct {
  uint32 id;                                                // 地锁id
  Point2f specificatio_points[CAMERA_PERCEPTION_PARKING_SLOT_SPECIFICATION_POINTS_NUM]; // 地锁点信息     (米)  <固定2个>
  float32 confidence;                                       // 识别置信度       [0.0-1.0]
} _STRUCT_ALIGNED_ VisionSlotSpecification;

typedef struct {
  MsgHeader msg_header;
  MsgMeta msg_meta;
  uint64 isp_timestamp;                                                 // 相机曝光时的时间戳    (微秒)
  uint8 parking_slots_size;                                             // 视觉车位数量
  CameraSlotType parking_slots[CAMERA_PERCEPTION_PARKING_SLOTS_NUM];    // 视觉车位信息   <最大12个>
  uint8 vision_slot_limiters_size;                                              // 视觉限位器数量
  VisionSlotLimiter vision_slot_limiters[CAMERA_PERCEPTION_PARKING_SLOTS_NUM];  // 视觉限位器信息   <最大12个>
  uint8 vision_slot_specificatios_size;                                                     // 视觉地锁数量
  VisionSlotSpecification vision_slot_specificatios[CAMERA_PERCEPTION_PARKING_SLOTS_NUM];   // 视觉地锁信息   <最大12个>
  float32 ego_motion[16];                           // 位姿信息
  uint8 is_after_reset;                             // 该帧为是否重置后第一帧
  uint64 images_timestamp[4];                       // 环视四个相机isp时间戳，前后左右
} _STRUCT_ALIGNED_ ParkingSlotSelectInfo;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_CAMERA_PERCEPTION_PARKING_SLOT_H_