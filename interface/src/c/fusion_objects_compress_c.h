// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef _IFLYAUTO_FUSION_OBJECTS_COMPRESS_H_
#define _IFLYAUTO_FUSION_OBJECTS_COMPRESS_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"

#ifdef __cplusplus
namespace iflyauto {
#endif
#define FUSION_COMPRESS_OBJECT_MAX_NUM 50

#pragma pack(4)

//发送到MCU障碍物目标
typedef struct {
  uint8 id;                          // 障碍物id
  ObjectType type;                   // 障碍物类型
  Shape3d shape;                     // 障碍物尺寸
  Point2f relative_velocity;         // 障碍物相对自车坐标系相对速度         (米/秒)
  Point2f relative_acceleration;     // 障碍物相对自车坐标系相对加速度 (米/秒^2)
  Point2f relative_center_position;  // 障碍物几何中心距离自车后轴中心的距离 (米)

  /* 障碍物与自车的相对朝向角
     单位：弧度rad
     备注：沿自车坐标系x轴正向逆时针旋转为正（0~Π），顺时针为负（0~-Π）
  */
  float32 relative_heading_angle;
  float32 relative_heading_angle_rate;  // 障碍物与自车的相对朝向角变化率
} _STRUCT_ALIGNED_ ObstacleCompress;

typedef struct {
  /** 融合障碍物的数据来源
   *  点的顺序是沿自车前进方向，由近到远
   *  第1位：有前相机来源
   *  第2位：有前毫米波雷达来源
   *  第3位：有左前毫米波雷达来源
   *  第4位：有右前毫米波雷达来源
   *  第5位：有左后毫米波雷达来源
   *  第6位：有右后毫米波雷达来源
   *  第7位：有超声波雷达来源
   *  第8位：有激光雷达来源
   **/
  uint32 fusion_source;
  uint16 track_id;               // 融合障碍物id
  uint16 track_age;              // 融合障碍物存在周期   (帧数)
  uint8 confidence;              // 障碍物置信度         [0-100]
} _STRUCT_ALIGNED_ FusionObjectsAdditionalCompress;

typedef struct {
  ObstacleCompress common_info;                     // 障碍物通用信息
  FusionObjectsAdditionalCompress additional_info;  // 障碍物附加信息
} _STRUCT_ALIGNED_ FusionObjectCompress;

typedef struct {
  MsgHeader msg_header;
  MsgMeta msg_meta;
  uint8 fusion_object_size;                        // 融合障碍物数量
  FusionObjectCompress fusion_object[FUSION_COMPRESS_OBJECT_MAX_NUM];  // 融合障碍物信息 <最大50个>
} _STRUCT_ALIGNED_ FusionObjectsInfoCompress;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_FUSION_OBJECTS_FUSION_H_