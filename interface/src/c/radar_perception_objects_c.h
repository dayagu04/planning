// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef _IFLYAUTO_RADAR_PERCEPTION_OBJECTS_H_
#define _IFLYAUTO_RADAR_PERCEPTION_OBJECTS_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"

#ifdef __cplusplus
namespace iflyauto {
#endif

#define RADAR_PERCEPTION_OBJ_MAX_NUM 40

#pragma pack(4)

// radar障碍物信息
typedef struct {
  uint32 id;                            // 障碍物id
  ObjectType type;                      // 障碍物类型
  Shape3d shape;                        // 障碍物尺寸 单位:m
  Point2f relative_position;            // 障碍物几何中心距离自车后轴中心的距离 单位:m
  Point2f relative_velocity;            // 障碍物相对自车坐标系相对速度 单位:m/s
  Point2f relative_acceleration;        // 障碍物相对自车坐标系相对加速度 单位:m/ss
  float32 relative_heading_angle;       // 障碍物与自车的相对朝向角 单位:rad
  float32 relative_heading_angle_rate;  // 障碍物与自车的相对朝向角变化率 单位:rad/s
  int32 age;                            // 障碍物存在的帧数
  float32 conf;                         // 障碍物置信度     [0.0-1.0]
  float32 orientation_angle_conf;       // 航向角置信度     [0.0-1.0]
  float32 vel_conf;                     // 速度置信度       [0.0-1.0]
  float32 accel_conf;                   // 加速度置信度     [0.0-1.0]
  UpdateFlag obj_update_flag;           // 更新标志
  ObjectMotionType obj_motion_pattern;  // 运动状态
  float32 obstacle_prob;
} _STRUCT_ALIGNED_ RadarPerceptionObject;

// radar障碍物信息集合
typedef struct {
  MsgHeader msg_header;                                         // SOC消息发送信息
  SensorMeta sensor_meta;                                       // CAN报文到达MCU信息
  SensorType sensor_type;                                       // 传感器类型
  uint8 object_list_size;                                       // 障碍物数量
  RadarPerceptionObject object_list[RADAR_PERCEPTION_OBJ_MAX_NUM];  // 障碍物列表  <最大30个>
  uint32 crc;                                                   // CRC校验
} _STRUCT_ALIGNED_ RadarPerceptionObjectsInfo;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_RADAR_PERCEPTION_OBJECTS_H_