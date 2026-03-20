// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef _IFLYAUTO_FUSION_OBJECTS_H_
#define _IFLYAUTO_FUSION_OBJECTS_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"

#ifdef __cplusplus
namespace iflyauto {
#endif

#define FUSION_OBJECT_MAX_NUM 128
#define FUSION_SENSOR_SOURCE_NUM 8
#define FUSION_OBJECTS_BOUNDING_BOX_POINTS_SET_MAX_NUM 8
#define FUSION_OBJECTS_POLYGON_POINTS_SET_MAX_NUM 64

#pragma pack(4)

typedef struct {
  ObjectMotionType motion_pattern_current;  // 当前运动状态
  ObjectMotionType motion_pattern_history;  // 历史运动状态
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
  UpdateFlag track_status;       // 跟踪状态
  uint8 bounding_box_points_size;                                               // 障碍物包络框有效信息数量
  Point3f bounding_box_points[FUSION_OBJECTS_BOUNDING_BOX_POINTS_SET_MAX_NUM];  // 障碍物包络框信息
  uint8 polygon_points_size;                                            // 障碍物轮廓有效信息数量
  Point3f polygon_points[FUSION_OBJECTS_POLYGON_POINTS_SET_MAX_NUM];    // 障碍物轮廓信息
  float32 relative_speed_angle;  // 障碍物速度角         (rad)   [-pi,pi]
  uint8 sensor_source_size;      // 原始传感器个数
  uint32 sensor_source_id
      [FUSION_SENSOR_SOURCE_NUM];  // 原始传感器id
                            // <数组大小8，第1位相机，第2位前雷达，第3位前左雷达，第4位前右雷达，第5位后左雷达，第6位后右雷达>
} _STRUCT_ALIGNED_ FusionObjectsAdditional;

typedef struct {
  Obstacle common_info;                     // 障碍物通用信息
  FusionObjectsAdditional additional_info;  // 障碍物附加信息
} _STRUCT_ALIGNED_ FusionObject;

typedef struct {
  MsgHeader msg_header;
  MsgMeta msg_meta;
  uint8 fusion_object_size;                           // 融合障碍物数量
  FusionObject fusion_object[FUSION_OBJECT_MAX_NUM];  // 融合障碍物信息 <最大128个>
  boolean local_point_valid;  // 障碍物数据中用到的绝对坐标是否有效 (true:有效 / false:无效)
  uint8 perception_mode;      // 0为默认状态，无效，1为使用行车感知模式，2为使用泊车感知模式
} _STRUCT_ALIGNED_ FusionObjectsInfo;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_FUSION_OBJECTS_H_