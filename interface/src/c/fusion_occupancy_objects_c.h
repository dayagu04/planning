// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef _IFLYAUTO_FUSION_OCCUPANCY_OBJECTS_H_
#define _IFLYAUTO_FUSION_OCCUPANCY_OBJECTS_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#include "camera_perception_occupancy_objects_c.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"

#ifdef __cplusplus
namespace iflyauto {
#endif

#define FUSION_OCCUPANCY_OBJECTS_MAX_NUM CAMERA_PERCEPTION_OCC_OBJECT_MAX_NUM
#define FUSION_OCCUPANCY_OBJECTS_BOUNDING_BOX_POINTS_SET_MAX_NUM 8
#define FUSION_OCCUPANCY_OBJECTS_POLYGON_POINTS_SET_MAX_NUM CAMERA_PERCEPTION_OCC_POINT_MAX_NUM

#pragma pack(4)

typedef struct {
  ObjectMotionType motion_pattern_current;  // 当前运动状态
  ObjectMotionType motion_pattern_history;  // 历史运动状态
  uint32 fusion_source;
  uint16 track_id;               // 融合障碍物id
  uint16 track_age;              // 融合障碍物存在周期   (帧数)
  uint8 confidence;              // 障碍物置信度         [0-100]
  UpdateFlag track_status;       // 跟踪状态
  uint8 bounding_box_points_size; // 障碍物包络框有效信息数量
  Point2f bounding_box_points[FUSION_OCCUPANCY_OBJECTS_BOUNDING_BOX_POINTS_SET_MAX_NUM];  // 障碍物包络框信息
  uint32 visable_seg_num;         // 值大于0点有序，数值表示前多少个可见点，其余点为预测点;值小于0点无序，点全为预测点
  uint32 polygon_points_size;     // 障碍物轮廓有效信息数量
  Point3f polygon_points[FUSION_OCCUPANCY_OBJECTS_POLYGON_POINTS_SET_MAX_NUM];  // 障碍物轮廓信息，xy是boot坐标,z是body坐标
  float32 relative_speed_angle;  // 障碍物速度角         (rad)   [-pi,pi]
} _STRUCT_ALIGNED_ FusionOccupancyAdditional;

typedef struct {
  Obstacle common_occupancy_info;                     // 障碍物通用信息
  FusionOccupancyAdditional additional_occupancy_info;  // 障碍物附加信息
} _STRUCT_ALIGNED_ FusionOccupancyObject;

typedef struct {
  MsgHeader msg_header;
  MsgMeta msg_meta;
  uint8 fusion_object_size;                        // 融合障碍物数量
  FusionOccupancyObject fusion_object[FUSION_OCCUPANCY_OBJECTS_MAX_NUM];  // 融合障碍物信息 <最大48个>
  boolean local_point_valid;  // 障碍物数据中用到的绝对坐标是否有效 (true:有效 / false:无效)
} _STRUCT_ALIGNED_ FusionOccupancyObjectsInfo;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_FUSION_OCCUPANCY_OBJECTS_H_