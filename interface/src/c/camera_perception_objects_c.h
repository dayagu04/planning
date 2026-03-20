// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef _IFLYAUTO_CAMERA_PERCEPTION_OBJECTS_H_
#define _IFLYAUTO_CAMERA_PERCEPTION_OBJECTS_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"

#define CAMERA_PERCEPTION_OBJECT_MAX_NUM 128

#ifdef __cplusplus
namespace iflyauto {
#endif

#define CAMERA_PERCEPTION_BOUNDING_BOX_POINTS_MAX_NUM 8
#define CAMERA_PERCEPTION_2D_BOUNDING_BOX_NUM         256
#define CAMERA_PERCEPTION_VIEW_NUM                    8
#define CAMERA_PERCEPTION_CONTOUR_POINTS_MAX_NUM      64

#pragma pack(4)

typedef struct {
  ObjectType  type;                   // 障碍物类型
  uint32      id;                     // 障碍物id
  uint16      top_left_x;             // 障碍物左上角x坐标
  uint16      top_left_y;             // 障碍物左上角y坐标
  uint16      bottom_right_x;         // 障碍物右下角x坐标
  uint16      bottom_right_y;         // 障碍物右下角y坐标
  uint16      mosaic_top_left_x;      // 马赛克框左上角x坐标
  uint16      mosaic_top_left_y;      // 马赛克框左上角x坐标
  uint16      mosaic_bottom_right_x;  // 马赛克框左上角x坐标
  uint16      mosaic_bottom_right_y;  // 马赛克框左上角x坐标
} _STRUCT_ALIGNED_ CameraPerception2DBoundingBox;

typedef struct {
  uint64                        isp_timestamp;          // 障碍物isp时间戳

  SensorType                    camera_type;            // 相机位置
  uint16                        camera_width;           // 图像宽度
  uint16                        camera_height;          // 图像高度

  uint8                         bounding_box_2d_size;   // box个数
  CameraPerception2DBoundingBox boundingbox_2d[CAMERA_PERCEPTION_2D_BOUNDING_BOX_NUM];  // box数组
} _STRUCT_ALIGNED_ CameraPerception2DBoundingBoxes;

typedef struct {
  MsgHeader                     msg_header;
  MsgMeta                       msg_meta;

  uint8                         view_size;               // 视角个数
  CameraPerception2DBoundingBoxes boundingboxes_2d[CAMERA_PERCEPTION_VIEW_NUM];  // 不同视角boundingbox信息
} _STRUCT_ALIGNED_ CameraPerceptionAllView2DBoundingBoxes;

typedef struct {
  SensorType sensor_type;  // 传感器类型

  /** 包络框各点坐标
   *  单位：米
   *  备注：自车坐标系，长方体。点0-3组成底部平面，点4-7组成顶部平面。
   *      点0和4在障碍物右后(点0和4组成长方体一条侧边)，点1和5在障碍物右前，点2和6在障碍物左前，点3和7在障碍物左后。
   **/
  uint8 bounding_box_points_size;
  Point3f bounding_box_points[CAMERA_PERCEPTION_BOUNDING_BOX_POINTS_MAX_NUM];

  /** 轮廓线各点坐标
   *  单位：米
   *  备注：未定义排布
   **/
  uint8 contour_points_size;
  Point3f contour_points[CAMERA_PERCEPTION_CONTOUR_POINTS_MAX_NUM];
  int32 life_time;                 // 生命周期         (毫秒)
  int32 age;                       // 障碍物存在的帧数
  float32 conf;                    // 障碍物置信度     [0.0-1.0]
  float32 orientation_angle_conf;  // 航向角置信度     [0.0-1.0]
  float32 vel_conf;                // 速度置信度       [0.0-1.0]
  float32 accel_conf;              // 加速度置信度     [0.0-1.0]
  boolean is_fusion;               // 是否是融合结果，RDG预留  (true:融合/false:未融合)
} _STRUCT_ALIGNED_ CameraPerceptionAdditional;

// 相机障碍物信息
typedef struct {
  Obstacle common_info;                        // 通用信息
  CameraPerceptionAdditional additional_info;  // 附加信息
} _STRUCT_ALIGNED_ CameraPerceptionObject;

// 相机障碍物信息集合
typedef struct {
  MsgHeader msg_header;
  MsgMeta msg_meta;
  uint64 isp_timestamp;                                                     // ISP出图时间戳    (微秒)
  uint8 camera_perception_objects_size;                                                 // 障碍物数量
  CameraPerceptionObject camera_perception_objects[CAMERA_PERCEPTION_OBJECT_MAX_NUM];   // 障碍物列表
  CommonCameraPerceptionInputTimestamp camera_perception_input_timestamp;   // 所有相机的ISP出图时间戳    (微秒)
} _STRUCT_ALIGNED_ CameraPerceptionObjectsInfo;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_CAMERA_PERCEPTION_OBJECTS_H_