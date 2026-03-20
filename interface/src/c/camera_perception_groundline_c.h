// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/12/14

#ifndef _IFLYAUTO_CAMERA_PERCEPTION_GROUNDLINE_H_
#define _IFLYAUTO_CAMERA_PERCEPTION_GROUNDLINE_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"

#define CAMERA_PERCEPTION_GROUND_LINE_POINTS_MAX_NUM 576
#define CAMERA_PERCEPTION_GROUND_LINES_MAX_NUM 10
#define CAMERA_PERCEPTION_GROUND_LINE_RESERVED_BYTES 64
#ifdef __cplusplus
namespace iflyauto {
#endif

#pragma pack(4)

// 图像来源
typedef enum {
  CAMERA_SOURCE_FRONT_FAR,              // 前视30°相机
  CAMERA_SOURCE_FRONT_MID,              // 前视相机
  CAMERA_SOURCE_FRONT_WIDE,             // 前视120°相机
  CAMERA_SOURCE_LEFT_FRONT,             // 左前相机
  CAMERA_SOURCE_LEFT_REAR,              // 左后相机
  CAMERA_SOURCE_REAR_MID,               // 后视相机
  CAMERA_SOURCE_REAR_MID_UP,            // 后视相机(视角向上)
  CAMERA_SOURCE_REAR_MID_DOWN,          // 后视相机(视角向下)
  CAMERA_SOURCE_RIGHT_FRONT,            // 右前相机
  CAMERA_SOURCE_RIGHT_REAR,             // 右后相机
  CAMERA_SOURCE_FRONT_FISHEYE,          // 前鱼眼相机
  CAMERA_SOURCE_LEFT_FISHEYE,           // 左鱼眼相机
  CAMERA_SOURCE_RIGHT_FISHEYE,          // 右鱼眼相机
  CAMERA_SOURCE_REAR_FISHEYE,           // 后鱼眼相机
  CAMERA_SOURCE_FISHEYE,                // 鱼眼相机
  CAMERA_SOURCE_FRONT_WIDE_SUB_LEFT,    // 前视120°相机与左鱼眼相机的组合视角
  CAMERA_SOURCE_FRONT_WIDE_SUB_RIGHT,   // 前视120°相机与右鱼眼相机的组合视角
  CAMERA_SOURCE_RIGHT_FRONT_SUB_LEFT,   // 右前相机与左鱼眼相机的组合视角
  CAMERA_SOURCE_RIGHT_FRONT_SUB_RIGHT,  // 右前相机与右鱼眼相机的组合视角
  CAMERA_SOURCE_LEFT_FRONT_SUB_LEFT,    // 左前相机与左鱼眼相机的组合视角
  CAMERA_SOURCE_LEFT_FRONT_SUB_RIGHT,   // 左前相机与右鱼眼相机的组合视角
  CAMERA_SOURCE_LEFT_REAR_SUB_LEFT,     // 左后相机与左鱼眼相机的组合视角
  CAMERA_SOURCE_RIGHT_REAR_SUB_RIGHT,   // 右后相机与右鱼眼相机的组合视角
  CAMERA_SOURCE_IPM,                    // 逆透视变换视角的鸟瞰图
} _ENUM_PACKED_ CameraSourceEnum;

// 相机模型
typedef enum {
  CAMERA_MODEL_UNKNOWN,     // 未知相机模型
  CAMERA_MODEL_RAW,         // 原始图像
  CAMERA_MODEL_CYLINDER,    // 柱形图像
  CAMERA_MODEL_DEDISTORT,   // 无畸变图像
  CAMERA_MODEL_PSTITCH,     // 全景拼接图像
  CAMERA_MODEL_RESERVED_A,  // 预留字段A
  CAMERA_MODEL_RESERVED_B,  // 预留字段B
  CAMERA_MODEL_RESERVED_C,  // 预留字段C
} _ENUM_PACKED_ CameraModelEnum;

// 接地线点的类型
typedef enum {
  GROUND_LINE_POINT_TYPE_2D,  // 2d点
  GROUND_LINE_POINT_TYPE_3D,  // 3d点
} _ENUM_PACKED_ GroundLinePointType;

typedef struct {
  CameraSourceEnum camera_source;    // 图像来源
  CameraModelEnum camera_model;      // 相机模型
  GroundLineType semantic_type;      // 接地线类型
  GroundLinePointType point_type;    // 接地线点的类型

  int32 visable_seg_num;             // 值大于0点有序，数值表示前多少个可见点，其余点为预测点;值小于0点无序，点全为预测点
  uint16 points_2d_size;
  uint16 points_3d_size;
  Point2f points_2d[CAMERA_PERCEPTION_GROUND_LINE_POINTS_MAX_NUM];  // 组成接地线的2d点
  Point3f points_3d[CAMERA_PERCEPTION_GROUND_LINE_POINTS_MAX_NUM];  // 组成接地线的3d点
  float32 confidence;                           // 接地线感知置信度 [0.0-1.0]
} _STRUCT_ALIGNED_ GroundLine;

typedef struct {
  MsgHeader msg_header;
  MsgMeta msg_meta;
  uint64 isp_timestamp;     // 相机曝光时的时间戳    (微秒)
  uint8 ground_lines_size;                                          // 视觉接地线数量
  GroundLine ground_lines[CAMERA_PERCEPTION_GROUND_LINES_MAX_NUM];  // 视觉接地线信息
  uint8 reserved_infos[CAMERA_PERCEPTION_GROUND_LINE_RESERVED_BYTES];   // 为后续扩展需求预留的消息字段
} _STRUCT_ALIGNED_ GroundLinePerceptionInfo;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_CAMERA_PERCEPTION_GROUNDLINE_H_