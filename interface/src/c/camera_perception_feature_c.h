// Copyright (C) iflyauto Technologies (2024). All rights reserved.
// Modified: 2024/02/21

#ifndef _IFLYAUTO_CAMERA_PERCEPTION_FEATURE_H_
#define _IFLYAUTO_CAMERA_PERCEPTION_FEATURE_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"
#include "camera_perception_objects_c.h"

#define CAMERA_PERCEPTION_BEV_FEATURE_MAX_NUM (1 * 128 * 100 * 190 / 2) // 感知bev特征张量的元素个数
#define CAMERA_PERCEPTION_IMG_FEATURE_MAX_NUM (8 * 128 * 30 * 52 / 2)   // 感知img特征张量的元素个数

#define CAMERA_PERCEPTION_LANE_FEATURE_MAX_NUM (7 * 128 * 30 * 52 / 2)   // 感知车道特征张量的元素个数
#define CAMERA_PERCEPTION_OBSTACLE_FEATURE_MAX_NUM (8 * 128 * 30 * 52 / 2)   // 感知障碍物特征张量的元素个数

#ifdef __cplusplus
namespace iflyauto {
#endif

#pragma pack(4)

// 感知特征信息集合
typedef struct {
  MsgHeader msg_header;
  MsgMeta msg_meta;
  uint64 isp_timestamp;                                                              // 图像的isp时间戳    (微秒)
  uint64 index;                                                                      // 感知特征索引号
  uint32 bev_feature_dim;                                                            // bev特征元素的维数
  uint32 bev_feature_num;                                                            // bev特征元素的个数
  float32 bev_feature[CAMERA_PERCEPTION_BEV_FEATURE_MAX_NUM];                        // bev特征元素数组 
  uint32 img_feature_dim;                                                            // img特征元素的维数
  uint32 img_feature_num;                                                            // img特征元素的个数
  float32 img_feature[CAMERA_PERCEPTION_IMG_FEATURE_MAX_NUM];                        // img特征元素数组
  uint32 reserved;                                                                   // 预留
} _STRUCT_ALIGNED_ CameraPerceptionFeatureInfo;


// 感知车道特征信息集合
typedef struct {
  MsgHeader msg_header;
  MsgMeta msg_meta;
  uint64 isp_timestamp;                                                              // 图像的isp时间戳    (微秒)
  uint64 index;                                                                      // 感知特征索引号
  uint32 img_feature_dim;                                                            // img特征元素的维数
  uint32 img_feature_num;                                                            // img特征元素的个数
  float32 img_feature[CAMERA_PERCEPTION_LANE_FEATURE_MAX_NUM];                        // img特征元素数组
  uint32 reserved;                                                                   // 预留
} _STRUCT_ALIGNED_ CameraPerceptionLaneFeatureInfo;


// 感知障碍物特征信息集合
typedef struct {
  MsgHeader msg_header;
  MsgMeta msg_meta;
  uint64 isp_timestamp;                                                              // 图像的isp时间戳    (微秒)
  uint64 index;                                                                      // 感知特征索引号
  uint32 img_feature_dim;                                                            // img特征元素的维数
  uint32 img_feature_num;                                                            // img特征元素的个数
  float32 img_feature[CAMERA_PERCEPTION_OBSTACLE_FEATURE_MAX_NUM];                   // img特征元素数组
  uint32 reserved;                                                                   // 预留
} _STRUCT_ALIGNED_ CameraPerceptionObstacleFeatureInfo;


#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_CAMERA_PERCEPTION_FEATURE_H_
