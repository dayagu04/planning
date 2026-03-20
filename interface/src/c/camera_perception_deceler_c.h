// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/12/14

#ifndef _IFLYAUTO_CAMERA_PERCEPTION_DECELER_H_
#define _IFLYAUTO_CAMERA_PERCEPTION_DECELER_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"

#define CAMERA_PERCEPTION_DECELER_POINTS_NUM 4
#define CAMERA_PERCEPTION_DECELERS_MAX_NUM 4
#ifdef __cplusplus
namespace iflyauto {
#endif

#pragma pack(4)

typedef struct {
  uint32 id;                                   // 减速带id
  Point2f deceler_points[CAMERA_PERCEPTION_DECELER_POINTS_NUM];  // 减速带顶点坐标  <固定4个,逆时针排列>
  float32 confidence;                          // 感知置信度      [0.0,1.0]
  uint32 life_time;                            // 生命周期
} _STRUCT_ALIGNED_ Deceler;

typedef struct {
  MsgHeader msg_header;
  MsgMeta msg_meta;
  uint64 isp_timestamp;  // 相机曝光时的时间戳    (微秒)
  uint8 decelers_size;                                    // 视觉减速带数量
  Deceler decelers[CAMERA_PERCEPTION_DECELERS_MAX_NUM];   // 视觉减速带信息
} _STRUCT_ALIGNED_ DecelerPerceptionInfo;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_CAMERA_PERCEPTION_DECELER_H_