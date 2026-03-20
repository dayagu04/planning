// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2024/10/28

#ifndef _IFLYAUTO_FUSION_DECELER_H_
#define _IFLYAUTO_FUSION_DECELER_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"

#define FUSION_DECELER_POINTS_NUM 4
#define FUSION_DECELERS_MAX_NUM 20

#ifdef __cplusplus
namespace iflyauto {
#endif

#pragma pack(4)

typedef struct {
  uint32 id;                                          // 地图中减速带id
  StaticFusionResourceType resource_type;             // 数据来源
  Point2f deceler_points[FUSION_DECELER_POINTS_NUM];  // 减速带顶点boot坐标  <固定4个,逆时针排列> 
  float32 confidence;                                 // 感知置信度         [0.0,1.0]
  uint32 reserved;                                    // 保留字段
} _STRUCT_ALIGNED_ FusionDeceler;

typedef struct {
  MsgHeader msg_header;
  MsgMeta msg_meta;
  uint64 isp_timestamp;  // 相机曝光时的时间戳    (微秒)
  uint8 decelers_size;                              // 减速带数量
  FusionDeceler decelers[FUSION_DECELERS_MAX_NUM];  // 减速带信息
} _STRUCT_ALIGNED_ FusionDecelerInfo;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_FUSION_DECELER_H_
