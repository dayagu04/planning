// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2024/01/17

#ifndef _IFLYAUTO_FUSION_ROAD_COMPRESS_H_
#define _IFLYAUTO_FUSION_ROAD_COMPRESS_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"
#include "fusion_road_c.h"

#define FUSION_ROAD_LINE_MAX_NUM 6

#ifdef __cplusplus
namespace iflyauto {
#endif

#pragma pack(4)

typedef struct {
  boolean existence;                                             // 数据有效性           (true:valid / false:invalid)
  LineType type;                                                 // 线类型               (车道线/路沿/栅栏)
  float64 poly_coefficient[FUSION_ROAD_LINE_POLYNOMIAL_NUM];     // 车道线方程系数       (固定4个，依次为常数项、一次项、二次项、三次项)
  float32 begin;                                                 // 车道线起始点         (米)
  float32 end;                                                   // 车道线终止点         (米)
  LaneBoundaryType line_type;                                    // 车道线虚实类型
} _STRUCT_ALIGNED_ LaneBoundaryCompress;

typedef struct {
  MsgHeader msg_header;
  MsgMeta msg_meta;
  uint64 isp_timestamp;                                          // ISP出图时间戳        (微秒)
  LaneBoundaryCompress line_info[FUSION_ROAD_LINE_MAX_NUM];      // 顺序依次为 左侧边沿、左二道线、左一道线、右一道线、右二道线、右侧边沿
} _STRUCT_ALIGNED_ RoadInfoCompress;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  //_IFLYAUTO_FUSION_ROAD_COMPRESS_H_
