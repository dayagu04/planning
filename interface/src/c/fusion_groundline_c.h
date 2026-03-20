// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2024/10/28

#ifndef _IFLYAUTO_FUSION_GROUNDLINE_H_
#define _IFLYAUTO_FUSION_GROUNDLINE_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"

#define FUSION_GROUNDLINE_POINT_MAX_NUM 60
#define FUSION_GROUNDLINE_MAX_NUM 40

#ifdef __cplusplus
namespace iflyauto {
#endif

#pragma pack(4)


// 接地线信息
typedef struct _FusionGroundLine {
  uint32 id;         // 接地线块id
  StaticFusionResourceType resource_type; // 数据来源
  GroundLineType type;    // 接地线类型
  uint8 groundline_point_size;
  Point3f groundline_point[FUSION_GROUNDLINE_POINT_MAX_NUM];  // 接地线位置信息,xy是boot坐标,z是body坐标
  uint32 reserved; //预留字段
} _STRUCT_ALIGNED_ FusionGroundLine;

typedef struct {
  MsgHeader msg_header;
  MsgMeta msg_meta;
  uint64 isp_timestamp;  // 相机曝光时的时间戳    (微秒)
  uint8 groundline_size;
  FusionGroundLine groundline[FUSION_GROUNDLINE_MAX_NUM];  // 接地线信息
} _STRUCT_ALIGNED_ FusionGroundLineInfo;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus

#endif  // _IFLYAUTO_FUSION_GROUNDLINE_H_
