// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef _IFLYAUTO_USS_PERCEPTION_INFO_H_
#define _IFLYAUTO_USS_PERCEPTION_INFO_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"

#ifdef __cplusplus
namespace iflyauto {
#endif

#define USS_PERCEPTION_APA_SLOT_OBJ_MAX_NUM 200
#define USS_PERCEPTION_OUTLINE_DATAORI_NUM 1
#define USS_PERCEPTION_OBJECT_DISTANCE_NUM 12

#pragma pack(4)

typedef enum {
  USS_PERCEPT_OBJ_POINT_TYPE_VIRTUAL = 0,  // 虚拟
  USS_PERCEPT_OBJ_POINT_TYPE_ACTUAL = 1,   // 真实
} _ENUM_PACKED_ UssPerceptObjPointType;

typedef struct {
  UssPerceptObjPointType obj_point_type;  // 障碍物点类型
  float32 obj_point_xpos;                 // 障碍物点x坐标    (毫米)
  float32 obj_point_ypos;                 // 障碍物点y坐标    (毫米)
} _STRUCT_ALIGNED_ UssObjPointInfo;

typedef struct {
  Point2d obj_pt_global[USS_PERCEPTION_APA_SLOT_OBJ_MAX_NUM];        // 障碍物点云全局坐标(米) <最大400个>
  Point2d obj_pt_local[USS_PERCEPTION_APA_SLOT_OBJ_MAX_NUM];         // 障碍物点云局部坐标 (米)   <最大400个>
  uint8 point_high[USS_PERCEPTION_APA_SLOT_OBJ_MAX_NUM];             // 点的高低，0：Low,1:High,2:Unkown
  int32 obj_type[USS_PERCEPTION_APA_SLOT_OBJ_MAX_NUM];               // 障碍物的类型 <预留>
  uint32 wr_index;                                                   // 数组写索引 <预留>
  uint32 obj_pt_cnt;                                                 // 障碍物数量
} _STRUCT_ALIGNED_ ApaSlotOutlineCoordinateDataType;

typedef struct {
  MsgHeader msg_header;                                                                     // SOC消息发送信息
  MsgMeta msg_meta;                                                                         // 用于记录模块各种信息
  float32 dis_from_car_to_obj[USS_PERCEPTION_OBJECT_DISTANCE_NUM];                           // 超声波障碍物距离跟踪信息[12] (米) <下标0为左前APA雷达，顺时针放置>
  ApaSlotOutlineCoordinateDataType out_line_dataori[USS_PERCEPTION_OUTLINE_DATAORI_NUM];    // 超声波侧边障碍物点云信息[2]
} _STRUCT_ALIGNED_ UssPerceptInfo;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_USS_PERCEPTION_INFO_H_