// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef _IFLYAUTO_USS_MODEL_INFO_H_
#define _IFLYAUTO_USS_MODEL_INFO_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"

#ifdef __cplusplus
namespace iflyauto {
#endif

#define USS_MODEL_APA_SLOT_OBJ_NUM 400
#define USS_MODEL_OUTLINE_DATAORI_NUM 2
#define USS_MODEL_USS_OBJECT_DISTANCE_NUM 12

#pragma pack(4)

typedef struct {
  Point2d obj_pt_global[USS_MODEL_APA_SLOT_OBJ_NUM];       // 障碍物点云全局坐标(毫米) <最大400个>
  Point2d obj_pt_local[USS_MODEL_APA_SLOT_OBJ_NUM];        // 障碍物点云局部坐标 (毫米)   <最大400个>
  int32 obj_type0[USS_MODEL_APA_SLOT_OBJ_NUM];            // 障碍物0的类型 <预留>
  uint32 wr_index;                                  // 数组写索引
  uint32 obj_pt_cnt;                                // 障碍物数量
} _STRUCT_ALIGNED_ UssModelCoordinateDataType;

typedef struct {
  Point2d position;                                    // 定位坐标
  float64 yaw;                                         // 欧拉角yaw值
} _STRUCT_ALIGNED_ UssModelPosition;

typedef struct {
  MsgHeader msg_header;                                                       // SOC消息发送信息
  MsgMeta msg_meta;                                                           // 用于记录模块各种信息
  uint32 uss_wave_distance[USS_MODEL_USS_OBJECT_DISTANCE_NUM];                // 超声感知使用的超声波距离
  UssModelPosition uss_wave_position;                                         // 超声感知使用的超声波距离时的定位
  uint32 dis_from_car_to_obj[USS_MODEL_USS_OBJECT_DISTANCE_NUM];              // 超声波障碍物距离跟踪信息[12] (米) <下标0为左前APA雷达，顺时针放置>
  UssModelCoordinateDataType out_line_dataori[USS_MODEL_OUTLINE_DATAORI_NUM]; // 超声波侧边障碍物点云信息[2]
} _STRUCT_ALIGNED_ UssModelInfo;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_USS_MODEL_INFO_H_