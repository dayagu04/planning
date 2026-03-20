// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2025/05/22

#ifndef _IFLYAUTO_CAMERA_PERCEPTION_SCENE_H_
#define _IFLYAUTO_CAMERA_PERCEPTION_SCENE_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"

#ifdef __cplusplus
namespace iflyauto {
#endif

#pragma pack(4)

typedef enum {
  CAMERA_PERCEPTION_ROAD_TYPE_UNKNOWN = 0,                                    // 未知
  CAMERA_PERCEPTION_ROAD_TYPE_URBAN,                                          // 城市道路
  CAMERA_PERCEPTION_ROAD_TYPE_HIGHWAY,                                        // 高速公路
  CAMERA_PERCEPTION_ROAD_TYPE_RURAL,                                          // 乡村道路
  CAMERA_PERCEPTION_ROAD_TYPE_UNDERGROUND,                                    // 地下车库
  CAMERA_PERCEPTION_ROAD_TYPE_TUNNEL                                          // 隧道
} _ENUM_PACKED_ CameraPerceptionRoadType;

typedef enum {
  CAMERA_PERCEPTION_WEATHER_CONDITION_UNKNOWN = 0,                            // 未知
  CAMERA_PERCEPTION_WEATHER_CONDITION_SUNNY,                                  // 晴天
  CAMERA_PERCEPTION_WEATHER_CONDITION_RAINY,                                  // 雨天
  CAMERA_PERCEPTION_WEATHER_CONDITION_SNOWY,                                  // 雪天
  CAMERA_PERCEPTION_WEATHER_CONDITION_FOGGY,                                  // 雾天
} _ENUM_PACKED_ CameraPerceptionWeatherCondition;

typedef enum {
  CAMERA_PERCEPTION_TRAFFIC_CONDITION_UNKNOWN = 0,                            // 未知
  CAMERA_PERCEPTION_TRAFFIC_CONDITION_BUSY,                                   // 繁忙
  CAMERA_PERCEPTION_TRAFFIC_CONDITION_IDLE,                                   // 空闲
  CAMERA_PERCEPTION_TRAFFIC_CONDITION_ROADWORKS,                              // 施工区域
  CAMERA_PERCEPTION_TRAFFIC_CONDITION_ACCIDENT,                               // 交通事故
} _ENUM_PACKED_ CameraPerceptionTrafficCondition;

typedef enum {
  CAMERA_PERCEPTION_LIGHTING_CONDITION_UNKNOWN = 0,                          // 未知
  CAMERA_PERCEPTION_LIGHTING_CONDITION_BRIGHT,                               // 明亮
  CAMERA_PERCEPTION_LIGHTING_CONDITION_MEDIUM,                               // 中等亮度
  CAMERA_PERCEPTION_LIGHTING_CONDITION_DARK,                                 // 昏暗
} _ENUM_PACKED_ CameraPerceptionLightingCondition;

typedef struct {
  MsgHeader msg_header;
  MsgMeta msg_meta;
  uint64 isp_timestamp;                                                      // 相机曝光时的时间戳    (微秒)
  CameraPerceptionRoadType road_type;                                        // 道路类型
  CameraPerceptionWeatherCondition weather_condition;                        // 天气条件
  CameraPerceptionTrafficCondition traffic_condition;                        // 交通条件
  CameraPerceptionLightingCondition lighting_condition;                      // 亮度条件
} _STRUCT_ALIGNED_ CameraPerceptionScene;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_CAMERA_PERCEPTION_SCENE_H_