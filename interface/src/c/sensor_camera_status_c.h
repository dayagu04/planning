// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2025/11/21

#ifndef _IFLYAUTO_SENSOR_CAMERA_STATUS_H_
#define _IFLYAUTO_SENSOR_CAMERA_STATUS_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"

#ifdef __cplusplus
namespace iflyauto {
#endif
#define SENSOR_CAMERA_NUM 4
#pragma pack(4)

typedef enum{
  SENSOR_CAMERA_NO_ERROR = 0x0,             // 无故障
  SENSOR_CAMERA_ERROR_CODE1 = 0x1,          // 故障
  SENSOR_CAMERA_ERROR_RESERVED_1 = 0x2,     // 预留故障码
  SENSOR_CAMERA_ERROR_RESERVED_2 = 0x3,     // 预留故障码
  SENSOR_CAMERA_ERROR_RESERVED_3 = 0x4      // 预留故障码
}_ENUM_PACKED_ SensorCameraErrorCode;       // 环视相机故障码

typedef struct {
  MsgHeader msg_header;
  // MsgMeta msg_meta;
  SensorCameraErrorCode CameraErrorCode[SENSOR_CAMERA_NUM];  // 0-left  1-right 2--front 3-rear (当前仅包含环视相机故障码)
} _STRUCT_ALIGNED_ SensorCameraStatusSoc2Mcu;                // 相机状态信息

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_SENSOR_CAMERA_STATUS_H_