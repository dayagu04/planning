// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2025/8/14

#ifndef _IFLYAUTO_DEGRADED_DRIVING_FUNCTION_H_
#define _IFLYAUTO_DEGRADED_DRIVING_FUNCTION_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include <stdint.h>
#include "common_c.h"

#ifdef __cplusplus
namespace iflyauto {
#endif
#pragma pack(4)
typedef uint64_t HmiTagMask;  // 64位掩码，最多支持64个HmiTag，bit位表示的HmiTag集合，举例：hmi_mask的第0位为1，表示存在HMI_NO_ERROR，为0，表示不存在HMI_NO_ERROR，以此类推

typedef enum {
  NO_ERROR = 0,                     // no error 
  INHIBIT_DEGRADED_TO_ACC = 1,      // inhibit(降级到acc)
  ERROR_DEGRADED_TO_ACC = 2,        // error(降级到acc)
  INHIBIT = 3,                      // inhibit
  ERROR_DEGRADED = 4,               // error
  ERROR_SAFE_STOP = 5,              // error(安全停车)
  MCU_COMM_SHUTDOWN = 6             // mcu关断对外通讯
} _ENUM_PACKED_ DegradedLevel;

typedef enum {
    HMI_NO_ERROR = 0,                        // 无错误

    // 环视摄像头
    HMI_CAM_SVC_FRONT_FAULT,                 // 前环视摄像头（故障）
    HMI_CAM_SVC_FRONT_BLOCKED,               // 前环视摄像头（遮挡）
    HMI_CAM_SVC_REAR_FAULT,                  // 后环视摄像头（故障）
    HMI_CAM_SVC_REAR_BLOCKED,                // 后环视摄像头（遮挡）
    HMI_CAM_SVC_LEFT_FAULT,                  // 左环视摄像头（故障）
    HMI_CAM_SVC_LEFT_BLOCKED,                // 左环视摄像头（遮挡）
    HMI_CAM_SVC_RIGHT_FAULT,                 // 右环视摄像头（故障）
    HMI_CAM_SVC_RIGHT_BLOCKED,               // 右环视摄像头（遮挡）
    HMI_CAM_GENERIC_FAULT,                   // 摄像头故障（通用）

    // 超声波雷达
    HMI_ULTRASONIC_LEFT_FRONT_SIDE_FAULT,    // 左前侧超声波雷达（故障）
    HMI_ULTRASONIC_LEFT_FRONT_SIDE_BLOCKED,  // 左前侧超声波雷达（遮挡）
    HMI_ULTRASONIC_LEFT_FRONT_CORNER_FAULT,  // 左前角超声波雷达（故障）
    HMI_ULTRASONIC_LEFT_FRONT_CORNER_BLOCKED,// 左前角超声波雷达（遮挡）
    HMI_ULTRASONIC_LEFT_FRONT_CENTER_FAULT,  // 左前中超声波雷达（故障）
    HMI_ULTRASONIC_LEFT_FRONT_CENTER_BLOCKED,// 左前中超声波雷达（遮挡）

    HMI_ULTRASONIC_RIGHT_FRONT_CENTER_FAULT, // 右前中超声波雷达（故障）
    HMI_ULTRASONIC_RIGHT_FRONT_CENTER_BLOCKED,// 右前中超声波雷达（遮挡）
    HMI_ULTRASONIC_RIGHT_FRONT_CORNER_FAULT, // 右前角超声波雷达（故障）
    HMI_ULTRASONIC_RIGHT_FRONT_CORNER_BLOCKED,// 右前角超声波雷达（遮挡）
    HMI_ULTRASONIC_RIGHT_FRONT_SIDE_FAULT,   // 右前侧超声波雷达（故障）
    HMI_ULTRASONIC_RIGHT_FRONT_SIDE_BLOCKED, // 右前侧超声波雷达（遮挡）

    HMI_ULTRASONIC_LEFT_REAR_SIDE_FAULT,     // 左后侧超声波雷达（故障）
    HMI_ULTRASONIC_LEFT_REAR_SIDE_BLOCKED,   // 左后侧超声波雷达（遮挡）
    HMI_ULTRASONIC_LEFT_REAR_CORNER_FAULT,   // 左后角超声波雷达（故障）
    HMI_ULTRASONIC_LEFT_REAR_CORNER_BLOCKED, // 左后角超声波雷达（遮挡）
    HMI_ULTRASONIC_LEFT_REAR_CENTER_FAULT,   // 左后中超声波雷达（故障）
    HMI_ULTRASONIC_LEFT_REAR_CENTER_BLOCKED, // 左后中超声波雷达（遮挡）

    HMI_ULTRASONIC_RIGHT_REAR_CENTER_FAULT,  // 右后中超声波雷达（故障）
    HMI_ULTRASONIC_RIGHT_REAR_CENTER_BLOCKED,// 右后中超声波雷达（遮挡）
    HMI_ULTRASONIC_RIGHT_REAR_CORNER_FAULT,  // 右后角超声波雷达（故障）
    HMI_ULTRASONIC_RIGHT_REAR_CORNER_BLOCKED,// 右后角超声波雷达（遮挡）
    HMI_ULTRASONIC_RIGHT_REAR_SIDE_FAULT,    // 右后侧超声波雷达（故障）
    HMI_ULTRASONIC_RIGHT_REAR_SIDE_BLOCKED,  // 右后侧超声波雷达（遮挡）
    HMI_ULTRASONIC_GENERIC_FAULT,            // 超声波雷达故障（共用）

    // 底盘/域控
    HMI_STEERING_FAULT,                      // 转向故障
    HMI_BRAKE_FAULT,                         // 制动故障
    HMI_DRIVE_FAULT,                         // 驱动故障
    HMI_OTHERS_FAULT,                        // 其余关联件故障
    HMI_DOMAIN_HW_FAULT,                     // 域控硬件故障
    HMI_DOMAIN_SW_FAULT,                     // 域控软件故障

    // 前视/周视摄像头
    HMI_CAM_FRONT_FAULT,                     // 前视摄像头（故障）
    HMI_CAM_FRONT_LEFT_FAULT,                // 左侧前周视（故障）
    HMI_CAM_REAR_LEFT_FAULT,                 // 左侧后周视（故障）
    HMI_CAM_FRONT_RIGHT_FAULT,               // 右侧前周视（故障）
    HMI_CAM_REAR_RIGHT_FAULT,                // 右侧后周视（故障）
    HMI_CAM_BACK_FAULT,                      // 后视摄像头（故障）

    HMI_CAM_FRONT_BLOCKED,                   // 前视摄像头（遮挡）
    HMI_CAM_FRONT_LEFT_BLOCKED,              // 左侧前周视（遮挡）
    HMI_CAM_REAR_LEFT_BLOCKED,               // 左侧后周视（遮挡）
    HMI_CAM_FRONT_RIGHT_BLOCKED,             // 右侧前周视（遮挡）
    HMI_CAM_REAR_RIGHT_BLOCKED,              // 右侧后周视（遮挡）
    HMI_CAM_BACK_BLOCKED,                    // 后视摄像头（遮挡）

    // 其他传感器
    HMI_IMU_FAULT,                           // IMU故障
    HMI_GNSS_FAULT,                          // GNSS故障
    HMI_RADAR_4D_FAULT,                      // 4D毫米波雷达故障
    HMI_RADAR_CORNER_FAULT,                  // 毫米波角雷达（故障）
    HMI_RADAR_CORNER_BLOCKED                 // 毫米波角雷达（遮挡）

} _ENUM_PACKED_ HmiTag;

typedef struct {
    DegradedLevel degraded;  // 降级等级
    HmiTagMask hmi_mask;     // bit位表示的HmiTag集合, 举例：hmi_mask的第0位为1，表示存在HMI_NO_ERROR，为0，表示不存在HMI_NO_ERROR，以此类推
} _STRUCT_ALIGNED_ FunctionStatus;

typedef struct { 
  MsgHeader msg_header;
  MsgMeta msg_meta;
  FunctionStatus acc;
  FunctionStatus lcc;
  FunctionStatus hnoa;
  FunctionStatus mnoa;
  FunctionStatus aeb;
  FunctionStatus ldw;
  FunctionStatus ldp;
  FunctionStatus elk;
  FunctionStatus tsr;
  FunctionStatus isli;
  FunctionStatus ihc;
  FunctionStatus rpa;
  FunctionStatus apa;
  FunctionStatus rads;
  FunctionStatus meb;
  FunctionStatus hpp;
  FunctionStatus mrm;
  FunctionStatus pa;
  FunctionStatus nra;
  FunctionStatus mapping;
  FunctionStatus fcta_fctb;
  FunctionStatus rcta_rctb;
  FunctionStatus rcw;
  FunctionStatus dow;
  FunctionStatus bsd;
  FunctionStatus amap;
  FunctionStatus dai;
} _STRUCT_ALIGNED_ DegradedDrivingFunction;

typedef struct 
{
    DegradedLevel acc;
    DegradedLevel lcc;
    DegradedLevel hnoa;
    DegradedLevel mnoa;
    DegradedLevel aeb;
    DegradedLevel ldw;
    DegradedLevel ldp;
    DegradedLevel elk;
    DegradedLevel tsr;
    DegradedLevel isli;
    DegradedLevel ihc;
    DegradedLevel rpa;
    DegradedLevel apa;
    DegradedLevel rads;
    DegradedLevel meb;
    DegradedLevel hpp;
    DegradedLevel mrm;
    DegradedLevel pa;
    DegradedLevel nra;
    DegradedLevel mapping;
    DegradedLevel fcta_fctb;
    DegradedLevel rcta_rctb;
    DegradedLevel rcw;
    DegradedLevel dow;
    DegradedLevel bsd;
    DegradedLevel amap;
    DegradedLevel dai;
}__attribute__((aligned(4))) DegradedDrivingFunctionMcu;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_DEGRADED_DRIVING_FUNCTION_H_
