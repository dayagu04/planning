// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2025/05/14

#ifndef _IFLYAUTO_CALIB_INFO_H_
#define _IFLYAUTO_CALIB_INFO_H_

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

#define CALIB_INFO_DESCRIBE_STR_LEN 128
#define CALIB_SENSORS_STR_LEN 128

typedef struct {
    MsgHeader msg_header;
    MsgMeta msg_meta;
    uint8 calib_type; // 标定类型，0：工厂标定，1：在线标定，2：售后标定
    uint8 calib_status; // 标定状态，0：完成，1：空闲，2：进行中
    uint8 action; // 行为，0：无，1：请求触发上传
    uint64 time; // 状态产生时间（unix时间戳，单位为毫秒）
    char calib_sensors[CALIB_SENSORS_STR_LEN]; // 标定传感器: FW/FL/FR/RL/RR/lidar_front/FN/RN/SFW/SLW/SRW/SRCW/imu/lidar_front/lidar_right
    char desc[CALIB_INFO_DESCRIBE_STR_LEN]; // 信息描述，最长为64字节
} _STRUCT_ALIGNED_ CalibInfo;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_CALIB_INFO_H_