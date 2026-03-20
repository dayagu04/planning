// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef OTA_LOG_UPLOAD_C_H
#define OTA_LOG_UPLOAD_C_H

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"

#ifdef __cplusplus
namespace iflyauto {
#endif
#define OTA_LOG_PATH_MAX_LEN 128
#pragma pack(4)

typedef enum {
    OTA_STATUS_NOT_START = 0,              // 未开始OTA
    OTA_STATUS_SUCCESS = 1,                // 升级成功
    OTA_STATUS_FAILED = 2                  //升级失败
} _ENUM_PACKED_ OTAStatusEnum;             // OTA状态枚举

typedef enum {
    OTA_LOG_GET_FAILED = 0,                 //获取OTA日志失败
    OTA_LOG_GET_SUCCESS = 1                 //获取OTA日志成功
} _ENUM_PACKED_ OTALogGetStatus;         // OTA日志获取状态

typedef struct {
    MsgHeader             msg_header;
    OTAStatusEnum         ota_status;
    OTALogGetStatus       ota_log_get_status;
    uint8_t               ota_log_path[OTA_LOG_PATH_MAX_LEN];   //日志路径
} _STRUCT_ALIGNED_ OTALogUploadRequest;  // OTA日志上传请求

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // OTA_LOG_UPLOAD_C_H