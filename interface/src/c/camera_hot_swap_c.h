// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2025/10/24

#ifndef _IFLYAUTO_CAMERA_HOT_SWAP_H_
#define _IFLYAUTO_CAMERA_HOT_SWAP_H_

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

// 相机重启状态枚举
typedef enum {
    CAMERA_RESTART_INVALID  = 0,   // 无效值
    CAMERA_RESTART_REQUEST  = 1,   // 重启请求
    CAMERA_RESTART_SUCCESS  = 2,   // 重启成功  
    CAMERA_RESTART_FAILED   = 3,   // 重启失败
} _ENUM_PACKED_ CameraRestartStatus;

// 相机启动
typedef enum {
    CAMERA_BOOT_INVALID  = 0,       // 无效值
    CAMERA_BOOT_NORMAL  = 1,        // 正常启动
    CAMERA_BOOT_HOT_SWAP  = 2,      // 热插拔重启  
} _ENUM_PACKED_ CameraBootReason;

typedef struct {
    MsgHeader msg_header; 
    uint8_t reqCnt;                 // 第几次请求
    CameraRestartStatus  type;
} _STRUCT_ALIGNED_ CameraRestartRequest;    // 环视相机 -> FM_A 请求重启的消息struct

typedef struct {
    MsgHeader msg_header;             
    CameraBootReason  reason;
} _STRUCT_ALIGNED_ CameraBootReasonMsg;    // ACAM -> BCAM 启动原因

typedef struct {
    MsgHeader msg_header; 
    uint8_t camera_fault_mask;
    uint8_t camera_link_mask;
} _STRUCT_ALIGNED_ CameraLinkStatusMsg;        // 周视相机 -> 环视相机 上报的相机故障和link状态信息struct

typedef struct {
    MsgHeader msg_header; 
    bool is_camera_restarting;                 // 相机是否在重启中，默认false，重启时为true
} _STRUCT_ALIGNED_ CameraHotSwapStatus;        // FM_A -> 状态机 上报相机是否在重启中

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_CAMERA_HOT_SWAP_H_
