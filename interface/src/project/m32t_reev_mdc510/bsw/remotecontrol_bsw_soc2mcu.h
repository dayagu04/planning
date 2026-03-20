// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef _IFLYAUTO_BSW_MCU2SOC_H_
#define _IFLYAUTO_BSW_MCU2SOC_H_

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

typedef struct {
    uint16      fmHeartBeat;
} _STRUCT_ALIGNED_ SocHeartBeat_T;

typedef struct {
    uint8       todalFrame; // 如果当前产生故障小于等于255个，则todalFrame = 1, 如果大于255个，则除以255 + 1， 如31个，则为2
    uint8       frameId;    // 如果当前产生故障小于等于255个，则frameId =1; 如果大于255个 则按序增加
    uint8       size;       // 本次DTC 个数
    uint8       ub;         // 本次DTC 较上次是否有变化，针对的是整体的DTC，比如多帧， 有变化1， 无变化0
    uint16      DTC_internal_number[255];
} _STRUCT_ALIGNED_ SocDiagInfo_T;

typedef struct {
    uint16      functionstate;
    uint16      machineState;
} _STRUCT_ALIGNED_ SocFunctionState_T;


typedef struct {
    MsgHeader               msg_header;
    // MsgMeta                 msg_meta;
    SocHeartBeat_T          SocHeartBeat;
    SocDiagInfo_T           SocDiagInfo;
    SocFunctionState_T      SocFunctionState;
    uint8                   reserve[128];
} _STRUCT_ALIGNED_ remotecontrol_bsw_soc2mcu_T;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_BSW_MCU2SOC_H_