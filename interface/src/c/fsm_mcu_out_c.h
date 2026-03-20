// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2024/03/29

#ifndef _IFLYAUTO_FSM_MCU_OUT_H_
#define _IFLYAUTO_FSM_MCU_OUT_H_

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
    MCU_STATE_MANUAL = 0,     
    MCU_STATE_PILOT = 1,          
    MCU_STATE_PARKING = 2,        
    MCU_STATE_MRC = 3,       
    MCU_STATE_GUARD = 4,
} _ENUM_PACKED_ McuCurrentState;

typedef enum {
   HEAT_OFF = 0, //  发送关闭加热信号
   HEAT_ON  = 1,  // 发送开启加热信号
} _ENUM_PACKED_ HeatReq;

typedef struct {
  MsgHeader msg_header;
  MsgMeta msg_meta;
  McuCurrentState mcu_state;   // 状态机当前状态 <需对照状态跳转图查询>
  boolean lat_override_apa;    // apa横向override    (true:有效/false:无效)
  boolean lat_override_pilot;  // pilot横向override  (true:有效/false:无效)
  HeatReq heat_req_info;    // 加热请求
} _STRUCT_ALIGNED_ FsmMcuOut;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_FSM_MCU_OUT_H_