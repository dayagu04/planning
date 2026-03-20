// Copyright (C) iflyauto Technologies (2024). All rights reserved.
// Modified: 2024/04/15

#ifndef _IFLYAUTO_TIME_SYNCHRONIZATION_H_
#define _IFLYAUTO_TIME_SYNCHRONIZATION_H_

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

typedef struct{
    MsgHeader msg_header;
    MsgMeta msg_meta;
    uint64_t vartualTimeStamp;       // 数据面时钟时间(微妙)  
    uint64_t realTimeStamp;          // 管理面时钟时间(微妙)                                 
} _STRUCT_ALIGNED_ TimeSynchronizationInfo;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_TIME_SYNCHRONIZATION_H_