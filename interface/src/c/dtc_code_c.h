// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef _IFLYAUTO_DTC_CODE_H_
#define _IFLYAUTO_DTC_CODE_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"

#ifdef __cplusplus
namespace iflyauto {
#endif

#define DTC_CODE_STR_LEN 4
#define DTC_CODE_ALL_STATE_MAX_NUM 400
#pragma pack(4)

typedef struct {
  uint8 code[DTC_CODE_STR_LEN];  // 前三字节代表dtc状态码，第四个字节代表（0:故障恢复，1：故障发生）
} _STRUCT_ALIGNED_ IFLYDtcCode;

typedef struct {
  MsgHeader msg_header;
  MsgMeta msg_meta;
  uint32 dtc_code_size;
  IFLYDtcCode dtc_code[DTC_CODE_ALL_STATE_MAX_NUM];  // dtc状态码
} _STRUCT_ALIGNED_ IFLYAllDtcState;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_LIDAR_OBJECTS_H_
