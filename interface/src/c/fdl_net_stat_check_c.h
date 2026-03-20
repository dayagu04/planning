// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2025/10/29

#ifndef _IFLYAUTO_FDL_NET_STAT_CHECK_H_
#define _IFLYAUTO_FDL_NET_STAT_CHECK_H_

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
//车云链路检测使用
typedef struct { 
  MsgHeader msg_header;
  MsgMeta msg_meta;
  uint8 net_check_msg_type;    //0: query request; 1: query ack; 2: result request; 3: result ack
  uint8 net_check_result;      //0: connect failed; 1: connected; 2: not certain
  uint8 bak[2];
} _STRUCT_ALIGNED_ FdlNetStatCheck;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_FDL_NET_STAT_CHECK_H_
