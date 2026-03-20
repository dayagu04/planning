// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2025/4/29

#ifndef _IFLYAUTO_FDL_UPLOAD_STATE_H_
#define _IFLYAUTO_FDL_UPLOAD_STATE_H_

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
  MsgHeader msg_header;
  MsgMeta msg_meta;
  uint8 upload_state; // 上传状态 0: 未上传 1: 上传中
} _STRUCT_ALIGNED_ FdlUploadState;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_FDL_UPLOAD_STATE_H_
