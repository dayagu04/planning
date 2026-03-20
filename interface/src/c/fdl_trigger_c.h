// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2025/4/29

#ifndef _IFLYAUTO_FDL_TRIGGER_H_
#define _IFLYAUTO_FDL_TRIGGER_H_

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

#define FDL_SFTP_PATH_LEN_MAX 256

typedef struct { 
  MsgHeader msg_header;
  MsgMeta msg_meta;
  size_t trigger_ack;
  uint32_t pack_log_before_duration;//打包起始时间，单位是秒，如3600，就是打包1小时前到当前这段时间的日志
  char sftp_path[FDL_SFTP_PATH_LEN_MAX];//sftp 的发送文件存储路径
  char sftp_A_log_file_name[FDL_SFTP_PATH_LEN_MAX];//A面的应用日志文件名
  char sftp_plat_log_file_name[FDL_SFTP_PATH_LEN_MAX];//打包mdc平台日志文件名
  char bak[FDL_SFTP_PATH_LEN_MAX];
} _STRUCT_ALIGNED_ FdlTriggerState;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_FDL_TRIGGER_H_
