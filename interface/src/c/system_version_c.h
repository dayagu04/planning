// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef _IFLYAUTO_SYSTEM_VERSION_H_
#define _IFLYAUTO_SYSTEM_VERSION_H_

#include "common_c.h"

#ifdef __cplusplus
namespace iflyauto {
#endif

#pragma pack(4)

#define VERSION_STR_LEN 32
#define MODULE_NAME_LEN 32
#define MODULE_COMMIT_ID_LEN 40
#define MAX_MODULES 30

// 单个模块信息
typedef struct {;
  uint8 name[MODULE_NAME_LEN];        // 模块名称
	uint8 branch[MODULE_NAME_LEN];      // 模块分支
  uint8 commitid[MODULE_COMMIT_ID_LEN]; // 提交ID
} _STRUCT_ALIGNED_ ModuleInfo;

// 系统版本号消息
typedef struct {
  MsgHeader msg_header;
  uint8 system_version[VERSION_STR_LEN];     // 系统版本号字符串
  uint8 interface_version[VERSION_STR_LEN];  // 接口版本号字符串
  uint8 module_count;                        // 模块数量
  ModuleInfo modules[MAX_MODULES];           // 模块信息数组
} _STRUCT_ALIGNED_ SystemVersion;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_SYSTEM_VERSION_H_