// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2024/03/29

#ifndef _IFLYAUTO_AVM_RELOAD_RESULT_H_
#define _IFLYAUTO_AVM_RELOAD_RESULT_H_

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
	AVM_NO_MESSAGE = 0,        // 未加载
	AVM_RELOAD_SUCCESS = 1,    // 重加载成功
	AVM_RELOAD_FAIL = 2,       // 重加载失败
	AVM_GET_IMAGE_SUCCESS = 3, // 获取环视图片成功
	AVM_GET_IMAGE_FAIL = 4,    // 获取环视图片失败
} _ENUM_PACKED_ AvmReloadState;

typedef struct {
	MsgHeader msg_header;
	MsgMeta msg_meta;
	AvmReloadState reload_state;    // AVM重加载新标定文件结果
} _STRUCT_ALIGNED_ AvmReloadResult;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_AVM_RELOAD_RESULT_H_