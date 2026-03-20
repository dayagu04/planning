// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef _IFLYAUTO_FM_INFO_H_
#define _IFLYAUTO_FM_INFO_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"

#ifdef __cplusplus
namespace iflyauto {
#endif

#define FM_MESSAGE_DESCRIBE_STR_LEN 64

#pragma pack(4)

/* 故障码详见wiki：http://wiki.iflytek.com/pages/viewpage.action?pageId=629248637
 * TODO：待争端成熟后迁移到本仓库中
 */
typedef struct {
    uint16 alarmId;                         // 告警ID
    uint16 alarmObj;                        // 具体检测的告警对象
    uint16 alarmCount;                      // 告警次数（可不填）
    uint8 clss;                             // 告警类别（可不填），0：状态型，1：事件型
    uint8 level;                            // 告警级别（可不填），0：无故障默认值，1：报警类故障，2：泊车功能故障，3：行车功能故障，4：智驾域SOC故障，5：智驾域严重故障
    uint8 status;                           // 告警状态，0：告警恢复，1：告警产生
    uint64 time;                            // 最后一次告警产生时间（unix时间戳，单位为毫秒）
    char desc[FM_MESSAGE_DESCRIBE_STR_LEN]; // 告警描述，最长为64字节
} _STRUCT_ALIGNED_ FmInfo;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_FM_INFO_H_