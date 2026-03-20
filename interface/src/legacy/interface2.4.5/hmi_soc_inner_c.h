// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2024/03/29

#ifndef _IFLYAUTO_LEGACY_INTERFACE2_4_5_HMI_SOC_INNER_H_
#define _IFLYAUTO_LEGACY_INTERFACE2_4_5_HMI_SOC_INNER_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "interface2.4.5/common_c.h"

#ifdef __cplusplus
namespace iflyauto {
  namespace interface_2_4_5 {
#endif

#pragma pack(4)

// 泊车模式
typedef enum {
  APA_WORK_MODE_PARKING_IN = 0,   // 泊入
  APA_WORK_MODE_PARKING_OUT = 1,  // 泊出
} _ENUM_PACKED_ ApaWorkMode;

// HMI输入信息
typedef struct {
  Header header;                // 头信息
  boolean apa_main_switch;      // 泊车主开关       (true:open / false:close)
  boolean apa_active_switch;    // 泊车激活开关     (true:response / false:no response)
  ApaWorkMode apa_active_mode;  // 泊车模式
  boolean apa_cancel_switch;    // 泊车取消         (true:response / false:no response)
  uint32 apa_select_slot_id;    // 选择的车位id
  boolean apa_start;            // 开始泊车         (true:response / false:no response)
  boolean apa_resume;           // 恢复泊车         (true:response / false:no response)
  boolean calib_active_switch;  // 标定激活按键     (true:response / false:no response)
} _STRUCT_ALIGNED_ HmiSocInner;

#pragma pack()
#ifdef __cplusplus
  }  // namespace interface_2_4_5
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_LEGACY_INTERFACE2_4_5_HMI_SOC_INNER_H_