// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef _IFLYAUTO_LEGACY_INTERFACE2_4_5_HMI_MCU_INNER_H_
#define _IFLYAUTO_LEGACY_INTERFACE2_4_5_HMI_MCU_INNER_H_

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

// noa语音提示开关反馈
typedef enum {
  PROMPT_SAFE,     // 安全
  PROMPT_CONCISE,  // 简洁
  PROMPT_CLOSE,    // 贴心
  PROMPT_OFF,      // 关闭
} _ENUM_PACKED_ NoaVoicePromptSwitch;

typedef enum {
  SENSITIVITY_LEVEL_INVALID,  // 无效
  SENSITIVITY_LEVEL_LOW,      // 低
  SENSITIVITY_LEVEL_MIDDLE,   // 中
  SENSITIVITY_LEVEL_HIGH,     // 高
} _ENUM_PACKED_ SensitivityLevel;

typedef enum {
  OPEN_PARK_NO_REQUEST,  // 无请求
  OPEN_PARK_INT,         // 开启泊入
  OPEN_PARK_OUT,         // 开启泊出
} _ENUM_PACKED_ ApaParkingDirection;

typedef enum {
  PARK_OUT_NO_REQUEST,  // 无请求
  PARK_OUT_TO_LEFT,     // 左侧泊出
  PARK_OUT_TO_RIGHT,    // 右侧泊出
} _ENUM_PACKED_ ApaParkOutDirection;

typedef struct {
  Header header;                                // 头信息
  boolean acc_main_switch;                      // acc功能软开关    (true:on / false:off)
  boolean acc_active_switch;                    // acc激活信号      (true:open acc / false:do nothing)
  boolean acc_cancel_switch;                    // acc关闭信号      (true:close acc / false:do nothing)
  float32 acc_set_real_speed;                   // acc跟车实际时速  (米/秒)
  float32 acc_set_disp_speed;                   // acc跟车表显时速  (米/秒)
  float32 acc_set_time_interval;                // acc跟车时距      (秒)
  boolean acc_stop2go;                          // acc跟车起步确认  (true:确认起步 / false:不起步)
  boolean scc_main_switch;                      // scc功能软开关    (true:on / false:off)
  boolean scc_active_switch;                    // scc激活信号      (true:open scc / false:do nothing)
  boolean scc_cancel_switch;                    // scc关闭信号      (true:close scc / false:do nothing)
  boolean noa_main_switch;                      // noa激活软开关    (true:on / false:off)
  boolean noa_active_switch;                    // noa激活信号      (true:open noa / false:do nothing)
  NoaVoicePromptSwitch noa_voice_promp_switch;  // noa语音提示开关
  boolean noa_cruise_dclc_switch;               // noa变道确认提醒开关(true: 提醒 false:不提醒)
  boolean noa_cancel_switch;                    // noa功能关闭      (true:close noa / false:do nothing)

  boolean fcw_main_switch;                     // fcw软开关        (true:on / false:off)
  SensitivityLevel fcw_set_sensitivity_level;  // fcw敏感度等级
  boolean aeb_main_switch;                     // aeb软开关        (true:on / false:off)
  boolean tsr_main_switch;                     // tsr软开关        (true:on / false:off)
  boolean ihc_main_switch;                     // ihc软开关        (true:on / false:off)
  boolean ldw_main_switch;                     // ldw软开关        (true:on / false:off)
  SensitivityLevel ldw_set_sensitivity_level;  // ldw敏感度等级
  boolean elk_main_switch;                     // elk软开关        (true:on / false:off)
  boolean bsd_main_switch;                     // bsd软开关        (true:on / false:off)
  boolean lca_main_switch;                     // lca软开关        (true:on / false:off)
  boolean dow_main_switch;                     // dow软开关        (true:on / false:off)
  boolean fcta_main_switch;                    // fcta软开关       (true:on / false:off)
  boolean rcta_main_switch;                    // rcta软开关       (true:on / false:off)
  boolean rcw_main_switch;                     // rcw软开关        (true:on / false:off)
  boolean ldp_main_switch;                     // ldp软开关        (true:on / false:off)

  boolean apa_main_switch;                     // apa软开关        (true:on / false:off)
  boolean apa_active_switch;                   // apa激活信号      (true:open acc / false:do nothing)
  boolean apa_cancel_switch;                   // apa取消信号      (true:open acc / false:do nothing)
  int32 apa_select_slot_id;                    // 选择车位id
  boolean apa_avm_main_switch;                 // 泊车avm开关(ture: on / false: no request)
  ApaParkingDirection apa_parking_direction;   // 泊入泊出
  ApaParkOutDirection apa_park_out_direction;  // 泊出方向
} _STRUCT_ALIGNED_ HmiMcuInner;

#pragma pack()
#ifdef __cplusplus
  }  // namespace interface_2_4_5
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_LEGACY_INTERFACE2_4_5_HMI_MCU_INNER_H_