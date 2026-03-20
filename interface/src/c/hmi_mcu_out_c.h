// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2024/03/29

#ifndef _IFLYAUTO_HMI_MCU_OUT_H_
#define _IFLYAUTO_HMI_MCU_OUT_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"
#include "adas_function_c.h"

#define HMI_PDC_RADAER_NUM 12
#ifdef __cplusplus
namespace iflyauto {
#endif

#pragma pack(4)

// PDC状态
typedef enum {
  PDC_INACTIVE,
  PDC_ACTIVE,
  PDC_FAILED,
} _ENUM_PACKED_ PdcSysSts;

// PDC告警等级
typedef enum {
  PDC_NO_WARNING,
  PDC_WARNING_LEVEL_1, 
  PDC_WARNING_LEVEL_2,
  PDC_WARNING_LEVEL_3,
  PDC_WARNING_LEVEL_4,
  PDC_WARNING_LEVEL_5,
} _ENUM_PACKED_ PdcWarningLevel;

// PDC雷达位置
typedef enum {
  PDC_RADAR_START,
  PDC_FLS_RADAR = PDC_RADAR_START,  // 前左侧雷达
  PDC_FL_RADAR,   // 左前侧雷达
  PDC_FLM_RADAR,  // 左前侧中雷达
  PDC_FRM_RADAR,  // 右前侧中雷达
  PDC_FR_RADAR,   // 右前雷达
  PDC_FRS_RADAR,  // 前右雷达
  PDC_FRONT_RADAR_NUM,
  PDC_RRS_RADAR = PDC_FRONT_RADAR_NUM,  // 后右雷达
  PDC_RR_RADAR,   // 右后雷达
  PDC_RRM_RADAR,  // 右后侧中雷达
  PDC_RLM_RADAR,  // 左后侧中雷达
  PDC_RL_RADAR,   // 左后侧雷达
  PDC_RLS_RADAR,  // 后左侧中雷达
  PDC_RADAER_NUM,
} _ENUM_PACKED_ PdcRadarPos;

// 单个雷达信息
typedef struct {
  int32 obj_dis;         // 最近障碍物距离，单位cm
  PdcRadarPos radar_pos; // 雷达位置
  PdcWarningLevel warning_level; // 告警等级
} _STRUCT_ALIGNED_ PdcRadarInfo;

typedef struct {
  PdcSysSts pdc_sys_sts;
  PdcRadarInfo pdc_info[HMI_PDC_RADAER_NUM]; // HMI_PDC_RADAR_NUM: 12, 雷达数量
} _STRUCT_ALIGNED_ HmiPdcInfo;

typedef struct {
  MsgHeader msg_header;
  MsgMeta msg_meta;
  ADASFunctionOutputInfo adas_out_info; // ADAS输出信息
  HmiPdcInfo hmi_pdc_info;              // PDC报警信息
} _STRUCT_ALIGNED_ HmiMcuOut;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_HMI_MCU_OUT_H_