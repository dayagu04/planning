// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2024/03/29

#ifndef _IFLYAUTO_LEGACY_INTERFACE2_4_5_FSM_H_
#define _IFLYAUTO_LEGACY_INTERFACE2_4_5_FSM_H_

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

typedef enum {
  FunctionalState_INIT = 0,
  FunctionalState_STANDBY = 1,
  FunctionalState_ERROR = 2,
  FunctionalState_DRIVING = 3,
  /* acc */
  FunctionalState_ACC = 4,
  FunctionalState_ACC_ACTIVATE = 5,
  FunctionalState_ACC_STANDSTILL = 6,
  FunctionalState_ACC_STAND_ACTIVATE = 7,
  FunctionalState_ACC_STAND_WAIT = 8,
  FunctionalState_ACC_OVERRIDE = 9,
  FunctionalState_ACC_SECURE = 10,
  /* scc */
  FunctionalState_SCC = 11,
  FunctionalState_SCC_ACTIVATE = 12,
  FunctionalState_SCC_STANDSTILL = 13,
  FunctionalState_SCC_STAND_ACTIVATE = 14,
  FunctionalState_SCC_STAND_WAIT = 15,
  FunctionalState_SCC_OVERRIDE = 16,
  FunctionalState_SCC_SECURE = 17,
  /* noa */
  FunctionalState_NOA = 18,
  FunctionalState_NOA_ACTIVATE = 19,
  FunctionalState_NOA_STANDSTILL = 20,
  FunctionalState_NOA_OVERRIDE = 21,
  FunctionalState_NOA_SECURE = 22,
  /* park */
  FunctionalState_PARK_IN_APA_IN = 23,
  FunctionalState_PARK_IN_SEARCHING = 24,
  FunctionalState_PARK_IN_SELECT = 25,
  FunctionalState_PARK_IN_READY = 26,
  FunctionalState_PARK_IN_NO_READY = 27,
  FunctionalState_PARK_IN_ACTIVATE = 28,
  FunctionalState_PARK_IN_ACTIVATE_WAIT = 29,
  FunctionalState_PARK_IN_ACTIVATE_CONTROL = 30,
  FunctionalState_PARK_IN_SUSPEND = 31,
  FunctionalState_PARK_IN_SUSPEND_ACTIVATE = 32,
  FunctionalState_PARK_IN_SUSPEND_CLOSE = 33,
  FunctionalState_PARK_IN_SECURE = 34,
  FunctionalState_PARK_IN_COMPLETED = 35,
  FunctionalState_PARK_OUT_SEARCHING = 36,
  FunctionalState_PARK_OUT_READY = 37,
  FunctionalState_PARK_OUT_NO_READY = 38,
  FunctionalState_PARK_OUT_ACTIVATE = 39,
  FunctionalState_PARK_OUT_SUSPEND_ACTIVATE = 40,
  FunctionalState_PARK_OUT_SUSPEND_CLOSE = 41,
  FunctionalState_PARK_OUT_SECURE = 42,
  FunctionalState_PARK_OUT_COMPLETED = 43,
  // hpp
  FunctionalState_HPP_IN_SILENT_MAPPING,
  FunctionalState_HPP_IN_READY,
  FunctionalState_HPP_IN_READY_RELEARNINGROUTE,
  FunctionalState_HPP_IN_READY_LEARNINGROUTE,
  FunctionalState_HPP_IN_READY_EXISTROUTE,
  FunctionalState_HPP_IN_READY_REENTRYROUTE,
  FunctionalState_HPP_IN_LEARNING,
  FunctionalState_HPP_IN_LEARNING_CRUISE,
  FunctionalState_HPP_IN_LEARNING_MANUAL_PARKING,
  FunctionalState_HPP_IN_LEARNING_UPLOAD_PATH,
  FunctionalState_HPP_IN_LEARNING_PARKING,
  FunctionalState_HPP_IN_LEARNING_PARKING_SELECT,
  FunctionalState_HPP_IN_LEARNING_PARKING_NO_READY,
  FunctionalState_HPP_IN_LEARNING_PARKING_READY,
  FunctionalState_HPP_IN_LEARNING_PARKING_ACTIVATE,
  FunctionalState_HPP_IN_LEARNING_PARKING_ACTIVATE_WAIT,
  FunctionalState_HPP_IN_LEARNING_PARKING_ACTIVATE_CONTROL,
  FunctionalState_HPP_IN_MEMORY,
  FunctionalState_HPP_IN_MEMORY_READY,
  FunctionalState_HPP_IN_MEMORY_CRUISE,
  FunctionalState_HPP_IN_MEMORY_PARKING,
  FunctionalState_HPP_IN_MEMORY_PARKING_SELECT,
  FunctionalState_HPP_IN_MEMORY_PARKING_NO_READY,
  FunctionalState_HPP_IN_MEMORY_PARKING_READY,
  FunctionalState_HPP_IN_MEMORY_PARKING_ACTIVATE,
  FunctionalState_HPP_IN_MEMORY_PARKING_CONTROLER_WAIT,  // 行泊车控制器切换（行车->泊车）
  FunctionalState_HPP_IN_MEMORY_PARKING_ACTIVATE_CONTROL,
  FunctionalState_HPP_IN_SECURE,
  FunctionalState_HPP_IN_OVERRIDE,
  FunctionalState_HPP_IN_SUSPEND,
  FunctionalState_HPP_IN_COMPLETED,
  // Calibration
  FunctionalState_CALIB_ACTIVATE,
  FunctionalState_CALIB_COMPLETED,
  FunctionalState_CALIB_ERROR,
  FunctionalState_AVM_GET_IMAGE,
  FunctionalState_AVM_RELOAD_COMPLETED,
  FunctionalState_AVM_ERROR,
} _ENUM_PACKED_ FunctionalState;

typedef struct {
  Header header;                  // 头信息
  FunctionalState current_state;  // 状态机当前状态 <需对照状态跳转图查询>
  uint32 state_duration;          // 当前状态已经持续时间         (毫秒)

  /** 附加消息
   *  单位：rad
   *  备注：每位表示一个消息。如状态受抑制的原因, 每位代表某个抑制原因
   *          第1位：未系安全带
   *          第2位：高精地图不可用
   *          第3位：刹车未松开
   **/
  uint32 message;
} _STRUCT_ALIGNED_ FuncStateMachine;

#pragma pack()
#ifdef __cplusplus
  }  // namespace interface_2_4_5
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_LEGACY_INTERFACE2_4_5_FSM_H_