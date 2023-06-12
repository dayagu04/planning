#ifndef _TSR_STEP_H_
#define _TSR_STEP_H_

#include "Platform_Types.h"
#include "frame.h"
#include "virtual_lane_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

// TSR算法输入信号结构体定义
typedef struct TSRSysInput {
  boolean tsr_main_switch;            // TSR开关 0:Off 1:On
  uint8 tsr_speed_limit;              // TSR识别到的限速标识牌 单位:km/h
  float32 vehicle_speed_display_kph;  // 本车车速 单位:kph
} TSRSysInput;

// TSR算法状态结构体定义
typedef struct TSRSysState {
  uint16 tsr_enable_code;
  uint16 tsr_disable_code;
  uint16 tsr_fault_code;
  uint8 tsr_state;        // TSR功能状态 0:Unavailable 1:Off 2:Standby 3:Active
  uint8 tsr_speed_limit;  // TSR识别到的限速标识牌 单位:km/h
  boolean tsr_warning;    // TSR超速报警标志位 0:No Warning 1:Warning
} TSRSysState;

// TSR算法结构体定义
typedef struct TSRSys {
  TSRSysInput input;
  TSRSysState state;
} TSRSys;

extern void TSRStep(planning::framework::Session *session);

#ifdef __cplusplus
}
#endif

#endif
