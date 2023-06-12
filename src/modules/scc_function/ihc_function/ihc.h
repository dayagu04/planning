#ifndef _IHC_STEP_H_
#define _IHC_STEP_H_

#include <math.h>
#include "Platform_Types.h"
#include "frame.h"
#include "obstacle_manager.h"
#include "virtual_lane_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

// IHC算法输入信号结构体定义
typedef struct IHCSysInput {
  boolean ihc_main_switch;            // IHC开关 0:Off 1:On
  float32 vehicle_speed_display_kph;  // 本车车速 单位:kph
  boolean auto_light_state;           //自动灯光控制状态 0:Off 1:On
} IHCSysInput;

// IHC算法状态结构体定义
typedef struct IHCSysState {
  uint16 ihc_enable_code;
  uint16 ihc_disable_code;
  uint16 ihc_fault_code;
  uint8 ihc_state;             // IHC功能状态 0:Unavailable 1:Off 2:Standby 3:Active
  boolean ihc_request_status;  // IHC请求状态 0:No Request 1:Request
  boolean ihc_request;         // IHC请求 0:LowBeam 1:HighBeam
} IHCSysState;

// IHC算法结构体定义
typedef struct IHCSys {
  IHCSysInput input;
  IHCSysState state;
} IHCSys;

extern void IHCStep(planning::framework::Session *session);

#ifdef __cplusplus
}
#endif

#endif
