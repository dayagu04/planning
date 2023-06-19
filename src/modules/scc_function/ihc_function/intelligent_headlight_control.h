#ifndef _IHC_STEP_H_
#define _IHC_STEP_H_

#include "Platform_Types.h"
#include "debug_info_log.h"
#include "environmental_model.h"
#include "frame.h"
#include "obstacle_manager.h"
#include "planning_output_context.h"
#include "virtual_lane_manager.h"

namespace planning {
#define IHC_StateMachine_IN_ACTIVE 1   // IHC一级主状态
#define IHC_StateMachine_IN_FAULT 2    // IHC一级主状态
#define IHC_StateMachine_IN_OFF 3      // IHC一级主状态
#define IHC_StateMachine_IN_STANDBY 4  // IHC一级主状态

// IHC算法输入信号结构体定义
typedef struct IHCSysInput {
  boolean ihc_main_switch;            // IHC开关 0:Off 1:On
  float32 vehicle_speed_display_kph;  // 本车车速 单位:kph
  boolean auto_light_state;           // 自动灯光控制状态 0:Off 1:On
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

class IntelligentHeadlightControl {
 public:
  IntelligentHeadlightControl(planning::framework::Session *session) { session_ = session; }
  void init(planning::framework::Session *session) { session_ = session; }
  void RunOnce();
  void Update();
  uint16 IHCEnableCode();
  uint16 IHCDisableCode();
  uint16 IHCFaultCode();
  uint8 IHCStateMachine();
  boolean IHCRequest();
  void set_ihc_output_info();
  boolean get_ihc_request_status_info() { return ihc_request_status_; }
  boolean get_ihc_request_info() { return ihc_request_; }
  uint8 get_ihc_state_info() { return ihc_state_; }

 private:
  planning::framework::Session *session_;
  IHCSys ihc_sys_;
  boolean ihc_request_status_;  // IHC请求状态 0:No Request 1:Request
  boolean ihc_request_;         // IHC请求 0:LowBeam 1:HighBeam
  uint8 ihc_state_;             // IHC功能状态 0:Unavailable 1:Off 2:Standby 3:Active
};
}
#endif
