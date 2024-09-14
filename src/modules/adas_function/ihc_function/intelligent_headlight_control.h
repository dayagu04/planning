#ifndef _IHC_STEP_H_
#define _IHC_STEP_H_

#include "Platform_Types.h"
#include "debug_info_log.h"
#include "environmental_model.h"
#include "obstacle_manager.h"
#include "planning_hmi_c.h"
#include "virtual_lane_manager.h"

namespace planning {
#define IHC_StateMachine_IN_ACTIVE 1   // IHC一级主状态
#define IHC_StateMachine_IN_FAULT 2    // IHC一级主状态
#define IHC_StateMachine_IN_OFF 3      // IHC一级主状态
#define IHC_StateMachine_IN_STANDBY 4  // IHC一级主状态

// IHC算法输入信号结构体定义
struct IHCSysInput {
  bool ihc_main_switch;               // IHC开关 0:Off 1:On
  float32 vehicle_speed_display_kph;  // 本车车速 单位:kph
  bool auto_light_state;              // 自动灯光控制状态 0:Off 1:On
};

// IHC算法状态结构体定义
struct IHCSysState {
  uint16 ihc_enable_code;
  uint16 ihc_disable_code;
  uint16 ihc_fault_code;
  uint8 ihc_state;  // IHC功能状态 0:Unavailable 1:Off 2:Standby 3:Active
  bool ihc_request_status;  // IHC请求状态 0:No Request 1:Request
  bool ihc_request;         // IHC请求 0:LowBeam 1:HighBeam
};

// IHC算法结构体定义
struct IHCSys {
  IHCSysInput input;
  IHCSysState state;
};

class IntelligentHeadlightControl {
 public:
  IntelligentHeadlightControl(planning::framework::Session *session) {
    session_ = session;
  }
  void init(planning::framework::Session *session) { session_ = session; }
  void RunOnce();
  bool get_ihc_request_status_info() { return ihc_request_status_; }
  bool get_ihc_request_info() { return ihc_request_; }
  iflyauto::IHCFunctionFSMWorkState get_ihc_state_info() { return ihc_state_; }
  ~IntelligentHeadlightControl() = default;

 private:
  void Update();
  uint16 IHCEnableCode();
  uint16 IHCDisableCode();
  uint16 IHCFaultCode();
  uint8 IHCStateMachine();
  bool IHCRequest();
  void set_ihc_output_info() {
    ihc_request_status_ =
        ihc_sys_.state
            .ihc_request_status;  // IHC请求状态 0:No Request 1:Request
    ihc_request_ =
        ihc_sys_.state
            .ihc_request;  // IHC请求 0:LowBeam 1:HighBeam
                           // IHC功能状态 0:Unavailable 1:Off 2:Standby 3:Active
    switch (ihc_sys_.state.ihc_state) {
      case 0:
        ihc_state_ = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_UNAVAILABLE;
        break;
      case 1:
        ihc_state_ = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_OFF;
        break;
      case 2:
        ihc_state_ = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_STANDBY;
        break;
      default:
        ihc_state_ = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_ACTIVE;
        break;
    }
  }

 private:
  planning::framework::Session *session_;
  IHCSys ihc_sys_;
  bool ihc_request_status_{true};  // IHC请求状态 0:No Request 1:Request
  bool ihc_request_{FALSE};        // IHC请求 0:LowBeam 1:HighBeam
  iflyauto::IHCFunctionFSMWorkState ihc_state_{
      iflyauto::IHC_FUNCTION_FSM_WORK_STATE_OFF};  // IHC功能状态
                                                   // 0:Unavailable 1:Off
                                                   // 2:Standby 3:Active
};
}
#endif
