
#ifndef _TSR_STEP_H_
#define _TSR_STEP_H_

#include "Platform_Types.h"
#include "debug_info_log.h"
#include "environmental_model.h"
#include "planning_hmi_c.h"
#include "virtual_lane_manager.h"

namespace planning {
#define TSR_StateMachine_IN_ACTIVE 1   // TSR一级主状态
#define TSR_StateMachine_IN_FAULT 2    // TSR一级主状态
#define TSR_StateMachine_IN_OFF 3      // TSR一级主状态
#define TSR_StateMachine_IN_STANDBY 4  // TSR一级主状态

// TSR算法输入信号结构体定义
typedef struct TSRSysInput {
  bool tsr_main_switch;   // TSR开关 0:Off 1:On
  uint8 tsr_speed_limit;  // TSR识别到的限速标识牌 单位:km/h
  float32 vehicle_speed_display_kph;  // 本车车速 单位:kph
} TSRSysInput;

// TSR算法状态结构体定义
typedef struct TSRSysState {
  uint16 tsr_enable_code;
  uint16 tsr_disable_code;
  uint16 tsr_fault_code;
  uint8 tsr_state;  // TSR功能状态 0:Unavailable 1:Off 2:Standby 3:Active
  uint8 tsr_speed_limit;  // TSR识别到的限速标识牌 单位:km/h
  bool tsr_warning;       // TSR超速报警标志位 0:No Warning 1:Warning
} TSRSysState;

// TSR算法结构体定义
typedef struct TSRSys {
  TSRSysInput input;
  TSRSysState state;
} TSRSys;

class TrafficSignRecognition {
 public:
  TrafficSignRecognition(planning::framework::Session *session) {
    session_ = session;
  }
  void init(planning::framework::Session *session) { session_ = session; }
  void RunOnce();
  iflyauto::TSRFunctionFSMWorkState get_tsr_state_info() {
    return tsr_state_;  // TSR功能状态 0:Unavailable 1:Off 2:Standby 3:Active
  }
  uint8 get_tsr_speed_limit_info() {
    return tsr_speed_limit_;  // TSR识别到的限速标识牌 单位:km/h
  }
  bool get_tsr_warning_info() {
    return tsr_warning_;  // TSR超速报警标志位 0:No Warning 1:Warning
  }
  ~TrafficSignRecognition() = default;

 private:
  void Update();
  uint16 TSREnableCode();
  uint16 TSRDisableCode();
  uint16 TSRFaultCode();
  uint8 TSRStateMachine();
  void set_tsr_output_info() {
    // TSR功能状态 0:Unavailable 1:Off 2:Standby 3:Active
    switch (tsr_sys_.state.tsr_state) {
      case 0:
        tsr_state_ = iflyauto::TSR_FUNCTION_FSM_WORK_STATE_UNAVAILABLE;
        break;
      case 1:
        tsr_state_ = iflyauto::TSR_FUNCTION_FSM_WORK_STATE_OFF;
        break;
      case 2:
        tsr_state_ = iflyauto::TSR_FUNCTION_FSM_WORK_STATE_STANDBY;
        break;
      default:
        tsr_state_ = iflyauto::TSR_FUNCTION_FSM_WORK_STATE_ACTIVE;
        break;
    }
    tsr_speed_limit_ =
        tsr_sys_.state.tsr_speed_limit;  // TSR识别到的限速标识牌 单位:km/h
    tsr_warning_ =
        tsr_sys_.state.tsr_warning;  // TSR超速报警标志位 0:No Warning 1:Warning
  }

 private:
  planning::framework::Session *session_;
  TSRSys tsr_sys_;
  iflyauto::TSRFunctionFSMWorkState tsr_state_{
      iflyauto::TSR_FUNCTION_FSM_WORK_STATE_OFF};  // TSR功能状态
                                                   // 0:Unavailable 1:Off
                                                   // 2:Standby 3:Active
  uint8 tsr_speed_limit_;  // TSR识别到的限速标识牌 单位:km/h
  bool tsr_warning_;       // TSR超速报警标志位 0:No Warning 1:Warning
};
}
#endif
