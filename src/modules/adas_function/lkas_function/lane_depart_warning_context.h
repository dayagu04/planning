#ifndef _LANE_DEPART_WARNING_CONTEXT_H_
#define _LANE_DEPART_WARNING_CONTEXT_H_
#include "lane_keep_assist_type.h"

namespace planning {
class LaneDepartWarning {
 public:
  LaneDepartWarning() = default;
  LaneDepartWarning(planning::LkasInput *lkas_input) {
    lkas_input_ = lkas_input;
  }
  void Init(planning::LkasInput *lkas_input);

  void RunOnce();
  bool get_ldw_left_warning_info() { return ldw_left_warning_; }
  bool get_ldw_right_warning_info() { return ldw_right_warning_; }
  PlanningHMI::LDWOutputInfoStr_LDWFunctionFSMWorkState get_ldw_state() {
    return ldw_state_;
  }
  ~LaneDepartWarning() = default;

 private:
  void Update();
  uint16 EnableCode();
  uint16 DisableCode();
  uint16 FaultCode();
  uint16 LeftSuppressionCode();
  uint16 LeftKickDownCode();
  uint16 RightSuppressionCode();
  uint16 RightKickDownCode();
  uint8 StateMachine();
  void set_ldw_output_info() {
    // ldw_state_ = measurement_str_.state;
    switch (measurement_str_.state) {
      case 0:
        ldw_state_ = PlanningHMI::
            LDWOutputInfoStr_LDWFunctionFSMWorkState_LDW_FUNCTION_FSM_WORK_STATE_UNAVAILABLE;
        break;
      case 1:
        ldw_state_ = PlanningHMI::
            LDWOutputInfoStr_LDWFunctionFSMWorkState_LDW_FUNCTION_FSM_WORK_STATE_OFF;
        break;
      case 2:
        ldw_state_ = PlanningHMI::
            LDWOutputInfoStr_LDWFunctionFSMWorkState_LDW_FUNCTION_FSM_WORK_STATE_STANDBY;
        break;
      case 3:
        ldw_state_ = PlanningHMI::
            LDWOutputInfoStr_LDWFunctionFSMWorkState_LDW_FUNCTION_FSM_WORK_STATE_ACTIVE_NO_INTERVENTION;
        break;
      case 4:
        ldw_state_ = PlanningHMI::
            LDWOutputInfoStr_LDWFunctionFSMWorkState_LDW_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION;
        break;
      default:
        ldw_state_ = PlanningHMI::
            LDWOutputInfoStr_LDWFunctionFSMWorkState_LDW_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION;
        break;
    }
    if (measurement_str_.state == 4) {
      ldw_left_warning_ = true;
    } else {
      ldw_left_warning_ = false;
    }
    if (measurement_str_.state == 5) {
      ldw_right_warning_ = true;
    } else {
      ldw_right_warning_ = false;
    }
  }

 private:
  planning::MeasurementPoint measurement_str_;
  planning::CalibrationParameter calribration_str_;
  PlanningHMI::LDWOutputInfoStr_LDWFunctionFSMWorkState ldw_state_{
      PlanningHMI::
          LDWOutputInfoStr_LDWFunctionFSMWorkState_LDW_FUNCTION_FSM_WORK_STATE_OFF}; /* LDW功能状态
0:Unavailable 1:Off 2:Standby 3:Active(No Intervention) 4:Active(Left
Intervention) 5:Active(Right Intervention) */
  bool ldw_left_warning_{
      false}; /* LDW功能触发左侧报警标志位 0:No Warning 1:Left Warning */
  bool ldw_right_warning_{
      false}; /* LDW功能触发右侧报警标志位 0:No Warning 1:Rirht Warning */
  planning::LkasInput *lkas_input_ = nullptr;
};

}  // namespace planning
#endif