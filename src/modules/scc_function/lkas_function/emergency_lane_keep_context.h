#ifndef EMERGENCY_LANE_KEEP_CONTEXT_H_
#define EMERGENCY_LANE_KEEP_CONTEXT_H_
#include "emergency_lane_keep_alert_context.h"
#include "lane_keep_assist_type.h"

namespace planning {
class EmergencyLaneKeep {
 public:
  EmergencyLaneKeep() = default;
  EmergencyLaneKeep(LkasInput *lkas_input, framework::Session *session) {
    lkas_input_ = lkas_input;
    session_ = session;
  }
  void Init(LkasInput *lkas_input, framework::Session *session);
  void RunOnce();
  bool get_elk_left_intervention_flag_info() {
    return elk_left_intervention_flag_;
  }
  bool get_elk_right_intervention_flag_info() {
    return elk_right_intervention_flag_;
  }
  PlanningHMI::ELKOutputInfoStr_ELKFunctionFSMWorkState get_elk_state() {
    return elk_state_;
  }
  ~EmergencyLaneKeep() = default;

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
  void ste_elk_output_info() {
    switch (measurement_str_.state) {
      case 0:
        elk_state_ = PlanningHMI::
            ELKOutputInfoStr_ELKFunctionFSMWorkState_ELK_FUNCTION_FSM_WORK_STATE_UNAVAILABLE;
        break;
      case 1:
        elk_state_ = PlanningHMI::
            ELKOutputInfoStr_ELKFunctionFSMWorkState_ELK_FUNCTION_FSM_WORK_STATE_OFF;
        break;
      case 2:
        elk_state_ = PlanningHMI::
            ELKOutputInfoStr_ELKFunctionFSMWorkState_ELK_FUNCTION_FSM_WORK_STATE_STANDBY;
        break;
      case 3:
        elk_state_ = PlanningHMI::
            ELKOutputInfoStr_ELKFunctionFSMWorkState_ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_NO_INTERVENTION;
        break;
      case 4:
        elk_state_ = PlanningHMI::
            ELKOutputInfoStr_ELKFunctionFSMWorkState_ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION;
        break;
      default:
        elk_state_ = PlanningHMI::
            ELKOutputInfoStr_ELKFunctionFSMWorkState_ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION;
        break;
    }
    if (measurement_str_.state == 4) {
      elk_left_intervention_flag_ = true;
    } else {
      elk_left_intervention_flag_ = false;
    }
    if (measurement_str_.state == 5) {
      elk_right_intervention_flag_ = true;
    } else {
      elk_right_intervention_flag_ = false;
    }
  }

 private:
  framework::Session *session_ = nullptr;
  planning::MeasurementPoint measurement_str_;
  planning::CalibrationParameter calribration_str_;
  planning::EmergencyLaneKeepAlert bsd_lca_;
  PlanningHMI::ELKOutputInfoStr_ELKFunctionFSMWorkState elk_state_{
      PlanningHMI::
          ELKOutputInfoStr_ELKFunctionFSMWorkState_ELK_FUNCTION_FSM_WORK_STATE_OFF}; /* ELK功能状态
0:Unavailable 1:Off 2:Standby 3:Active(No Intervention) 4:Active(Left
Intervention) 5:Active(Right Intervention) */
  bool
      elk_left_intervention_flag_{false}; /* ELK功能触发左侧报警标志位 0:No
                                             Intervention 1:Left Intervention */
  bool elk_right_intervention_flag_{
      false}; /* ELK功能触发右侧报警标志位 0:No Intervention 1:Rirht
                 Intervention */
  planning::LkasInput *lkas_input_ = nullptr;
  // planning::LkasOutput *lkas_output;
};

}  // namespace planning
#endif