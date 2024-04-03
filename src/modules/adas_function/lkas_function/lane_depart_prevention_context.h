#ifndef _LANE_DEPART_PREVEMTION_CONTEXT_H_
#define _LANE_DEPART_PREVEMTION_CONTEXT_H_
#include "lane_keep_assist_type.h"

namespace planning {
class LaneDepartPrevention {
 public:
  LaneDepartPrevention() = default;
  LaneDepartPrevention(planning::LkasInput *lkas_input) {
    lkas_input_ = lkas_input;
  }
  void Init(planning::LkasInput *lkas_input);
  void RunOnce();
  bool get_ldp_left_intervention_flag_info() {
    return ldp_left_intervention_flag_;
  }
  bool get_ldp_right_intervention_flag_info() {
    return ldp_right_intervention_flag_;
  }
  iflyauto::LDPFunctionFSMWorkState get_ldp_state() { return ldp_state_; }
  ~LaneDepartPrevention() = default;

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
  void set_ldp_output_info() {
    switch (measurement_str_.state) {
      case 0:
        ldp_state_ = iflyauto::LDP_FUNCTION_FSM_WORK_STATE_UNAVAILABLE;
        break;
      case 1:
        ldp_state_ = iflyauto::LDP_FUNCTION_FSM_WORK_STATE_OFF;
        break;
      case 2:
        ldp_state_ = iflyauto::LDP_FUNCTION_FSM_WORK_STATE_STANDBY;
        break;
      case 3:
        ldp_state_ =
            iflyauto::LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_NO_INTERVENTION;
        break;
      case 4:
        ldp_state_ =
            iflyauto::LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION;
        break;
      default:
        ldp_state_ =
            iflyauto::LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION;
        break;
    }
    if (measurement_str_.state == 4) {
      ldp_left_intervention_flag_ = true;
    } else {
      ldp_left_intervention_flag_ = false;
    }
    if (measurement_str_.state == 5) {
      ldp_right_intervention_flag_ = true;
    } else {
      ldp_right_intervention_flag_ = false;
    }
  }

 private:
  planning::MeasurementPoint measurement_str_;
  planning::CalibrationParameter calribration_str_;
  iflyauto::LDPFunctionFSMWorkState ldp_state_{
      iflyauto::LDP_FUNCTION_FSM_WORK_STATE_OFF}; /* LDP功能状态
0:Unavailable 1:Off 2:Standby 3:Active(No Intervention) 4:Active(Left
Intervention) 5:Active(Right Intervention) */
  bool ldp_left_intervention_flag_{
      false}; /* LDP功能触发左侧报警标志位 0:No
                 Intervention 1:Left Intervention */
  bool ldp_right_intervention_flag_{
      false}; /* LDP功能触发右侧报警标志位 0:No Intervention 1:Rirht
                 Intervention */
  planning::LkasInput *lkas_input_ = nullptr;
};

}  // namespace planning
#endif