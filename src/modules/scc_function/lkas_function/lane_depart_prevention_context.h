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
  boolean get_ldp_left_intervention_flag_info() {
    return ldp_left_intervention_flag_;
  }
  boolean get_ldp_right_intervention_flag_info() {
    return ldp_right_intervention_flag_;
  }
  uint16 get_ldp_state() { return ldp_state_; }
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
    ldp_state_ = measurement_str_.state;
    if (measurement_str_.state == 4) {
      ldp_left_intervention_flag_ = TRUE;
    } else {
      ldp_left_intervention_flag_ = FALSE;
    }
    if (measurement_str_.state == 5) {
      ldp_right_intervention_flag_ = TRUE;
    } else {
      ldp_right_intervention_flag_ = FALSE;
    }
  }

 private:
  planning::MeasurementPoint measurement_str_;
  planning::CalibrationParameter calribration_str_;
  uint32 ldp_state_{0}; /* LDP功能状态 0:Unavailable 1:Off 2:Standby 3:Active(No
                           Intervention) 4:Active(Left Intervention)
                           5:Active(Right Intervention) */
  boolean
      ldp_left_intervention_flag_{FALSE}; /* LDP功能触发左侧报警标志位 0:No
                                             Intervention 1:Left Intervention */
  boolean ldp_right_intervention_flag_{
      FALSE}; /* LDP功能触发右侧报警标志位 0:No Intervention 1:Rirht
                 Intervention */
  planning::LkasInput *lkas_input_ = nullptr;
  // planning::LkasOutput *lkas_output;
};

}  // namespace planning
#endif