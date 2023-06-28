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
  boolean get_elk_left_intervention_flag_info() {
    return elk_left_intervention_flag_;
  }
  boolean get_elk_right_intervention_flag_info() {
    return elk_right_intervention_flag_;
  }
  uint16 get_elk_state() { return elk_state_; }
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
    elk_state_ = measurement_str_.state;
    if (measurement_str_.state == 4) {
      elk_left_intervention_flag_ = TRUE;
    } else {
      elk_left_intervention_flag_ = FALSE;
    }
    if (measurement_str_.state == 5) {
      elk_right_intervention_flag_ = TRUE;
    } else {
      elk_right_intervention_flag_ = FALSE;
    }
  }

 private:
  framework::Session *session_ = nullptr;
  planning::MeasurementPoint measurement_str_;
  planning::CalibrationParameter calribration_str_;
  planning::EmergencyLaneKeepAlert bsd_lca_;
  uint32 elk_state_{0}; /* ELK功能状态 0:Unavailable 1:Off 2:Standby 3:Active(No
                           Intervention) 4:Active(Left Intervention)
                           5:Active(Right Intervention) */
  boolean elk_left_intervention_flag_{
      FALSE}; /* ELK功能触发左侧报警标志位 0:No
                 Intervention 1:Left Intervention */
  boolean elk_right_intervention_flag_{
      FALSE}; /* ELK功能触发右侧报警标志位 0:No Intervention 1:Rirht
                 Intervention */
  planning::LkasInput *lkas_input_ = nullptr;
  // planning::LkasOutput *lkas_output;
};

}  // namespace planning
#endif