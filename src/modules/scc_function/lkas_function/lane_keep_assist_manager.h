#ifndef _LANE_KEEP_ASSIST_MANAGER_H_
#define _LANE_KEEP_ASSIST_MANAGER_H_
#include "Platform_Types.h"
#include "debug_info_log.h"
#include "environmental_model.h"
#include "frame.h"
#include "planning_output_context.h"
#include "virtual_lane_manager.h"

#include "emergency_lane_keep_context.h"
#include "lane_depart_prevention_context.h"
#include "lane_depart_warning_context.h"

namespace planning {
class LaneKeepAssistManager {
 public:
  LaneKeepAssistManager() = default;
  LaneKeepAssistManager(framework::Session *session) {
    session_ = session;
    // ldw init
    lane_depart_warning_.Init(&lkas_input_);
    // ldp init
    lane_depart_prevention_.Init(&lkas_input_);
    // elk init
    emergency_lane_keep_.Init(&lkas_input_, session_);
  }
  void Init(framework::Session *session);
  void RunOnce();
  uint32 get_ldw_state_info() { return ldw_state_; }
  boolean get_ldw_left_warning_info() { return ldw_left_warning_; }
  boolean get_ldw_right_warning_info() { return ldw_right_warning_; }
  uint32 get_ldp_state_info() { return ldp_state_; }
  boolean get_ldp_left_intervention_flag_info() {
    return ldp_left_intervention_flag_;
  }
  boolean get_ldp_right_intervention_flag_info() {
    return ldp_right_intervention_flag_;
  }
  uint32 get_elk_state_info() { return elk_state_; }
  boolean get_elk_left_intervention_flag_info() {
    return elk_left_intervention_flag_;
  }
  boolean get_elk_right_intervention_flag_info() {
    return elk_right_intervention_flag_;
  }
  ~LaneKeepAssistManager() = default;

 private:
  void Update();
  void CalculateWheelToLine();
  void Output();
  void set_lka_output_info();

 private:
  // planning::framework::Session *session_ = nullptr;
  framework::Session *session_ = nullptr;
  planning::LkasInput lkas_input_;
  planning::LaneDepartWarning lane_depart_warning_;        // ldw;
  planning::LaneDepartPrevention lane_depart_prevention_;  // ldp;
  planning::EmergencyLaneKeep emergency_lane_keep_;        // elk;
  uint32 ldw_state_{0}; /* LDW功能状态 0:Unavailable 1:Off 2:Standby 3:Active(No
                           Intervention) 4:Active(Left Intervention)
                           5:Active(Right Intervention) */
  boolean ldw_left_warning_{
      FALSE}; /* LDW功能触发左侧报警标志位 0:No Warning 1:Left Warning */
  boolean ldw_right_warning_{
      FALSE}; /* LDW功能触发右侧报警标志位 0:No Warning 1:Rirht Warning */
  uint32 ldp_state_{0}; /* LDP功能状态 0:Unavailable 1:Off 2:Standby 3:Active(No
                           Intervention) 4:Active(Left Intervention)
                           5:Active(Right Intervention) */
  boolean
      ldp_left_intervention_flag_{FALSE}; /* LDP功能触发左侧报警标志位 0:No
                                             Intervention 1:Left Intervention */
  boolean ldp_right_intervention_flag_{
      FALSE}; /* LDP功能触发右侧报警标志位 0:No Intervention 1:Rirht
                 Intervention */
  uint32 elk_state_{0}; /* ELK功能状态 0:Unavailable 1:Off 2:Standby 3:Active(No
                           Intervention) 4:Active(Left Intervention)
                           5:Active(Right Intervention) */
  boolean
      elk_left_intervention_flag_{FALSE}; /* ELK功能触发左侧报警标志位 0:No
                                             Intervention 1:Left Intervention */
  boolean elk_right_intervention_flag_{
      FALSE}; /* ELK功能触发右侧报警标志位 0:No Intervention 1:Rirht
                 Intervention */
};
}  // namespace planning
#endif