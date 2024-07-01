#ifndef _LANE_KEEP_ASSIST_MANAGER_H_
#define _LANE_KEEP_ASSIST_MANAGER_H_
#include "Platform_Types.h"
#include "debug_info_log.h"
#include "emergency_lane_keep_context.h"
#include "environmental_model.h"
#include "lane_depart_prevention_context.h"
#include "lane_depart_warning_context.h"
#include "session.h"
#include "virtual_lane_manager.h"

#define ADAS_JSON_READ_VALUE(var_name, type, json_name) \
  var_name = adas_config.get<type>(json_name, false, var_name)

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
  iflyauto::LDWFunctionFSMWorkState get_ldw_state_info() { return ldw_state_; }
  bool get_ldw_left_warning_info() { return ldw_left_warning_; }
  bool get_ldw_right_warning_info() { return ldw_right_warning_; }
  iflyauto::LDPFunctionFSMWorkState get_ldp_state_info() { return ldp_state_; }
  bool get_ldp_left_intervention_flag_info() {
    return ldp_left_intervention_flag_;
  }
  bool get_ldp_right_intervention_flag_info() {
    return ldp_right_intervention_flag_;
  }
  iflyauto::ELKFunctionFSMWorkState get_elk_state_info() { return elk_state_; }
  bool get_elk_left_intervention_flag_info() {
    return elk_left_intervention_flag_;
  }
  bool get_elk_right_intervention_flag_info() {
    return elk_right_intervention_flag_;
  }
  ~LaneKeepAssistManager() = default;

 private:
  void Update();
  void CalculateWheelToLine();
  void Output();
  void set_lka_output_info();
  void SyncParameters();

 private:
  bool init_flag_ = false;
  // planning::framework::Session *session_ = nullptr;
  framework::Session *session_ = nullptr;
  planning::LkasInput lkas_input_;
  planning::LaneDepartWarning lane_depart_warning_;        // ldw;
  planning::LaneDepartPrevention lane_depart_prevention_;  // ldp;
  planning::EmergencyLaneKeep emergency_lane_keep_;        // elk;
  iflyauto::LDWFunctionFSMWorkState ldw_state_{
      iflyauto::LDW_FUNCTION_FSM_WORK_STATE_OFF}; /* LDW功能状态
0:Unavailable 1:Off 2:Standby 3:Active(No Intervention) 4:Active(Left
Intervention) 5:Active(Right Intervention) */
  bool ldw_left_warning_{
      false}; /* LDW功能触发左侧报警标志位 0:No Warning 1:Left Warning */
  bool ldw_right_warning_{
      false}; /* LDW功能触发右侧报警标志位 0:No Warning 1:Rirht Warning */
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
  iflyauto::ELKFunctionFSMWorkState elk_state_{
      iflyauto::ELK_FUNCTION_FSM_WORK_STATE_OFF}; /* ELK功能状态
0:Unavailable 1:Off 2:Standby 3:Active(No Intervention) 4:Active(Left
Intervention) 5:Active(Right Intervention) */
  bool elk_left_intervention_flag_{
      false}; /* ELK功能触发左侧报警标志位 0:No
                 Intervention 1:Left Intervention */
  bool elk_right_intervention_flag_{
      false}; /* ELK功能触发右侧报警标志位 0:No Intervention 1:Rirht
                 Intervention */
};
}  // namespace planning
#endif