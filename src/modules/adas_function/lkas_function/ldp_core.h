#ifndef ldp_CORE_H_
#define ldp_CORE_H_

#include "adas_function_context.h"
#include "debug_info_log.h"

using namespace planning;
namespace adas_function {
namespace ldp_core {

struct LdpParameters {
  double enable_vehspd_display_min =
      60.0 / 3.6;  // 激活的最小仪表车速，单位：m/s
  double enable_vehspd_display_max =
      120.0 / 3.6;  // 激活的最大仪表车速，单位：m/s
  double disable_vehspd_display_min =
      55.0 / 3.6;  // 退出的最小仪表车速，单位：m/s
  double disable_vehspd_display_max =
      125.0 / 3.6;  // 退出的最大仪表车速，单位：m/s

  double earliest_warning_line = 1.5;  // 触发的最早报警线，单位：m
  double latest_warning_line = -0.3;   // 触发的最晚报警线，单位：m
  double reset_warning_line = 0.15;    // 触发的报警重置线，单位：m
  double roadedge_earliest_warning_line = 1.8;  // 触发的最早报警线，单位：m
  double roadedge_latest_warning_line = -0.3;  // 触发的最晚报警线，单位：m
  double roadedge_reset_warning_line = 0.15;  // 触发的报警重置线，单位：m
  double supp_turn_light_recovery_time = 2.0;  // 转向灯抑制恢复时长，单位：s
  double warning_time_max = 8.0;  // 单次最大报警时长，单位：s
  double suppression_driver_hand_trq =
      2.0;  // 抑制报警的驾驶员手力矩(绝对值)阈值，单位：Nm
  double kickdown_driver_hand_trq =
      2.5;  // 抑制报警的驾驶员手力矩(绝对值)阈值，单位：Nm
};

class LdpCore {
 public:
  void RunOnce(void);

 private:
  LdpParameters ldp_param_;

  bool ldp_main_switch_ = false;  // ldp功能开关状态 0:Off  1:On
  bool UpdateLdpMainSwitch(void);

  uint16 ldp_enable_code_ = 255;
  uint16 UpdateLdpEnableCode(void);

  uint16 ldp_disable_code_ = 255;
  uint16 UpdateLdpDisableCode(void);

  uint16 ldp_fault_code_ = 255;
  uint16 UpdateLdpFaultCode(void);

  // 是否抑制重复报警的标志位 false:不抑制 true:抑制
  bool left_suppress_repeat_warning_flag_ = false;
  uint16 ldp_left_suppression_code_ = 255;
  uint16 UpdateLdpLeftSuppressionCode(void);

  double ldp_left_warning_time_ = 0.0;  // 处于左侧报警状态的时长，单位:s
  uint16 ldp_left_kickdown_code_ = 255;
  uint16 UpdateLdpLeftKickDownCode(void);

  // 是否抑制重复报警的标志位 false:不抑制 true:抑制
  bool right_suppress_repeat_warning_flag_ = false;
  uint16 ldp_right_suppression_code_ = 255;
  uint16 UpdateLdpRightSuppressionCode(void);

  double ldp_right_warning_time_ = 0.0;  // 处于右侧报警状态的时长，单位:s
  uint16 ldp_right_kickdown_code_ = 255;
  uint16 UpdateLdpRightKickDownCode(void);

  double ldp_tlc_threshold_ = 0.6;
  double ldp_roadedge_tlc_threshold_ = 0.6;
  // 输出ldp计算结果
  void SetLdpOutputInfo();

  bool ldp_left_intervention_ = false;

  bool ldp_right_intervention_ = false;

  iflyauto::LDPFunctionFSMWorkState ldp_state_ = iflyauto::
      LDPFunctionFSMWorkState::LDP_FUNCTION_FSM_WORK_STATE_UNAVAILABLE;

  bool ldp_state_machine_init_flag_ = false;
  iflyauto::LDPFunctionFSMWorkState LdpStateMachine(void);

  double UpdateTlcThreshold(void);
};

}  // namespace ldp_core
}  // namespace adas_function

#endif