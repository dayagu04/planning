#ifndef elk_CORE_H_
#define elk_CORE_H_

#include "adas_function_context.h"
#include "debug_info_log.h"

using namespace planning;
namespace adas_function {
namespace elk_core {

struct ElkParameters {
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

class ElkCore {
 public:
  void RunOnce(void);

 private:
  ElkParameters elk_param_;


  bool elk_main_switch_ = false;  // elk功能开关状态 0:Off  1:On
  bool UpdateElkMainSwitch(void);

  uint16 elk_enable_code_ = 255;
  uint16 UpdateElkEnableCode(void);

  uint16 elk_disable_code_ = 255;
  uint16 UpdateElkDisableCode(void);

  uint16 elk_fault_code_ = 255;
  uint16 UpdateElkFaultCode(void);

  // 是否抑制重复报警的标志位 false:不抑制 true:抑制
  bool left_suppress_repeat_warning_flag_ = false;
  uint16 elk_left_suppression_code_ = 255;
  uint16 UpdateElkLeftSuppressionCode(void);

  double elk_left_warning_time_ = 0.0;  // 处于左侧报警状态的时长，单位:s
  uint16 elk_left_kickdown_code_ = 255;
  uint16 UpdateElkLeftKickDownCode(void);

  // 是否抑制重复报警的标志位 false:不抑制 true:抑制
  bool right_suppress_repeat_warning_flag_ = false;
  uint16 elk_right_suppression_code_ = 255;
  uint16 UpdateElkRightSuppressionCode(void);

  double elk_right_warning_time_ = 0.0;  // 处于右侧报警状态的时长，单位:s
  uint16 elk_right_kickdown_code_ = 255;
  uint16 UpdateElkRightKickDownCode(void);

  double elk_tlc_threshold_ = 0.6;
  double elk_roadedge_tlc_threshold_ = 0.6;
  // 输出elk计算结果
  void SetElkOutputInfo();

  uint16 RiskAlertJudge(const context::FusionObjExtractInfo &obj);
//   std::vector<double>ObjCornersCalculate(const context::FusionObjExtractInfo& obj);

  bool elk_left_intervention_ = false;

  bool elk_right_intervention_ = false;

  iflyauto::ELKFunctionFSMWorkState elk_state_ = iflyauto::
      ELKFunctionFSMWorkState::ELK_FUNCTION_FSM_WORK_STATE_UNAVAILABLE;

  bool elk_state_machine_init_flag_ = false;
  iflyauto::ELKFunctionFSMWorkState ElkStateMachine(void);
};

}  // namespace elk_core
}  // namespace adas_function

#endif