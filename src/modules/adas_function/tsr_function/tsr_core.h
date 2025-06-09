#ifndef TSR_CORE_H_
#define TSR_CORE_H_

#include "adas_function_context.h"
#include "debug_info_log.h"

using namespace planning;
namespace adas_function {
namespace tsr_core {

struct TsrParameters {
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
  double supp_turn_light_recovery_time = 2.0;  // 转向灯抑制恢复时长，单位：s
  double warning_time_max = 2.0;  // 单次最大报警时长，单位：s
};

class TsrCore {
 public:
  void RunOnce(void);

 private:
  TsrParameters tsr_param_;

  bool tsr_main_switch_ = false;  // TSR功能开关状态 0:Off  1:On
  bool UpdateTsrMainSwitch(void);

  uint16 tsr_enable_code_ = 255;
  uint16 UpdateTsrEnableCode(void);

  uint16 tsr_disable_code_ = 255;
  uint16 UpdateTsrDisableCode(void);

  uint16 tsr_fault_code_ = 255;
  uint16 UpdateTsrFaultCode(void);

  iflyauto::TSRFunctionFSMWorkState tsr_state_ = iflyauto::
      TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_UNAVAILABLE;

  bool tsr_state_machine_init_flag_ = false;
  iflyauto::TSRFunctionFSMWorkState TsrStateMachine(void);
  bool tsr_speed_limit_valid_ =false;
  uint32 tsr_speed_limit_ = 0;  // 限速值 单位:kph
  bool tsr_speed_limit_change_flag_ = false;
  bool speed_limit_exist_in_view_flag_ = false;
  uint32 speed_limit_exist_in_view_ = 0;
  double accumulated_path_length_ = 0.0;
  void UpdateTsrSpeedLimit(void);

  bool tsr_warning_image_ = false;  // 视觉提醒
  bool tsr_warning_voice_ = false;  // 声音提醒
  bool overspeed_status_ = false;  // false:未处于超速状态 true:处于超速状态
  double overspeed_duration_time_ = 0.0;  // 处于超速状态持续的时间 单位:s
  void UpdateTsrWarning(void);
  // 输出ldw计算结果
  void SetTsrOutputInfo();
  void CalculatePathLengthAccumulated();
};

}  // namespace tsr_core
}  // namespace adas_function

#endif