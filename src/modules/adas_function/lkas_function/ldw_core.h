#ifndef LDW_CORE_H_
#define LDW_CORE_H_

#include "adas_function_context.h"
#include "debug_info_log.h"

using namespace planning;
namespace adas_function {
namespace ldw_core {

struct LdwParameters {
  double enable_vehspd_display_min =
      60.0 / 3.6;  // 激活的最小仪表车速，单位：m/s
  double enable_vehspd_display_max =
      150.0 / 3.6;  // 激活的最大仪表车速，单位：m/s
  double disable_vehspd_display_min =
      55.0 / 3.6;  // 退出的最小仪表车速，单位：m/s
  double disable_vehspd_display_max =
      155.0 / 3.6;  // 退出的最大仪表车速，单位：m/s

  double earliest_warning_line = 1.5;          // 触发的最早报警线，单位：m
  double latest_warning_line = -0.3;           // 触发的最晚报警线，单位：m
  double reset_warning_line = 0.15;            // 触发的报警重置线，单位：m
  double supp_turn_light_recovery_time = 2.0;  // 转向灯抑制恢复时长，单位：s
  double warning_time_min = 1.0;               // 单次最大报警时长，单位：s
  double warning_time_max = 2.0;               // 单次最大报警时长，单位：s
};

class LdwCore {
 public:
  void RunOnce(void);

 private:
  LdwParameters ldw_param_;

  bool ldw_main_switch_ = false;  // LDW功能开关状态 0:Off  1:On
  bool UpdateLdwMainSwitch(void);

  // 车身YawRate满足取消抑制阈值持续时间 单位:s
  double yaw_rate_supp_recover_duration_ = 0.0;
  // 油门踏板变化率满足取消抑制阈值持续时间 单位:s
  double acc_pedal_pos_rate_supp_recover_duration_ = 0.0;
  // 制动踏板表现满足取消抑制阈值持续时间 单位:s
  double brake_pedal_pressed_supp_recover_duration_ = 0.0;
  // 当前道线的弯道半径满足取消抑制阈值持续时间 单位:s
  double curve_C2_supp_recover_duration_ = 0.0;
  // 当前方向盘转速条件满足取消抑制阈值持续时间 单位:s
  double str_wheel_ang_speed_recover_duration_ = 0.0;
  // 定义当前挡位值
  iflyauto::ShiftLeverStateEnum shift_lever_state_ =
      iflyauto::ShiftLeverStateEnum::ShiftLeverState_P;
  uint32 ldw_enable_code_ = 255;
  uint32 UpdateLdwEnableCode(void);

  // 油门踏板变化率满足抑制阈值持续时间 单位:s
  // double acc_pedal_pos_rate_supp_duration_ = 0.0;

  // 车身YawRate满足抑制阈值持续时间 单位:s
  double yaw_rate_supp_duration_ = 0.0;
  // 刹车踏板主缸压力满足抑制阈值持续时间 单位:s
  double brake_pedal_pressed_supp_duration_ = 0.0;
  // 雨刮持续时间满足抑制阈值持续时间 单位:s
  double wiper_state_supp_duration_ = 0.0;
  uint32 ldw_disable_code_ = 255;
  uint32 UpdateLdwDisableCode(void);

  uint32 ldw_fault_code_ = 255;
  uint32 UpdateLdwFaultCode(void);

  // 是否抑制重复报警的标志位 false:不抑制 true:抑制
  bool left_suppress_repeat_warning_flag_ = false;
  // LDW报警冷却时间阈值 单位:s
  double LDW_CoolingTime_duration_ = 0.0;
  // LDW变道报警抑制条件阈值 单位:s 默认10，避免开始就抑制两秒
  double LDW_LaneChange_duration_ = 10.0;
  uint32 ldw_left_suppression_code_ = 255;
  uint32 UpdateLdwLeftSuppressionCode(void);

  double ldw_left_warning_time_ = 0.0;  // 处于左侧报警状态的时长，单位:s
  uint32 ldw_left_kickdown_code_ = 255;
  uint32 UpdateLdwLeftKickDownCode(void);

  // 是否抑制重复报警的标志位 false:不抑制 true:抑制
  bool right_suppress_repeat_warning_flag_ = false;
  uint32 ldw_right_suppression_code_ = 255;
  uint32 UpdateLdwRightSuppressionCode(void);

  double ldw_right_warning_time_ = 0.0;  // 处于右侧报警状态的时长，单位:s
  uint32 ldw_right_kickdown_code_ = 255;
  uint32 UpdateLdwRightKickDownCode(void);

  double ldw_tlc_far_ = 1.0;     // 针对道线触发报警的高灵敏度阈值，单位：s
  double ldw_tlc_medium_ = 0.6;  // 针对道线触发报警的中灵敏度阈值，单位：s
  double ldw_tlc_near_ = 0.2;    // 针对道线触发报警的低灵敏度阈值，单位：s
  double ldw_tlc_threshold_ = 0.6;
  double UpdateTlcThreshold(void);

  bool ldw_left_intervention_ = false;

  bool ldw_right_intervention_ = false;

  iflyauto::LDWFunctionFSMWorkState ldw_state_ = iflyauto::
      LDWFunctionFSMWorkState::LDW_FUNCTION_FSM_WORK_STATE_UNAVAILABLE;

  bool ldw_state_machine_init_flag_ = false;
  iflyauto::LDWFunctionFSMWorkState LdwStateMachine(void);
  // 输出ldw计算结果
  void SetLdwOutputInfo();
};

}  // namespace ldw_core
}  // namespace adas_function

#endif