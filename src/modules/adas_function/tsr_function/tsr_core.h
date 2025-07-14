#ifndef TSR_CORE_H_
#define TSR_CORE_H_

#include "adas_function_context.h"
#include "adas_function_struct.h"
#include "camera_perception_tsr_c.h"
#include "debug_info_log.h"
#include "func_state_machine_c.h"

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

  iflyauto::NotificationMainSwitch tsr_main_switch_ =
      iflyauto::NotificationMainSwitch::NOTIFICATION_MAIN_SWITCH_NONE;
  iflyauto::NotificationMainSwitch UpdateTsrMainSwitch(void);

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

  // 限速标识牌信息 (视觉信息)
  uint32 tsr_speed_limit_ = 0;  // 限速值 单位:kph
  bool tsr_speed_limit_change_flag_ = false;
  bool speed_limit_exist_in_view_flag_ = false;
  uint32 speed_limit_exist_in_view_ = 0;
  double accumulated_path_length_ = 0.0;
  // 解除限速标识牌信息 (视觉信息)
  uint32 end_of_speed_sign_value_ = 0; // 解除限速牌值
  double end_of_speed_sign_display_time_ = 0.0; // 解除限速牌显示时间 单位:s
  bool end_of_speed_sign_display_flag_ = false; // 解除限速牌显示标志位
  // 当前道路限速信息 (道路信息)
  bool current_map_speed_limit_valid_ = false;

  // 新道路标志位, 需要清掉视觉限速信息
  bool new_road_flag_ = false;
  // 视觉限速抑制标志位
  bool speed_limit_suppression_flag_ = false;
  // 限速标识牌列表
  std::vector<adas_function::context::SpeedSignInfo> speed_limit_sign_info_vector_;
  // 解除限速列表
  std::vector<adas_function::context::SpeedSignInfo> end_of_speed_sign_info_vector_;

  double current_map_speed_limit_ = 0.0;
  void UpdateMapSpeedLimit(void);
  // 更新限速标识牌信息
  void UpdateTsrSpeedLimit(void);

  // 新版更新限速信息
  void UpdateTsrSpeedLimitNew(void);

  // 实时辅助标识牌, 不一定输出
  iflyauto::SuppSignType realtime_supp_sign_info_ = iflyauto::SuppSignType::SUPP_SIGN_TYPE_UNKNOWN;
  // 输出辅助标识牌
  iflyauto::SuppSignType output_supp_sign_info_ = iflyauto::SuppSignType::SUPP_SIGN_TYPE_UNKNOWN;
  // 辅助标识牌有效标志位
  bool supp_sign_valid_flag_ = false;
  // 辅助标识变化标志位
  bool supp_sign_change_flag_ = false;
  // 辅助标识牌抑制显示标志位
  bool supp_sign_in_suppression_flag_ = false;
  // 辅助标识牌保持时间
  double supp_sign_hold_time_ = 0.0;

  // 更新辅助标识牌信息
  void UpdateTsrSuppInfo(void);
  // 辅助标识牌置位, 用于输出优先级最高的标识牌
  uint16_t supp_sign_code_ = 0;
  // 获取限速标识牌中的最高限速值
  uint32 GetHighestSpeedLimit(void);
  
  bool tsr_warning_image_ = false;  // 视觉提醒
  bool tsr_warning_voice_ = false;  // 声音提醒
  bool overspeed_status_ = false;  // false:未处于超速状态 true:处于超速状态
  double overspeed_duration_time_ = 0.0;  // 处于超速状态持续的时间 单位:s
  void UpdateTsrWarning(void);
  // 输出ldw计算结果
  void SetTsrOutputInfo();
  void CalculatePathLengthAccumulated();
  void CalculateDurationTime(void);
  // TSR实时信息重置
  void ResetRealTimeTsrInfo(void);
};

}  // namespace tsr_core
}  // namespace adas_function

#endif