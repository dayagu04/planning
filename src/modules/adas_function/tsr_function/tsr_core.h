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

  uint16 tsr_disable_code_ = 255;
  uint16 UpdateTsrDisableCode(void);

  uint16 tsr_fault_code_ = 255;
  uint16 UpdateTsrFaultCode(void);

  iflyauto::TSRFunctionFSMWorkState tsr_state_ = iflyauto::
      TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_UNAVAILABLE;
  iflyauto::TSRFunctionFSMWorkState tsr_state_prev_ = iflyauto::
      TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_UNAVAILABLE;  // 上一时刻的状态

  bool tsr_state_machine_init_flag_ = false;
  iflyauto::TSRFunctionFSMWorkState TsrStateMachine(void);

  // 限速标识牌信息 (视觉信息)
  uint32 tsr_speed_limit_ = 0;  // 限速值 单位:kph
  double accumulated_path_length_ = 0.0;
  bool has_perception_speed_limit_ = false;
  bool speed_limit_renew_flag_ = false;  // 限速牌刷新标志位
  std::unordered_set<uint8_t> speed_limit_set_; // 限速标识牌值集合
  // 未感知到任何限速标识持续时间
  double no_speed_limit_duration_time_ = 0.0;
  bool speed_limit_out_flag_ = false;
  bool speed_limit_ever_appeared_ = false;  // 限速牌是否曾经出现过

  // 解除限速标识牌信息 (视觉信息)
  uint32 end_of_speed_sign_value_ = 0;           // 解除限速牌值
  bool has_perception_end_of_speed_limit_ = false;
  double end_of_speed_sign_display_time_ = 0.0;  // 解除限速牌显示持续时间 单位:s
  std::unordered_set<uint8_t> end_of_speed_limit_set_; // 解除限速标识牌值集合
  // 未感知到任何解除限速标识牌持续时间
  double no_end_of_speed_limit_duration_time_ = 0.0;
  bool end_of_speed_limit_out_flag_ = false;
  bool end_of_speed_limit_ever_appeared_ = false;  // 解除限速牌是否曾经出现过

  // 当前道路限速信息 (道路信息)
  bool current_map_speed_limit_valid_ = false;

  // 新道路标志位, 需要清掉视觉限速信息
  bool new_road_flag_ = false;
  // 视觉限速抑制标志位
  bool speed_limit_suppression_flag_ = false;
  // 限速标识牌列表
  std::vector<adas_function::context::SpeedSignInfo>
      speed_limit_sign_info_vector_;
  // 解除限速列表
  std::vector<adas_function::context::SpeedSignInfo>
      end_of_speed_sign_info_vector_;

  double current_map_speed_limit_ = 0.0;
  uint16 current_map_type_ = 0;  // 地图类型: 0-无地图, 1-sd_map, 2-sd_pro_map
  iflyauto::DrivingRoadType current_road_type_ = iflyauto::DrivingRoadType::DRIVING_ROAD_TYPE_NONE;  // 当前道路类型

  // 更新限速标识牌信息
  void UpdateTsrSpeedLimit(void);
  // 只使用地图限速的更新函数
  void UpdateTsrSpeedLimitOnlyByMap(void);

  // 实时辅助标识牌, 不一定输出
  iflyauto::SuppSignType realtime_supp_sign_info_ =
      iflyauto::SuppSignType::SUPP_SIGN_TYPE_UNKNOWN;
  // 输出辅助标识牌
  iflyauto::SuppSignType output_supp_sign_info_ =
      iflyauto::SuppSignType::SUPP_SIGN_TYPE_UNKNOWN;
  // 辅助标识牌有效标志位
  bool supp_sign_valid_flag_ = false;
  // 辅助标识变化标志位
  bool supp_sign_change_flag_ = false;
  // 辅助标识牌抑制显示标志位
  bool supp_sign_in_suppression_flag_ = false;
  // 辅助标识牌保持时间
  double supp_sign_hold_time_ = 0.0;
  //sdmappro_导航模式 =1导航中 =0 非导航中
  bool tsr_navi_flag_ = false; 

  // 更新辅助标识牌信息
  void UpdateTsrSuppInfo(void);
  // 辅助标识牌置位, 用于输出优先级最高的标识牌
  uint16_t supp_sign_code_ = 0;
  // 获取集合中的最高限速值
  uint32 GetHighestFromSet(const std::unordered_set<uint8_t>& input_set);

  bool tsr_warning_flag_ = false;  // 超速报警标志位
  bool overspeed_status_ = false;  // false:未处于超速状态 true:处于超速状态
  double overspeed_duration_time_ = 0.0;  // 处于超速状态持续的时间 单位:s
  double tsr_reset_path_length_ = 1000.0;  // 限速重置行驶距离 单位:m
  void UpdateTsrWarning(void);
  // 输出限速结果
  void SetTsrOutputInfo();
  // 行驶距离累计计算
  void CalculatePathLengthAccumulated();
  // 持续时间计算模块
  void CalculateDurationTime(void);
  // TSR实时信息重置
  void ResetRealTimeTsrInfo(void);

  // 测试函数
  // 模式0: 关闭测试
  // 模式1: 输出限速80，不报警
  // 模式2: 输出限速80，报警
  // 模式3: 输出解除限速80
  // 模式4: 辅助标识依次以2s为周期显示所有种类
  void TsrTestFunction(void);

};

}  // namespace tsr_core
}  // namespace adas_function

#endif