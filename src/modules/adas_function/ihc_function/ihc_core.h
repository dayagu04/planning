#ifndef _IHC_STEP_H_
#define _IHC_STEP_H_

#include "Platform_Types.h"
#include "adas_function_context.h"
#include "adas_function_lib.h"
#include "camera_perception_scene_c.h"
#include "debug_info_log.h"
#include "environmental_model.h"
#include "obstacle_manager.h"
#include "planning_hmi_c.h"
#include "vehicle_service_c.h"
#include "virtual_lane_manager.h"

using namespace planning;
namespace adas_function {
namespace ihc_core {
#define IHC_StateMachine_IN_UNAVAILABLE 0  // IHC一级主状态
#define IHC_StateMachine_IN_OFF 1          // IHC一级主状态
#define IHC_StateMachine_IN_STANDBY 2      // IHC一级主状态
#define IHC_StateMachine_IN_ACTIVE 3       // IHC一级主状态
#define IHC_StateMachine_IN_FAULT 4        // IHC一级主状态

// IHC算法输入信号结构体定义
struct IHCSysInput {
  bool ihc_main_switch;               // IHC开关 0:Off 1:On
  float32 vehicle_speed_display_kph;  // 本车车速 单位:kph
  bool auto_light_state;              // 自动灯光控制状态 0:Off 1:On
  iflyauto::ShiftLeverStateEnum shift_lever_state;  // 换档杆状态
  bool low_beam_state;                              // 近光灯状态
  bool high_beam_state;                             // 远光灯状态
  bool front_fog_light_state;                       // 前雾灯状态
  bool rear_fog_light_state;                        // 后雾灯状态
  uint32 wiper_state;                               // 雨刮状态
                                                    // 环境光线条件
  iflyauto::CameraPerceptionLightingCondition
      lighting_condition;  // 环境亮度条件：0-UNKNOWN, 1-BRIGHT, 2-MEDIUM,
                           // 3-DARK
};

// IHC算法状态结构体定义
struct IHCSysState {
  uint16 ihc_high_beam_code = 0;
  uint16 ihc_low_beam_code = 0;
  uint16 ihc_fault_code = 0;
  uint16 ihc_active_code = 0;  // IHC激活码: 0: 使能, 其他: 禁用
  iflyauto::IHCFunctionFSMWorkState
      ihc_state;  // IHC功能状态 0:Unavailable 1:Off 2:Standby 3:Active 4:Fault
  bool ihc_request_status;  // IHC请求状态 0:No Request 1:Request
  bool ihc_request;         // IHC请求 0:LowBeam 1:HighBeam

  // Debug变量：记录切换近光灯的原因
  bool low_beam_due_to_same_dir_vehicle;  // 是否由于同向车辆导致近光灯
  bool low_beam_due_to_oncomming_vehicle;  // 是否由于对向机动车导致近光灯
  bool low_beam_due_to_oncomming_cycle;  // 是否由于对向非机动车导致近光灯
};

// IHC算法结构体定义
struct IHCSys {
  IHCSysInput input;
  IHCSysState state;
};

class IhcCore {
 public:
  void RunOnce(void);

 private:
  // IHC获取所有输入信息
  void GetInputInfo(void);

  // IHC active码: 0: 使能, 其他: 禁用 (全部满足)
  uint16 IhcActiveCode(void);

  // IHC 远光灯使能码，根据文档需求更改
  uint16 UpdateIhcHighBeamCode(void);
  // IHC 近光灯使能码，根据文档需求更改
  uint16 UpdateIhcLowBeamCode(void);

  // IHC 故障码，根据文档需求更改
  uint16 UpdateIhcFaultCode(void);

  // 初始化状态机，决定是off还是standby
  iflyauto::IHCFunctionFSMWorkState IHCStateMachine(void);

  // 感知环境亮度硬滤波
  bool IHCRequestLightingFilter(bool ihc_request_lighting, uint8_t window_size,
                                float ratio_threshold, uint8_t max_trasition);
  // 动态障碍物检查，判断范围内是否有稳定的障碍物
  bool DynamicObstacleCheck(void);
  // 判断雨刷不为快速档是否超过60s
  bool IsWiperNotHighSpeedLast(void);
  // IHC 灯光处理部分，如感知到车辆在前方等，需要近光灯
  bool IHCRequest(void);

  // 输出信息，IHC状态与灯光请求内容
  void SetIhcOutputInfo(void);

  //测试用
  void JsonSwitchIhcMainSwitch(void);

 private:
  IHCSys ihc_sys_;
  bool last_high_beam_request_ = false;
  // 附近车辆状态，判断范围内是否有车
  bool dynamic_obstacle_check_;
  // 雨刷不为快速档是否超过60s
  bool wiper_not_high_speed_last_;
  // 当前雨刷是否为快速档
  bool wiper_is_high_speed_;
  std::vector<bool> ihc_request_lighting_buffer_;
  // 雨刮持续时间满足抑制阈值持续时间 单位:s
  double wiper_state_supp_duration_ = 70.0;

  // 速度持续检测相关变量
  float speed_above_40kph_duration_ = 0.0f;  // 车速>=40kph的持续时间
  float speed_above_30kph_duration_ = 0.0f;  // 车速>=30kph的持续时间
  
  // 已验证的可信障碍物ID集合（曾经同时被相机和雷达检测到的障碍物）
  std::set<uint16> verified_obstacle_ids_;
};
}  // namespace ihc_core
}  // namespace adas_function
#endif
