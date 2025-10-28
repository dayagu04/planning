#include "ihc_core.h"

using namespace planning;
namespace adas_function {
namespace ihc_core {

// IHC计时器（使用dt累积）
static float ihc_low_beam_on_duration_s =
    0.0f;  // 无车空窗累计时间（用于近->远）
static float ihc_high_beam_on_duration_s = 0.0f;  // 远光持续时间（用于最小2s）

static inline void ResetIhcDynamicObstacleTimers() {
  ihc_low_beam_on_duration_s = 0.0f;
  ihc_high_beam_on_duration_s = 0.0f;
}

bool IhcCore::IsWiperNotHighSpeedLast(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  // 更新当前雨刷是否为快速档的状态
  wiper_is_high_speed_ = (ihc_sys_.input.wiper_state ==
                          iflyauto::WiperStateEnum::WiperState_HighSpeed);

  // 雨刷状态时间累积逻辑
  if (!wiper_is_high_speed_) {
    // 非快速档，累积时间
    wiper_state_supp_duration_ += GetContext.get_param()->dt;
    if (wiper_state_supp_duration_ > 70.0) {
      wiper_state_supp_duration_ = 70.0;  // 防止溢出
    }
  } else {
    // 快速档，清零时间
    wiper_state_supp_duration_ = 0.0;
  }

  // 返回雨刷不为快速档是否超过60s
  return (wiper_state_supp_duration_ >= 60.0);
}

void IhcCore::RunOnce(void) {
  // 更新输入信息
  GetInputInfo();
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  // 读取配置文件是否强制打开IHC软开关
  if (GetContext.get_param()->ihc_set_main_switch) {
    ihc_sys_.input.ihc_main_switch = true;
  }

  // 获取范围内是否有车
  dynamic_obstacle_check_ = DynamicObstacleCheck();

  // 判断雨刷不为快速档是否超过60s
  wiper_not_high_speed_last_ = IsWiperNotHighSpeedLast();

  // 根据输入信息，更新远光灯使能码、近光灯使能码、故障码、激活码
  ihc_sys_.state.ihc_high_beam_code = UpdateIhcHighBeamCode();
  ihc_sys_.state.ihc_low_beam_code = UpdateIhcLowBeamCode();  // 现阶段未使用
  ihc_sys_.state.ihc_fault_code = UpdateIhcFaultCode();
  ihc_sys_.state.ihc_active_code = IhcActiveCode();

  if (GetContext.get_param()->ihc_use_json_code) {
    // 如果使用json，则使用配置文件中的使能码、禁用码、故障码
    ihc_sys_.state.ihc_high_beam_code =
        GetContext.get_param()->ihc_high_beam_code;
    ihc_sys_.state.ihc_low_beam_code =
        GetContext.get_param()->ihc_low_beam_code;
    ihc_sys_.state.ihc_fault_code = GetContext.get_param()->ihc_fault_code;
    ihc_sys_.state.ihc_active_code = GetContext.get_param()->ihc_active_code;
  }

  ihc_sys_.state.ihc_state = IHCStateMachine();

  // 记录上次远光灯请求状态
  last_high_beam_request_ =
      GetContext.get_output_info()->ihc_output_info_.ihc_request_;

  // 应用时间保护策略
  const float dt = GetContext.get_param()->dt;

  // 累计灯光持续时间
  if (last_high_beam_request_) {
    // 累计远光持续时间, 清零近光持续时间
    ihc_low_beam_on_duration_s = 0.0f;
    ihc_high_beam_on_duration_s += dt;
    if (ihc_high_beam_on_duration_s > 10) {
      ihc_high_beam_on_duration_s = 10.0f;
    }
  } else {
    // 累计近光持续时间, 清零远光持续时间
    ihc_high_beam_on_duration_s = 0.0f;
    ihc_low_beam_on_duration_s += dt;
    if (ihc_low_beam_on_duration_s > 10) {
      ihc_low_beam_on_duration_s = 10.0f;
    }
  }

  // 根据状态机和功能逻辑确定灯光状态（时间保护策略已在IHCRequest中实现）
  if (ihc_sys_.state.ihc_state ==
      iflyauto::IHC_FUNCTION_FSM_WORK_STATE_ACTIVE) {
    ihc_sys_.state.ihc_request_status = true;
    ihc_sys_.state.ihc_request =
        IHCRequest();  // IHCRequest中已包含时间保护策略
  } else {
    ihc_sys_.state.ihc_request_status = false;
    ihc_sys_.state.ihc_request = false;  // 非激活状态强制近光
  }

  // 灯光状态已在时间保护策略中设置
  SetIhcOutputInfo();

  // 强制打开远光灯, 测试用
  if (GetContext.get_param()->ihc_high_beam_switch) {
    JsonSwitchIhcMainSwitch();
  } else {
    // do nothing
  }

  JSON_DEBUG_VALUE("ihc_function::ihc_high_beam_code",
                   ihc_sys_.state.ihc_high_beam_code);
  JSON_DEBUG_VALUE("ihc_function::ihc_low_beam_code",
                   ihc_sys_.state.ihc_low_beam_code);
  JSON_DEBUG_VALUE("ihc_function::ihc_fault_code",
                   ihc_sys_.state.ihc_fault_code);
  JSON_DEBUG_VALUE("ihc_function::ihc_active_code",
                   ihc_sys_.state.ihc_active_code);
  JSON_DEBUG_VALUE("ihc_function::ihc_state", int(ihc_sys_.state.ihc_state));
  JSON_DEBUG_VALUE("ihc_function::ihc_request_status",
                   ihc_sys_.state.ihc_request_status);
  JSON_DEBUG_VALUE("ihc_function::ihc_request", ihc_sys_.state.ihc_request);
  JSON_DEBUG_VALUE("ihc_function::ihc_main_switch",
                   ihc_sys_.input.ihc_main_switch);
  JSON_DEBUG_VALUE("ihc_function::auto_light_state",
                   ihc_sys_.input.auto_light_state);
  JSON_DEBUG_VALUE("ihc_function::low_beam_due_to_same_dir_vehicle",
                   ihc_sys_.state.low_beam_due_to_same_dir_vehicle);
  JSON_DEBUG_VALUE("ihc_function::low_beam_due_to_oncomming_vehicle",
                   ihc_sys_.state.low_beam_due_to_oncomming_vehicle);
  JSON_DEBUG_VALUE("ihc_function::low_beam_due_to_oncomming_cycle",
                   ihc_sys_.state.low_beam_due_to_oncomming_cycle);

  // 输出环境光线条件（已在IHCRequest中设置）
  JSON_DEBUG_VALUE("ihc_function::lighting_condition",
                   int(ihc_sys_.input.lighting_condition));
}

void IhcCore::GetInputInfo() {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  // 获取功能状态机信息
  auto function_state_machine_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .function_state_machine_info;

  // 获取车辆服务输出信息
  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;

  // 获取环境光线条件记录信息
  const auto scene_info_ptr = &GetContext.mutable_session()
                                   ->mutable_environmental_model()
                                   ->get_local_view()
                                   .perception_scene_info;

  ihc_sys_.input.lighting_condition = scene_info_ptr->lighting_condition;

  // 获取IHC开关状态
  ihc_sys_.input.ihc_main_switch =
      function_state_machine_info_ptr->switch_sts.ihc_main_switch;

  // 获取当前仪表车速
  ihc_sys_.input.vehicle_speed_display_kph =
      vehicle_service_output_info_ptr->vehicle_speed_display *
      3.6F;  // 当前车速 单位:kph

  // 获取换档杆状态
  ihc_sys_.input.shift_lever_state = static_cast<iflyauto::ShiftLeverStateEnum>(
      vehicle_service_output_info_ptr->shift_lever_state);

  // 获取近光灯状态
  ihc_sys_.input.low_beam_state =
      vehicle_service_output_info_ptr->low_beam_state;

  // 获取远光灯状态
  ihc_sys_.input.high_beam_state =
      vehicle_service_output_info_ptr->high_beam_state;

  // 获取前雾灯状态
  ihc_sys_.input.front_fog_light_state =
      vehicle_service_output_info_ptr->front_fog_light_state;

  // 后雾灯状态
  ihc_sys_.input.rear_fog_light_state =
      vehicle_service_output_info_ptr->rear_fog_light_state;

  // 雨刮运行状态
  ihc_sys_.input.wiper_state = vehicle_service_output_info_ptr->wiper_state;

  // 获取自动灯光控制状态
  ihc_sys_.input.auto_light_state =
      vehicle_service_output_info_ptr->auto_light_state;
}

// IHC standby->active码: 0: 使能, 其他: 禁用 (全部满足)
uint16 IhcCore::IhcActiveCode() {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  uint16 ihc_active_code_temp = 0;
  // condition0: IGN ON
  // 待补充

  // condition1: 大灯开关挡位在AUTO档 (auto_light_state一直为false)
  if (ihc_sys_.input.auto_light_state != true) {
    ihc_active_code_temp += uint16_bit[0];
  } else {
    // do nothing
  }

  // condition2: 近光灯点亮
  // if (ihc_sys_.input.low_beam_state != true) {
  //   ihc_active_code_temp += uint16_bit[2];
  // } else {
  //   // do nothing
  // }

  return ihc_active_code_temp;
}

// 远光灯使能码: 0: 使能, 其他: 禁用 (全部满足)
uint16 IhcCore::UpdateIhcHighBeamCode() {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  uint16 ihc_enable_code_temp = 0;

  // condition0: 范围内有车, 置位
  if (dynamic_obstacle_check_ == true) {
    ihc_enable_code_temp += uint16_bit[0];
  } else {
    // do nothing
  }

  // condition1: 车速是否小于40kph, 或者车速大于等于40kph但未持续0.5s, 置位
  const float dt = GetContext.get_param()->dt;
  const float SPEED_40KPH_THRESHOLD_S = 0.5f;  // 40kph持续时间阈值

  if (ihc_sys_.input.vehicle_speed_display_kph >= 40.0F) {
    // 车速>=40kph，累计持续时间
    speed_above_40kph_duration_ += dt;
    if (speed_above_40kph_duration_ > 10.0f) {
      speed_above_40kph_duration_ = 10.0f;  // 防止溢出
    }
  } else {
    // 车速<40kph，清零持续时间
    speed_above_40kph_duration_ = 0.0f;
  }

  // 判断是否需要置位：车速<40kph 或者 车速>=40kph但未持续0.5s
  if (ihc_sys_.input.vehicle_speed_display_kph < 40.0F ||
      speed_above_40kph_duration_ < SPEED_40KPH_THRESHOLD_S) {
    ihc_enable_code_temp += uint16_bit[1];
  } else {
    // do nothing
  }

  // condition2: 雨刮运行速度相关条件
  // 2.1: 雨刷快速档时禁用远光灯
  // 2.2: 雨刷非快速档且未满60s时也禁用远光灯（避免频繁切换）
  if (wiper_is_high_speed_ || !wiper_not_high_speed_last_) {
    ihc_enable_code_temp += uint16_bit[2];
  } else {
    // do nothing
  }

  // condition3: 摄像头未出现临时遮挡问题
  // TODO: thzhang5 0907 需要根据文档需求更改

  // condition4：雾灯状态为false
  if (ihc_sys_.input.rear_fog_light_state ||
      ihc_sys_.input.front_fog_light_state) {
    ihc_enable_code_temp += uint16_bit[4];
  } else {
    // do nothing
  }

  // condition5: 转向灯关闭
  // 待确认

  // condition6: 环境昏暗
  if (ihc_sys_.input.lighting_condition !=
      iflyauto::CameraPerceptionLightingCondition::
          CAMERA_PERCEPTION_LIGHTING_CONDITION_DARK) {
    ihc_enable_code_temp += uint16_bit[6];
  } else {
    // do nothing
  }

  return ihc_enable_code_temp;
}

// 近光灯使能码: 0: 禁用, 其他: 使能 (任一满足)
uint16 IhcCore::UpdateIhcLowBeamCode() {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  uint16 ihc_disable_code_temp = 0;

  // condition0: 范围内有车, 置位
  if (dynamic_obstacle_check_ == true) {
    ihc_disable_code_temp += uint16_bit[0];
  } else {
    // do nothing
  }

  // condition1: 仪表车速降至30km/h以下，持续时间0.5s(可标定)
  const float dt = GetContext.get_param()->dt;
  const float SPEED_30KPH_THRESHOLD_S = 0.5f;  // 30kph持续时间阈值

  if (ihc_sys_.input.vehicle_speed_display_kph < 30.0F) {
    // 车速<30kph，累计持续时间
    speed_above_30kph_duration_ += dt;
    if (speed_above_30kph_duration_ > 10.0f) {
      speed_above_30kph_duration_ = 10.0f;  // 防止溢出
    }
  } else {
    // 车速>=30kph，清零持续时间
    speed_above_30kph_duration_ = 0.0f;
  }

  // 判断是否满足持续时间要求（车速低于30kph且持续0.5s就置位）
  if (speed_above_30kph_duration_ >= SPEED_30KPH_THRESHOLD_S) {
    ihc_disable_code_temp += uint16_bit[1];
  } else {
    // do nothing
  }

  // condition2：雨刮运行速度不是切换为非快速档位后持续时间未超过60s, 近光
  if (!wiper_not_high_speed_last_) {
    ihc_disable_code_temp += uint16_bit[2];
  } else {
    /*do nothing*/
  }

  // condition3：雾灯状态为true
  if (ihc_sys_.input.rear_fog_light_state == true ||
      ihc_sys_.input.front_fog_light_state == true) {
    ihc_disable_code_temp += uint16_bit[3];
  } else {
    // do nothing
  }

  // condition4：摄像头等关联键故障
  // TODO: thzhang5 0907 状态机还未给出

  // 环境亮度过高
  if (ihc_sys_.input.lighting_condition ==
      iflyauto::CameraPerceptionLightingCondition::
          CAMERA_PERCEPTION_LIGHTING_CONDITION_BRIGHT) {
    ihc_disable_code_temp += uint16_bit[4];
  } else {
    // do nothing
  }

  return ihc_disable_code_temp;
}

// 故障码: 0: 无故障, 其他: 故障 (任一满足)
uint16 IhcCore::UpdateIhcFaultCode() {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;

  uint16 fault_code = 0;

  // bit 0
  // 前视摄像头有故障(信号没有给出)
  // if (vehicle_service_output_info_ptr->... == false) {
  //   fault_code += uint16_bit[0];
  // } else {
  //   /*do nothing*/
  // }

  // bit 1
  // IHC感知模块节点通讯丢失，持续5s
  if (GetContext.mutable_state_info()->ihc_info_node_valid == false) {
    fault_code += uint16_bit[1];
  } else {
    /*do nothing*/
  }

  // bit 2
  // 障碍物融合模块节点通讯丢失，持续0.5s
  if (GetContext.mutable_state_info()->obstacle_fusion_info_node_valid ==
      false) {
    fault_code += uint16_bit[2];
  } else {
    /*do nothing*/
  }

  // bit 3
  // vehicle_service模块节点通讯丢失，持续0.5s
  if (GetContext.mutable_state_info()->vehicle_service_node_valid == false) {
    fault_code += uint16_bit[3];
  } else {
    /*do nothing*/
  }

  // bit 4
  // 实际车速信号无效
  if (vehicle_service_output_info_ptr->vehicle_speed_available == false) {
    fault_code += uint16_bit[4];
  } else {
    /*do nothing*/
  }

  // bit 5
  // 仪表车速信号无效
  if (vehicle_service_output_info_ptr->vehicle_speed_display_available ==
      false) {
    fault_code += uint16_bit[5];
  } else {
    /*do nothing*/
  }

  // bit 6
  // 雾灯信号无效
  if (vehicle_service_output_info_ptr->front_fog_light_state_available ==
          false ||
      vehicle_service_output_info_ptr->front_fog_light_state_available ==
          false) {
    fault_code += uint16_bit[6];
  } else {
    /*do nothing*/
  }

  // bit 7
  // 雨刮信号无效
  if (vehicle_service_output_info_ptr->wiper_state_available == false) {
    fault_code += uint16_bit[7];
  } else {
    /*do nothing*/
  }

  // bit 8
  // 故障降级
  auto degraded_driving_function_info_ptr =
      &GetContext.mutable_session()
           ->mutable_environmental_model()
           ->get_local_view()
           .degraded_driving_function_info;

  if ((degraded_driving_function_info_ptr->ihc.degraded == iflyauto::INHIBIT) ||
       (degraded_driving_function_info_ptr->ihc.degraded ==
           iflyauto::ERROR_DEGRADED) ||
       (degraded_driving_function_info_ptr->ihc.degraded ==
           iflyauto::ERROR_SAFE_STOP) ||
       (degraded_driving_function_info_ptr->ihc.degraded ==
           iflyauto::MCU_COMM_SHUTDOWN)) {
    fault_code += uint16_bit[8];
  } else {
    /*do nothing*/
  }

  return fault_code;
}

iflyauto::IHCFunctionFSMWorkState IhcCore::IHCStateMachine() {
  bool main_switch = ihc_sys_.input.ihc_main_switch;  // IHC开关状态
  uint16 fault_code =
      ihc_sys_.state
          .ihc_fault_code;  // 故障码: 0: 无故障, 其他: 故障 (任一满足)
  uint16 active_code =
      ihc_sys_.state.ihc_active_code;  // 激活码: 0: 使能, 其他: 禁用 (全部满足)

  static uint8 ihc_state_machine_init_flag =
      0;  // IHC状态机初始化状态 0:未初始化过 1:已完成过初始化
  static uint8 ihc_state_fault_off_standby_active =
      0;  // IHC一级主状态 FAULT OFF STANDBY ACTIVE
  iflyauto::IHCFunctionFSMWorkState
      ihc_state_temp;  // 用于存储状态机跳转完状态的临时变量

  if (ihc_state_machine_init_flag == 0) {
    // 状态机处于初始化状态 根据开关状态,决定第一个周期是输出OFF还是STANDBY
    ihc_state_machine_init_flag = 1;
    if (!main_switch) {
      // 开关关闭 -> OFF
      ihc_state_fault_off_standby_active = IHC_StateMachine_IN_OFF;
      ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_OFF;
    } else {
      // 开关打开 -> 根据故障码决定状态
      if (fault_code > 0) {
        // 有故障 -> FAULT
        ihc_state_fault_off_standby_active = IHC_StateMachine_IN_FAULT;
        ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_FAULT;
      } else {
        // 无故障 -> STANDBY
        ihc_state_fault_off_standby_active = IHC_StateMachine_IN_STANDBY;
        ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_STANDBY;
      }
    }
  } else {
    // 状态机处于完成过初始化的状态
    switch (ihc_state_fault_off_standby_active) {
      case IHC_StateMachine_IN_ACTIVE:
        if (!main_switch) {  // 1. 优先级最高：开关关闭 -> OFF
          ihc_state_fault_off_standby_active = IHC_StateMachine_IN_OFF;
          ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_OFF;
        } else if (fault_code) {  // 2. 其次：有故障 -> FAULT
          ihc_state_fault_off_standby_active = IHC_StateMachine_IN_FAULT;
          ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_FAULT;
        } else if (active_code > 0) {  // 3. standby条件 -> STANDBY, >0为禁用
          ihc_state_fault_off_standby_active = IHC_StateMachine_IN_STANDBY;
          ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_STANDBY;
        } else {  // 维持ACTIVE
          ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_ACTIVE;
        }
        break;
      case IHC_StateMachine_IN_FAULT:
        if (!main_switch) {  // 1. 优先级最高：开关关闭 -> OFF
          ihc_state_fault_off_standby_active = IHC_StateMachine_IN_OFF;
          ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_OFF;
        } else if (!fault_code) {  // 2. 故障消失 -> STANDBY
          ihc_state_fault_off_standby_active = IHC_StateMachine_IN_STANDBY;
          ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_STANDBY;
        } else {  // 3. 有故障时维持FAULT状态
          ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_FAULT;
        }
        break;
      case IHC_StateMachine_IN_OFF:
        if (fault_code && main_switch) {  // 1. 有故障且开关打开 -> FAULT
          ihc_state_fault_off_standby_active = IHC_StateMachine_IN_FAULT;
          ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_FAULT;
        } else if (main_switch) {  // 2. 无故障且开关打开 -> STANDBY
          ihc_state_fault_off_standby_active = IHC_StateMachine_IN_STANDBY;
          ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_STANDBY;
        } else {  // 3. 维持OFF
          ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_OFF;
        }
        break;
      default:             // STANDBY
        if (!main_switch) {  // 1. 优先级最高：开关关闭 -> OFF
          ihc_state_fault_off_standby_active = IHC_StateMachine_IN_OFF;
          ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_OFF;
        } else if (fault_code) {  // 2. 其次：有故障 -> FAULT
          ihc_state_fault_off_standby_active = IHC_StateMachine_IN_FAULT;
          ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_FAULT;
        } else if (active_code == 0) {  // 3. active条件 -> ACTIVE, ==0为使能
          ihc_state_fault_off_standby_active = IHC_StateMachine_IN_ACTIVE;
          ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_ACTIVE;
        } else {  // 4. 维持STANDBY
          ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_STANDBY;
        }
        break;
    }
  }
  return ihc_state_temp;
}

bool IhcCore::IHCRequestLightingFilter(bool ihc_request_lighting,
                                       uint8_t window_size,
                                       float ratio_threshold,
                                       uint8_t max_trasition) {
  bool ihc_request_lighting_filter_temp = false;  // 默认近光灯
  if (ihc_request_lighting_buffer_.size() < window_size) {
    ihc_request_lighting_buffer_.push_back(ihc_request_lighting);
    ihc_request_lighting_filter_temp = false;
  } else {
    ihc_request_lighting_buffer_.erase(ihc_request_lighting_buffer_.begin());
    ihc_request_lighting_buffer_.push_back(ihc_request_lighting);
    // 计算信号是否稳定
    uint8_t transitions = 0;
    for (uint8_t i = 0; i < ihc_request_lighting_buffer_.size() - 1; ++i) {
      if (ihc_request_lighting_buffer_[i] !=
          ihc_request_lighting_buffer_[i + 1]) {
        transitions++;
      }
    }
    if (transitions <= max_trasition) {
      // 选择输出结果
      // 统计true和false的数量，选择出现最多的结果
      uint8_t true_count = 0;
      uint8_t false_count = 0;

      for (uint8_t i = 0; i < ihc_request_lighting_buffer_.size(); ++i) {
        if (ihc_request_lighting_buffer_[i]) {
          true_count++;
        } else {
          false_count++;
        }
      }

      // 选择出现次数最多的结果，如果相等则默认选择false
      ihc_request_lighting_filter_temp = (true_count > false_count);

    } else {
      ihc_request_lighting_filter_temp = false;
    }
  }
  return ihc_request_lighting_filter_temp;
}

/*
  动态障碍物检查函数
  功能：持续检测范围内是否有稳定的障碍物
  返回值：true - 检测到稳定的障碍物（需要近光灯）
         false - 无稳定障碍物（可以使用远光灯）
*/
bool IhcCore::DynamicObstacleCheck(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  bool last_high_beam_request =
      GetContext.get_output_info()->ihc_output_info_.ihc_request_;

  // 获取自车速度信息
  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;
  double ego_speed_kph =
      vehicle_service_output_info_ptr->vehicle_speed_display *
      3.6;  // 转换为km/h

  // 初始化debug变量（仅在持续满足阈值时间后才会被置为true）
  ihc_sys_.state.low_beam_due_to_same_dir_vehicle = false;
  ihc_sys_.state.low_beam_due_to_oncomming_vehicle = false;
  ihc_sys_.state.low_beam_due_to_oncomming_cycle = false;

  // 本周期即时检测标志（不直接触发近光，仅用于时间累计）
  bool detected_same_dir = false;
  bool detected_oncoming_vehicle = false;
  bool detected_oncoming_cycle = false;

  // 获取动态障碍物消息 - 优化：减少重复的调用链访问
  const auto &fusion_objects_info = GetContext.get_session()
                                        ->environmental_model()
                                        .get_local_view()
                                        .fusion_objects_info;
  const auto &fusion_objs = fusion_objects_info.fusion_object;
  const int fusion_objs_num = fusion_objects_info.fusion_object_size;

  // 步骤1: 收集当前帧所有障碍物ID，用于后续清理
  std::set<uint16> current_frame_ids;
  
  // 步骤2: 第一次遍历，更新verified_obstacle_ids_（同时被相机和雷达检测到的障碍物）
  for (int i = 0; i < fusion_objs_num; i++) {
    uint16 track_id = fusion_objs[i].additional_info.track_id;
    current_frame_ids.insert(track_id);
    
    // 判断障碍物的track_age
    if (fusion_objs[i].additional_info.track_age < 500) {
      continue;  // 跳过track_age小于500ms的障碍物
    }
    
    // 判断障碍物数据来源, 必须是视觉和雷达都检测出才行, 使用fusion_source判断
    uint32_t fusion_source = fusion_objs[i].additional_info.fusion_source;
    bool has_camera = (fusion_source & 0x01) != 0;  // 第1位：前相机来源
    bool has_radar = (fusion_source & 0xFE) != 0;   // 第2-8位：雷达来源（前毫米波、左前、右前、左后、右后、超声波、激光雷达）
    
    // 如果同时有相机和雷达来源，则加入到可信障碍物集合中
    if (has_camera && has_radar) {
      verified_obstacle_ids_.insert(track_id);
    }
  }
  
  // 步骤3: 第二次遍历，只处理在verified_obstacle_ids_中的障碍物
  for (int i = 0; i < fusion_objs_num; i++) {
    uint16 track_id = fusion_objs[i].additional_info.track_id;
    
    // 只有在verified_obstacle_ids_中的障碍物才被认为是真实障碍物
    if (verified_obstacle_ids_.find(track_id) == verified_obstacle_ids_.end()) {
      continue;  // 跳过不在可信列表中的障碍物
    }
    
    float distance_x = fusion_objs[i].common_info.relative_center_position.x;
    float distance_y = fusion_objs[i].common_info.relative_center_position.y;

    // 筛选前方的车辆动态障碍物，使用滞回控制
    if (distance_x > 0 && distance_x < 230.0F) {  // 扩大检测范围
      // 判断障碍物是否为机动车
      if (fusion_objs[i].common_info.type >=
              iflyauto::ObjectType::OBJECT_TYPE_COUPE &&
          fusion_objs[i].common_info.type <=
              iflyauto::ObjectType::OBJECT_TYPE_TRAILER) {
        // 判断障碍物是否为对向车辆
        if (fusion_objs[i].additional_info.motion_pattern_current ==
            iflyauto::ObjectMotionType::OBJECT_MOTION_TYPE_ONCOME) {
          // 对向机动车
          // 检测对向车1s后是否仍然在车辆前方，防止因为对向来车误检,导致频繁闪灯
          if (distance_x +
                  fusion_objs[i].common_info.relative_velocity.x * 1.0f <=
              0) {
            // 1s后在自车后方, 不管是否在滞回区间, 则不在灯光影响区域
            continue;
          }
          // 滞回控制，200m~230m为滞回区间
          if (distance_x < 200.0f) {
            // 明确进入近光区域（即时检测为true，用于时间累计）
            detected_oncoming_vehicle = true;
          } else if (distance_x <= 230.0f) {
            // 200m~230m滞回区间，保持当前状态
            if (!last_high_beam_request) {
              detected_oncoming_vehicle = true;
            }
          }
          // distance > 230.0f 时继续检查其他障碍物
        } else if (fusion_objs[i].additional_info.motion_pattern_current ==
                   iflyauto::ObjectMotionType::OBJECT_MOTION_TYPE_MOVING) {
          // 同向机动车：滞回控制，100m~120m为滞回区间
          if (distance_x < 100.0f) {
            // 明确进入近光区域（即时检测为true，用于时间累计）
            detected_same_dir = true;
          } else if (distance_x <= 120.0f) {
            // 100m~120m滞回区间，保持当前状态
            if (!last_high_beam_request) {
              detected_same_dir = true;
            }
          }
          // distance > 120.0f 时继续检查其他障碍物
        }
      } else if (fusion_objs[i].common_info.type >=
                     iflyauto::ObjectType::OBJECT_TYPE_CYCLE_RIDING &&
                 fusion_objs[i].common_info.type <=
                     iflyauto::ObjectType::OBJECT_TYPE_TRICYCLE_RIDING) {
        // 对向非机动车：滞回控制，75m~95m为滞回区间
        if (fusion_objs[i].additional_info.motion_pattern_current ==
            iflyauto::ObjectMotionType::OBJECT_MOTION_TYPE_ONCOME) {
          if (distance_x < 75.0f) {
            // 明确进入近光区域（即时检测为true，用于时间累计）
            detected_oncoming_cycle = true;
          } else if (distance_x <= 95.0f) {
            // 75m~95m滞回区间，保持当前状态
            if (!last_high_beam_request) {
              detected_oncoming_cycle = true;
            }
          }
          // distance > 95.0f 时继续检查其他障碍物
        }
      }
    }
  }

  // 步骤4: 清理不在当前帧的障碍物ID
  // 遍历verified_obstacle_ids_，删除不在current_frame_ids中的ID
  for (auto it = verified_obstacle_ids_.begin(); it != verified_obstacle_ids_.end(); ) {
    if (current_frame_ids.find(*it) == current_frame_ids.end()) {
      // 不在当前帧中，删除
      it = verified_obstacle_ids_.erase(it);
    } else {
      ++it;
    }
  }

  // 同向车辆
  if (detected_same_dir) {
    ihc_sys_.state.low_beam_due_to_same_dir_vehicle = true;
  }
  // 对向机动车
  if (detected_oncoming_vehicle) {
    ihc_sys_.state.low_beam_due_to_oncomming_vehicle = true;
  }

  // 对向非机动车
  if (detected_oncoming_cycle) {
    ihc_sys_.state.low_beam_due_to_oncomming_cycle = true;
  }
  
  // 返回是否检测到稳定的障碍物
  return (ihc_sys_.state.low_beam_due_to_same_dir_vehicle ||
          ihc_sys_.state.low_beam_due_to_oncomming_vehicle ||
          ihc_sys_.state.low_beam_due_to_oncomming_cycle);
}

bool IhcCore::IHCRequest() {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  bool ihc_request_temp =
      GetContext.get_output_info()->ihc_output_info_.ihc_request_;

  uint16 high_beam_code =
      ihc_sys_.state
          .ihc_high_beam_code;  // 远光灯使能码: 0: 使能, 其他: 禁用 (全部满足)
  uint16 low_beam_code =
      ihc_sys_.state
          .ihc_low_beam_code;  // 近光灯使能码: 0: 禁用, 其他: 使能 (任一满足)

  // 时间保护策略常量
  const float MIN_HIGH_BEAM_ON_S = 2.0f;  // 远光最小持续时间
  const float MIN_LOW_BEAM_ON_S = 1.0f;   // 近光最小持续时间

  if (last_high_beam_request_ == true) {
    // 当前远光灯(上次为远光灯请求), 在满足前置条件的情况下,
    // 根据近光灯使能码判断是否需要切近光 前置条件: 1. 处于active, 2.
    // 远光灯最小点亮时长满足
    if (ihc_sys_.state.ihc_state ==
            iflyauto::IHC_FUNCTION_FSM_WORK_STATE_ACTIVE &&
        ihc_high_beam_on_duration_s >= MIN_HIGH_BEAM_ON_S) {
      if (low_beam_code > 0) {
        ihc_request_temp = false;
      }
    }
  } else {
    // 当前近光灯, 在满足前置条件的情况下, 根据远光灯使能码判断是否需要切远光
    // 前置条件: 1. 处于active, 2. 近光灯最小点亮时长满足
    if (ihc_sys_.state.ihc_state ==
            iflyauto::IHC_FUNCTION_FSM_WORK_STATE_ACTIVE &&
        ihc_low_beam_on_duration_s >= MIN_LOW_BEAM_ON_S) {
      if (high_beam_code == 0) {
        ihc_request_temp = true;
      }
    }
  }

  return ihc_request_temp;
}

void IhcCore::SetIhcOutputInfo(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  // 状态安全检查：只有在ACTIVE状态下才能输出有效的请求
  if (ihc_sys_.state.ihc_state ==
      iflyauto::IHC_FUNCTION_FSM_WORK_STATE_ACTIVE) {
    GetContext.mutable_output_info()->ihc_output_info_.ihc_request_status_ =
        ihc_sys_.state.ihc_request_status;
    GetContext.mutable_output_info()->ihc_output_info_.ihc_request_ =
        ihc_sys_.state.ihc_request;
  } else {
    // 非ACTIVE状态下，强制设置为无请求状态
    GetContext.mutable_output_info()->ihc_output_info_.ihc_request_status_ =
        false;
    GetContext.mutable_output_info()->ihc_output_info_.ihc_request_ = false;
  }

  // 输出状态信息
  switch (ihc_sys_.state.ihc_state) {
    case iflyauto::IHC_FUNCTION_FSM_WORK_STATE_FAULT:
      GetContext.mutable_output_info()->ihc_output_info_.ihc_state_ =
          iflyauto::IHC_FUNCTION_FSM_WORK_STATE_FAULT;
      break;
    case iflyauto::IHC_FUNCTION_FSM_WORK_STATE_OFF:
      GetContext.mutable_output_info()->ihc_output_info_.ihc_state_ =
          iflyauto::IHC_FUNCTION_FSM_WORK_STATE_OFF;
      break;
    case iflyauto::IHC_FUNCTION_FSM_WORK_STATE_STANDBY:
      GetContext.mutable_output_info()->ihc_output_info_.ihc_state_ =
          iflyauto::IHC_FUNCTION_FSM_WORK_STATE_STANDBY;
      break;
    case iflyauto::IHC_FUNCTION_FSM_WORK_STATE_ACTIVE:
      GetContext.mutable_output_info()->ihc_output_info_.ihc_state_ =
          iflyauto::IHC_FUNCTION_FSM_WORK_STATE_ACTIVE;
      break;
    default:
      // 不可用状态，设置为 OFF
      GetContext.mutable_output_info()->ihc_output_info_.ihc_state_ =
          iflyauto::IHC_FUNCTION_FSM_WORK_STATE_OFF;
      break;
  }
}

void IhcCore::JsonSwitchIhcMainSwitch(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  GetContext.mutable_output_info()->ihc_output_info_.ihc_state_ =
      iflyauto::IHC_FUNCTION_FSM_WORK_STATE_ACTIVE;
  GetContext.mutable_output_info()->ihc_output_info_.ihc_request_status_ = true;
  GetContext.mutable_output_info()->ihc_output_info_.ihc_request_ = true;
}

}  // namespace ihc_core
}  // namespace adas_function
