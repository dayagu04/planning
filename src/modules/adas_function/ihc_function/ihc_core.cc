#include "ihc_core.h"

using namespace planning;
namespace adas_function {
namespace ihc_core {

void IhcCore::RunOnce(void) {
  // 更新输入信息
  GetInputInfo();

  // 根据输入信息，更新使能码、禁用码、故障码、状态机跳转
  ihc_sys_.state.ihc_enable_code = UpdateIhcEnableCode();
  ihc_sys_.state.ihc_disable_code = UpdateIhcDisableCode();
  ihc_sys_.state.ihc_fault_code = UpdateIhcFaultCode();

  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  if (GetContext.get_param()->ihc_use_json_code) {
    // 如果使用json，则使用配置文件中的使能码、禁用码、故障码
    ihc_sys_.input.ihc_main_switch = GetContext.get_param()->ihc_main_switch;
    ihc_sys_.state.ihc_enable_code = GetContext.get_param()->ihc_enable_code_maskcode;
    ihc_sys_.state.ihc_disable_code = GetContext.get_param()->ihc_disable_code_maskcode;
    ihc_sys_.state.ihc_fault_code = GetContext.get_param()->ihc_fault_code_maskcode;
  }

  ihc_sys_.state.ihc_state = IHCStateMachine();

  // IHC功能处于激活状态
  if (ihc_sys_.state.ihc_state == iflyauto::IHC_FUNCTION_FSM_WORK_STATE_ACTIVE) {
    ihc_sys_.state.ihc_request_status = true;
    ihc_sys_.state.ihc_request = IHCRequest();
  } else {
    ihc_sys_.state.ihc_request_status = false;
    ihc_sys_.state.ihc_request = false;
  }
  SetIhcOutputInfo();

  // 从配置文件中读取ihc_main_switch值, 测试用, 后续需要移动到XXXcode中
  if (GetContext.get_param()->ihc_use_json_switch) {
    JsonSwitchIhcMainSwitch();
  } else {
    // do nothing
  }

  JSON_DEBUG_VALUE("ihc_function::ihc_enable_code",
                   ihc_sys_.state.ihc_enable_code);
  JSON_DEBUG_VALUE("ihc_function::ihc_disable_code",
                   ihc_sys_.state.ihc_disable_code);
  JSON_DEBUG_VALUE("ihc_function::ihc_fault_code",
                   ihc_sys_.state.ihc_fault_code);
  JSON_DEBUG_VALUE("ihc_function::ihc_state", int(ihc_sys_.state.ihc_state));
  JSON_DEBUG_VALUE("ihc_function::ihc_request_status",
                   ihc_sys_.state.ihc_request_status);
  JSON_DEBUG_VALUE("ihc_function::ihc_request", ihc_sys_.state.ihc_request);
  JSON_DEBUG_VALUE("ihc_function::ihc_main_switch",
                   ihc_sys_.input.ihc_main_switch);
  JSON_DEBUG_VALUE("ihc_function::auto_light_state",
                   ihc_sys_.input.auto_light_state);
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

  // 获取IHC开关状态
  ihc_sys_.input.ihc_main_switch =
      function_state_machine_info_ptr->switch_sts.ihc_main_switch;

  // 获取当前仪表车速
  ihc_sys_.input.vehicle_speed_display_kph =
      vehicle_service_output_info_ptr->vehicle_speed_display * 3.6F;  // 当前车速 单位:kph

  // 获取换档杆状态
  ihc_sys_.input.shift_lever_state =
      static_cast<iflyauto::ShiftLeverStateEnum>(vehicle_service_output_info_ptr->shift_lever_state);

  // 获取前雾灯状态
  ihc_sys_.input.front_fog_light_state =
      vehicle_service_output_info_ptr->front_fog_light_state;

  // 获取自动灯光控制状态
  ihc_sys_.input.auto_light_state = vehicle_service_output_info_ptr->auto_light_state;
  
  // 前雾灯状态
  ihc_sys_.input.front_fog_light_state =
      vehicle_service_output_info_ptr->front_fog_light_state;

  // 后雾灯状态
  ihc_sys_.input.rear_fog_light_state =
      vehicle_service_output_info_ptr->rear_fog_light_state;
}

uint16 IhcCore::UpdateIhcEnableCode() {
  uint16 ihc_enable_code_temp = 0;

  // condition0: 换挡杆是否处于D
  if (ihc_sys_.input.shift_lever_state == iflyauto::ShiftLeverStateEnum::ShiftLeverState_D) {
    ihc_enable_code_temp += uint16_bit[0];
  } else {
      // do nothing
    }

  // condition1: 灯光挡位处于auto
  if (ihc_sys_.input.auto_light_state == true) {
    ihc_enable_code_temp += uint16_bit[1];
  } else {
    // do nothing
  }

  // condition2: 车速是否大于等于40kph
  if (ihc_sys_.input.vehicle_speed_display_kph >= 40.0F) {
    ihc_enable_code_temp += uint16_bit[2];
  } else {
    // do nothing
  }

  // codition3：车速是否小于等于150kph
  if (ihc_sys_.input.vehicle_speed_display_kph <= 150.0F) {
    ihc_enable_code_temp += uint16_bit[3];
  } else {
    // do nothing
  }

  // condition4: 前雾灯状态是否为false
  if (ihc_sys_.input.front_fog_light_state == false) {
    ihc_enable_code_temp += uint16_bit[4];
  } else {
    // do nothing
  }

  return ihc_enable_code_temp;
}
uint16 IhcCore::UpdateIhcDisableCode() {
  uint16 ihc_disable_code_temp = 0;

  // condition0：挡位不是D档
  if (ihc_sys_.input.shift_lever_state != iflyauto::ShiftLeverStateEnum::ShiftLeverState_D) {
    ihc_disable_code_temp += uint16_bit[0];
  } else {
    // do nothing
  }

  // condition1：灯光挡位不是auto
  if (ihc_sys_.input.auto_light_state == false) {
    ihc_disable_code_temp += uint16_bit[1];
  } else {
    // do nothing
  }

  // condition2: 车速小于等于25kph
  if (ihc_sys_.input.vehicle_speed_display_kph < 25.0F) {
    ihc_disable_code_temp += uint16_bit[2];
  } else {
    // do nothing
  }

  // condition3：车速是否大于155kph
  if (ihc_sys_.input.vehicle_speed_display_kph > 155.0F) {
    ihc_disable_code_temp += uint16_bit[3];
  } else {
    // do nothing
  }

  // condition4：前雾灯状态为true
  if (ihc_sys_.input.front_fog_light_state == true) {
    ihc_disable_code_temp += uint16_bit[4];
  } else {
    // do nothing
  }

  return ihc_disable_code_temp;
}

// TODO: thzhang5 0714 需要根据文档需求更改
uint16 IhcCore::UpdateIhcFaultCode() {
  uint16 ihc_fault_code_temp = 0;

  return ihc_fault_code_temp;
}

iflyauto::IHCFunctionFSMWorkState IhcCore::IHCStateMachine() {
  bool main_switch = ihc_sys_.input.ihc_main_switch;
  uint16 fault_code = ihc_sys_.state.ihc_fault_code;
  uint16 enable_code = ihc_sys_.state.ihc_enable_code;
  uint16 disable_code = ihc_sys_.state.ihc_disable_code;

  static uint8 ihc_state_machine_init_flag =
      0;  // IHC状态机初始化状态 0:未初始化过 1:已完成过初始化
  static uint8 ihc_state_fault_off_standby_active =
      0;                 // IHC一级主状态 FAULT OFF STANDBY ACTIVE
  iflyauto::IHCFunctionFSMWorkState ihc_state_temp;  // 用于存储状态机跳转完状态的临时变量

  if (ihc_state_machine_init_flag == 0) {
    // 状态机处于初始化状态 根据开关状态,决定第一个周期是输出OFF还是STANDBY
    ihc_state_machine_init_flag = 1;
    if (!main_switch) {
      ihc_state_fault_off_standby_active = IHC_StateMachine_IN_OFF;
      ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_OFF;
    } else {
      ihc_state_fault_off_standby_active = IHC_StateMachine_IN_STANDBY;
      ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_STANDBY;
    }
  } else {
    // 状态机处于完成过初始化的状态
    switch (ihc_state_fault_off_standby_active) {
      case IHC_StateMachine_IN_ACTIVE:
        if (!main_switch) {  // ACTIVE->OFF
          ihc_state_fault_off_standby_active = IHC_StateMachine_IN_OFF;
          ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_OFF;
        } else if (fault_code) {  // ACTIVE->FAULT
          ihc_state_fault_off_standby_active = IHC_StateMachine_IN_FAULT;
          ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_UNAVAILABLE;
        } else if (disable_code) {  // ACTIVE->STANDBY
          ihc_state_fault_off_standby_active = IHC_StateMachine_IN_STANDBY;
          ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_STANDBY;
        } else {  // 继续维持在ACTIVE状态
          ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_ACTIVE;
        }
        break;
      case IHC_StateMachine_IN_FAULT:
        if (!main_switch) {  // FAULT->OFF
          ihc_state_fault_off_standby_active = IHC_StateMachine_IN_OFF;
          ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_OFF;
        } else if (!fault_code) {  // FAULT->STANDBY
          ihc_state_fault_off_standby_active = IHC_StateMachine_IN_STANDBY;
          ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_STANDBY;
        } else {  // 继续维持在FAULT状态
          ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_UNAVAILABLE;
        }
        break;
      case IHC_StateMachine_IN_OFF:
        if (main_switch) {  // OFF->STANDBY
          ihc_state_fault_off_standby_active = IHC_StateMachine_IN_STANDBY;
          ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_STANDBY;
        } else {  // 继续维持在OFF状态
          ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_OFF;
        }
        break;
      default:
        if (!main_switch) {  // STANDBY->OFF
          ihc_state_fault_off_standby_active = IHC_StateMachine_IN_OFF;
          ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_OFF;
        } else if (fault_code) {  // STANDBY->FAULT
          ihc_state_fault_off_standby_active = IHC_StateMachine_IN_FAULT;
          ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_UNAVAILABLE;
        } else if (enable_code == 31) {  // STANDBY->ACTIVE
          ihc_state_fault_off_standby_active = IHC_StateMachine_IN_ACTIVE;
          ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_ACTIVE;
        } else {  // 继续维持在STANDBY状态
          ihc_state_temp = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_STANDBY;
        }
        break;
    }
  }
  return ihc_state_temp;
}

bool IhcCore::IHCRequestLightingFilter(bool ihc_request_lighting, uint8_t window_size, float ratio_threshold, uint8_t max_trasition) {
  bool ihc_request_lighting_filter_temp = false; // 默认近光灯
  if (ihc_request_lighting_buffer_.size() < window_size) {
    ihc_request_lighting_buffer_.push_back(ihc_request_lighting);
    ihc_request_lighting_filter_temp = false;
  } else {
    ihc_request_lighting_buffer_.erase(ihc_request_lighting_buffer_.begin());
    ihc_request_lighting_buffer_.push_back(ihc_request_lighting);
    // 计算信号是否稳定
    uint8_t transitions = 0;
    for (uint8_t i = 0; i < ihc_request_lighting_buffer_.size(); ++i) {
      if (ihc_request_lighting_buffer_[i] != ihc_request_lighting_buffer_[i + 1]) {
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

bool IhcCore::IHCRequest() {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  bool ihc_request_temp = GetContext.get_output_info()->ihc_output_info_.ihc_request_;
  
  // 获取环境信息
  // TODO: thzhang5 0718 如果感知不准确，是否需要设置观察一段时间后在跳变
  auto scene_info_ptr = &GetContext.mutable_session()
                                     ->mutable_environmental_model()
                                     ->get_local_view()
                                     .perception_scene_info;

  // 环境亮度条件
  if (scene_info_ptr->lighting_condition == iflyauto::CameraPerceptionLightingCondition::CAMERA_PERCEPTION_LIGHTING_CONDITION_DARK) {
    // 昏暗环境
    ihc_request_temp = true;
  } else if (scene_info_ptr->lighting_condition == iflyauto::CameraPerceptionLightingCondition::CAMERA_PERCEPTION_LIGHTING_CONDITION_BRIGHT) {
    // 明亮环境
    ihc_request_temp = false;
  } else {
    // 中等亮度环境，保持
    // do nothing
  }
  
  return ihc_request_temp;
}



void IhcCore::SetIhcOutputInfo(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  GetContext.mutable_output_info()->ihc_output_info_.ihc_request_status_ =
    ihc_sys_.state
        .ihc_request_status;  // IHC请求状态 0:No Request 1:Request
  GetContext.mutable_output_info()->ihc_output_info_.ihc_request_ =
      ihc_sys_.state
          .ihc_request;  // IHC请求 0:LowBeam 1:HighBeam
                          // IHC功能状态 0:Unavailable 1:Off 2:Standby 3:Active
  switch (ihc_sys_.state.ihc_state) {
    case iflyauto::IHC_FUNCTION_FSM_WORK_STATE_UNAVAILABLE:
      GetContext.mutable_output_info()->ihc_output_info_.ihc_state_ = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_UNAVAILABLE;
      break;
    case iflyauto::IHC_FUNCTION_FSM_WORK_STATE_OFF:
      GetContext.mutable_output_info()->ihc_output_info_.ihc_state_ = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_OFF;
      break;
    case iflyauto::IHC_FUNCTION_FSM_WORK_STATE_STANDBY:
      GetContext.mutable_output_info()->ihc_output_info_.ihc_state_ = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_STANDBY;
      break;
    case iflyauto::IHC_FUNCTION_FSM_WORK_STATE_ACTIVE:
      GetContext.mutable_output_info()->ihc_output_info_.ihc_state_ = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_ACTIVE;
      break;
    default:
      GetContext.mutable_output_info()->ihc_output_info_.ihc_state_ = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_OFF;
      break;
  }
}

bool IhcCore::JsonSwitchIhcMainSwitch() {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  // 读取配置文件中的 ihc_main_switch 值
  bool ihc_switch = GetContext.get_param()->ihc_main_switch;
  
  if (ihc_switch) {
    // 如果为1，设置为true
    GetContext.mutable_output_info()->ihc_output_info_.ihc_state_ = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_ACTIVE;
    GetContext.mutable_output_info()->ihc_output_info_.ihc_request_status_ = true;
    GetContext.mutable_output_info()->ihc_output_info_.ihc_request_ = true;
  } else {
    // 如果为0，设置为false 
    GetContext.mutable_output_info()->ihc_output_info_.ihc_state_ = iflyauto::IHC_FUNCTION_FSM_WORK_STATE_ACTIVE;
    GetContext.mutable_output_info()->ihc_output_info_.ihc_request_status_ = true;
    GetContext.mutable_output_info()->ihc_output_info_.ihc_request_ = false;
  }
  
  return ihc_switch;
}

}  // namespace ihc_core
}  // namespace adas_function