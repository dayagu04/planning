#include "traffic_sign_recognition.h"

namespace planning {

void TrafficSignRecognition::Update() {
  // 获取TSR开关状态
  tsr_sys_.input.tsr_main_switch =
      session_->mutable_environmental_model()->get_hmi_info().tsr_main_switch();

  // 当前车道的限速值 缺少接口
  auto ptr_current_lane = session_->mutable_environmental_model()
                              ->get_virtual_lane_manager()
                              ->get_current_lane();
  tsr_sys_.input.tsr_speed_limit = ptr_current_lane->get_ego_lateral_offset();

  // 获取当前仪表车速
  auto ptr_ego_state_manager =
      session_->mutable_environmental_model()->get_ego_state_manager();
  tsr_sys_.input.vehicle_speed_display_kph =
      ptr_ego_state_manager->ego_hmi_v() * 3.6F;  // 当前车速 单位:m/s
}

void TrafficSignRecognition::RunOnce() {
  // 更新输入信息
  Update();

  // 状态机跳转
  tsr_sys_.state.tsr_enable_code = TSREnableCode();
  tsr_sys_.state.tsr_disable_code = TSRDisableCode();
  tsr_sys_.state.tsr_fault_code = TSRFaultCode();
  tsr_sys_.state.tsr_state = TSRStateMachine();

  // TSR功能处于激活状态
  if (tsr_sys_.state.tsr_state == 3) {
    // TSR识别到的限速标识牌赋值
    tsr_sys_.state.tsr_speed_limit = tsr_sys_.input.tsr_speed_limit;

    // TSR超速报警标志位赋值
    if (tsr_sys_.input.vehicle_speed_display_kph >
        tsr_sys_.state.tsr_speed_limit) {
      tsr_sys_.state.tsr_warning = true;
    } else {
      tsr_sys_.state.tsr_warning = false;
    }
  } else {
    tsr_sys_.state.tsr_speed_limit = 0;
    tsr_sys_.state.tsr_warning = false;
  }
  set_tsr_output_info();

  JSON_DEBUG_VALUE("tsr_function::tsr_enable_code",
                   tsr_sys_.state.tsr_enable_code);
  JSON_DEBUG_VALUE("tsr_function::tsr_disable_code",
                   tsr_sys_.state.tsr_disable_code);
  JSON_DEBUG_VALUE("tsr_function::tsr_fault_code",
                   tsr_sys_.state.tsr_fault_code);
  JSON_DEBUG_VALUE("tsr_function::tsr_state", tsr_sys_.state.tsr_state);
  JSON_DEBUG_VALUE("tsr_function::tsr_warning", tsr_sys_.state.tsr_warning);
  JSON_DEBUG_VALUE("tsr_function::tsr_main_switch",
                   tsr_sys_.input.tsr_main_switch);
}

uint16 TrafficSignRecognition::TSREnableCode() {
  uint16 uint16_bit[16] = {1,   2,   4,    8,    16,   32,   64,    128,
                           256, 512, 1024, 2048, 4096, 8192, 16384, 32768};
  uint16 tsr_enable_code_temp = 0;

  return tsr_enable_code_temp;
}

uint16 TrafficSignRecognition::TSRDisableCode() {
  uint16 uint16_bit[16] = {1,   2,   4,    8,    16,   32,   64,    128,
                           256, 512, 1024, 2048, 4096, 8192, 16384, 32768};
  uint16 tsr_disable_code_temp = 0;

  return tsr_disable_code_temp;
}

uint16 TrafficSignRecognition::TSRFaultCode() {
  uint16 uint16_bit[16] = {1,   2,   4,    8,    16,   32,   64,    128,
                           256, 512, 1024, 2048, 4096, 8192, 16384, 32768};
  uint16 tsr_fault_code_temp = 0;

  return tsr_fault_code_temp;
}

uint8 TrafficSignRecognition::TSRStateMachine() {
  boolean main_switch = tsr_sys_.input.tsr_main_switch;
  uint16 fault_code = tsr_sys_.state.tsr_fault_code;
  uint16 enable_code = tsr_sys_.state.tsr_enable_code;
  uint16 disable_code = tsr_sys_.state.tsr_disable_code;

  static uint8 tsr_state_machine_init_flag =
      0;  // TSR状态机初始化状态 0:未初始化过 1:已完成过初始化
  static uint8 tsr_state_fault_off_standby_active =
      0;                 // tsr一级主状态 FAULT OFF STANDBY ACTIVE
  uint8 tsr_state_temp;  // 用于存储状态机跳转完状态的临时变量

  if (tsr_state_machine_init_flag == 0) {
    // 状态机处于初始化状态 根据开关状态,决定第一个周期是输出OFF还是STANDBY
    tsr_state_machine_init_flag = 1;
    if (!main_switch) {
      tsr_state_fault_off_standby_active = TSR_StateMachine_IN_OFF;
      tsr_state_temp = 1;
    } else {
      tsr_state_fault_off_standby_active = TSR_StateMachine_IN_STANDBY;
      tsr_state_temp = 2;
    }
  } else {
    // 状态机处于完成过初始化的状态
    switch (tsr_state_fault_off_standby_active) {
      case TSR_StateMachine_IN_ACTIVE:
        if (!main_switch) {  // ACTIVE->OFF
          tsr_state_fault_off_standby_active = TSR_StateMachine_IN_OFF;
          tsr_state_temp = 1;
        } else if (fault_code) {  // ACTIVE->FAULT
          tsr_state_fault_off_standby_active = TSR_StateMachine_IN_FAULT;
          tsr_state_temp = 0;
        } else if (disable_code) {  // ACTIVE->STANDBY
          tsr_state_fault_off_standby_active = TSR_StateMachine_IN_STANDBY;
          tsr_state_temp = 2;
        } else {  // 继续维持在ACTIVE状态
          tsr_state_temp = 3;
        }
        break;
      case TSR_StateMachine_IN_FAULT:
        if (!main_switch) {  // FAULT->OFF
          tsr_state_fault_off_standby_active = TSR_StateMachine_IN_OFF;
          tsr_state_temp = 1;
        } else if (!fault_code) {  // FAULT->STANDBY
          tsr_state_fault_off_standby_active = TSR_StateMachine_IN_STANDBY;
          tsr_state_temp = 2;
        } else {  // 继续维持在FAULT状态
          tsr_state_temp = 0;
        }
        break;
      case TSR_StateMachine_IN_OFF:
        if (main_switch) {  // OFF->STANDBY
          tsr_state_fault_off_standby_active = TSR_StateMachine_IN_STANDBY;
          tsr_state_temp = 2;
        } else {  // 继续维持在OFF状态
          tsr_state_temp = 1;
        }
        break;
      default:
        if (!main_switch) {  // STANDBY->OFF
          tsr_state_fault_off_standby_active = TSR_StateMachine_IN_OFF;
          tsr_state_temp = 1;
        } else if (fault_code) {  // STANDBY->FAULT
          tsr_state_fault_off_standby_active = TSR_StateMachine_IN_FAULT;
          tsr_state_temp = 0;
        } else if (enable_code == 0) {  // STANDBY->ACTIVE
          tsr_state_fault_off_standby_active = TSR_StateMachine_IN_ACTIVE;
          tsr_state_temp = 3;
        } else {  // 继续维持在STANDBY状态
          tsr_state_temp = 2;
        }
        break;
    }
  }
  return tsr_state_temp;
}

}  // namespace planning