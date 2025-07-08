#include "tsr_core.h"

#include <iostream>

#include "adas_function_lib.h"
#include "planning_hmi_c.h"

namespace adas_function {
namespace tsr_core {

bool TsrCore::UpdateTsrMainSwitch(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  auto hmi_mcu_inner_info_ptr = &GetContext.mutable_session()
                                     ->mutable_environmental_model()
                                     ->get_local_view()
                                     .hmi_mcu_inner_info;
  if (GetContext.get_param()->tsr_main_switch) {
    return GetContext.get_param()->tsr_main_switch;
  }
  return hmi_mcu_inner_info_ptr->tsr_main_switch;
}

uint16 TsrCore::UpdateTsrEnableCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;

  uint16 enable_code = 0;

  // bit 0
  // 判断是否为前进挡
  if (vehicle_service_output_info_ptr->shift_lever_state !=
      iflyauto::ShiftLeverStateEnum::ShiftLeverState_D) {
    enable_code += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // if (tsr_speed_limit_valid_ == false) {
  //   enable_code += uint16_bit[1];
  // } else {
  //   /*do nothing*/
  // }

  return enable_code;
}

uint16 TsrCore::UpdateTsrDisableCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;

  uint16 disable_code = 0;

  // bit 0
  // 判断是否为前进挡
  if (vehicle_service_output_info_ptr->shift_lever_state !=
      iflyauto::ShiftLeverStateEnum::ShiftLeverState_D) {
    disable_code += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // if (tsr_speed_limit_valid_ == false) {
  //   disable_code += uint16_bit[1];
  // } else {
  //   /*do nothing*/
  // }

  return disable_code;
}

uint16 TsrCore::UpdateTsrFaultCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;

  uint16 fault_code = 0;

  // bit 0
  // 判断挡位信号有效性
  if (vehicle_service_output_info_ptr->shift_lever_state_available == false) {
    fault_code += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // bit 1
  // 判断仪表车速信号有效性
  if (vehicle_service_output_info_ptr->vehicle_speed_display_available ==
      false) {
    fault_code += uint16_bit[1];
  } else {
    /*do nothing*/
  }

  return fault_code;
}

iflyauto::TSRFunctionFSMWorkState TsrCore::TsrStateMachine(void) {
  iflyauto::TSRFunctionFSMWorkState tsr_state;
  iflyauto::TSRFunctionFSMWorkState tsr_state_delay = tsr_state_;

  if (tsr_state_machine_init_flag_ == false) {
    // 状态机处于初始化状态 根据开关状态,决定第一个周期是输出OFF还是STANDBY
    tsr_state_machine_init_flag_ = true;
    if (tsr_main_switch_ == false) {
      tsr_state =
          iflyauto::TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_OFF;
    } else {
      tsr_state = iflyauto::TSRFunctionFSMWorkState::
          TSR_FUNCTION_FSM_WORK_STATE_STANDBY;
    }
    return tsr_state;
  }

  // 状态机处于完成过初始化的状态
  if (tsr_state_delay == iflyauto::TSRFunctionFSMWorkState::
                             TSR_FUNCTION_FSM_WORK_STATE_UNAVAILABLE) {
    // 上一时刻处于TSR_FUNCTION_FSM_WORK_STATE_UNAVAILABLE状态
    if (tsr_main_switch_ == false) {
      tsr_state =
          iflyauto::TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_OFF;
    } else if (tsr_fault_code_ == 0) {
      tsr_state = iflyauto::TSRFunctionFSMWorkState::
          TSR_FUNCTION_FSM_WORK_STATE_STANDBY;
    } else {
      tsr_state = iflyauto::TSRFunctionFSMWorkState::
          TSR_FUNCTION_FSM_WORK_STATE_UNAVAILABLE;
    }
  } else if (tsr_state_delay == iflyauto::TSRFunctionFSMWorkState::
                                    TSR_FUNCTION_FSM_WORK_STATE_OFF) {
    // 上一时刻处于TSR_FUNCTION_FSM_WORK_STATE_OFF状态
    if (tsr_main_switch_ == true) {
      tsr_state = iflyauto::TSRFunctionFSMWorkState::
          TSR_FUNCTION_FSM_WORK_STATE_STANDBY;
    } else {
      tsr_state =
          iflyauto::TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_OFF;
    }
  } else if (tsr_state_delay == iflyauto::TSRFunctionFSMWorkState::
                                    TSR_FUNCTION_FSM_WORK_STATE_STANDBY) {
    // 上一时刻处于TSR_FUNCTION_FSM_WORK_STATE_STANDBY状态
    if (tsr_main_switch_ == false) {
      tsr_state =
          iflyauto::TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_OFF;
    } else if (tsr_fault_code_) {
      tsr_state = iflyauto::TSRFunctionFSMWorkState::
          TSR_FUNCTION_FSM_WORK_STATE_UNAVAILABLE;
    } else if (tsr_enable_code_ == 0) {
      tsr_state =
          iflyauto::TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_ACTIVE;
    } else {
      tsr_state = iflyauto::TSRFunctionFSMWorkState::
          TSR_FUNCTION_FSM_WORK_STATE_STANDBY;
    }
  } else if (tsr_state_delay == iflyauto::TSRFunctionFSMWorkState::
                                    TSR_FUNCTION_FSM_WORK_STATE_ACTIVE) {
    // 上一时刻处于TSR_FUNCTION_FSM_WORK_STATE_ACTIVE状态
    if (tsr_main_switch_ == false) {
      tsr_state =
          iflyauto::TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_OFF;
    } else if (tsr_fault_code_) {
      tsr_state = iflyauto::TSRFunctionFSMWorkState::
          TSR_FUNCTION_FSM_WORK_STATE_UNAVAILABLE;
    } else if (tsr_disable_code_) {
      tsr_state = iflyauto::TSRFunctionFSMWorkState::
          TSR_FUNCTION_FSM_WORK_STATE_STANDBY;
    } else {
      tsr_state =
          iflyauto::TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_ACTIVE;
    }
  } else {
    // 处于异常状态
    tsr_state =
        iflyauto::TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_OFF;
  }

  return tsr_state;
}

void TsrCore::UpdateTsrSpeedLimit(void) {
  // 需要从规划拿数据
  // 奇瑞要求功能关闭时也显示限速值,只是不会报警
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  auto peception_tsr_info = GetContext.get_session()
                                ->environmental_model()
                                .get_local_view()
                                .perception_tsr_info;
  auto supp_signs_array = peception_tsr_info.supp_signs;
  int supp_signs_array_size = peception_tsr_info.supp_signs_size;
  bool sign_vald = false;
  uint32 lane_speed_limit_value = tsr_speed_limit_;
  uint32 tsr_speed_limit_last = tsr_speed_limit_;
  for (int i = 0; i < supp_signs_array_size; i++) {
    auto single_sign = supp_signs_array[i];
    if (single_sign.supp_sign_type != iflyauto::SUPP_SIGN_TYPE_MAXIMUM_SPEED) {
      continue;
    } else {
      if (sign_vald == false) {
        lane_speed_limit_value = single_sign.speed_limit;
        sign_vald = true;
        continue;
      } else {
        if (single_sign.speed_limit > lane_speed_limit_value) {
          lane_speed_limit_value = single_sign.speed_limit;
        }
      }
    }
  }
  bool speed_limit_exist_in_view_flag_last = speed_limit_exist_in_view_flag_;
  uint32 speed_limit_exist_in_view_last = speed_limit_exist_in_view_;
  if (sign_vald == true) {
    speed_limit_exist_in_view_flag_ = true;
    speed_limit_exist_in_view_ = lane_speed_limit_value;
    // tsr_speed_limit_ = lane_speed_limit_value;
    // tsr_speed_limit_valid_ = true;
  } else {
    speed_limit_exist_in_view_flag_ = false;
    speed_limit_exist_in_view_ = 0;
  }

  if (speed_limit_exist_in_view_flag_last == true &&
      speed_limit_exist_in_view_flag_ == false) {
    // 通过限速标志消失与否判断车辆是否驶入限速路段
    if (tsr_speed_limit_valid_ == false) {
      tsr_speed_limit_ = speed_limit_exist_in_view_last;
    } else {
      if (speed_limit_exist_in_view_last != tsr_speed_limit_) {
        tsr_speed_limit_ = speed_limit_exist_in_view_last;
      }
    }
    tsr_speed_limit_valid_ = true;
  } else if (speed_limit_exist_in_view_flag_ == true) {
    // 当标志牌一直存在时，如果限速值存在变化时，判断进入了限速路段
    if ((speed_limit_exist_in_view_last != speed_limit_exist_in_view_) &&
        (speed_limit_exist_in_view_flag_last == true)) {
      tsr_speed_limit_ = speed_limit_exist_in_view_last;
      tsr_speed_limit_valid_ = true;
    }
  } else {
    /*do nothing*/
  }

  if (tsr_speed_limit_valid_ == false) {
    tsr_speed_limit_ = 0;
  }
  if (tsr_speed_limit_valid_ == true) {
    if (tsr_speed_limit_last != tsr_speed_limit_) {
      tsr_speed_limit_change_flag_ = true;
    }
  }
  if (accumulated_path_length_ >
      GetContext.get_param()->tsr_reset_path_length) {
    tsr_speed_limit_valid_ = false;
    tsr_speed_limit_ = 0;
  }
  return;
}

void TsrCore::UpdateTsrWarning(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;

  // 更新overspeed_status_
  if (((vehicle_service_output_info_ptr->vehicle_speed_display * 3.6) >
       tsr_speed_limit_) &&
      (tsr_speed_limit_valid_ == true)) {
    overspeed_status_ = true;
  } else {
    overspeed_status_ = false;
  }

  // 更新overspeed_duration_time_
  if (overspeed_status_) {
    overspeed_duration_time_ += GetContext.get_param()->dt;
  } else {
    overspeed_duration_time_ = 0.0;
  }

  // 更新tsr_warning_image_
  if (overspeed_duration_time_ >= 1.5) {
    tsr_warning_image_ = true;
  } else {
    tsr_warning_image_ = false;
  }

  // 发起声音报警需要的确认时长
  double image2voice_confirm_time;
  if ((vehicle_service_output_info_ptr->vehicle_speed_display * 3.6) >=
      (tsr_speed_limit_ * 1.3)) {
    image2voice_confirm_time = 3.0;
  } else if ((vehicle_service_output_info_ptr->vehicle_speed_display * 3.6) >=
             (tsr_speed_limit_ * 1.2)) {
    image2voice_confirm_time = 4.0;
  } else if ((vehicle_service_output_info_ptr->vehicle_speed_display * 3.6) >=
             (tsr_speed_limit_ * 1.1)) {
    image2voice_confirm_time = 5.0;
  } else if ((vehicle_service_output_info_ptr->vehicle_speed_display * 3.6) >=
             (tsr_speed_limit_ * 1.0)) {
    image2voice_confirm_time = 6.0;
  } else {
    image2voice_confirm_time = 6.0;
  }

  // 更新tsr_warning_voice_
  if (tsr_warning_voice_ == false) {
    // 上一时刻未发起声音报警
    if (overspeed_duration_time_ >= image2voice_confirm_time) {
      tsr_warning_voice_ = true;
    } else {
      tsr_warning_voice_ = false;
    }
  } else {
    // do nothing
  }
  // 功能未激活时取消声音报警
  if (tsr_state_ !=
      iflyauto::TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_ACTIVE) {
    tsr_warning_voice_ = false;
  } else {
    // do nothing
  }
  // 未超速时取消声音报警
  if (overspeed_status_ == false) {
    tsr_warning_voice_ = false;
  } else {
    // do nothing
  }
  // 声音报警超过最大时长时取消声音报警
  if (overspeed_duration_time_ >= (image2voice_confirm_time + 5.0)) {
    tsr_warning_voice_ = false;
  } else {
    // do nothing
  }
  // 驾驶员未踩下油门踏板时取消声音报警
  if ((vehicle_service_output_info_ptr->accelerator_pedal_pos < 1.0) &&
      vehicle_service_output_info_ptr->accelerator_pedal_pos_available) {
    tsr_warning_voice_ = false;
  } else {
    // do nothing
  }
  if (vehicle_service_output_info_ptr->brake_pedal_pressed_available &&
      vehicle_service_output_info_ptr->brake_pedal_pressed) {
    tsr_warning_voice_ = false;
  } else {
    // do nothing
  }
  if (tsr_speed_limit_change_flag_ == true) {
    tsr_speed_limit_change_flag_ = false;
  }
  return;
}
void TsrCore::CalculatePathLengthAccumulated() {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  if (tsr_speed_limit_change_flag_ == true || tsr_speed_limit_valid_ == false) {
    accumulated_path_length_ = 0.0;
  } else {
    accumulated_path_length_ =
        accumulated_path_length_ +
        GetContext.get_state_info()->vehicle_speed * GetContext.get_param()->dt;
  }
  // accumulated_path_length_ = accumulated_path_length_;
  return;
}
void TsrCore::SetTsrOutputInfo() {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  GetContext.mutable_output_info()->tsr_output_info_.tsr_state_ = tsr_state_;
  if (tsr_state_ ==
      iflyauto::TSRFunctionFSMWorkState::TSR_FUNCTION_FSM_WORK_STATE_ACTIVE) {
    GetContext.mutable_output_info()->tsr_output_info_.tsr_speed_limit_ =
        tsr_speed_limit_;
    GetContext.mutable_output_info()->tsr_output_info_.tsr_warning_ =
        tsr_warning_image_;
  } else {
    GetContext.mutable_output_info()->tsr_output_info_.tsr_warning_ = false;
    GetContext.mutable_output_info()->tsr_output_info_.tsr_speed_limit_ = 0;
  }
  return;
}

void TsrCore::RunOnce(void) {
  // 更新tsr开关状态
  tsr_main_switch_ = UpdateTsrMainSwitch();

  // 更新tsr_enable_code_
  tsr_enable_code_ = UpdateTsrEnableCode();

  // 更新tsr_disable_code_
  tsr_disable_code_ = UpdateTsrDisableCode();

  // 更新tsr_fault_code_
  tsr_fault_code_ = UpdateTsrFaultCode();

  // 更新tsr_state_
  tsr_state_ = TsrStateMachine();

  // 更新tsr_speed_limit_
  UpdateTsrSpeedLimit();

  // 计算accumulated_path_length_
  CalculatePathLengthAccumulated();

  // 更新tsr_warning_image_&&tsr_warning_voice_
  UpdateTsrWarning();

  //output
  SetTsrOutputInfo();

  // log
  JSON_DEBUG_VALUE("tsr_main_switch_", tsr_main_switch_);
  JSON_DEBUG_VALUE("tsr_enable_code_", tsr_enable_code_);
  JSON_DEBUG_VALUE("tsr_disable_code_", tsr_disable_code_);
  JSON_DEBUG_VALUE("tsr_fault_code_", tsr_fault_code_);
  JSON_DEBUG_VALUE("tsr_state_", (int)tsr_state_);
  JSON_DEBUG_VALUE("tsr_speed_limit_", tsr_speed_limit_);

  JSON_DEBUG_VALUE("tsr_speed_limit_valid_", tsr_speed_limit_valid_);
  JSON_DEBUG_VALUE("tsr_warning_image_", tsr_warning_image_);
  JSON_DEBUG_VALUE("tsr_warning_voice_", tsr_warning_voice_);
  JSON_DEBUG_VALUE("tsr_overspeed_status_", overspeed_status_);
  JSON_DEBUG_VALUE("tsr_overspeed_duration_time_", overspeed_duration_time_);
  JSON_DEBUG_VALUE("tsr_speed_limit_change_flag_",
                   tsr_speed_limit_change_flag_);
  JSON_DEBUG_VALUE("tsr_speed_limit_exist_in_view_flag_",
                   speed_limit_exist_in_view_flag_);
  JSON_DEBUG_VALUE("tsr_speed_limit_exist_in_view_",
                   speed_limit_exist_in_view_);
  JSON_DEBUG_VALUE("tsr_accumulated_path_length_", accumulated_path_length_);
}

}  // namespace tsr_core
}  // namespace adas_function