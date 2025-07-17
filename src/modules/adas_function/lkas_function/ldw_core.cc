#include "ldw_core.h"

#include <iostream>

#include "adas_function_lib.h"
#include "planning_hmi_c.h"

namespace adas_function {
namespace ldw_core {

bool LdwCore::UpdateLdwMainSwitch(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  auto function_state_machine_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .function_state_machine_info;
  if (GetContext.get_param()->ldw_main_switch) {
    return GetContext.get_param()->ldw_main_switch;
  }
  return function_state_machine_info_ptr->switch_sts.ldp_main_switch;
}

uint16 LdwCore::UpdateLdwEnableCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;

  uint16 enable_code = 0;

  // bit 0
  // 判断车速是否处于工作车速范围内
  if (vehicle_service_output_info_ptr->vehicle_speed_display <
      ldw_param_.enable_vehspd_display_min) {
    enable_code += uint16_bit[0];
  } else if (vehicle_service_output_info_ptr->vehicle_speed_display >
             ldw_param_.enable_vehspd_display_max) {
    enable_code += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // bit 1
  // 判断是否至少识别到一侧道线或一侧路沿
  if ((GetContext.get_road_info()->current_lane.left_line.valid == false) &&
      (GetContext.get_road_info()->current_lane.right_line.valid == false)) {
    enable_code += uint16_bit[1];
  } else {
    /*do nothing*/
  }

  // bit 2
  // 判断当前车道宽度是否满足激活条件
  if (GetContext.mutable_road_info()->current_lane.lane_width < 2.8) {
    enable_code += uint16_bit[2];
  } else if (GetContext.mutable_road_info()->current_lane.lane_width > 4.5) {
    enable_code += uint16_bit[2];
  } else {
    /*do nothing*/
  }

  return enable_code;
}

uint16 LdwCore::UpdateLdwDisableCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;

  uint16 disable_code = 0;

  // bit 0
  // 判断车速是否处于工作车速范围内
  if (vehicle_service_output_info_ptr->vehicle_speed_display <
      ldw_param_.disable_vehspd_display_min) {
    disable_code += uint16_bit[0];
  } else if (vehicle_service_output_info_ptr->vehicle_speed_display >
             ldw_param_.disable_vehspd_display_max) {
    disable_code += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // bit 1
  // 判断是否至少识别到一侧道线或一侧路沿
  if ((GetContext.get_road_info()->current_lane.left_line.valid == false) &&
      (GetContext.get_road_info()->current_lane.right_line.valid == false)) {
    disable_code += uint16_bit[1];
  } else {
    /*do nothing*/
  }

  // bit 2
  // 判断当前车道宽度是否满足激活条件
  if (GetContext.mutable_road_info()->current_lane.lane_width < 2.5) {
    disable_code += uint16_bit[2];
  } else if (GetContext.mutable_road_info()->current_lane.lane_width > 4.8) {
    disable_code += uint16_bit[2];
  } else {
    /*do nothing*/
  }

  return disable_code;
}

uint16 LdwCore::UpdateLdwFaultCode(void) {
  uint16 ldw_fault_code = 0;
  return ldw_fault_code;
}

uint16 LdwCore::UpdateLdwLeftSuppressionCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto function_state_machine_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .function_state_machine_info;

  uint16 ldw_left_suppression_code = 0;

  // left_line_valid
  bool left_line_valid;
  if (GetContext.get_road_info()->current_lane.left_line.valid == true) {
    left_line_valid = true;
  } else {
    left_line_valid = false;
  }

  if ((ldw_state_ ==
       iflyauto::LDWFunctionFSMWorkState::
           LDW_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION) ||
      (GetContext.get_road_info()->current_lane.lane_changed_flag ==
       true)) {  // 检测到左侧报警
    left_suppress_repeat_warning_flag_ = true;
  } else {  // 检测到左侧报警结束
    if ((left_suppress_repeat_warning_flag_ == true) &&
        (left_line_valid == true) &&
        (GetContext.mutable_state_info()->fl_wheel_distance_to_line >
         ldw_param_.reset_warning_line)) {
      // 左侧边界线有效&&检测到越过了重置线
      left_suppress_repeat_warning_flag_ = false;
    } else {
      /*do nothing*/
    }
  }

  // Condition1
  // 判断左转向灯处于关闭时长是否满足
  if (GetContext.mutable_state_info()->left_turn_light_off_time <
      ldw_param_.supp_turn_light_recovery_time) {
    ldw_left_suppression_code += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // Condition2
  // 判断是否处于允许左侧报警区域内
  if (left_line_valid == true) {
    if (GetContext.mutable_state_info()->fl_wheel_distance_to_line < 0.0) {
      ldw_left_suppression_code += uint16_bit[1];
    } else if (GetContext.mutable_state_info()->fl_wheel_distance_to_line >
               ldw_param_.earliest_warning_line) {
      ldw_left_suppression_code += uint16_bit[1];
    } else {
      /*do nothing*/
    }
  } else {
    ldw_left_suppression_code += uint16_bit[1];
  }

  // Condition3
  // 距上次触发报警后,越过了重置线
  if (left_suppress_repeat_warning_flag_ == true) {
    ldw_left_suppression_code += uint16_bit[2];
  } else {
    /*do nothing*/
  }

  // Condition4
  if (((function_state_machine_info_ptr->current_state >=
        iflyauto::FunctionalState_SCC_ACTIVATE) &&
       (function_state_machine_info_ptr->current_state <=
        iflyauto::FunctionalState_SCC_OVERRIDE)) ||
      ((function_state_machine_info_ptr->current_state >=
        iflyauto::FunctionalState_NOA_ACTIVATE) &&
       (function_state_machine_info_ptr->current_state <=
        iflyauto::FunctionalState_NOA_OVERRIDE))) {
    ldw_left_suppression_code += uint16_bit[3];
  }
  if (GetContext.get_road_info()->current_lane.left_line.line_type ==
      context::Enum_LineType::Enum_LineType_Virtual) {
    ldw_left_suppression_code += uint16_bit[4];
  } else {
    /*do nothing*/
  }

  if (GetContext.get_road_info()->current_lane.left_sideway_exist_flag &&
      (GetContext.get_param()->force_no_sideway_switch == false)) {
    // 左侧有分流口
    // ldw_left_suppression_code += uint16_bit[5];
  }

  if (GetContext.get_road_info()
          ->current_lane.left_safe_departure_permission_flag == true) {
    // 右侧有并行车或右前方一定区域内有车、或者正前方一定区域内有车，且有碰撞风险
    ldw_left_suppression_code += uint16_bit[6];
  }

  return ldw_left_suppression_code;
}

uint16 LdwCore::UpdateLdwLeftKickDownCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;

  auto function_state_machine_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .function_state_machine_info;

  uint16 ldw_left_kickdown_code = 0;

  // left_line_valid
  bool left_line_valid;
  if (GetContext.mutable_road_info()->current_lane.left_line.valid == true) {
    left_line_valid = true;
  } else {
    left_line_valid = false;
  }

  if (ldw_state_ == iflyauto::LDWFunctionFSMWorkState::
                        LDW_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION) {
    ldw_left_warning_time_ += GetContext.mutable_param()->dt;
    if (ldw_left_warning_time_ >= 60.0) {
      ldw_left_warning_time_ = 60.0;
    } else {
      /*do nothing*/
    }
  } else {
    ldw_left_warning_time_ = 0.0;
  }

  // Condition1
  if (vehicle_service_output_info_ptr->left_turn_light_state == true) {
    ldw_left_kickdown_code += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // Condition2
  // 判断是否处于允许左侧报警区域内
  if (left_line_valid == true) {
    if (GetContext.mutable_state_info()->fl_wheel_distance_to_line <
        ldw_param_.latest_warning_line) {
      ldw_left_kickdown_code += uint16_bit[1];
    } else if (GetContext.mutable_state_info()->fl_wheel_distance_to_line >
               ldw_param_.earliest_warning_line) {
      ldw_left_kickdown_code += uint16_bit[1];
    } else {
      /*do nothing*/
    }
  } else {
    ldw_left_kickdown_code += uint16_bit[1];
  }

  // Condition3
  if (ldw_left_warning_time_ > ldw_param_.warning_time_max) {
    ldw_left_kickdown_code += uint16_bit[2];
  }

  // Condition4
  if (ldw_left_intervention_ == false) {
    ldw_left_kickdown_code += uint16_bit[3];
  }

  // Condition5
  if (((function_state_machine_info_ptr->current_state >=
        iflyauto::FunctionalState_SCC_ACTIVATE) &&
       (function_state_machine_info_ptr->current_state <=
        iflyauto::FunctionalState_SCC_OVERRIDE)) ||
      ((function_state_machine_info_ptr->current_state >=
        iflyauto::FunctionalState_NOA_ACTIVATE) &&
       (function_state_machine_info_ptr->current_state <=
        iflyauto::FunctionalState_NOA_OVERRIDE))) {
    ldw_left_kickdown_code += uint16_bit[3];
  }

  if (GetContext.get_road_info()->current_lane.left_line.line_type ==
      context::Enum_LineType::Enum_LineType_Virtual) {
    ldw_left_kickdown_code += uint16_bit[4];
  } else {
    /*do nothing*/
  }
  return ldw_left_kickdown_code;
}

uint16 LdwCore::UpdateLdwRightSuppressionCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto function_state_machine_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .function_state_machine_info;

  uint16 ldw_right_suppression_code = 0;

  // right_line_valid
  bool right_line_valid;
  if (GetContext.mutable_road_info()->current_lane.right_line.valid == true) {
    right_line_valid = true;
  } else {
    right_line_valid = false;
  }

  if ((ldw_state_ ==
       iflyauto::LDWFunctionFSMWorkState::
           LDW_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION) ||
      (GetContext.get_road_info()->current_lane.lane_changed_flag ==
       true)) {  // 检测到右侧报警
    right_suppress_repeat_warning_flag_ = true;
  } else {  // 检测到右侧报警结束
    if ((right_suppress_repeat_warning_flag_ == true) &&
        (right_line_valid == true) &&
        (GetContext.mutable_state_info()->fr_wheel_distance_to_line <
         (-1.0 * ldw_param_.reset_warning_line))) {
      // 右侧边界线有效&&检测到越过了重置线
      right_suppress_repeat_warning_flag_ = false;
    } else {
      /*do nothing*/
    }
  }

  // Condition1
  // 判断右转向灯处于关闭时长是否满足
  if (GetContext.mutable_state_info()->right_turn_light_off_time <
      ldw_param_.supp_turn_light_recovery_time) {
    ldw_right_suppression_code += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // Condition2
  // 判断是否处于允许右侧报警区域内
  if (right_line_valid == true) {
    if (GetContext.mutable_state_info()->fr_wheel_distance_to_line > 0.0) {
      ldw_right_suppression_code += uint16_bit[1];
    } else if (GetContext.mutable_state_info()->fr_wheel_distance_to_line <
               (-1.0 * ldw_param_.earliest_warning_line)) {
      ldw_right_suppression_code += uint16_bit[1];
    } else {
      /*do nothing*/
    }
  } else {
    ldw_right_suppression_code += uint16_bit[1];
  }

  // Condition3
  // 距上次触发报警后,越过了重置线
  if (right_suppress_repeat_warning_flag_ == true) {
    ldw_right_suppression_code += uint16_bit[2];
  } else {
    /*do nothing*/
  }

  // Condition4
  if (((function_state_machine_info_ptr->current_state >=
        iflyauto::FunctionalState_SCC_ACTIVATE) &&
       (function_state_machine_info_ptr->current_state <=
        iflyauto::FunctionalState_SCC_OVERRIDE)) ||
      ((function_state_machine_info_ptr->current_state >=
        iflyauto::FunctionalState_NOA_ACTIVATE) &&
       (function_state_machine_info_ptr->current_state <=
        iflyauto::FunctionalState_NOA_OVERRIDE))) {
    ldw_right_suppression_code += uint16_bit[3];
  }

  if (GetContext.get_road_info()->current_lane.right_line.line_type ==
      context::Enum_LineType::Enum_LineType_Virtual) {
    ldw_right_suppression_code += uint16_bit[4];
  } else {
    /*do nothing*/
  }

  if (GetContext.get_road_info()->current_lane.right_sideway_exist_flag &&
      (GetContext.get_param()->force_no_sideway_switch == false)) {
    // 右侧有分流口
    // ldw_right_suppression_code += uint16_bit[5];
  }

  if (GetContext.get_road_info()
          ->current_lane.right_safe_departure_permission_flag == true) {
    // 左侧有并行车或左前方一定区域内有车、或者正前方一定区域内有车，且有碰撞风险
    ldw_right_suppression_code += uint16_bit[6];
  }

  return ldw_right_suppression_code;
}

uint16 LdwCore::UpdateLdwRightKickDownCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;

  auto function_state_machine_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .function_state_machine_info;

  uint16 ldw_right_kickdown_code = 0;

  // right_line_valid
  bool right_line_valid;
  if (GetContext.mutable_road_info()->current_lane.right_line.valid == true) {
    right_line_valid = true;
  } else {
    right_line_valid = false;
  }

  if (ldw_state_ == iflyauto::LDWFunctionFSMWorkState::
                        LDW_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION) {
    ldw_right_warning_time_ += GetContext.mutable_param()->dt;
    if (ldw_right_warning_time_ >= 60.0) {
      ldw_right_warning_time_ = 60.0;
    } else {
      /*do nothing*/
    }
  } else {
    ldw_right_warning_time_ = 0.0;
  }

  // Condition1
  if (vehicle_service_output_info_ptr->right_turn_light_state == true) {
    ldw_right_kickdown_code += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // Condition2
  // 判断是否处于允许右侧报警区域内
  if (right_line_valid == true) {
    if (GetContext.mutable_state_info()->fr_wheel_distance_to_line >
        (-1.0 * ldw_param_.latest_warning_line)) {
      ldw_right_kickdown_code += uint16_bit[1];
    } else if (GetContext.mutable_state_info()->fr_wheel_distance_to_line <
               (-1.0 * ldw_param_.earliest_warning_line)) {
      ldw_right_kickdown_code += uint16_bit[1];
    } else {
      /*do nothing*/
    }
  } else {
    ldw_right_kickdown_code += uint16_bit[1];
  }

  // Condition3
  if (ldw_right_warning_time_ > ldw_param_.warning_time_max) {
    ldw_right_kickdown_code += uint16_bit[2];
  }

  // Condition4
  if (ldw_right_intervention_ == false) {
    ldw_right_kickdown_code += uint16_bit[3];
  }

  // Condition5
  if (((function_state_machine_info_ptr->current_state >=
        iflyauto::FunctionalState_SCC_ACTIVATE) &&
       (function_state_machine_info_ptr->current_state <=
        iflyauto::FunctionalState_SCC_OVERRIDE)) ||
      ((function_state_machine_info_ptr->current_state >=
        iflyauto::FunctionalState_NOA_ACTIVATE) &&
       (function_state_machine_info_ptr->current_state <=
        iflyauto::FunctionalState_NOA_OVERRIDE))) {
    ldw_right_kickdown_code += uint16_bit[3];
  }

  if (GetContext.get_road_info()->current_lane.right_line.line_type ==
      context::Enum_LineType::Enum_LineType_Virtual) {
    ldw_right_kickdown_code += uint16_bit[4];
  } else {
    /*do nothing*/
  }

  return ldw_right_kickdown_code;
}

double LdwCore::UpdateTlcThreshold(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  auto hmi_mcu_inner_info_ptr = &GetContext.mutable_session()
                                     ->mutable_environmental_model()
                                     ->get_local_view()
                                     .hmi_mcu_inner_info;
  if (GetContext.get_param()->ldw_tlc_thrd > 0.1) {
    return GetContext.get_param()->ldw_tlc_thrd;
  } else {
    if (hmi_mcu_inner_info_ptr->ldw_set_sensitivity_level ==
        iflyauto::interface_2_4_5::SensitivityLevel::SENSITIVITY_LEVEL_LOW) {
      return ldw_tlc_near_;
    } else if (hmi_mcu_inner_info_ptr->ldw_set_sensitivity_level ==
               iflyauto::interface_2_4_5::SensitivityLevel::
                   SENSITIVITY_LEVEL_MIDDLE) {
      return ldw_tlc_medium_;
    } else if (hmi_mcu_inner_info_ptr->ldw_set_sensitivity_level ==
               iflyauto::interface_2_4_5::SensitivityLevel::
                   SENSITIVITY_LEVEL_HIGH) {
      return ldw_tlc_far_;
    } else {
      return ldw_tlc_medium_;
    }
  }
}

iflyauto::LDWFunctionFSMWorkState LdwCore::LdwStateMachine(void) {
  iflyauto::LDWFunctionFSMWorkState ldw_state;

  if (ldw_state_machine_init_flag_ == false) {
    // 状态机处于初始化状态 根据开关状态,决定第一个周期是输出OFF还是STANDBY
    ldw_state_machine_init_flag_ = true;
    if (ldw_main_switch_ == false) {
      ldw_state =
          iflyauto::LDWFunctionFSMWorkState::LDW_FUNCTION_FSM_WORK_STATE_OFF;
    } else {
      ldw_state = iflyauto::LDWFunctionFSMWorkState::
          LDW_FUNCTION_FSM_WORK_STATE_STANDBY;
    }
    return ldw_state;
  }

  // 状态机处于完成过初始化的状态
  if (ldw_state_ == iflyauto::LDWFunctionFSMWorkState::
                        LDW_FUNCTION_FSM_WORK_STATE_UNAVAILABLE) {
    // 上一时刻处于LDW_FUNCTION_FSM_WORK_STATE_UNAVAILABLE状态
    if (ldw_main_switch_ == false) {
      ldw_state =
          iflyauto::LDWFunctionFSMWorkState::LDW_FUNCTION_FSM_WORK_STATE_OFF;
    } else if (ldw_fault_code_ == 0) {
      ldw_state = iflyauto::LDWFunctionFSMWorkState::
          LDW_FUNCTION_FSM_WORK_STATE_STANDBY;
    } else {
      // do nothing
    }
  } else if (ldw_state_ == iflyauto::LDWFunctionFSMWorkState::
                               LDW_FUNCTION_FSM_WORK_STATE_OFF) {
    // 上一时刻处于LDW_FUNCTION_FSM_WORK_STATE_OFF状态
    if (ldw_main_switch_ == true) {
      ldw_state = iflyauto::LDWFunctionFSMWorkState::
          LDW_FUNCTION_FSM_WORK_STATE_STANDBY;
    } else {
      ldw_state =
          iflyauto::LDWFunctionFSMWorkState::LDW_FUNCTION_FSM_WORK_STATE_OFF;
    }
  } else if (ldw_state_ == iflyauto::LDWFunctionFSMWorkState::
                               LDW_FUNCTION_FSM_WORK_STATE_STANDBY) {
    // 上一时刻处于LDW_FUNCTION_FSM_WORK_STATE_STANDBY状态
    if (ldw_main_switch_ == false) {
      ldw_state =
          iflyauto::LDWFunctionFSMWorkState::LDW_FUNCTION_FSM_WORK_STATE_OFF;
    } else if (ldw_fault_code_) {
      ldw_state = iflyauto::LDWFunctionFSMWorkState::
          LDW_FUNCTION_FSM_WORK_STATE_UNAVAILABLE;
    } else if (ldw_enable_code_ == 0) {
      ldw_state = iflyauto::LDWFunctionFSMWorkState::
          LDW_FUNCTION_FSM_WORK_STATE_ACTIVE_NO_INTERVENTION;
    } else {
      ldw_state = ldw_state_;
    }
  } else if ((ldw_state_ ==
              iflyauto::LDWFunctionFSMWorkState::
                  LDW_FUNCTION_FSM_WORK_STATE_ACTIVE_NO_INTERVENTION) ||
             (ldw_state_ ==
              iflyauto::LDWFunctionFSMWorkState::
                  LDW_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION) ||
             (ldw_state_ ==
              iflyauto::LDWFunctionFSMWorkState::
                  LDW_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION)) {
    // 上一时刻处于LDW_FUNCTION_FSM_WORK_STATE_ACTIVE状态
    if (ldw_main_switch_ == false) {
      ldw_state =
          iflyauto::LDWFunctionFSMWorkState::LDW_FUNCTION_FSM_WORK_STATE_OFF;
    } else if (ldw_fault_code_) {
      ldw_state = iflyauto::LDWFunctionFSMWorkState::
          LDW_FUNCTION_FSM_WORK_STATE_UNAVAILABLE;
    } else if (ldw_disable_code_) {
      ldw_state = iflyauto::LDWFunctionFSMWorkState::
          LDW_FUNCTION_FSM_WORK_STATE_STANDBY;
    } else if (ldw_state_ ==
               iflyauto::LDWFunctionFSMWorkState::
                   LDW_FUNCTION_FSM_WORK_STATE_ACTIVE_NO_INTERVENTION) {
      if (ldw_left_intervention_ && (ldw_left_suppression_code_ == 0)) {
        ldw_state = iflyauto::LDWFunctionFSMWorkState::
            LDW_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION;
      } else if (ldw_right_intervention_ &&
                 (ldw_right_suppression_code_ == 0)) {
        ldw_state = iflyauto::LDWFunctionFSMWorkState::
            LDW_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION;
      } else {
        ldw_state = ldw_state_;
      }
    } else if (ldw_state_ ==
               iflyauto::LDWFunctionFSMWorkState::
                   LDW_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION) {
      if (ldw_left_kickdown_code_) {
        ldw_state = iflyauto::LDWFunctionFSMWorkState::
            LDW_FUNCTION_FSM_WORK_STATE_ACTIVE_NO_INTERVENTION;
      } else {
        ldw_state = ldw_state_;
      }
    } else if (ldw_state_ ==
               iflyauto::LDWFunctionFSMWorkState::
                   LDW_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION) {
      if (ldw_right_kickdown_code_) {
        ldw_state = iflyauto::LDWFunctionFSMWorkState::
            LDW_FUNCTION_FSM_WORK_STATE_ACTIVE_NO_INTERVENTION;
      } else {
        ldw_state = ldw_state_;
      }
    } else {
      ldw_state = ldw_state_;
    }
  } else {
    // 处于异常状态
    ldw_state =
        iflyauto::LDWFunctionFSMWorkState::LDW_FUNCTION_FSM_WORK_STATE_OFF;
  }
  return ldw_state;
}

void LdwCore::SetLdwOutputInfo() {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  GetContext.mutable_output_info()->ldw_output_info_.ldw_state_ = ldw_state_;
  if (ldw_state_ == iflyauto::LDWFunctionFSMWorkState::
                        LDW_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION) {
    GetContext.mutable_output_info()->ldw_output_info_.ldw_left_warning_ = true;
  } else {
    GetContext.mutable_output_info()->ldw_output_info_.ldw_left_warning_ =
        false;
  }
  if (ldw_state_ == iflyauto::LDWFunctionFSMWorkState::
                        LDW_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION) {
    GetContext.mutable_output_info()->ldw_output_info_.ldw_right_warning_ =
        true;
  } else {
    GetContext.mutable_output_info()->ldw_output_info_.ldw_right_warning_ =
        false;
  }
  return;
}

void LdwCore::RunOnce(void) {
  // 更新Ldw开关状态
  ldw_main_switch_ = UpdateLdwMainSwitch();

  // 更新ldw_enable_code_
  ldw_enable_code_ = UpdateLdwEnableCode();

  // 更新ldw_disable_code_
  ldw_disable_code_ = UpdateLdwDisableCode();

  // 更新ldw_fault_code_
  ldw_fault_code_ = UpdateLdwFaultCode();

  // 更新ldw_left_suppression_code_
  ldw_left_suppression_code_ = UpdateLdwLeftSuppressionCode();

  // 更新ldw_left_kickdown_code_
  ldw_left_kickdown_code_ = UpdateLdwLeftKickDownCode();

  // 更新ldw_right_suppression_code_
  ldw_right_suppression_code_ = UpdateLdwRightSuppressionCode();

  // 更新ldw_right_kickdown_code_
  ldw_right_kickdown_code_ = UpdateLdwRightKickDownCode();

  // 更新tlc_to_line_threshold_
  ldw_tlc_threshold_ = UpdateTlcThreshold();

  // 更新ldw_left_intervention_
  double preview_left_y_gap =
      adas_function::LkasLineLeftIntervention(ldw_tlc_threshold_);
  if (preview_left_y_gap < 0.0) {
    ldw_left_intervention_ = true;
  } else {
    ldw_left_intervention_ = false;
  }

  // 更新ldw_right_intervention_
  double preview_right_y_gap =
      adas_function::LkasLineRightIntervention(ldw_tlc_threshold_);
  if (preview_right_y_gap > 0.0) {
    ldw_right_intervention_ = true;
  } else {
    ldw_right_intervention_ = false;
  }

  ldw_state_ = LdwStateMachine();

  // 输出ldw计算结果
  SetLdwOutputInfo();

  // log
  JSON_DEBUG_VALUE("ldw_main_switch_", ldw_main_switch_);
  JSON_DEBUG_VALUE("ldw_enable_code_", ldw_enable_code_);
  JSON_DEBUG_VALUE("ldw_disable_code_", ldw_disable_code_);
  JSON_DEBUG_VALUE("ldw_fault_code_", ldw_fault_code_);
  JSON_DEBUG_VALUE("ldw_left_suppression_code_", ldw_left_suppression_code_);
  JSON_DEBUG_VALUE("ldw_left_kickdown_code_", ldw_left_kickdown_code_);
  JSON_DEBUG_VALUE("ldw_right_suppression_code_", ldw_right_suppression_code_);
  JSON_DEBUG_VALUE("ldw_right_kickdown_code_", ldw_right_kickdown_code_);
  JSON_DEBUG_VALUE("ldw_tlc_threshold_", ldw_tlc_threshold_);
  JSON_DEBUG_VALUE("ldw_left_intervention_", ldw_left_intervention_);
  JSON_DEBUG_VALUE("ldw_right_intervention_", ldw_right_intervention_);
  JSON_DEBUG_VALUE("ldw_state_", (int)ldw_state_);
  JSON_DEBUG_VALUE("ldw_preview_left_y_gap", preview_left_y_gap);
  JSON_DEBUG_VALUE("ldw_preview_right_y_gap", preview_right_y_gap);
}
}  // namespace ldw_core
}  // namespace adas_function