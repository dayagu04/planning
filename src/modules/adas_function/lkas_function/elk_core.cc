#include "elk_core.h"

#include <iostream>

#include "adas_function_lib.h"
#include "common_platform_type_soc.h"
#include "debug_info_log.h"
#include "planning_hmi_c.h"

namespace adas_function {
namespace elk_core {

bool ElkCore::UpdateElkMainSwitch(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  auto function_state_machine_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .function_state_machine_info;
  if (GetContext.get_param()->elk_main_switch) {
    return GetContext.get_param()->elk_main_switch;
  }
  return function_state_machine_info_ptr->switch_sts.elk_main_switch;
}

uint16 ElkCore::UpdateElkEnableCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;

  uint16 enable_code = 0;

  // bit 0
  // 判断车速是否处于工作车速范围内
  if (vehicle_service_output_info_ptr->vehicle_speed_display <
      elk_param_.enable_vehspd_display_min) {
    enable_code += uint16_bit[0];
  } else if (vehicle_service_output_info_ptr->vehicle_speed_display >
             elk_param_.enable_vehspd_display_max) {
    enable_code += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // bit 1
  // 判断是否至少识别到一侧道线或一侧路沿
  if ((!GetContext.mutable_road_info()->current_lane.left_line.valid) &&
      (!GetContext.mutable_road_info()->current_lane.right_line.valid) &&
      (!GetContext.get_road_info()->current_lane.left_roadedge.valid) &&
      (!GetContext.get_road_info()->current_lane.right_roadedge.valid)) {
    enable_code += uint16_bit[1];
  } else {
    /*do nothing*/
  }

  // bit 2
  // 判断当前车道宽度是否满足激活条件
  if (GetContext.get_road_info()->current_lane.lane_width_valid) {
    if ((GetContext.mutable_road_info()->current_lane.lane_width < 2.8 ||
         GetContext.mutable_road_info()->current_lane.lane_width > 4.5)) {
      enable_code += uint16_bit[2];
    } else {
      /*do nothing*/
    }
  } else {
    /*当一侧既没有路沿也没有道线时，不作道宽判断*/
  }

  return enable_code;
}

uint16 ElkCore::UpdateElkDisableCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;

  uint16 disable_code = 0;

  // bit 0
  // 判断车速是否处于工作车速范围内
  if (vehicle_service_output_info_ptr->vehicle_speed_display <
      elk_param_.disable_vehspd_display_min) {
    disable_code += uint16_bit[0];
  } else if (vehicle_service_output_info_ptr->vehicle_speed_display >
             elk_param_.disable_vehspd_display_max) {
    disable_code += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // bit 1
  // 判断是否至少识别到一侧道线或一侧路沿
  if ((!GetContext.mutable_road_info()->current_lane.left_line.valid) &&
      (!GetContext.mutable_road_info()->current_lane.right_line.valid) &&
      (!GetContext.get_road_info()->current_lane.left_roadedge.valid) &&
      (!GetContext.get_road_info()->current_lane.right_roadedge.valid)) {
    disable_code += uint16_bit[1];
  } else {
    /*do nothing*/
  }

  // bit 2
  // 判断当前车道宽度是否满足激活条件
  if (GetContext.get_road_info()->current_lane.lane_width_valid) {
    if ((GetContext.mutable_road_info()->current_lane.lane_width < 2.8 ||
         GetContext.mutable_road_info()->current_lane.lane_width > 4.5)) {
      disable_code += uint16_bit[2];
    } else {
      /*do nothing*/
    }
  } else {
    /*当一侧既没有路沿也没有道线时，不作道宽判断*/
  }

  return disable_code;
}

uint16 ElkCore::UpdateElkFaultCode(void) {
  uint16 elk_fault_code = 0;
  return elk_fault_code;
}

uint16 ElkCore::UpdateElkLeftSuppressionCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto function_state_machine_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .function_state_machine_info;

  uint16 elk_left_suppression_code = 0;

  if ((elk_state_ ==
       iflyauto::ELKFunctionFSMWorkState::
           ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION) ||
      (GetContext.get_road_info()->current_lane.lane_changed_flag ==
       true)) {  // 检测到左侧报警
    left_suppress_repeat_warning_flag_ = true;
  } else {  // 检测到左侧报警结束
    if (left_suppress_repeat_warning_flag_ == true) {
      if ((GetContext.get_road_info()->current_lane.left_line.valid == true) &&
          (GetContext.mutable_state_info()->fl_wheel_distance_to_line >
           elk_param_.reset_warning_line)) {
        // 左侧边界线有效&&检测到越过了重置线
        left_suppress_repeat_warning_flag_ = false;
      } else if ((GetContext.get_road_info()
                      ->current_lane.left_roadedge.valid == true) &&
                 (GetContext.mutable_state_info()
                      ->fl_wheel_distance_to_roadedge >
                  elk_param_.roadedge_reset_warning_line)) {
        // 左侧路沿边界线有效&&检测到越过了重置线
        left_suppress_repeat_warning_flag_ = false;
      } else {
        /*do nothing*/
      }
    } else {
      /*do nothing*/
    }
  }

  // Condition1
  // 判断左转向灯处于关闭时长是否满足
  if (GetContext.mutable_state_info()->left_turn_light_off_time <
      elk_param_.supp_turn_light_recovery_time) {
    // elk_left_suppression_code += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // Condition2
  // 判断是否处于道线允许左侧报警区域内
  bool in_left_line_aera_flag = false;
  bool in_left_roadedge_aera_flag = false;
  if ((GetContext.get_road_info()->current_lane.left_line.valid == true) &&
      (GetContext.mutable_state_info()->fl_wheel_distance_to_line > 0.0) &&
      (GetContext.mutable_state_info()->fl_wheel_distance_to_line <
       elk_param_.earliest_warning_line)) {
    in_left_line_aera_flag = true;
  } else {
    in_left_line_aera_flag = false;
  }
  // 判断是否处于路沿允许左侧报警区域内
  if ((GetContext.get_road_info()->current_lane.left_roadedge.valid == true) &&
      (GetContext.mutable_state_info()->fl_wheel_distance_to_roadedge > 0.0) &&
      (GetContext.mutable_state_info()->fl_wheel_distance_to_roadedge <
       elk_param_.roadedge_earliest_warning_line)) {
    in_left_roadedge_aera_flag = true;
  } else {
    in_left_roadedge_aera_flag = false;
  }
  if ((in_left_line_aera_flag == false) &&
      (in_left_roadedge_aera_flag == false)) {
    elk_left_suppression_code += uint16_bit[1];
  }

  // Condition3
  // 距上次触发报警后,越过了重置线
  if (left_suppress_repeat_warning_flag_ == true) {
    elk_left_suppression_code += uint16_bit[2];
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
    elk_left_suppression_code += uint16_bit[3];
  }

  // Condition5
  // 驾驶员手力矩条件
  if (fabs(GetContext.mutable_state_info()->driver_hand_trq) >
      elk_param_.suppression_driver_hand_trq) {
    elk_left_suppression_code += uint16_bit[4];
  } else {
    /*do nothing*/
  }

  if (GetContext.get_road_info()->current_lane.left_line.line_type ==
      context::Enum_LineType::Enum_LineType_Virtual) {
    elk_left_suppression_code += uint16_bit[5];
  } else {
    /*do nothing*/
  }

  return elk_left_suppression_code;
}

uint16 ElkCore::UpdateElkLeftKickDownCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;

  auto function_state_machine_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .function_state_machine_info;

  uint16 elk_left_kickdown_code = 0;

  if (elk_state_ == iflyauto::ELKFunctionFSMWorkState::
                        ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION) {
    elk_left_warning_time_ += GetContext.mutable_param()->dt;
    if (elk_left_warning_time_ >= 60.0) {
      elk_left_warning_time_ = 60.0;
    } else {
      /*do nothing*/
    }
  } else {
    elk_left_warning_time_ = 0.0;
  }

  // Condition1
  if (vehicle_service_output_info_ptr->left_turn_light_state == true) {
    // elk_left_kickdown_code += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // Condition2
  // 判断是否处于允许左侧报警区域内
  bool in_left_line_aera_flag = false;
  bool in_left_roadedge_aera_flag = false;
  if ((GetContext.get_road_info()->current_lane.left_line.valid) &&
      (GetContext.mutable_state_info()->fl_wheel_distance_to_line >
       elk_param_.latest_warning_line) &&
      (GetContext.mutable_state_info()->fl_wheel_distance_to_line <
       elk_param_.earliest_warning_line)) {
    in_left_line_aera_flag = true;
  } else {
    in_left_line_aera_flag = false;
  }
  if ((GetContext.get_road_info()->current_lane.left_roadedge.valid) &&
      (GetContext.mutable_state_info()->fl_wheel_distance_to_roadedge >
       elk_param_.roadedge_latest_warning_line) &&
      (GetContext.mutable_state_info()->fl_wheel_distance_to_roadedge <
       elk_param_.roadedge_earliest_warning_line)) {
    in_left_roadedge_aera_flag = true;
  } else {
    in_left_roadedge_aera_flag = false;
  }
  if ((in_left_line_aera_flag == false) &&
      (in_left_roadedge_aera_flag == false)) {
    elk_left_kickdown_code += uint16_bit[1];
  }
  // Condition3
  if (elk_left_warning_time_ > elk_param_.warning_time_max) {
    elk_left_kickdown_code += uint16_bit[2];
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
    elk_left_kickdown_code += uint16_bit[3];
  }

  // Condition6
  // 驾驶员手力矩条件
  if (fabs(GetContext.mutable_state_info()->driver_hand_trq) >
      elk_param_.kickdown_driver_hand_trq) {
    elk_left_kickdown_code += uint16_bit[4];
  } else {
    /*do nothing*/
  }

  if (GetContext.get_road_info()->current_lane.left_line.line_type ==
      context::Enum_LineType::Enum_LineType_Virtual) {
    elk_left_kickdown_code += uint16_bit[5];
  } else {
    /*do nothing*/
  }

  return elk_left_kickdown_code;
}

uint16 ElkCore::UpdateElkRightSuppressionCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto function_state_machine_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .function_state_machine_info;

  uint16 elk_right_suppression_code = 0;

  if ((elk_state_ ==
       iflyauto::ELKFunctionFSMWorkState::
           ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION) ||
      (GetContext.get_road_info()->current_lane.lane_changed_flag ==
       true)) {  // 检测到右侧报警
    right_suppress_repeat_warning_flag_ = true;
  } else {  // 检测到右侧报警结束
    if (right_suppress_repeat_warning_flag_ == true) {
      if ((GetContext.get_road_info()->current_lane.right_line.valid == true) &&
          (GetContext.mutable_state_info()->fr_wheel_distance_to_line <
           (-1.0 * elk_param_.reset_warning_line))) {
        // 左侧边界线有效&&检测到越过了重置线
        right_suppress_repeat_warning_flag_ = false;
      } else if ((GetContext.get_road_info()
                      ->current_lane.right_roadedge.valid == true) &&
                 (GetContext.mutable_state_info()
                      ->fr_wheel_distance_to_roadedge <
                  (-1.0 * elk_param_.roadedge_reset_warning_line))) {
        // 左侧路沿边界线有效&&检测到越过了重置线
        right_suppress_repeat_warning_flag_ = false;
      } else {
        /*do nothing*/
      }
    } else {
      /*do nothing*/
    }
  }

  // Condition1
  // 判断右转向灯处于关闭时长是否满足
  if (GetContext.mutable_state_info()->right_turn_light_off_time <
      elk_param_.supp_turn_light_recovery_time) {
    // elk_right_suppression_code += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // Condition2
  // 判断是否处于允许右侧报警区域内
  bool in_right_line_aera_flag = false;
  bool in_right_roadedge_aera_flag = false;
  if ((GetContext.get_road_info()->current_lane.right_line.valid == true) &&
      (GetContext.mutable_state_info()->fr_wheel_distance_to_line < 0.0) &&
      (GetContext.mutable_state_info()->fr_wheel_distance_to_line >
       (-1.0 * elk_param_.earliest_warning_line))) {
    in_right_line_aera_flag = true;
  } else {
    in_right_line_aera_flag = false;
  }
  // 判断是否处于路沿允许右侧报警区域内
  if ((GetContext.get_road_info()->current_lane.right_roadedge.valid == true) &&
      (GetContext.mutable_state_info()->fr_wheel_distance_to_roadedge < 0.0) &&
      (GetContext.mutable_state_info()->fr_wheel_distance_to_roadedge >
       (-1.0 * elk_param_.roadedge_earliest_warning_line))) {
    in_right_roadedge_aera_flag = true;
  } else {
    in_right_roadedge_aera_flag = false;
  }
  if ((in_right_line_aera_flag == false) &&
      (in_right_roadedge_aera_flag == false)) {
    elk_right_suppression_code += uint16_bit[1];
  }

  // Condition3
  // 距上次触发报警后,越过了重置线
  if (right_suppress_repeat_warning_flag_ == true) {
    elk_right_suppression_code += uint16_bit[2];
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
    elk_right_suppression_code += uint16_bit[3];
  }

  // Condition5
  // 驾驶员手力矩条件
  if (fabs(GetContext.mutable_state_info()->driver_hand_trq) >
      elk_param_.suppression_driver_hand_trq) {
    elk_right_suppression_code += uint16_bit[4];
  } else {
    /*do nothing*/
  }

  if (GetContext.get_road_info()->current_lane.right_line.line_type ==
      context::Enum_LineType::Enum_LineType_Virtual) {
    elk_right_suppression_code += uint16_bit[5];
  } else {
    /*do nothing*/
  }

  return elk_right_suppression_code;
}

uint16 ElkCore::UpdateElkRightKickDownCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;

  auto function_state_machine_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .function_state_machine_info;

  uint16 elk_right_kickdown_code = 0;

  if (elk_state_ == iflyauto::ELKFunctionFSMWorkState::
                        ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION) {
    elk_right_warning_time_ += GetContext.mutable_param()->dt;
    if (elk_right_warning_time_ >= 60.0) {
      elk_right_warning_time_ = 60.0;
    } else {
      /*do nothing*/
    }
  } else {
    elk_right_warning_time_ = 0.0;
  }

  // Condition1
  if (vehicle_service_output_info_ptr->right_turn_light_state == true) {
    // elk_right_kickdown_code += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // Condition2
  // 判断是否处于允许右侧报警区域内
  bool in_right_line_aera_flag = false;
  bool in_right_roadedge_aera_flag = false;
  if ((GetContext.get_road_info()->current_lane.right_line.valid) &&
      (GetContext.mutable_state_info()->fr_wheel_distance_to_line <
       (-1.0 * elk_param_.latest_warning_line)) &&
      (GetContext.mutable_state_info()->fr_wheel_distance_to_line >
       (-1.0 * elk_param_.earliest_warning_line))) {
    in_right_line_aera_flag = true;
  } else {
    in_right_line_aera_flag = false;
  }
  if ((GetContext.get_road_info()->current_lane.right_roadedge.valid) &&
      (GetContext.mutable_state_info()->fr_wheel_distance_to_roadedge <
       (-1.0 * elk_param_.roadedge_latest_warning_line)) &&
      (GetContext.mutable_state_info()->fr_wheel_distance_to_roadedge >
       (-1.0 * elk_param_.roadedge_earliest_warning_line))) {
    in_right_roadedge_aera_flag = true;
  } else {
    in_right_roadedge_aera_flag = false;
  }
  if ((in_right_line_aera_flag == false) &&
      (in_right_roadedge_aera_flag == false)) {
    elk_right_kickdown_code += uint16_bit[1];
  }

  // Condition3
  if (elk_right_warning_time_ > elk_param_.warning_time_max) {
    elk_right_kickdown_code += uint16_bit[2];
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
    elk_right_kickdown_code += uint16_bit[3];
  }

  // Condition6
  // 驾驶员手力矩条件
  if (fabs(GetContext.mutable_state_info()->driver_hand_trq) >
      elk_param_.kickdown_driver_hand_trq) {
    elk_right_kickdown_code += uint16_bit[4];
  } else {
    /*do nothing*/
  }

  if (GetContext.get_road_info()->current_lane.right_line.line_type ==
      context::Enum_LineType::Enum_LineType_Virtual) {
    elk_right_kickdown_code += uint16_bit[5];
  } else {
    /*do nothing*/
  }
  return elk_right_kickdown_code;
}

iflyauto::ELKFunctionFSMWorkState ElkCore::ElkStateMachine(void) {
  iflyauto::ELKFunctionFSMWorkState elk_state;

  if (elk_state_machine_init_flag_ == false) {
    // 状态机处于初始化状态 根据开关状态,决定第一个周期是输出OFF还是STANDBY
    elk_state_machine_init_flag_ = true;
    if (elk_main_switch_ == false) {
      elk_state =
          iflyauto::ELKFunctionFSMWorkState::ELK_FUNCTION_FSM_WORK_STATE_OFF;
    } else {
      elk_state = iflyauto::ELKFunctionFSMWorkState::
          ELK_FUNCTION_FSM_WORK_STATE_STANDBY;
    }
    return elk_state;
  }

  // 状态机处于完成过初始化的状态
  if (elk_state_ == iflyauto::ELKFunctionFSMWorkState::
                        ELK_FUNCTION_FSM_WORK_STATE_UNAVAILABLE) {
    // 上一时刻处于ELK_FUNCTION_FSM_WORK_STATE_UNAVAILABLE状态
    if (elk_main_switch_ == false) {
      elk_state =
          iflyauto::ELKFunctionFSMWorkState::ELK_FUNCTION_FSM_WORK_STATE_OFF;
    } else if (elk_fault_code_ == 0) {
      elk_state = iflyauto::ELKFunctionFSMWorkState::
          ELK_FUNCTION_FSM_WORK_STATE_STANDBY;
    } else {
      // do nothing
    }
  } else if (elk_state_ == iflyauto::ELKFunctionFSMWorkState::
                               ELK_FUNCTION_FSM_WORK_STATE_OFF) {
    // 上一时刻处于ELK_FUNCTION_FSM_WORK_STATE_OFF状态
    if (elk_main_switch_ == true) {
      elk_state = iflyauto::ELKFunctionFSMWorkState::
          ELK_FUNCTION_FSM_WORK_STATE_STANDBY;
    } else {
      elk_state =
          iflyauto::ELKFunctionFSMWorkState::ELK_FUNCTION_FSM_WORK_STATE_OFF;
    }
  } else if (elk_state_ == iflyauto::ELKFunctionFSMWorkState::
                               ELK_FUNCTION_FSM_WORK_STATE_STANDBY) {
    // 上一时刻处于ELK_FUNCTION_FSM_WORK_STATE_STANDBY状态
    if (elk_main_switch_ == false) {
      elk_state =
          iflyauto::ELKFunctionFSMWorkState::ELK_FUNCTION_FSM_WORK_STATE_OFF;
    } else if (elk_fault_code_) {
      elk_state = iflyauto::ELKFunctionFSMWorkState::
          ELK_FUNCTION_FSM_WORK_STATE_UNAVAILABLE;
    } else if (elk_enable_code_ == 0) {
      elk_state = iflyauto::ELKFunctionFSMWorkState::
          ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_NO_INTERVENTION;
    } else {
      elk_state = elk_state_;
    }
  } else if ((elk_state_ ==
              iflyauto::ELKFunctionFSMWorkState::
                  ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_NO_INTERVENTION) ||
             (elk_state_ ==
              iflyauto::ELKFunctionFSMWorkState::
                  ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION) ||
             (elk_state_ ==
              iflyauto::ELKFunctionFSMWorkState::
                  ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION)) {
    // 上一时刻处于ELK_FUNCTION_FSM_WORK_STATE_ACTIVE状态
    if (elk_main_switch_ == false) {
      elk_state =
          iflyauto::ELKFunctionFSMWorkState::ELK_FUNCTION_FSM_WORK_STATE_OFF;
    } else if (elk_fault_code_) {
      elk_state = iflyauto::ELKFunctionFSMWorkState::
          ELK_FUNCTION_FSM_WORK_STATE_UNAVAILABLE;
    } else if (elk_disable_code_) {
      elk_state = iflyauto::ELKFunctionFSMWorkState::
          ELK_FUNCTION_FSM_WORK_STATE_STANDBY;
    } else if (elk_state_ ==
               iflyauto::ELKFunctionFSMWorkState::
                   ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_NO_INTERVENTION) {
      if (elk_left_intervention_ && (elk_left_suppression_code_ == 0)) {
        elk_state = iflyauto::ELKFunctionFSMWorkState::
            ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION;
      } else if (elk_right_intervention_ &&
                 (elk_right_suppression_code_ == 0)) {
        elk_state = iflyauto::ELKFunctionFSMWorkState::
            ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION;
      } else {
        elk_state = elk_state_;
      }
    } else if (elk_state_ ==
               iflyauto::ELKFunctionFSMWorkState::
                   ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION) {
      if (elk_left_kickdown_code_) {
        elk_state = iflyauto::ELKFunctionFSMWorkState::
            ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_NO_INTERVENTION;
      } else {
        elk_state = elk_state_;
      }
    } else if (elk_state_ ==
               iflyauto::ELKFunctionFSMWorkState::
                   ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION) {
      if (elk_right_kickdown_code_) {
        elk_state = iflyauto::ELKFunctionFSMWorkState::
            ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_NO_INTERVENTION;
      } else {
        elk_state = elk_state_;
      }
    } else {
      elk_state = elk_state_;
    }
  } else {
    // 处于异常状态
    elk_state =
        iflyauto::ELKFunctionFSMWorkState::ELK_FUNCTION_FSM_WORK_STATE_OFF;
  }
  return elk_state;
}

void ElkCore::SetElkOutputInfo() {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  GetContext.mutable_output_info()->elk_output_info_.elk_state_ = elk_state_;
  if (elk_state_ == iflyauto::ELKFunctionFSMWorkState::
                        ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION) {
    GetContext.mutable_output_info()
        ->elk_output_info_.elk_left_intervention_flag_ = true;
  } else {
    GetContext.mutable_output_info()
        ->elk_output_info_.elk_left_intervention_flag_ = false;
  }
  if (elk_state_ == iflyauto::ELKFunctionFSMWorkState::
                        ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION) {
    GetContext.mutable_output_info()
        ->elk_output_info_.elk_right_intervention_flag_ = true;
  } else {
    GetContext.mutable_output_info()
        ->elk_output_info_.elk_right_intervention_flag_ = false;
  }
  return;
}
uint16 ElkCore::RiskAlertJudge(const context::FusionObjExtractInfo &obj) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  uint16 risk_condition_code = 0;
  double collision_x = 0.0;
  double obj_collision_x = 0.0;
  bool requre_obj_speed_direction_positive = true;
  if (obj.relative_position_x > 0) {
    collision_x = GetContext.get_param()->origin_2_front_bumper;
    requre_obj_speed_direction_positive = false;
    obj_collision_x = obj.relative_position_x_down;
  } else {
    collision_x = -1.0 * GetContext.get_param()->origin_2_rear_bumper;
    obj_collision_x = obj.relative_position_x_up;
  }

  if (requre_obj_speed_direction_positive) {
    if (obj.relative_speed_x < 0.01) {
      risk_condition_code += uint16_bit[0];
    }
  } else {
    if (obj.relative_speed_x > -0.01) {
      risk_condition_code += uint16_bit[0];
    }
  }
  // double relative_position_yl = 0.0;
  // double relative_position_yr = 0.0;
  // double obj_projection_inner_lane_latdist = 0.0;
  double ttc_collision_tmp = 10.0;
  double ttc_collision_thrd_tmp = 10.0;
  std::vector<double> ttc_collision_thrd_tab = {2.0, 2.5, 3.0};
  std::vector<double> obj_relative_speed_tab = {5.0, 10.0, 15.0};
  ttc_collision_thrd_tmp =
      pnc::mathlib::Interp1(obj_relative_speed_tab, ttc_collision_thrd_tab,
                            std::fabs(obj.relative_speed_x));
  ttc_collision_tmp =
      std::fabs((obj_collision_x - collision_x) / obj.relative_speed_x);
  if (ttc_collision_tmp > ttc_collision_thrd_tmp) {
    risk_condition_code += uint16_bit[1];
  }
  return risk_condition_code;
}

// std::vector<double> ElkCore::ObjCornersCalculate(
//     const context::FusionObjExtractInfo &obj) {
//   std::vector<double> obj_vec;
//   obj_vec.clear();
//   obj_vec.resize(8);
//   double cal_x_tmp = 0.0;
//   double cal_y_tmp = 0.0;
//   cal_x_tmp = obj.relative_position_x + 0.5 * obj.length;
//   cal_y_tmp = obj.relative_position_y + 0.5 * obj.width;
//   obj_vec[0] = cal_x_tmp;
//   obj_vec[1] = cal_y_tmp;
//   cal_x_tmp = obj.relative_position_x + 0.5 * obj.length;
//   cal_y_tmp = obj.relative_position_y - 0.5 * obj.width;
//   obj_vec[2] = cal_x_tmp;
//   obj_vec[3] = cal_y_tmp;
//   cal_x_tmp = obj.relative_position_x - 0.5 * obj.length;
//   cal_y_tmp = obj.relative_position_y + 0.5 * obj.width;
//   obj_vec[4] = cal_x_tmp;
//   obj_vec[5] = cal_y_tmp;
//   cal_x_tmp = obj.relative_position_x - 0.5 * obj.length;
//   cal_y_tmp = obj.relative_position_y - 0.5 * obj.width;
//   obj_vec[6] = cal_x_tmp;
//   obj_vec[7] = cal_y_tmp;
//   return obj_vec;
// }

void ElkCore::RunOnce(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  // 更新Elk开关状态
  elk_main_switch_ = UpdateElkMainSwitch();

  // 更新elk_enable_code_
  elk_enable_code_ = UpdateElkEnableCode();

  // 更新elk_disable_code_
  elk_disable_code_ = UpdateElkDisableCode();

  // 更新elk_fault_code_
  elk_fault_code_ = UpdateElkFaultCode();

  // 更新elk_left_suppression_code_
  elk_left_suppression_code_ = UpdateElkLeftSuppressionCode();

  // 更新elk_left_kickdown_code_
  elk_left_kickdown_code_ = UpdateElkLeftKickDownCode();

  // 更新elk_right_suppression_code_
  elk_right_suppression_code_ = UpdateElkRightSuppressionCode();

  // 更新elk_right_kickdown_code_
  elk_right_kickdown_code_ = UpdateElkRightKickDownCode();

  // 更新tlc_to_line_threshold_
  elk_tlc_threshold_ = GetContext.get_param()->elk_tlc_thrd;
  elk_roadedge_tlc_threshold_ = GetContext.get_param()->elk_roadedge_tlc_thrd;
  // 获取tlc秒后的车辆位置
  std::vector<double> preview_ego_pos_vec;
  PreviewEgoPosisation(elk_tlc_threshold_, preview_ego_pos_vec);
  // 更新ldw_left_intervention_
  bool left_has_risk = false;
  uint16 fl_risk_code = 5;
  uint16 ml_risk_code = 5;
  uint16 rl_risk_code = 5;
  bool left_sideway_exist_flag = false;
  if (GetContext.get_param()->force_no_sideway_switch == true) {
    // 分流口是否作为抑制功能条件，待确定
    left_sideway_exist_flag = true;
  } else {
    left_sideway_exist_flag =
        GetContext.get_road_info()->current_lane.left_sideway_exist_flag;
  }
  if (((GetContext.get_road_info()->current_lane.left_line.line_type ==
        context::Enum_LineType::Enum_LineType_Solid) &&
       (left_sideway_exist_flag &&
        GetContext.get_road_info()
                ->current_lane.left_safe_departure_permission_flag == false)) ||
      (GetContext.get_road_info()->current_lane.left_roadedge.valid)) {
    // 当一侧是路沿时，不需要判断与目标物的碰撞风险，功能触发
    // 当一侧是实线，且没有分流岔路口同时不需要安全偏离，不需要判断与目标物的碰撞风险，功能触发
    left_has_risk = true;
    fl_risk_code = 4;
    ml_risk_code = 4;
    rl_risk_code = 4;
  } else {
    if (GetContext.get_objs_info()->objs_selected.ml_objs.vehicle_info_valid) {
      left_has_risk = true;
      ml_risk_code = 0;
    }
    if (GetContext.get_objs_info()->objs_selected.fl_objs.vehicle_info_valid) {
      fl_risk_code = RiskAlertJudge(
          GetContext.get_objs_info()->objs_selected.fl_objs.vehicle_info);
      if (fl_risk_code == 0) {
        left_has_risk = true;
      }
    }
    if (GetContext.get_objs_info()->objs_selected.rl_objs.vehicle_info_valid) {
      rl_risk_code = RiskAlertJudge(
          GetContext.get_objs_info()->objs_selected.rl_objs.vehicle_info);
      if (rl_risk_code == 0) {
        left_has_risk = true;
      }
    }
  }
  double preview_left_y_gap =
      adas_function::LkasLineLeftIntervention(elk_tlc_threshold_);
  double preview_left_roadedge_y_gap =
      adas_function::LkasRoadedgeLeftIntervention(
          elk_roadedge_tlc_threshold_,
          GetContext.get_param()->elk_roadedge_offset);
  bool elk_left_intervention_by_line = false;
  bool elk_left_intervention_by_roadedge = false;
  if (preview_left_y_gap < 0.0) {
    elk_left_intervention_by_line = true;
  }
  if (preview_left_roadedge_y_gap < 0.0 &&
      GetContext.get_road_info()->current_lane.left_roadedge.end_x >
          GetContext.get_param()->ego_length) {
    elk_left_intervention_by_roadedge = true;
  }
  if ((elk_left_intervention_by_line || elk_left_intervention_by_roadedge) &&
      left_has_risk == true) {
    elk_left_intervention_ = true;
  } else {
    elk_left_intervention_ = false;
  }

  // 更新ldw_right_intervention_
  bool right_has_risk = false;
  uint16 fr_risk_code = 5;
  uint16 mr_risk_code = 5;
  uint16 rr_risk_code = 5;
  bool right_sideway_exist_flag = false;
  if (GetContext.get_param()->force_no_sideway_switch == true) {
    // 分流口是否作为抑制功能条件，待确定
    right_sideway_exist_flag = true;
  } else {
    right_sideway_exist_flag =
        GetContext.get_road_info()->current_lane.right_sideway_exist_flag;
  }
  if (((GetContext.get_road_info()->current_lane.right_line.line_type ==
        context::Enum_LineType::Enum_LineType_Solid) &&
       (right_sideway_exist_flag &&
        GetContext.get_road_info()
                ->current_lane.right_safe_departure_permission_flag ==
            false)) ||
      GetContext.get_road_info()->current_lane.right_roadedge.valid) {
    // 当一侧是路沿时，不需要判断与目标物的碰撞风险，均不会抑制功能触发
    // 当一侧是实线，且没有分流岔路口同时没有分流岔路口且不需要安全偏离，不需要判断与目标物的碰撞风险，均不会抑制功能触发
    right_has_risk = true;
    fr_risk_code = 4;
    mr_risk_code = 4;
    rr_risk_code = 4;
  } else {
    if (GetContext.get_objs_info()->objs_selected.mr_objs.vehicle_info_valid) {
      right_has_risk = true;
      mr_risk_code = 0;
    }
    if (GetContext.get_objs_info()->objs_selected.fr_objs.vehicle_info_valid) {
      fr_risk_code = RiskAlertJudge(
          GetContext.get_objs_info()->objs_selected.fr_objs.vehicle_info);
      if (fr_risk_code == 0) {
        right_has_risk = true;
      }
    }
    if (GetContext.get_objs_info()->objs_selected.rr_objs.vehicle_info_valid) {
      rr_risk_code = RiskAlertJudge(
          GetContext.get_objs_info()->objs_selected.rr_objs.vehicle_info);
      if (rr_risk_code == 0) {
        right_has_risk = true;
      }
    }
  }
  double preview_right_y_gap =
      adas_function::LkasLineRightIntervention(elk_tlc_threshold_);
  double preview_right_roadedge_y_gap =
      adas_function::LkasRoadedgeRightIntervention(
          elk_roadedge_tlc_threshold_,
          GetContext.get_param()->elk_roadedge_offset);
  bool elk_right_intervention_by_line = false;
  bool elk_right_intervention_by_roadedge = false;
  if (preview_right_y_gap > 0.0) {
    elk_right_intervention_by_line = true;
  }
  if (preview_right_roadedge_y_gap > 0.0 &&
      GetContext.get_road_info()->current_lane.right_roadedge.end_x >
          GetContext.get_param()->ego_length) {
    elk_right_intervention_by_roadedge = true;
  }
  if ((elk_right_intervention_by_line || elk_right_intervention_by_roadedge) &&
      right_has_risk == true) {
    elk_right_intervention_ = true;
  } else {
    elk_right_intervention_ = false;
  }

  // 更新elk_state_
  elk_state_ = ElkStateMachine();

  // 输出elk计算结果
  SetElkOutputInfo();
  // log
  JSON_DEBUG_VALUE("elk_main_switch_", elk_main_switch_);
  JSON_DEBUG_VALUE("elk_enable_code_", elk_enable_code_);
  JSON_DEBUG_VALUE("elk_disable_code_", elk_disable_code_);
  JSON_DEBUG_VALUE("elk_fault_code_", elk_fault_code_);
  JSON_DEBUG_VALUE("elk_left_suppression_code_", elk_left_suppression_code_);
  JSON_DEBUG_VALUE("elk_left_kickdown_code_", elk_left_kickdown_code_);
  JSON_DEBUG_VALUE("elk_right_suppression_code_", elk_right_suppression_code_);
  JSON_DEBUG_VALUE("elk_right_kickdown_code_", elk_right_kickdown_code_);
  JSON_DEBUG_VALUE("elk_tlc_threshold_", elk_tlc_threshold_);
  JSON_DEBUG_VALUE("elk_roadedge_tlc_threshold_", elk_roadedge_tlc_threshold_);
  JSON_DEBUG_VALUE("elk_left_intervention_", elk_left_intervention_);
  JSON_DEBUG_VALUE("elk_left_intervention_by_line",
                   elk_left_intervention_by_line);
  JSON_DEBUG_VALUE("elk_left_intervention_by_roadedge",
                   elk_left_intervention_by_roadedge);

  JSON_DEBUG_VALUE("elk_left_has_risk", left_has_risk);
  JSON_DEBUG_VALUE("elk_right_has_risk", right_has_risk);

  JSON_DEBUG_VALUE("elk_right_intervention_", elk_right_intervention_);
  JSON_DEBUG_VALUE("elk_right_intervention_by_line",
                   elk_right_intervention_by_line);
  JSON_DEBUG_VALUE("elk_right_intervention_by_roadedge",
                   elk_right_intervention_by_roadedge);
  JSON_DEBUG_VALUE("elk_state_", (int)elk_state_);
  JSON_DEBUG_VALUE("elk_preview_left_y_gap", preview_left_y_gap);
  JSON_DEBUG_VALUE("elk_preview_right_y_gap", preview_right_y_gap);
  JSON_DEBUG_VALUE("elk_preview_left_roadedge_y_gap",
                   preview_left_roadedge_y_gap);
  JSON_DEBUG_VALUE("elk_preview_right_roadedge_y_gap",
                   preview_right_roadedge_y_gap);
  JSON_DEBUG_VALUE("elk_roadedge_offset",
                   GetContext.get_param()->elk_roadedge_offset);
  JSON_DEBUG_VECTOR("elk_preview_ego_pos_vec", preview_ego_pos_vec, 2);
  JSON_DEBUG_VALUE("elk_fl_risk_code", fl_risk_code);
  JSON_DEBUG_VALUE("elk_ml_risk_code", ml_risk_code);
  JSON_DEBUG_VALUE("elk_rl_risk_code", rl_risk_code);
  JSON_DEBUG_VALUE("elk_fr_risk_code", fr_risk_code);
  JSON_DEBUG_VALUE("elk_mr_risk_code", mr_risk_code);
  JSON_DEBUG_VALUE("elk_rr_risk_code", rr_risk_code);
}

}  // namespace elk_core
}  // namespace adas_function