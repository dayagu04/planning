#include "ldp_core.h"

#include <iostream>

#include "adas_function_lib.h"
#include "debug_info_log.h"
#include "planning_hmi_c.h"

namespace adas_function {
namespace ldp_core {

bool LdpCore::UpdateLdpMainSwitch(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  auto function_state_machine_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .function_state_machine_info;
  if (GetContext.get_param()->ldp_main_switch) {
    return GetContext.get_param()->ldp_main_switch;
  }
  return function_state_machine_info_ptr->switch_sts.ldp_main_switch;
}

uint16 LdpCore::UpdateLdpEnableCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;

  uint16 enable_code = 0;

  // bit 0
  // 判断车速是否处于工作车速范围内
  if (vehicle_service_output_info_ptr->vehicle_speed_display <
      ldp_param_.enable_vehspd_display_min) {
    enable_code += uint16_bit[0];
  } else if (vehicle_service_output_info_ptr->vehicle_speed_display >
             ldp_param_.enable_vehspd_display_max) {
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

uint16 LdpCore::UpdateLdpDisableCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;

  uint16 disable_code = 0;

  // bit 0
  // 判断车速是否处于工作车速范围内
  if (vehicle_service_output_info_ptr->vehicle_speed_display <
      ldp_param_.disable_vehspd_display_min) {
    disable_code += uint16_bit[0];
  } else if (vehicle_service_output_info_ptr->vehicle_speed_display >
             ldp_param_.disable_vehspd_display_max) {
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

uint16 LdpCore::UpdateLdpFaultCode(void) {
  uint16 ldp_fault_code = 0;
  return ldp_fault_code;
}

uint16 LdpCore::UpdateLdpLeftSuppressionCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto function_state_machine_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .function_state_machine_info;

  uint16 ldp_left_suppression_code = 0;

  if ((ldp_state_ ==
       iflyauto::LDPFunctionFSMWorkState::
           LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION) ||
      (GetContext.get_road_info()->current_lane.lane_changed_flag ==
       true)) {  // 检测到左侧报警
    left_suppress_repeat_warning_flag_ = true;
  } else {  // 检测到左侧报警结束
    if (left_suppress_repeat_warning_flag_ == true) {
      if ((GetContext.get_road_info()->current_lane.left_line.valid == true) &&
          (GetContext.mutable_state_info()->fl_wheel_distance_to_line >
           ldp_param_.reset_warning_line)) {
        // 左侧边界线有效&&检测到越过了重置线
        left_suppress_repeat_warning_flag_ = false;
      } else if ((GetContext.get_road_info()
                      ->current_lane.left_roadedge.valid == true) &&
                 (GetContext.mutable_state_info()
                      ->fl_wheel_distance_to_roadedge >
                  ldp_param_.roadedge_reset_warning_line)) {
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
      ldp_param_.supp_turn_light_recovery_time) {
    ldp_left_suppression_code += uint16_bit[0];
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
       ldp_param_.earliest_warning_line)) {
    in_left_line_aera_flag = true;
  } else {
    in_left_line_aera_flag = false;
  }
  // 判断是否处于路沿允许左侧报警区域内
  if ((GetContext.get_road_info()->current_lane.left_roadedge.valid == true) &&
      (GetContext.mutable_state_info()->fl_wheel_distance_to_roadedge > 0.0) &&
      (GetContext.mutable_state_info()->fl_wheel_distance_to_roadedge <
       ldp_param_.roadedge_earliest_warning_line)) {
    in_left_roadedge_aera_flag = true;
  } else {
    in_left_roadedge_aera_flag = false;
  }
  if ((in_left_line_aera_flag == false) &&
      (in_left_roadedge_aera_flag == false)) {
    ldp_left_suppression_code += uint16_bit[1];
  }

  // Condition3
  // 距上次触发报警后,越过了重置线
  if (left_suppress_repeat_warning_flag_ == true) {
    ldp_left_suppression_code += uint16_bit[2];
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
    ldp_left_suppression_code += uint16_bit[3];
  }

  // Condition5
  // 驾驶员手力矩条件
  if (fabs(GetContext.mutable_state_info()->driver_hand_trq) >
      ldp_param_.suppression_driver_hand_trq) {
    ldp_left_suppression_code += uint16_bit[4];
  } else {
    /*do nothing*/
  }
  if (GetContext.get_road_info()->current_lane.left_line.line_type ==
      context::Enum_LineType::Enum_LineType_Virtual) {
    ldp_left_suppression_code += uint16_bit[5];
  } else {
    /*do nothing*/
  }

  if (GetContext.get_road_info()->current_lane.left_sideway_exist_flag&& (GetContext.get_param()->force_no_sideway_switch == false)) {
    // 左侧有分流口
    ldp_left_suppression_code += uint16_bit[6];
  }

  if (GetContext.get_road_info()
          ->current_lane.left_safe_departure_permission_flag == true) {
    // 右侧有并行车或右前方一定区域内有车、或者正前方一定区域内有车，且有碰撞风险
    ldp_left_suppression_code += uint16_bit[7];
  }

  return ldp_left_suppression_code;
}

uint16 LdpCore::UpdateLdpLeftKickDownCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;

  auto function_state_machine_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .function_state_machine_info;

  uint16 ldp_left_kickdown_code = 0;

  if (ldp_state_ == iflyauto::LDPFunctionFSMWorkState::
                        LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION) {
    ldp_left_warning_time_ += GetContext.mutable_param()->dt;
    if (ldp_left_warning_time_ >= 60.0) {
      ldp_left_warning_time_ = 60.0;
    } else {
      /*do nothing*/
    }
  } else {
    ldp_left_warning_time_ = 0.0;
  }

  // Condition1
  if (vehicle_service_output_info_ptr->left_turn_light_state == true) {
    ldp_left_kickdown_code += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // Condition2
  // 判断是否处于允许左侧报警区域内
  bool in_left_line_aera_flag = false;
  bool in_left_roadedge_aera_flag = false;
  if ((GetContext.get_road_info()->current_lane.left_line.valid) &&
      (GetContext.mutable_state_info()->fl_wheel_distance_to_line >
       ldp_param_.latest_warning_line) &&
      (GetContext.mutable_state_info()->fl_wheel_distance_to_line <
       ldp_param_.earliest_warning_line)) {
    in_left_line_aera_flag = true;
  } else {
    in_left_line_aera_flag = false;
  }
  if ((GetContext.get_road_info()->current_lane.left_roadedge.valid) &&
      (GetContext.mutable_state_info()->fl_wheel_distance_to_roadedge >
       ldp_param_.roadedge_latest_warning_line) &&
      (GetContext.mutable_state_info()->fl_wheel_distance_to_roadedge <
       ldp_param_.roadedge_earliest_warning_line)) {
    in_left_roadedge_aera_flag = true;
  } else {
    in_left_roadedge_aera_flag = false;
  }
  if ((in_left_line_aera_flag == false) &&
      (in_left_roadedge_aera_flag == false)) {
    ldp_left_kickdown_code += uint16_bit[1];
  }
  // Condition3
  if (ldp_left_warning_time_ > ldp_param_.warning_time_max) {
    ldp_left_kickdown_code += uint16_bit[2];
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
    ldp_left_kickdown_code += uint16_bit[3];
  }

  // Condition6
  // 驾驶员手力矩条件
  if (fabs(GetContext.mutable_state_info()->driver_hand_trq) >
      ldp_param_.kickdown_driver_hand_trq) {
    ldp_left_kickdown_code += uint16_bit[4];
  } else {
    /*do nothing*/
  }

  if (GetContext.get_road_info()->current_lane.left_line.line_type ==
      context::Enum_LineType::Enum_LineType_Virtual) {
    ldp_left_kickdown_code += uint16_bit[5];
  } else {
    /*do nothing*/
  }

  return ldp_left_kickdown_code;
}

uint16 LdpCore::UpdateLdpRightSuppressionCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto function_state_machine_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .function_state_machine_info;

  uint16 ldp_right_suppression_code = 0;

  if ((ldp_state_ ==
       iflyauto::LDPFunctionFSMWorkState::
           LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION) ||
      (GetContext.get_road_info()->current_lane.lane_changed_flag ==
       true)) {  // 检测到右侧报警
    right_suppress_repeat_warning_flag_ = true;
  } else {  // 检测到右侧报警结束
    if (right_suppress_repeat_warning_flag_ == true) {
      if ((GetContext.get_road_info()->current_lane.right_line.valid == true) &&
          (GetContext.mutable_state_info()->fr_wheel_distance_to_line <
           (-1.0 * ldp_param_.reset_warning_line))) {
        // 左侧边界线有效&&检测到越过了重置线
        right_suppress_repeat_warning_flag_ = false;
      } else if ((GetContext.get_road_info()
                      ->current_lane.right_roadedge.valid == true) &&
                 (GetContext.mutable_state_info()
                      ->fr_wheel_distance_to_roadedge <
                  (-1.0 * ldp_param_.roadedge_reset_warning_line))) {
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
      ldp_param_.supp_turn_light_recovery_time) {
    ldp_right_suppression_code += uint16_bit[0];
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
       (-1.0 * ldp_param_.earliest_warning_line))) {
    in_right_line_aera_flag = true;
  } else {
    in_right_line_aera_flag = false;
  }
  // 判断是否处于路沿允许右侧报警区域内
  if ((GetContext.get_road_info()->current_lane.right_roadedge.valid == true) &&
      (GetContext.mutable_state_info()->fr_wheel_distance_to_roadedge < 0.0) &&
      (GetContext.mutable_state_info()->fr_wheel_distance_to_roadedge >
       (-1.0 * ldp_param_.roadedge_earliest_warning_line))) {
    in_right_roadedge_aera_flag = true;
  } else {
    in_right_roadedge_aera_flag = false;
  }
  if ((in_right_line_aera_flag == false) &&
      (in_right_roadedge_aera_flag == false)) {
    ldp_right_suppression_code += uint16_bit[1];
  }

  // Condition3
  // 距上次触发报警后,未越过了重置线
  if (right_suppress_repeat_warning_flag_ == true) {
    ldp_right_suppression_code += uint16_bit[2];
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
    ldp_right_suppression_code += uint16_bit[3];
  }

  // Condition5
  // 驾驶员手力矩条件
  if (fabs(GetContext.mutable_state_info()->driver_hand_trq) >
      ldp_param_.suppression_driver_hand_trq) {
    ldp_right_suppression_code += uint16_bit[4];
  } else {
    /*do nothing*/
  }

  if (GetContext.get_road_info()->current_lane.right_line.line_type ==
      context::Enum_LineType::Enum_LineType_Virtual) {
    ldp_right_suppression_code += uint16_bit[5];
  } else {
    /*do nothing*/
  }

  if (GetContext.get_road_info()->current_lane.right_sideway_exist_flag && (GetContext.get_param()->force_no_sideway_switch == false)) {
    // 右侧有分流口
    ldp_right_suppression_code += uint16_bit[6];
  }

  if (GetContext.get_road_info()
          ->current_lane.right_safe_departure_permission_flag == true) {
    // 左侧有并行车或左前方一定区域内有车、或者正前方一定区域内有车，且有碰撞风险
    ldp_right_suppression_code += uint16_bit[7];
  }

  return ldp_right_suppression_code;
}

uint16 LdpCore::UpdateLdpRightKickDownCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;

  auto function_state_machine_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .function_state_machine_info;

  uint16 ldp_right_kickdown_code = 0;

  if (ldp_state_ == iflyauto::LDPFunctionFSMWorkState::
                        LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION) {
    ldp_right_warning_time_ += GetContext.mutable_param()->dt;
    if (ldp_right_warning_time_ >= 60.0) {
      ldp_right_warning_time_ = 60.0;
    } else {
      /*do nothing*/
    }
  } else {
    ldp_right_warning_time_ = 0.0;
  }

  // Condition1
  if (vehicle_service_output_info_ptr->right_turn_light_state == true) {
    ldp_right_kickdown_code += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // Condition2
  // 判断是否处于允许右侧报警区域内
  bool in_right_line_aera_flag = false;
  bool in_right_roadedge_aera_flag = false;
  if ((GetContext.get_road_info()->current_lane.right_line.valid) &&
      (GetContext.mutable_state_info()->fr_wheel_distance_to_line <
       (-1.0 * ldp_param_.latest_warning_line)) &&
      (GetContext.mutable_state_info()->fr_wheel_distance_to_line >
       (-1.0 * ldp_param_.earliest_warning_line))) {
    in_right_line_aera_flag = true;
  } else {
    in_right_line_aera_flag = false;
  }
  if ((GetContext.get_road_info()->current_lane.right_roadedge.valid) &&
      (GetContext.mutable_state_info()->fr_wheel_distance_to_roadedge <
       (-1.0 * ldp_param_.roadedge_latest_warning_line)) &&
      (GetContext.mutable_state_info()->fr_wheel_distance_to_roadedge >
       (-1.0 * ldp_param_.roadedge_earliest_warning_line))) {
    in_right_roadedge_aera_flag = true;
  } else {
    in_right_roadedge_aera_flag = false;
  }
  if ((in_right_line_aera_flag == false) &&
      (in_right_roadedge_aera_flag == false)) {
    ldp_right_kickdown_code += uint16_bit[1];
  }

  // Condition3
  if (ldp_right_warning_time_ > ldp_param_.warning_time_max) {
    ldp_right_kickdown_code += uint16_bit[2];
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
    ldp_right_kickdown_code += uint16_bit[3];
  }

  // Condition6
  // 驾驶员手力矩条件
  if (fabs(GetContext.mutable_state_info()->driver_hand_trq) >
      ldp_param_.kickdown_driver_hand_trq) {
    ldp_right_kickdown_code += uint16_bit[4];
  } else {
    /*do nothing*/
  }
  if (GetContext.get_road_info()->current_lane.right_line.line_type ==
      context::Enum_LineType::Enum_LineType_Virtual) {
    ldp_right_kickdown_code += uint16_bit[5];
  } else {
    /*do nothing*/
  }
  return ldp_right_kickdown_code;
}

iflyauto::LDPFunctionFSMWorkState LdpCore::LdpStateMachine(void) {
  iflyauto::LDPFunctionFSMWorkState ldp_state;

  if (ldp_state_machine_init_flag_ == false) {
    // 状态机处于初始化状态 根据开关状态,决定第一个周期是输出OFF还是STANDBY
    ldp_state_machine_init_flag_ = true;
    if (ldp_main_switch_ == false) {
      ldp_state =
          iflyauto::LDPFunctionFSMWorkState::LDP_FUNCTION_FSM_WORK_STATE_OFF;
    } else {
      ldp_state = iflyauto::LDPFunctionFSMWorkState::
          LDP_FUNCTION_FSM_WORK_STATE_STANDBY;
    }
    return ldp_state;
  }

  // 状态机处于完成过初始化的状态
  if (ldp_state_ == iflyauto::LDPFunctionFSMWorkState::
                        LDP_FUNCTION_FSM_WORK_STATE_UNAVAILABLE) {
    // 上一时刻处于LDP_FUNCTION_FSM_WORK_STATE_UNAVAILABLE状态
    if (ldp_main_switch_ == false) {
      ldp_state =
          iflyauto::LDPFunctionFSMWorkState::LDP_FUNCTION_FSM_WORK_STATE_OFF;
    } else if (ldp_fault_code_ == 0) {
      ldp_state = iflyauto::LDPFunctionFSMWorkState::
          LDP_FUNCTION_FSM_WORK_STATE_STANDBY;
    } else {
      // do nothing
    }
  } else if (ldp_state_ == iflyauto::LDPFunctionFSMWorkState::
                               LDP_FUNCTION_FSM_WORK_STATE_OFF) {
    // 上一时刻处于LDP_FUNCTION_FSM_WORK_STATE_OFF状态
    if (ldp_main_switch_ == true) {
      ldp_state = iflyauto::LDPFunctionFSMWorkState::
          LDP_FUNCTION_FSM_WORK_STATE_STANDBY;
    } else {
      ldp_state =
          iflyauto::LDPFunctionFSMWorkState::LDP_FUNCTION_FSM_WORK_STATE_OFF;
    }
  } else if (ldp_state_ == iflyauto::LDPFunctionFSMWorkState::
                               LDP_FUNCTION_FSM_WORK_STATE_STANDBY) {
    // 上一时刻处于LDP_FUNCTION_FSM_WORK_STATE_STANDBY状态
    if (ldp_main_switch_ == false) {
      ldp_state =
          iflyauto::LDPFunctionFSMWorkState::LDP_FUNCTION_FSM_WORK_STATE_OFF;
    } else if (ldp_fault_code_) {
      ldp_state = iflyauto::LDPFunctionFSMWorkState::
          LDP_FUNCTION_FSM_WORK_STATE_UNAVAILABLE;
    } else if (ldp_enable_code_ == 0) {
      ldp_state = iflyauto::LDPFunctionFSMWorkState::
          LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_NO_INTERVENTION;
    } else {
      ldp_state = ldp_state_;
    }
  } else if ((ldp_state_ ==
              iflyauto::LDPFunctionFSMWorkState::
                  LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_NO_INTERVENTION) ||
             (ldp_state_ ==
              iflyauto::LDPFunctionFSMWorkState::
                  LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION) ||
             (ldp_state_ ==
              iflyauto::LDPFunctionFSMWorkState::
                  LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION)) {
    // 上一时刻处于LDP_FUNCTION_FSM_WORK_STATE_ACTIVE状态
    if (ldp_main_switch_ == false) {
      ldp_state =
          iflyauto::LDPFunctionFSMWorkState::LDP_FUNCTION_FSM_WORK_STATE_OFF;
    } else if (ldp_fault_code_) {
      ldp_state = iflyauto::LDPFunctionFSMWorkState::
          LDP_FUNCTION_FSM_WORK_STATE_UNAVAILABLE;
    } else if (ldp_disable_code_) {
      ldp_state = iflyauto::LDPFunctionFSMWorkState::
          LDP_FUNCTION_FSM_WORK_STATE_STANDBY;
    } else if (ldp_state_ ==
               iflyauto::LDPFunctionFSMWorkState::
                   LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_NO_INTERVENTION) {
      if (ldp_left_intervention_ && (ldp_left_suppression_code_ == 0)) {
        ldp_state = iflyauto::LDPFunctionFSMWorkState::
            LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION;
      } else if (ldp_right_intervention_ &&
                 (ldp_right_suppression_code_ == 0)) {
        ldp_state = iflyauto::LDPFunctionFSMWorkState::
            LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION;
      } else {
        ldp_state = ldp_state_;
      }
    } else if (ldp_state_ ==
               iflyauto::LDPFunctionFSMWorkState::
                   LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION) {
      if (ldp_left_kickdown_code_) {
        ldp_state = iflyauto::LDPFunctionFSMWorkState::
            LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_NO_INTERVENTION;
      } else {
        ldp_state = ldp_state_;
      }
    } else if (ldp_state_ ==
               iflyauto::LDPFunctionFSMWorkState::
                   LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION) {
      if (ldp_right_kickdown_code_) {
        ldp_state = iflyauto::LDPFunctionFSMWorkState::
            LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_NO_INTERVENTION;
      } else {
        ldp_state = ldp_state_;
      }
    } else {
      ldp_state = ldp_state_;
    }
  } else {
    // 处于异常状态
    ldp_state =
        iflyauto::LDPFunctionFSMWorkState::LDP_FUNCTION_FSM_WORK_STATE_OFF;
  }
  return ldp_state;
}

void LdpCore::SetLdpOutputInfo() {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  GetContext.mutable_output_info()->ldp_output_info_.ldp_state_ = ldp_state_;
  if (ldp_state_ == iflyauto::LDPFunctionFSMWorkState::
                        LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION) {
    GetContext.mutable_output_info()
        ->ldp_output_info_.ldp_left_intervention_flag_ = true;
  } else {
    GetContext.mutable_output_info()
        ->ldp_output_info_.ldp_left_intervention_flag_ = false;
  }
  if (ldp_state_ == iflyauto::LDPFunctionFSMWorkState::
                        LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION) {
    GetContext.mutable_output_info()
        ->ldp_output_info_.ldp_right_intervention_flag_ = true;
  } else {
    GetContext.mutable_output_info()
        ->ldp_output_info_.ldp_right_intervention_flag_ = false;
  }
  return;
}

double LdpCore::UpdateTlcThreshold(void){
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
double tlc_calculate =GetContext.get_param()->ldp_tlc_thrd;
 tlc_calculate = pnc::mathlib::Interp1(
                       GetContext.get_param()->ldp_vel_vector,
                       GetContext.get_param()->ldp_tlc_vector,(GetContext.get_state_info()->display_vehicle_speed *3.6));
 return tlc_calculate;
}

void LdpCore::RunOnce(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  // 更新Ldp开关状态
  ldp_main_switch_ = UpdateLdpMainSwitch();

  // 更新ldp_enable_code_
  ldp_enable_code_ = UpdateLdpEnableCode();

  // 更新ldp_disable_code_
  ldp_disable_code_ = UpdateLdpDisableCode();

  // 更新ldp_fault_code_
  ldp_fault_code_ = UpdateLdpFaultCode();

  // 更新ldp_left_suppression_code_
  ldp_left_suppression_code_ = UpdateLdpLeftSuppressionCode();

  // 更新ldp_left_kickdown_code_
  ldp_left_kickdown_code_ = UpdateLdpLeftKickDownCode();

  // 更新ldp_right_suppression_code_
  ldp_right_suppression_code_ = UpdateLdpRightSuppressionCode();

  // 更新ldp_right_kickdown_code_
  ldp_right_kickdown_code_ = UpdateLdpRightKickDownCode();

  // 更新tlc_to_line_threshold_
  ldp_tlc_threshold_ = UpdateTlcThreshold();
  ldp_roadedge_tlc_threshold_ = GetContext.get_param()->ldp_roadedge_tlc_thrd;
  // 获取tlc秒后的车辆位置
  std::vector<double> preview_ego_pos_vec;
  PreviewEgoPosisation(ldp_tlc_threshold_, preview_ego_pos_vec);
  // 更新ldw_left_intervention_
  double preview_left_y_gap =
      adas_function::LkasLineLeftIntervention(ldp_tlc_threshold_);
  double preview_left_roadedge_y_gap =
      adas_function::LkasRoadedgeLeftIntervention(
          ldp_roadedge_tlc_threshold_,
          GetContext.get_param()->ldp_roadedge_offset);
  bool ldp_left_intervention_by_line = false;
  bool ldp_left_intervention_by_roadedge = false;
  if (preview_left_y_gap < 0.0) {
    ldp_left_intervention_by_line = true;
  }
  if (preview_left_roadedge_y_gap < 0.0 &&
      GetContext.get_road_info()->current_lane.left_roadedge.end_x >
          GetContext.get_param()->ego_length) {
    ldp_left_intervention_by_roadedge = true;
  }
  if (ldp_left_intervention_by_line || ldp_left_intervention_by_roadedge) {
    ldp_left_intervention_ = true;
  } else {
    ldp_left_intervention_ = false;
  }

  // 更新ldw_right_intervention_
  double preview_right_y_gap =
      adas_function::LkasLineRightIntervention(ldp_tlc_threshold_);
  double preview_right_roadedge_y_gap =
      adas_function::LkasRoadedgeRightIntervention(
          ldp_roadedge_tlc_threshold_,
          GetContext.get_param()->ldp_roadedge_offset);
  bool ldp_right_intervention_by_line = false;
  bool ldp_right_intervention_by_roadedge = false;
  if (preview_right_y_gap > 0.0) {
    ldp_right_intervention_by_line = true;
  }
  if (preview_right_roadedge_y_gap > 0.0 &&
      GetContext.get_road_info()->current_lane.right_roadedge.end_x >
          GetContext.get_param()->ego_length) {
    ldp_right_intervention_by_roadedge = true;
  }
  if (ldp_right_intervention_by_line || ldp_right_intervention_by_roadedge) {
    ldp_right_intervention_ = true;
  } else {
    ldp_right_intervention_ = false;
  }

  // 更新ldp_state_
  ldp_state_ = LdpStateMachine();

  // 输出ldp计算结果
  SetLdpOutputInfo();
  // log
  JSON_DEBUG_VALUE("ldp_main_switch_", ldp_main_switch_);
  JSON_DEBUG_VALUE("ldp_enable_code_", ldp_enable_code_);
  JSON_DEBUG_VALUE("ldp_disable_code_", ldp_disable_code_);
  JSON_DEBUG_VALUE("ldp_fault_code_", ldp_fault_code_);
  JSON_DEBUG_VALUE("ldp_left_suppression_code_", ldp_left_suppression_code_);
  JSON_DEBUG_VALUE("ldp_left_kickdown_code_", ldp_left_kickdown_code_);
  JSON_DEBUG_VALUE("ldp_right_suppression_code_", ldp_right_suppression_code_);
  JSON_DEBUG_VALUE("ldp_right_kickdown_code_", ldp_right_kickdown_code_);
  JSON_DEBUG_VALUE("ldp_tlc_threshold_", ldp_tlc_threshold_);
  JSON_DEBUG_VALUE("ldp_roadedge_tlc_threshold_", ldp_roadedge_tlc_threshold_);
  JSON_DEBUG_VALUE("ldp_left_intervention_", ldp_left_intervention_);
  JSON_DEBUG_VALUE("ldp_left_intervention_by_line",
                   ldp_left_intervention_by_line);
  JSON_DEBUG_VALUE("ldp_left_intervention_by_roadedge",
                   ldp_left_intervention_by_roadedge);
  JSON_DEBUG_VALUE("ldp_right_intervention_", ldp_right_intervention_);
  JSON_DEBUG_VALUE("ldp_right_intervention_by_line",
                   ldp_right_intervention_by_line);
  JSON_DEBUG_VALUE("ldp_right_intervention_by_roadedge",
                   ldp_right_intervention_by_roadedge);
  JSON_DEBUG_VALUE("ldp_state_", (int)ldp_state_);
  JSON_DEBUG_VALUE("ldp_preview_left_y_gap", preview_left_y_gap);
  JSON_DEBUG_VALUE("ldp_preview_right_y_gap", preview_right_y_gap);
  JSON_DEBUG_VALUE("ldp_preview_left_roadedge_y_gap",
                   preview_left_roadedge_y_gap);
  JSON_DEBUG_VALUE("ldp_preview_right_roadedge_y_gap",
                   preview_right_roadedge_y_gap);
  JSON_DEBUG_VALUE("ldp_roadedge_offset",
                   GetContext.get_param()->ldp_roadedge_offset);
  JSON_DEBUG_VECTOR("ldp_preview_ego_pos_vec", preview_ego_pos_vec, 2);
}

}  // namespace ldp_core
}  // namespace adas_function