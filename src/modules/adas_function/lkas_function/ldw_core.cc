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
  return function_state_machine_info_ptr->switch_sts.ldw_main_switch;
}

uint32 LdwCore::UpdateLdwEnableCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;

  uint32 enable_code = 0;

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
  } else if (GetContext.mutable_road_info()->current_lane.lane_width > 5.2) {
    enable_code += uint16_bit[2];
  } else {
    /*do nothing*/
  }

  // bit 3
  // 判断挡位是否处于D档
  if (GetContext.get_state_info()->shift_lever_state !=
      iflyauto::ShiftLeverStateEnum::ShiftLeverState_D) {
    enable_code += uint16_bit[3];
  } else {
    /*do nothing*/
  }

  // bit 4
  // 判断四门两盖,只要有一个门/一个舱盖打开，enable条件置1
  if ((vehicle_service_output_info_ptr->fl_door_state == true) ||
      (vehicle_service_output_info_ptr->fr_door_state == true) ||
      (vehicle_service_output_info_ptr->rl_door_state == true) ||
      (vehicle_service_output_info_ptr->rr_door_state == true) ||
      (vehicle_service_output_info_ptr->hood_state == true) ||
      (vehicle_service_output_info_ptr->trunk_door_state == true)) {
    enable_code += uint16_bit[4];
  } else {
    /*do nothing*/
  }

  // bit 5
  // 判断危险报警灯是否开启:使用双闪来判断
  if (vehicle_service_output_info_ptr->hazard_light_state == true) {
    enable_code += uint16_bit[5];
  } else {
    /*do nothing*/
  }

  // bit 6
  // 判断横摆角速度是否超限,12deg/s对应0.2094rad/s.横摆角速度绝对值<12deg/s，持续1s
  if (fabs(vehicle_service_output_info_ptr->yaw_rate) < 12.0 / 57.3) {
    yaw_rate_supp_recover_duration_ += GetContext.get_param()->dt;
    if (yaw_rate_supp_recover_duration_ > 60.0) {
      yaw_rate_supp_recover_duration_ = 60.0;
    } else {
      /*do nothing*/
    }
  } else {
    yaw_rate_supp_recover_duration_ = 0.0;
  }
  if (yaw_rate_supp_recover_duration_ < 1.0) {
    enable_code += uint16_bit[6];
  } else {
    /*do nothing*/
  }

  // bit 7
  // 方向盘转速(deg/s)和车速(km/h)满足如下公式,持续1s:方向盘转速≥145-0.5*v，0<v≤100
  // 方向盘转速≥95，v＞100.
  // PS1:1.8：mps转换为kph   PS2:使用实际车速.持续1秒的算法未完成
  if ((vehicle_service_output_info_ptr->vehicle_speed <= 100.0 / 3.6) &&
      (fabs(vehicle_service_output_info_ptr->steering_wheel_angle_speed) >=
       (145.0 - 1.8 * vehicle_service_output_info_ptr->vehicle_speed) / 57.3)) {
    str_wheel_ang_speed_recover_duration_ += GetContext.get_param()->dt;
    if (str_wheel_ang_speed_recover_duration_ > 60.0) {
      str_wheel_ang_speed_recover_duration_ = 60.0;
    };
  } else if ((vehicle_service_output_info_ptr->vehicle_speed > 100.0 / 3.6) &&
             (fabs(
                  vehicle_service_output_info_ptr->steering_wheel_angle_speed) >
              95.0 / 57.3)) {
    str_wheel_ang_speed_recover_duration_ += GetContext.get_param()->dt;
    if (str_wheel_ang_speed_recover_duration_ > 60.0) {
      str_wheel_ang_speed_recover_duration_ = 60.0;
    }
  } else {
    /*do nothing*/
  }
  if (str_wheel_ang_speed_recover_duration_ > 1.0) {
    enable_code += uint16_bit[7];
  } else {
    /*do nothing*/
  }

  // bit 8
  // 驾驶员未踩下制动踏板:制动力<3bar，持续2s.后续再讨论
  if (vehicle_service_output_info_ptr->esp_pressure > 3.0) {
    brake_pedal_pressed_supp_recover_duration_ += GetContext.get_param()->dt;
    if (brake_pedal_pressed_supp_recover_duration_ > 60.0) {
      brake_pedal_pressed_supp_recover_duration_ = 60.0;
    } else {
      /*do nothing*/
    }
  } else {
    brake_pedal_pressed_supp_recover_duration_ = 0.0;
  }
  if (brake_pedal_pressed_supp_recover_duration_ > 2.0) {
    enable_code += uint16_bit[8];
  } else {
    /*do nothing*/
  }

  // bit 9
  // 油门踏板变化率<30%/s，持续1s，未完成
  if (GetContext.get_state_info()->accelerator_pedal_pos_rate < 30.0) {
    acc_pedal_pos_rate_supp_recover_duration_ += GetContext.get_param()->dt;
    if (acc_pedal_pos_rate_supp_recover_duration_ > 60.0) {
      acc_pedal_pos_rate_supp_recover_duration_ = 60.0;
    } else {
      /*do nothing*/
    }
  } else {
    acc_pedal_pos_rate_supp_recover_duration_ = 0.0;
  }
  if (acc_pedal_pos_rate_supp_recover_duration_ < 1.0) {
    enable_code += uint16_bit[9];
  } else {
    /*do nothing*/
  }

  // // bit 10
  // // AEB未激活，未完成
  if (vehicle_service_output_info_ptr->aeb_actuator_status == 2) {
    enable_code += uint16_bit[10];
  } else { /*do nothing*/
  }

  // bit 11
  // ABS/TCS/VDC未激活
  if ((vehicle_service_output_info_ptr->abs_active == true) ||
      (vehicle_service_output_info_ptr->tcs_active == true) ||
      (vehicle_service_output_info_ptr->esp_vdc_active == true)) {
    enable_code += uint16_bit[11];
  } else {
    /*do nothing*/
  }

  // bit 12
  // 弯道半径>=220m，持续2s
  double left_line_C2_temp =
      GetContext.get_road_info()->current_lane.left_line.c2;
  double right_line_C2_temp =
      GetContext.get_road_info()->current_lane.right_line.c2;

  if (((fabs(left_line_C2_temp) < 0.5 / 220) &&
       ((GetContext.get_road_info()->current_lane.left_line.valid == true))) ||
      (fabs(right_line_C2_temp) < 0.5 / 220) &&
          (GetContext.get_road_info()->current_lane.right_line.valid == true)) {
    curve_C2_supp_recover_duration_ += GetContext.get_param()->dt;
    if (curve_C2_supp_recover_duration_ > 60.0) {
      curve_C2_supp_recover_duration_ = 60.0;
    } else {
      /*do nothing*/
    }
  } else {
    curve_C2_supp_recover_duration_ = 0.0;
  }

  if (curve_C2_supp_recover_duration_ < 2.0) {
    enable_code += uint16_bit[12];
  } else {
    /*do nothing*/
  }

  // bit 13
  // 雨刮状态判断
  if ((vehicle_service_output_info_ptr->wiper_state ==
       iflyauto::WiperStateEnum::WiperState_HighSpeed)) {
    enable_code += uint16_bit[13];
  } else {
    /*do nothing*/
  }

  // bit 14
  // 安全带状态判断
  if ((vehicle_service_output_info_ptr->fl_seat_belt_state == false)) {
    enable_code += uint16_bit[14];
  } else {
    /*do nothing*/
  }

  return enable_code & GetContext.get_param()->ldw_enable_code_maskcode;
}

uint32 LdwCore::UpdateLdwDisableCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;

  uint32 disable_code = 0;

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
  // 判断是否至少识别到一侧道线
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
  } else if (GetContext.mutable_road_info()->current_lane.lane_width > 5.5) {
    disable_code += uint16_bit[2];
  } else {
    /*do nothing*/
  }

  // bit 3
  // 判断挡位是否处于D档
  if (GetContext.get_state_info()->shift_lever_state !=
      iflyauto::ShiftLeverStateEnum::ShiftLeverState_D) {
    disable_code += uint16_bit[3];
  } else {
    /*do nothing*/
  }

  // bit 4
  // 判断四门两盖,只要有一个门/一个舱盖打开，disable条件置1
  if ((vehicle_service_output_info_ptr->fl_door_state == true) ||
      (vehicle_service_output_info_ptr->fr_door_state == true) ||
      (vehicle_service_output_info_ptr->rl_door_state == true) ||
      (vehicle_service_output_info_ptr->rr_door_state == true) ||
      (vehicle_service_output_info_ptr->hood_state == true) ||
      (vehicle_service_output_info_ptr->trunk_door_state == true)) {
    disable_code += uint16_bit[4];
  } else {
    /*do nothing*/
  }

  // bit 5
  // 判断危险报警灯是否开启:使用双闪来判断
  if (vehicle_service_output_info_ptr->hazard_light_state == true) {
    disable_code += uint16_bit[5];
  } else {
    /*do nothing*/
  }

  // bit 6
  // 判断横摆角速度是否超限,15deg/s对应0.2094rad/s
  if (fabs(vehicle_service_output_info_ptr->yaw_rate) > 15.0 / 57.3) {
    yaw_rate_supp_duration_ += GetContext.get_param()->dt;
    if (yaw_rate_supp_duration_ > 60.0) {
      yaw_rate_supp_duration_ = 60.0;
    } else {
      /*do nothing*/
    }
  } else {
    yaw_rate_supp_duration_ = 0.0;
  }
  if (yaw_rate_supp_duration_ > 5.0) {
    disable_code += uint16_bit[6];
  } else {
    /*do nothing*/
  }

  // bit 7
  // 方向盘转速过大，方向盘转速(deg/s)和车速(km/h)满足如下公式:
  // 方向盘转速≥150-0.5*v，0<v≤100
  // 方向盘转速≥100，v＞100
  // PS1:1.8：mps转换为kph   PS2:使用实际车速 PS:滞回区间
  double steering_wheel_angle_speed_supp_thrd = 0.0;
  if (vehicle_service_output_info_ptr->vehicle_speed <= 100.0 / 3.6) {
    steering_wheel_angle_speed_supp_thrd =
        (150.0 - 1.8 * vehicle_service_output_info_ptr->vehicle_speed) / 57.3;
  } else {
    steering_wheel_angle_speed_supp_thrd = 100.0;
  }
  if ((fabs(vehicle_service_output_info_ptr->steering_wheel_angle_speed) >=
       steering_wheel_angle_speed_supp_thrd) &&
      (ldw_state_ !=
       iflyauto::LDWFunctionFSMWorkState::
           LDW_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION) &&
      (ldw_state_ != iflyauto::LDWFunctionFSMWorkState::
                         LDW_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION)

  ) {
    disable_code += uint16_bit[7];
  } else {
    /*do nothing*/
  }

  // bit 8
  // 驾驶员未踩下制动踏板:制动力>3bar，持续0.2s
  if (vehicle_service_output_info_ptr->esp_pressure > 10.0) {
    brake_pedal_pressed_supp_duration_ += GetContext.get_param()->dt;
    if (brake_pedal_pressed_supp_duration_ > 60.0) {
      brake_pedal_pressed_supp_duration_ = 60.0;
    } else {
      /*do nothing*/
    }
  } else {
    brake_pedal_pressed_supp_duration_ = 0.0;
  }
  if (brake_pedal_pressed_supp_duration_ > 0.2) {
    disable_code += uint16_bit[8];
  } else {
    /*do nothing*/
  }
  // bit 9
  // 油门踏板变化率>70%/s
  if (GetContext.get_state_info()->accelerator_pedal_pos_rate > 70.0) {
    disable_code += uint16_bit[9];
  } else {
    /*do nothing*/
  }

  // bit 10
  // AEB未激活
  // 0:Not Ready 1:Ready 2:Active 3:Temporary Failed 4:Permanently Failed
  if (vehicle_service_output_info_ptr->aeb_actuator_status == 2) {
    disable_code += uint16_bit[10];
  } else { /*do nothing*/
  }

  // bit 11
  // ABS/TCS/VDC未激活
  if ((vehicle_service_output_info_ptr->abs_active == true) ||
      (vehicle_service_output_info_ptr->tcs_active == true) ||
      (vehicle_service_output_info_ptr->esp_vdc_active == true)) {
    disable_code += uint16_bit[11];
  } else {
    /*do nothing*/
  }

  // bit 12
  // 弯道半径>200m
  // current_lane_curv_enable_flag当前道线曲率是否满足条件
  // 1：满足条件，0：不满足条件,弯道过急，功能退出.
  bool current_lane_curv_enable_flag = true;
  if (((GetContext.get_road_info()->current_lane.left_line.c2 > 0.5 / 200.0) &&
       ((GetContext.get_road_info()->current_lane.left_line.valid == true))) ||
      ((GetContext.get_road_info()->current_lane.right_line.c2 > 0.5 / 200.0) &&
       (GetContext.get_road_info()->current_lane.right_line.valid == true))) {
    current_lane_curv_enable_flag = false;
  } else {
    current_lane_curv_enable_flag = true;
  }
  if (current_lane_curv_enable_flag =
          false &&
          (ldw_state_ !=
           iflyauto::LDWFunctionFSMWorkState::
               LDW_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION) &&
          (ldw_state_ !=
           iflyauto::LDWFunctionFSMWorkState::
               LDW_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION)) {
    disable_code += uint16_bit[12];
  } else {
    /*do nothing*/
  }

  // bit 13
  // 雨刮状态判断
  if (vehicle_service_output_info_ptr->wiper_state ==
      iflyauto::WiperStateEnum::WiperState_HighSpeed) {
    wiper_state_supp_duration_ += GetContext.get_param()->dt;
    if (wiper_state_supp_duration_ > 60.0) {
      wiper_state_supp_duration_ = 60.0;
    } else {
      /*do nothing*/
    }
  } else {
    wiper_state_supp_duration_ = 0.0;
  }
  if (wiper_state_supp_duration_ > 15.0) {
    disable_code += uint16_bit[13];
  } else {
    /*do nothing*/
  }

  // bit 14
  // 安全带状态判断
  if ((vehicle_service_output_info_ptr->fl_seat_belt_state == false)) {
    disable_code += uint16_bit[14];
  } else {
    /*do nothing*/
  }
  return disable_code & GetContext.get_param()->ldw_disable_code_maskcode;
}

uint32 LdwCore::UpdateLdwFaultCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;

  uint32 ldw_fault_code = 0;

  // bit 0
  // 实际车速信号无效
  if ((vehicle_service_output_info_ptr->vehicle_speed_available == false)) {
    ldw_fault_code += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // bit 1
  // 仪表车速信号无效
  if ((vehicle_service_output_info_ptr->vehicle_speed_display_available ==
       false)) {
    ldw_fault_code += uint16_bit[1];
  } else {
    /*do nothing*/
  }

  // bit 2
  // 横摆角速度信号无效
  if ((vehicle_service_output_info_ptr->yaw_rate_available == false)) {
    ldw_fault_code += uint16_bit[2];
  } else {
    /*do nothing*/
  }

  // bit 3
  // 方向盘转角速度信号无效
  if ((vehicle_service_output_info_ptr->steering_wheel_angle_available ==
       false)) {
    ldw_fault_code += uint16_bit[3];
  } else {
    /*do nothing*/
  }

  // bit 4
  // 方向盘转速信号无效
  if ((vehicle_service_output_info_ptr->steering_wheel_angle_speed_available ==
       false)) {
    ldw_fault_code += uint16_bit[4];
  } else {
    /*do nothing*/
  }

  // bit 5
  // 油门踏板信号无效
  if ((vehicle_service_output_info_ptr->accelerator_pedal_pos_available ==
       false)) {
    ldw_fault_code += uint16_bit[5];
  } else {
    /*do nothing*/
  }

  // bit 6
  // 制动压力信号无效，使用实际制动踏板开度有效性 (true:有效/false:无效)
  if ((vehicle_service_output_info_ptr->esp_pressure_available == false)) {
    ldw_fault_code += uint16_bit[6];
  } else {
    /*do nothing*/
  }

  // bit 7
  // 转向灯信号无效
  if ((vehicle_service_output_info_ptr->turn_switch_state_available == false)) {
    ldw_fault_code += uint16_bit[7];
  } else {
    /*do nothing*/
  }

  // bit 8
  // 车道线融合模块节点通讯丢失
  if ((GetContext.mutable_state_info()->road_info_node_valid == false)) {
    ldw_fault_code += uint16_bit[8];
  } else {
    /*do nothing*/
  }

  // bit 9
  // vehicle_service模块节点通讯丢失
  if ((GetContext.mutable_state_info()->vehicle_service_node_valid == false)) {
    ldw_fault_code += uint16_bit[9];
  } else {
    /*do nothing*/
  }

  // bit 10
  // 定位模块节点通讯丢失
  if ((GetContext.mutable_state_info()->localization_info_node_valid ==
       false)) {
    ldw_fault_code += uint16_bit[10];
  } else {
    /*do nothing*/
  }

  return ldw_fault_code & GetContext.get_param()->ldw_fault_code_maskcode;
}

uint32 LdwCore::UpdateLdwLeftSuppressionCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto function_state_machine_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .function_state_machine_info;

  uint32 ldw_left_suppression_code = 0;

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

  // bit 0
  // 判断左转向灯处于关闭时长是否满足
  if (GetContext.mutable_state_info()->left_turn_light_off_time <
      ldw_param_.supp_turn_light_recovery_time) {
    ldw_left_suppression_code += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // bit 1
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

  // bit 2
  // 距上次触发报警后,越过了重置线
  if (left_suppress_repeat_warning_flag_ == true) {
    ldw_left_suppression_code += uint16_bit[2];
  } else {
    /*do nothing*/
  }

  // bit3
  // 距离上一次LDW报警结束后，冷却超过3s
  if (ldw_state_ != iflyauto::LDWFunctionFSMWorkState::
                        LDW_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION &&
      ldw_state_ != iflyauto::LDWFunctionFSMWorkState::
                        LDW_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION) {
    LDW_CoolingTime_duration_ += GetContext.get_param()->dt;
    if (LDW_CoolingTime_duration_ > 60.0) {
      LDW_CoolingTime_duration_ = 60.0;
    } else {
      /*do nothing*/
    }
  } else {
    LDW_CoolingTime_duration_ = 0.0;
  }
  if (LDW_CoolingTime_duration_ < 3.0) {
    ldw_left_suppression_code += uint16_bit[3];
  } else {
    /*do nothing*/
  }

  // bit 4
  // 左侧纠偏
  if ((ldw_left_intervention_ == false) &&
      (ldw_state_ !=
       iflyauto::LDWFunctionFSMWorkState::
           LDW_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION)) {
    ldw_left_suppression_code += uint16_bit[4];
  } else {
    /*do nothing*/
  }

  // bit 5
  if (((function_state_machine_info_ptr->current_state >=
        iflyauto::FunctionalState_SCC_ACTIVATE) &&
       (function_state_machine_info_ptr->current_state <=
        iflyauto::FunctionalState_SCC_OVERRIDE)) ||
      ((function_state_machine_info_ptr->current_state >=
        iflyauto::FunctionalState_NOA_ACTIVATE) &&
       (function_state_machine_info_ptr->current_state <=
        iflyauto::FunctionalState_NOA_OVERRIDE))) {
    ldw_left_suppression_code += uint16_bit[5];
  } else {
    /*do nothing*/
  }

  // bit 6
  // 左侧为非虚拟线
  if (GetContext.get_road_info()->current_lane.left_line.line_type ==
      context::Enum_LineType::Enum_LineType_Virtual) {
    ldw_left_suppression_code += uint16_bit[6];
  } else {
    /*do nothing*/
  }

  // bit 7
  // 左侧有分流口
  if ((GetContext.get_road_info()->current_lane.left_sideway_exist_flag ||
       GetContext.get_road_info()->current_lane.right_sideway_exist_flag) &&
      (GetContext.get_param()->force_no_sideway_switch == false)) {
    // 左侧有分流口
    ldw_left_suppression_code += uint16_bit[7];
  } else {
    /*do nothing*/
  }

  // bit 8
  // 触发侧有旁侧车or前车  非触发侧有旁侧车or前车
  if (GetContext.get_road_info()->current_lane.right_parallel_car_flag ==
          true ||
      GetContext.get_road_info()->current_lane.right_front_car_flag == true) {
    // 右侧有并行车或右前方一定区域内有车、或者正前方一定区域内有车，且有碰撞风险
    ldw_left_suppression_code += uint16_bit[8];
  } else {
    /*do nothing*/
  }

  
    // bit 9
  // 换道之后抑制两秒
  if (GetContext.get_road_info()->current_lane.lane_changed_flag == false) {
    LDW_LaneChange_duration_ += GetContext.get_param()->dt;
    if (LDW_LaneChange_duration_ > 60.0) {
      LDW_LaneChange_duration_ = 60.0;
    } else {
      /*do nothing*/
    }
  } else {
    LDW_LaneChange_duration_ = 0.0;
  }
  if (LDW_LaneChange_duration_ <= 2) {
    ldw_left_suppression_code += uint16_bit[9];
  } else {
    /*do nothing*/
  }  

  return ldw_left_suppression_code &
         GetContext.get_param()->ldw_left_suppression_code_maskcode;
}

uint32 LdwCore::UpdateLdwLeftKickDownCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;

  auto function_state_machine_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .function_state_machine_info;

  uint32 ldw_left_kickdown_code = 0;

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

  // bit 0
  if (vehicle_service_output_info_ptr->left_turn_light_state == true) {
    ldw_left_kickdown_code += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // bit 1
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

  // bit 2
  // 单次报警时间不超过2秒
  if (ldw_left_warning_time_ > ldw_param_.warning_time_max) {
    ldw_left_kickdown_code += uint16_bit[2];
  } else {
    /*do nothing*/
  }

  // bit 3
  // 暂时空置

  // bit 4
  // 左侧纠偏
  if (ldw_left_intervention_ == false &&
      ldw_left_warning_time_ > ldw_param_.warning_time_min) {
    ldw_left_kickdown_code += uint16_bit[4];
  } else {
    /*do nothing*/
  }

  // bit 5
  // 当前状态
  if (((function_state_machine_info_ptr->current_state >=
        iflyauto::FunctionalState_SCC_ACTIVATE) &&
       (function_state_machine_info_ptr->current_state <=
        iflyauto::FunctionalState_SCC_OVERRIDE)) ||
      ((function_state_machine_info_ptr->current_state >=
        iflyauto::FunctionalState_NOA_ACTIVATE) &&
       (function_state_machine_info_ptr->current_state <=
        iflyauto::FunctionalState_NOA_OVERRIDE))) {
    ldw_left_kickdown_code += uint16_bit[5];
  } else {
    /*do nothing*/
  }

  // bit 6
  // 偏离侧为虚拟道线
  if (GetContext.get_road_info()->current_lane.left_line.line_type ==
      context::Enum_LineType::Enum_LineType_Virtual) {
    ldw_left_kickdown_code += uint16_bit[6];
  } else {
    /*do nothing*/
  }

  return ldw_left_kickdown_code &
         GetContext.get_param()->ldw_left_kickdown_code_maskcode;
}

uint32 LdwCore::UpdateLdwRightSuppressionCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto function_state_machine_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .function_state_machine_info;

  uint32 ldw_right_suppression_code = 0;

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

  // bit 0
  // 判断右转向灯处于关闭时长是否满足
  if (GetContext.mutable_state_info()->right_turn_light_off_time <
      ldw_param_.supp_turn_light_recovery_time) {
    ldw_right_suppression_code += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // bit 1
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

  // bit 2
  // 距上次触发报警后,越过了重置线
  if (right_suppress_repeat_warning_flag_ == true) {
    ldw_right_suppression_code += uint16_bit[2];
  } else {
    /*do nothing*/
  }

  // bit 3
  // 距离上一次LDW报警结束后，冷却超过3s
  if (ldw_state_ != iflyauto::LDWFunctionFSMWorkState::
                        LDW_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION &&
      ldw_state_ != iflyauto::LDWFunctionFSMWorkState::
                        LDW_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION) {
    LDW_CoolingTime_duration_ += GetContext.get_param()->dt;
    if (LDW_CoolingTime_duration_ > 60.0) {
      LDW_CoolingTime_duration_ = 60.0;
    } else {
      /*do nothing*/
    }
  } else {
    LDW_CoolingTime_duration_ = 0.0;
  }
  if (LDW_CoolingTime_duration_ < 3.0) {
    ldw_right_suppression_code += uint16_bit[3];
  } else {
    /*do nothing*/
  }

  // bit 4
  // 右侧纠偏
  if ((ldw_right_intervention_ == false) &&
      (ldw_state_ !=
       iflyauto::LDWFunctionFSMWorkState::
           LDW_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION)) {
    ldw_right_suppression_code += uint16_bit[4];
  } else {
    /*do nothing*/
  }

  // bit 5
  if (((function_state_machine_info_ptr->current_state >=
        iflyauto::FunctionalState_SCC_ACTIVATE) &&
       (function_state_machine_info_ptr->current_state <=
        iflyauto::FunctionalState_SCC_OVERRIDE)) ||
      ((function_state_machine_info_ptr->current_state >=
        iflyauto::FunctionalState_NOA_ACTIVATE) &&
       (function_state_machine_info_ptr->current_state <=
        iflyauto::FunctionalState_NOA_OVERRIDE))) {
    ldw_right_suppression_code += uint16_bit[5];
  }

  // bit6
  // 右侧为虚拟道线
  if (GetContext.get_road_info()->current_lane.right_line.line_type ==
      context::Enum_LineType::Enum_LineType_Virtual) {
    ldw_right_suppression_code += uint16_bit[6];
  } else {
    /*do nothing*/
  }
  // bit 7
  // 右侧有分流口
  if ((GetContext.get_road_info()->current_lane.left_sideway_exist_flag ||
       GetContext.get_road_info()->current_lane.right_sideway_exist_flag) &&
      (GetContext.get_param()->force_no_sideway_switch == false)) {
    // 右侧有分流口
    ldw_right_suppression_code += uint16_bit[7];
  } else {
    /*do nothing*/
  }

  // bit 8
  //
  if (GetContext.get_road_info()->current_lane.left_parallel_car_flag == true ||
      GetContext.get_road_info()->current_lane.left_front_car_flag == true) {
    // 左侧有并行车或左前方一定区域内有车、或者正前方一定区域内有车，且有碰撞风险
    ldw_right_suppression_code += uint16_bit[8];
  } else {
    /*do nothing*/
  }

    // bit 10
  // 换道之后抑制两秒
  if (GetContext.get_road_info()->current_lane.lane_changed_flag == false) {
    LDW_LaneChange_duration_ += GetContext.get_param()->dt;
    if (LDW_LaneChange_duration_ > 60.0) {
      LDW_LaneChange_duration_ = 60.0;
    } else {
      /*do nothing*/
    }
  } else {
    LDW_LaneChange_duration_ = 0.0;
  }
  if (LDW_LaneChange_duration_ <= 2) {
    ldw_right_suppression_code += uint16_bit[9];
  } else {
    /*do nothing*/
  }  

  return ldw_right_suppression_code &
         GetContext.get_param()->ldw_right_suppression_code_maskcode;
}

uint32 LdwCore::UpdateLdwRightKickDownCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;

  auto function_state_machine_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .function_state_machine_info;

  uint32 ldw_right_kickdown_code = 0;

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

  // bit 0
  if (vehicle_service_output_info_ptr->right_turn_light_state == true) {
    ldw_right_kickdown_code += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // bit 1
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

  // bit 2
  if (ldw_right_warning_time_ > ldw_param_.warning_time_max) {
    ldw_right_kickdown_code += uint16_bit[2];
  }

  // bit4
  if (ldw_right_intervention_ == false &&
      ldw_right_warning_time_ > ldw_param_.warning_time_min) {
    ldw_right_kickdown_code += uint16_bit[4];
  } else {
    /*do nothing*/
  }

  // bit 5
  if (((function_state_machine_info_ptr->current_state >=
        iflyauto::FunctionalState_SCC_ACTIVATE) &&
       (function_state_machine_info_ptr->current_state <=
        iflyauto::FunctionalState_SCC_OVERRIDE)) ||
      ((function_state_machine_info_ptr->current_state >=
        iflyauto::FunctionalState_NOA_ACTIVATE) &&
       (function_state_machine_info_ptr->current_state <=
        iflyauto::FunctionalState_NOA_OVERRIDE))) {
    ldw_right_kickdown_code += uint16_bit[5];
  } else {
    /*do nothing*/
  }

  // bit 6
  if (GetContext.get_road_info()->current_lane.right_line.line_type ==
      context::Enum_LineType::Enum_LineType_Virtual) {
    ldw_right_kickdown_code += uint16_bit[6];
  } else {
    /*do nothing*/
  }

  return ldw_right_kickdown_code &
         GetContext.get_param()->ldw_right_kickdown_code_maskcode;
}

double LdwCore::UpdateTlcThreshold(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  auto hmi_mcu_inner_info_ptr = &GetContext.mutable_session()
                                     ->mutable_environmental_model()
                                     ->get_local_view()
                                     .hmi_mcu_inner_info;
  // 定义基础tlc_base值
  double tlc_calculate_base = 0;
  if (GetContext.get_param()->ldw_tlc_thrd > 0.1) {
    // return GetContext.get_param()->ldw_tlc_thrd;
    tlc_calculate_base = GetContext.get_param()->ldw_tlc_thrd;
    ;
  } else {
    if (hmi_mcu_inner_info_ptr->ldw_set_sensitivity_level ==
        iflyauto::interface_2_4_5::SensitivityLevel::SENSITIVITY_LEVEL_LOW) {
      tlc_calculate_base = ldw_tlc_near_;
    } else if (hmi_mcu_inner_info_ptr->ldw_set_sensitivity_level ==
               iflyauto::interface_2_4_5::SensitivityLevel::
                   SENSITIVITY_LEVEL_MIDDLE) {
      tlc_calculate_base = ldw_tlc_medium_;
    } else if (hmi_mcu_inner_info_ptr->ldw_set_sensitivity_level ==
               iflyauto::interface_2_4_5::SensitivityLevel::
                   SENSITIVITY_LEVEL_HIGH) {
      tlc_calculate_base = ldw_tlc_far_;
    } else {
      tlc_calculate_base = ldw_tlc_medium_;
    }
  }

  // 定义根据车速确定基础tlc
  //  tlc_calculate_base = pnc::mathlib::Interp1(
  //      GetContext.get_param()->lka_vel_vector,
  //      GetContext.get_param()->lka_tlc_vector,
  //      (GetContext.get_state_info()->display_vehicle_speed * 3.6));

  // 定义弯道tlc减少
  double tlc_dec_by_curv = 0.0;
  double C2_temp_thr = 0.0;
  if (GetContext.get_road_info()->current_lane.left_line.valid == true) {
    C2_temp_thr = GetContext.get_road_info()->current_lane.left_line.c2;
  } else if (GetContext.get_road_info()->current_lane.right_line.valid ==
             true) {
    C2_temp_thr = GetContext.get_road_info()->current_lane.right_line.c2;
  } else {
    C2_temp_thr = 0.00005;
  }
  tlc_dec_by_curv = pnc::mathlib::Interp1(
      GetContext.get_param()->lka_c2_vector,
      GetContext.get_param()->lka_dec_tlc_by_c2_vector, fabs(C2_temp_thr));

  // 定义窄道tlc减少
  double tlc_dec_by_narrowroad = 0.0;
  double lane_width_temp_thr = 0.0;
  if (GetContext.get_road_info()->current_lane.lane_width_valid) {
    lane_width_temp_thr = GetContext.get_road_info()->current_lane.lane_width;
  } else {
    lane_width_temp_thr = 10.0;  // 道宽信息不可用，默认不减益tlc
  }
  tlc_dec_by_narrowroad = pnc::mathlib::Interp1(
      GetContext.get_param()->lka_lane_width_vector,
      GetContext.get_param()->lka_tlc_dec_by_lane_width_vector,
      lane_width_temp_thr);

  double tlc_thrd =
      tlc_calculate_base - tlc_dec_by_curv - tlc_dec_by_narrowroad;
  if (tlc_thrd < 0.1) {
    tlc_thrd = 0.1;
  } else {
    // do nothing
  }

  return tlc_thrd;
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