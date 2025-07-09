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

uint32 LdpCore::UpdateLdpEnableCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;

  uint32 enable_code = 0;

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
  // 判断是否至少识别到一侧道线
  if ((!GetContext.mutable_road_info()->current_lane.left_line.valid) &&
      (!GetContext.mutable_road_info()->current_lane.right_line.valid)) {
    enable_code += uint16_bit[1];
  } else {
    /*do nothing*/
  }

  // bit 2
  // 判断当前车道宽度是否满足激活条件
  if (GetContext.get_road_info()->current_lane.lane_width_valid) {
    if ((GetContext.mutable_road_info()->current_lane.lane_width < 2.8 ||
         GetContext.mutable_road_info()->current_lane.lane_width > 5.2)) {
      enable_code += uint16_bit[2];
    } else {
      /*do nothing*/
    }
  } else {
    /*当一侧既没有路沿也没有道线时，不作道宽判断*/
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
  // 判断横摆角速度是否超限,12deg/s对应0.2094rad/s
  // :横摆角速度绝对值<12deg/s，持续1s
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
  // PS1:1.8：mps转换为kph   PS2:使用实际车速
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
    str_wheel_ang_speed_recover_duration_ = 0.0;
  }
  if (str_wheel_ang_speed_recover_duration_ > 1.0) {
    enable_code += uint16_bit[7];
  } else {
    /*do nothing*/
  }

  // bit 8
  // 驾驶员未踩下制动踏板:制动力<3bar，持续2s.后续再讨论
  if (vehicle_service_output_info_ptr->esp_pressure < 3.0) {
    brake_pedal_pressed_supp_recover_duration_ += GetContext.get_param()->dt;
    if (brake_pedal_pressed_supp_recover_duration_ > 60.0) {
      brake_pedal_pressed_supp_recover_duration_ = 60.0;
    } else {
      /*do nothing*/
    }
  } else {
    brake_pedal_pressed_supp_recover_duration_ = 0.0;
  }
  if (brake_pedal_pressed_supp_recover_duration_ < 2.0) {
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

  // bit 10
  // AEB未激活
  // 0:Not Ready 1:Ready 2:Active 3:Temporary Failed 4:Permanently Failed
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

  // // bit 15
  // // ESP系统处于开启状态
  // if ((vehicle_service_output_info_ptr->esp_active == false)) {
  //   enable_code += uint16_bit[15];
  // } else {
  //   /*do nothing*/
  // }

  // bit 15
  // ESP系统无故障
  if ((vehicle_service_output_info_ptr->esp_vdc_fault == true)) {
    enable_code += uint16_bit[15];
  } else {
    /*do nothing*/
  }

  return enable_code & GetContext.get_param()->ldp_enable_code_maskcode;
}

uint32 LdpCore::UpdateLdpDisableCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;

  uint32 disable_code = 0;

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
  // 判断是否至少识别到一侧道线
  if ((!GetContext.mutable_road_info()->current_lane.left_line.valid) &&
      (!GetContext.mutable_road_info()->current_lane.right_line.valid)) {
    disable_code += uint16_bit[1];
  } else {
    /*do nothing*/
  }

  // bit 2
  // 判断当前车道宽度是否满足激活条件
  if (GetContext.get_road_info()->current_lane.lane_width_valid) {
    if ((GetContext.mutable_road_info()->current_lane.lane_width < 2.5 ||
         GetContext.mutable_road_info()->current_lane.lane_width > 5.5)) {
      disable_code += uint16_bit[2];
    } else {
      /*do nothing*/
    }
  } else {
    /*当一侧既没有路沿也没有道线时，不作道宽判断*/
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
      (ldp_state_ !=
       iflyauto::LDPFunctionFSMWorkState::
           LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION) &&
      (ldp_state_ != iflyauto::LDPFunctionFSMWorkState::
                         LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION)

  ) {
    disable_code += uint16_bit[7];
  } else {
    /*do nothing*/
  }

  // bit 8
  // 驾驶员未踩下制动踏板:制动力>10bar，持续0.2s
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
          (ldp_state_ !=
           iflyauto::LDPFunctionFSMWorkState::
               LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION) &&
          (ldp_state_ !=
           iflyauto::LDPFunctionFSMWorkState::
               LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION)) {
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

  // bit 15
  // ESP系统无故障
  if ((vehicle_service_output_info_ptr->esp_vdc_fault == true)) {
    disable_code += uint16_bit[15];
  } else {
    /*do nothing*/
  }
  // // bit 16
  // // ESP系统处于开启状态
  // if ((vehicle_service_output_info_ptr->esp_active == false)) {
  //   disable_code += uint16_bit[16];
  // } else {
  //   /*do nothing*/
  // }

  return disable_code & GetContext.get_param()->ldp_disable_code_maskcode;
}

uint32 LdpCore::UpdateLdpFaultCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;

  uint32 ldp_fault_code = 0;

  // bit 0
  // 实际车速信号无效
  if ((vehicle_service_output_info_ptr->vehicle_speed_available == false)) {
    ldp_fault_code += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // bit 1
  // 仪表车速信号无效
  if ((vehicle_service_output_info_ptr->vehicle_speed_display_available ==
       false)) {
    ldp_fault_code += uint16_bit[1];
  } else {
    /*do nothing*/
  }

  // bit 2
  // 横摆角速度信号无效
  if ((vehicle_service_output_info_ptr->yaw_rate_available == false)) {
    ldp_fault_code += uint16_bit[2];
  } else {
    /*do nothing*/
  }

  // bit 3
  // 方向盘转角速度信号无效
  if ((vehicle_service_output_info_ptr->steering_wheel_angle_available ==
       false)) {
    ldp_fault_code += uint16_bit[3];
  } else {
    /*do nothing*/
  }

  // bit 4
  // 方向盘转速信号无效
  if ((vehicle_service_output_info_ptr->steering_wheel_angle_speed_available ==
       false)) {
    ldp_fault_code += uint16_bit[4];
  } else {
    /*do nothing*/
  }

  // bit 5
  // 油门踏板信号无效
  if ((vehicle_service_output_info_ptr->accelerator_pedal_pos_available ==
       false)) {
    ldp_fault_code += uint16_bit[5];
  } else {
    /*do nothing*/
  }

  // bit 6
  // 制动压力信号无效，使用实际制动踏板开度有效性 (true:有效/false:无效)
  if ((vehicle_service_output_info_ptr->esp_pressure_available == false)) {
    ldp_fault_code += uint16_bit[6];
  } else {
    /*do nothing*/
  }

  // bit 7
  // 转向灯信号无效
  if ((vehicle_service_output_info_ptr->turn_switch_state_available == false)) {
    ldp_fault_code += uint16_bit[7];
  } else {
    /*do nothing*/
  }

  // bit 8
  // 驾驶员手力矩信号无效
  if ((vehicle_service_output_info_ptr->driver_hand_torque_available ==
       false)) {
    ldp_fault_code += uint16_bit[8];
  } else {
    /*do nothing*/
  }

  // bit 9
  // 横向控制执行器状态异常
  if ((vehicle_service_output_info_ptr
           ->pilot_lat_control_actuator_status_available == false)) {
    ldp_fault_code += uint16_bit[9];
  } else {
    /*do nothing*/
  }

  // bit 10
  // 车道线融合模块节点通讯丢失
  if ((GetContext.mutable_state_info()->road_info_node_valid == false)) {
    ldp_fault_code += uint16_bit[10];
  } else {
    /*do nothing*/
  }

  // bit 11
  // vehicle_service模块节点通讯丢失
  if ((GetContext.mutable_state_info()->vehicle_service_node_valid == false)) {
    ldp_fault_code += uint16_bit[11];
  } else {
    /*do nothing*/
  }

  // bit 12
  // 定位模块节点通讯丢失
  if ((GetContext.mutable_state_info()->localization_info_node_valid ==
       false)) {
    ldp_fault_code += uint16_bit[12];
  } else {
    /*do nothing*/
  }

  return ldp_fault_code & GetContext.get_param()->ldp_fault_code_maskcode;
}

uint32 LdpCore::UpdateLdpLeftSuppressionCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto function_state_machine_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .function_state_machine_info;

  uint32 ldp_left_suppression_code = 0;

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

  // bit 0
  // 判断左转向灯处于关闭时长是否满足
  if (GetContext.mutable_state_info()->left_turn_light_off_time <
      ldp_param_.supp_turn_light_recovery_time) {
    ldp_left_suppression_code += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // bit 1
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
  } else {
    /*do nothing*/
  }

  // bit 2
  // 距上次触发报警后,越过了重置线
  if (left_suppress_repeat_warning_flag_ == true) {
    ldp_left_suppression_code += uint16_bit[2];
  } else {
    /*do nothing*/
  }

  // bit 3
  // 车辆偏离本车道，满足触发阈值,暂时保留
  if ((ldp_left_intervention_ == false) &&
      (ldp_state_ !=
       iflyauto::LDPFunctionFSMWorkState::
           LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION)) {
    ldp_left_suppression_code += uint16_bit[3];
  } else {
    /*do nothing*/
  }

  // bit4
  // 驾驶员手力矩条件,手力矩绝对值<=1.5Nm，持续超过0.5s

  if (fabs(GetContext.mutable_state_info()->driver_hand_trq) <
      GetContext.get_param()->LDP_suppression_driver_hand_trq) {
    driver_hand_trq_supp_duration_ += GetContext.get_param()->dt;
    if (driver_hand_trq_supp_duration_ > 60.0) {
      driver_hand_trq_supp_duration_ = 60.0;
    } else {
      /*do nothing*/
    }
  } else {
    driver_hand_trq_supp_duration_ = 0.0;
  }
  if (driver_hand_trq_supp_duration_ < 0.5) {
    ldp_left_suppression_code += uint16_bit[4];
  } else {
    /*do nothing*/
  }

  // bit 5
  // 距离上一次LDp报警结束后，冷却超过3s
  if (ldp_state_ != iflyauto::LDPFunctionFSMWorkState::
                        LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION &&
      ldp_state_ != iflyauto::LDPFunctionFSMWorkState::
                        LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION) {
    LDP_CoolingTime_duration_ += GetContext.get_param()->dt;
    if (LDP_CoolingTime_duration_ > 60.0) {
      LDP_CoolingTime_duration_ = 60.0;
    } else {
      /*do nothing*/
    }
  } else {
    LDP_CoolingTime_duration_ = 0.0;
  }
  if (LDP_CoolingTime_duration_ < 3.0 &&
      fabs(GetContext.mutable_state_info()->driver_hand_trq >= 0.5)) {
    ldp_left_suppression_code += uint16_bit[5];
  } else {
    /*do nothing*/
  }

  // bit 6
  if (((function_state_machine_info_ptr->current_state >=
        iflyauto::FunctionalState_SCC_ACTIVATE) &&
       (function_state_machine_info_ptr->current_state <=
        iflyauto::FunctionalState_SCC_OVERRIDE)) ||
      ((function_state_machine_info_ptr->current_state >=
        iflyauto::FunctionalState_NOA_ACTIVATE) &&
       (function_state_machine_info_ptr->current_state <=
        iflyauto::FunctionalState_NOA_OVERRIDE))) {
    ldp_left_suppression_code += uint16_bit[6];
  } else {
    /*do nothing*/
  }

  // bit 7
  // 纠偏侧为虚拟道线
  if (GetContext.get_road_info()->current_lane.left_line.line_type ==
      context::Enum_LineType::Enum_LineType_Virtual) {
    ldp_left_suppression_code += uint16_bit[7];
  } else {
    /*do nothing*/
  }
  // bit 8
  // 纠偏侧为分流口
  if ((GetContext.get_road_info()->current_lane.left_sideway_exist_flag ||
       GetContext.get_road_info()->current_lane.right_sideway_exist_flag) &&
      (GetContext.get_param()->force_no_sideway_switch == false)) {
    // 左侧有分流口
    ldp_left_suppression_code += uint16_bit[8];
  }
  // bit 9
  // 旁侧or前方有车，且有碰撞风险
  if (GetContext.get_road_info()->current_lane.right_parallel_car_flag ==
          true ||
      GetContext.get_road_info()->current_lane.right_front_car_flag == true) {
    // 右侧有并行车或右前方一定区域内有车、或者正前方一定区域内有车，且有碰撞风险
    ldp_left_suppression_code += uint16_bit[9];
  }

  return ldp_left_suppression_code &
         GetContext.get_param()->ldp_left_suppression_code_maskcode;
}

uint32 LdpCore::UpdateLdpLeftKickDownCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;

  auto function_state_machine_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .function_state_machine_info;

  uint32 ldp_left_kickdown_code = 0;

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

  // bit 1
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
  // bit 2
  // 纠偏持续时间>最大纠偏时长(8s)
  if (ldp_left_warning_time_ > ldp_param_.warning_time_max) {
    ldp_left_kickdown_code += uint16_bit[2];
  } else {
    /*do nothing*/
  }

  // bit 3
  // 干预完成:干预触发后，横向偏离速度绝对值<0.3m/s，持续一段时间。
  bool intervention_finish_flag_departure_speed = false;
  if (GetContext.get_road_info()->current_lane.left_line.valid) {
    if (fabs(GetContext.mutable_state_info()->veh_left_departure_speed) < 0.3) {
      intervention_finish_flag_departure_speed = true;
    }
  } else if (GetContext.get_road_info()->current_lane.right_line.valid) {
    if (fabs(GetContext.mutable_state_info()->veh_right_departure_speed) <
        0.3) {
      intervention_finish_flag_departure_speed = true;
    }
  } else {
    /*do nothing*/
  }

  if (intervention_finish_flag_departure_speed &&
      ldp_state_ == iflyauto::LDPFunctionFSMWorkState::
                        LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION) {
    ldp_left_kickdown_lat_v_duration_ += GetContext.get_param()->dt;
    if (ldp_left_kickdown_lat_v_duration_ > 60.0) {
      ldp_left_kickdown_lat_v_duration_ = 60.0;
    } else {
      /*do nothing*/
    }
  } else {
    ldp_left_kickdown_lat_v_duration_ = 0.0;
  }
  if ((ldp_left_kickdown_lat_v_duration_ >
       GetContext.get_param()->ldp_kickdown_lat_v_dur) &&
      (GetContext.mutable_state_info()->veh_left_departure_speed > 0.05)) {
    ldp_left_kickdown_code += uint16_bit[3];
  } else {
    /*do nothing*/
  }

  // bit 4
  // 驾驶员手力矩条件,反向手力矩>3.5Nm 或 同向手力矩>2.5Nm
  // 驾驶员手力矩 单位:Nm 左转弯为正，右转弯为负
  bool ldp_handtrq_abs_flag = false;
  if (fabs(GetContext.mutable_state_info()->driver_hand_trq) >
          GetContext.get_param()->LDP_kickdown_abs_hand_trq &&
      ldp_state_ == iflyauto::LDPFunctionFSMWorkState::
                        LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION) {
    ldp_left_handtrq_kickdown_duration_ += GetContext.get_param()->dt;
    if (ldp_left_handtrq_kickdown_duration_ > 60.0) {
      ldp_left_handtrq_kickdown_duration_ = 60.0;
    } else {
      /*do nothing*/
    }
  } else {
    ldp_left_handtrq_kickdown_duration_ = 0.0;
  }
  if (ldp_left_handtrq_kickdown_duration_ >
      GetContext.get_param()->LDP_kickdown_hand_trq_dur) {
    ldp_handtrq_abs_flag = true;
  } else {
    ldp_handtrq_abs_flag = false;
  }

  if (GetContext.mutable_state_info()->driver_hand_trq >
          GetContext.get_param()->LDP_kickdown_oppodir_hand_trq ||
      GetContext.mutable_state_info()->driver_hand_trq <
          -GetContext.get_param()->LDP_kickdown_samedir_hand_trq ||
      ldp_handtrq_abs_flag) {
    ldp_left_kickdown_code += uint16_bit[4];
  } else {
    /*do nothing*/
  }

  // bit 5
  // 预留

  // bit 6
  if (((function_state_machine_info_ptr->current_state >=
        iflyauto::FunctionalState_SCC_ACTIVATE) &&
       (function_state_machine_info_ptr->current_state <=
        iflyauto::FunctionalState_SCC_OVERRIDE)) ||
      ((function_state_machine_info_ptr->current_state >=
        iflyauto::FunctionalState_NOA_ACTIVATE) &&
       (function_state_machine_info_ptr->current_state <=
        iflyauto::FunctionalState_NOA_OVERRIDE))) {
    ldp_left_kickdown_code += uint16_bit[6];
  } else {
    /*do nothing*/
  }

  // bit 7
  // 偏离侧道线为虚拟线
  if (GetContext.get_road_info()->current_lane.left_line.line_type ==
      context::Enum_LineType::Enum_LineType_Virtual) {
    ldp_left_kickdown_code += uint16_bit[7];
  } else {
    /*do nothing*/
  }

  return ldp_left_kickdown_code &
         GetContext.get_param()->ldp_left_kickdown_code_maskcode;
}

uint32 LdpCore::UpdateLdpRightSuppressionCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto function_state_machine_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .function_state_machine_info;

  uint32 ldp_right_suppression_code = 0;

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

  // bit 0
  // 判断右转向灯处于关闭时长是否满足
  if (GetContext.mutable_state_info()->right_turn_light_off_time <
      ldp_param_.supp_turn_light_recovery_time) {
    ldp_right_suppression_code += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // bit 1
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

  // bit 2
  // 距上次触发报警后,未越过了重置线
  if (right_suppress_repeat_warning_flag_ == true) {
    ldp_right_suppression_code += uint16_bit[2];
  } else {
    /*do nothing*/
  }

  // bit 3
  // 车辆偏离本车道，满足触发阈值
  if ((ldp_right_intervention_ == false) &&
      (ldp_state_ !=
       iflyauto::LDPFunctionFSMWorkState::
           LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION)) {
    ldp_right_suppression_code += uint16_bit[3];
  } else {
    /*do nothing*/
  }

  // bit 4
  // 驾驶员手力矩条件
  if (fabs(GetContext.mutable_state_info()->driver_hand_trq) <
      GetContext.get_param()->LDP_suppression_driver_hand_trq) {
    driver_hand_trq_supp_duration_ += GetContext.get_param()->dt;
    if (driver_hand_trq_supp_duration_ > 60.0) {
      driver_hand_trq_supp_duration_ = 60.0;
    } else {
      /*do nothing*/
    }
  } else {
    driver_hand_trq_supp_duration_ = 0.0;
  }
  if (driver_hand_trq_supp_duration_ < 0.5) {
    ldp_right_suppression_code += uint16_bit[4];
  } else {
    /*do nothing*/
  }

  // bit5
  // 距离上一次LDW报警结束后，冷却超过3s
  if (ldp_state_ != iflyauto::LDPFunctionFSMWorkState::
                        LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION &&
      ldp_state_ != iflyauto::LDPFunctionFSMWorkState::
                        LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION) {
    LDP_CoolingTime_duration_ += GetContext.get_param()->dt;
    if (LDP_CoolingTime_duration_ > 60.0) {
      LDP_CoolingTime_duration_ = 60.0;
    } else {
      /*do nothing*/
    }
  } else {
    LDP_CoolingTime_duration_ = 0.0;
  }
  if (LDP_CoolingTime_duration_ < 3.0 &&
      fabs(GetContext.mutable_state_info()->driver_hand_trq >= 0.5)) {
    ldp_right_suppression_code += uint16_bit[5];
  } else {
    /*do nothing*/
  }

  // bit 6
  if (((function_state_machine_info_ptr->current_state >=
        iflyauto::FunctionalState_SCC_ACTIVATE) &&
       (function_state_machine_info_ptr->current_state <=
        iflyauto::FunctionalState_SCC_OVERRIDE)) ||
      ((function_state_machine_info_ptr->current_state >=
        iflyauto::FunctionalState_NOA_ACTIVATE) &&
       (function_state_machine_info_ptr->current_state <=
        iflyauto::FunctionalState_NOA_OVERRIDE))) {
    ldp_right_suppression_code += uint16_bit[6];
  } else {
    /*do nothing*/
  }

  // bit 7
  // 触发侧为虚拟道线
  if (GetContext.get_road_info()->current_lane.right_line.line_type ==
      context::Enum_LineType::Enum_LineType_Virtual) {
    ldp_right_suppression_code += uint16_bit[7];
  } else {
    /*do nothing*/
  }

  // bit 8
  // 触发侧为分流口
  if ((GetContext.get_road_info()->current_lane.left_sideway_exist_flag ||
       GetContext.get_road_info()->current_lane.right_sideway_exist_flag) &&
      (GetContext.get_param()->force_no_sideway_switch == false)) {
    // 右侧有分流口
    ldp_right_suppression_code += uint16_bit[8];
  }
  // bit9
  // 左侧有并行车或左前方一定区域内有车、或者正前方一定区域内有车，且有碰撞风险
  if (GetContext.get_road_info()->current_lane.left_parallel_car_flag == true ||
      GetContext.get_road_info()->current_lane.left_front_car_flag == true) {
    // 左侧有并行车或左前方一定区域内有车、或者正前方一定区域内有车，且有碰撞风险
    ldp_right_suppression_code += uint16_bit[9];
  }

  return ldp_right_suppression_code &
         GetContext.get_param()->ldp_right_suppression_code_maskcode;
}

uint32 LdpCore::UpdateLdpRightKickDownCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;

  auto function_state_machine_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .function_state_machine_info;

  uint32 ldp_right_kickdown_code = 0;

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

  // bit 0
  // 转向灯
  if (vehicle_service_output_info_ptr->right_turn_light_state == true) {
    ldp_right_kickdown_code += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // bit 1
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

  // bit 2
  if (ldp_right_warning_time_ > ldp_param_.warning_time_max) {
    ldp_right_kickdown_code += uint16_bit[2];
  }
  // bit 3
  bool intervention_finish_flag_departure_speed = false;
  if (GetContext.get_road_info()->current_lane.left_line.valid) {
    if (fabs(GetContext.mutable_state_info()->veh_left_departure_speed) < 0.3) {
      intervention_finish_flag_departure_speed = true;
    }
  } else if (GetContext.get_road_info()->current_lane.right_line.valid) {
    if (fabs(GetContext.mutable_state_info()->veh_right_departure_speed) <
        0.3) {
      intervention_finish_flag_departure_speed = true;
    }
  } else {
    /*do nothing*/
  }

  if (intervention_finish_flag_departure_speed &&
      ldp_state_ == iflyauto::LDPFunctionFSMWorkState::
                        LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION) {
    ldp_right_kickdown_lat_v_duration_ += GetContext.get_param()->dt;
    if (ldp_right_kickdown_lat_v_duration_ > 60.0) {
      ldp_right_kickdown_lat_v_duration_ = 60.0;
    } else {
      /*do nothing*/
    }
  } else {
    ldp_right_kickdown_lat_v_duration_ = 0.0;
  }
  if ((ldp_right_kickdown_lat_v_duration_ >
       GetContext.get_param()->ldp_kickdown_lat_v_dur) &&
      (GetContext.mutable_state_info()->veh_right_departure_speed < -0.05)) {
    ldp_right_kickdown_code += uint16_bit[3];
  } else {
    /*do nothing*/
  }

  // bit 4,手力矩条件
  // 反向手力矩>3.5Nm 或 同向手力矩>2.5Nm. 单位:Nm 左转弯为正，右转弯为负

  bool ldp_handtrq_abs_flag = false;
  if (fabs(GetContext.mutable_state_info()->driver_hand_trq) >
          GetContext.get_param()->LDP_kickdown_abs_hand_trq &&
      ldp_state_ == iflyauto::LDPFunctionFSMWorkState::
                        LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION) {
    ldp_right_handtrq_kickdown_duration_ += GetContext.get_param()->dt;
    if (ldp_right_handtrq_kickdown_duration_ > 60.0) {
      ldp_right_handtrq_kickdown_duration_ = 60.0;
    } else {
      /*do nothing*/
    }
  } else {
    ldp_right_handtrq_kickdown_duration_ = 0.0;
  }
  if (ldp_right_handtrq_kickdown_duration_ >
      GetContext.get_param()->LDP_kickdown_hand_trq_dur) {
    ldp_handtrq_abs_flag = true;
  } else {
    ldp_handtrq_abs_flag = false;
  }

  if (GetContext.mutable_state_info()->driver_hand_trq <
          -GetContext.get_param()->LDP_kickdown_oppodir_hand_trq ||
      GetContext.mutable_state_info()->driver_hand_trq >
          GetContext.get_param()->LDP_kickdown_samedir_hand_trq ||
      ldp_handtrq_abs_flag) {
    ldp_right_kickdown_code += uint16_bit[4];
  } else {
    /*do nothing*/
  }

  // // bit 4
  // // 驾驶员手力矩条件
  // // 驾驶员手力矩 单位:Nm 左转弯为正，右转弯为负
  // if (fabs(GetContext.mutable_state_info()->driver_hand_trq) >
  //     ldp_param_.kickdown_driver_hand_trq) {
  //   ldp_right_kickdown_code += uint16_bit[4];
  // } else {
  //   /*do nothing*/
  // }

  // bit 5

  // bit 6
  if (((function_state_machine_info_ptr->current_state >=
        iflyauto::FunctionalState_SCC_ACTIVATE) &&
       (function_state_machine_info_ptr->current_state <=
        iflyauto::FunctionalState_SCC_OVERRIDE)) ||
      ((function_state_machine_info_ptr->current_state >=
        iflyauto::FunctionalState_NOA_ACTIVATE) &&
       (function_state_machine_info_ptr->current_state <=
        iflyauto::FunctionalState_NOA_OVERRIDE))) {
    ldp_right_kickdown_code += uint16_bit[6];
  } else {
    /*do nothing*/
  }

  // bit 7
  if (GetContext.get_road_info()->current_lane.right_line.line_type ==
      context::Enum_LineType::Enum_LineType_Virtual) {
    ldp_right_kickdown_code += uint16_bit[7];
  } else {
    /*do nothing*/
  }

  return ldp_right_kickdown_code &
         GetContext.get_param()->ldp_right_kickdown_code_maskcode;
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

double LdpCore::UpdateTlcThreshold(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  double tlc_calculate_base = GetContext.get_param()->ldp_tlc_thrd;
  tlc_calculate_base = pnc::mathlib::Interp1(
      GetContext.get_param()->lka_vel_vector,
      GetContext.get_param()->lka_tlc_vector,
      (GetContext.get_state_info()->display_vehicle_speed * 3.6));

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

  // 测试代码可删除
  //  for (int i = 0; i < 50; i++) {
  //    double R = 10000 - i * 500;
  //    double curv = 1.0 / 2.0 / R;
  //    tlc_dec_by_curv = pnc::mathlib::Interp1(
  //        GetContext.get_param()->ldp_C2_vector,
  //        GetContext.get_param()->ldp_dec_tlc_vector, fabs(curv));
  //    std::cout << "R = " << R << std::endl;
  //    std::cout << "tlc_dec_by_curv = " << tlc_dec_by_curv << std::endl;
  //  }

  double tlc_thrd =
      tlc_calculate_base - tlc_dec_by_curv - tlc_dec_by_narrowroad;
  if (tlc_thrd < 0.1) {
    tlc_thrd = 0.1;
  } else {
    // do nothing
  }

  return tlc_thrd;
}

void LdpCore::RunOnce(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

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
  if (ldp_left_intervention_by_line) {
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
  if (ldp_right_intervention_by_line) {
    ldp_right_intervention_ = true;
  } else {
    ldp_right_intervention_ = false;
  }

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