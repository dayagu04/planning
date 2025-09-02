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
  if (GetContext.get_state_info()->accelerator_pedal_pos_rate <
      GetContext.get_param()->elk_enable_accel_pedal_pos_rate) {
    acc_pedal_pos_rate_supp_recover_duration_ += GetContext.get_param()->dt;
    if (acc_pedal_pos_rate_supp_recover_duration_ > 60.0) {
      acc_pedal_pos_rate_supp_recover_duration_ = 60.0;
    } else {
      /*do nothing*/
    }
  } else {
    acc_pedal_pos_rate_supp_recover_duration_ = 0.0;
  }
  if (acc_pedal_pos_rate_supp_recover_duration_ <
      GetContext.get_param()->elk_enable_accel_pedal_pos_rate_dur) {
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
  double left_roadedge_C2_temp =
      GetContext.get_road_info()->current_lane.left_roadedge.c2;
  double right_roadedge_C2_temp =
      GetContext.get_road_info()->current_lane.right_roadedge.c2;

  if (((fabs(left_line_C2_temp) < 0.5 / 220) &&
       (GetContext.get_road_info()->current_lane.left_line.valid == true)) ||
      ((fabs(right_line_C2_temp) < 0.5 / 220) &&
       (GetContext.get_road_info()->current_lane.right_line.valid == true)) ||
      ((fabs(left_roadedge_C2_temp) < 0.5 / 220) &&
       (GetContext.get_road_info()->current_lane.left_roadedge.valid ==
        true)) ||
      ((fabs(right_roadedge_C2_temp) < 0.5 / 220) &&
       (GetContext.get_road_info()->current_lane.right_roadedge.valid ==
        true))) {
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
  //   enable_code += uint16_bit[16];
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

  return enable_code & GetContext.get_param()->elk_enable_code_maskcode;
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
      (elk_state_ !=
       iflyauto::ELKFunctionFSMWorkState::
           ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION) &&
      (elk_state_ != iflyauto::ELKFunctionFSMWorkState::
                         ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION)

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
  if (GetContext.get_state_info()->accelerator_pedal_pos_rate >
      GetContext.get_param()->elk_disable_accel_pedal_pos_rate) {
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
  if (((fabs(GetContext.get_road_info()->current_lane.left_line.c2) >
        0.5 / 200.0) &&
       ((GetContext.get_road_info()->current_lane.left_line.valid == true))) ||
      ((fabs(GetContext.get_road_info()->current_lane.right_line.c2) >
        0.5 / 200.0) &&
       (GetContext.get_road_info()->current_lane.right_line.valid == true))) {
    current_lane_curv_enable_flag = false;
  } else {
    current_lane_curv_enable_flag = true;
  }
  if (current_lane_curv_enable_flag == false &&
      (elk_state_ !=
       iflyauto::ELKFunctionFSMWorkState::
           ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION) &&
      (elk_state_ !=
       iflyauto::ELKFunctionFSMWorkState::
           ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION)) {
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

  return disable_code & GetContext.get_param()->elk_disable_code_maskcode;
}

uint16 ElkCore::UpdateElkFaultCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;
  uint16 elk_fault_code = 0;
  // bit 0
  // 实际车速信号无效
  if ((vehicle_service_output_info_ptr->vehicle_speed_available == false)) {
    elk_fault_code += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // bit 1
  // 仪表车速信号无效
  if ((vehicle_service_output_info_ptr->vehicle_speed_display_available ==
       false)) {
    elk_fault_code += uint16_bit[1];
  } else {
    /*do nothing*/
  }

  // bit 2
  // 横摆角速度信号无效
  if ((vehicle_service_output_info_ptr->yaw_rate_available == false)) {
    elk_fault_code += uint16_bit[2];
  } else {
    /*do nothing*/
  }

  // bit 3
  // 方向盘转角速度信号无效
  if ((vehicle_service_output_info_ptr->steering_wheel_angle_available ==
       false)) {
    elk_fault_code += uint16_bit[3];
  } else {
    /*do nothing*/
  }

  // bit 4
  // 方向盘转速信号无效
  if ((vehicle_service_output_info_ptr->steering_wheel_angle_speed_available ==
       false)) {
    elk_fault_code += uint16_bit[4];
  } else {
    /*do nothing*/
  }

  // bit 5
  // 油门踏板信号无效
  if ((vehicle_service_output_info_ptr->accelerator_pedal_pos_available ==
       false)) {
    elk_fault_code += uint16_bit[5];
  } else {
    /*do nothing*/
  }

  // bit 6
  // 制动压力信号无效，使用实际制动踏板开度有效性 (true:有效/false:无效)
  if ((vehicle_service_output_info_ptr->esp_pressure_available == false)) {
    elk_fault_code += uint16_bit[6];
  } else {
    /*do nothing*/
  }

  // bit 7
  // 转向灯信号无效
  if ((vehicle_service_output_info_ptr->turn_switch_state_available == false)) {
    elk_fault_code += uint16_bit[7];
  } else {
    /*do nothing*/
  }

  // bit 8
  // 驾驶员手力矩信号无效
  if ((vehicle_service_output_info_ptr->driver_hand_torque_available ==
       false)) {
    elk_fault_code += uint16_bit[8];
  } else {
    /*do nothing*/
  }

  // bit 9
  // 横向控制执行器状态异常
  if ((vehicle_service_output_info_ptr
           ->pilot_lat_control_actuator_status_available == false)) {
    elk_fault_code += uint16_bit[9];
  } else {
    /*do nothing*/
  }

  // bit 10
  // 车道线融合模块节点通讯丢失
  if ((GetContext.mutable_state_info()->road_info_node_valid == false)) {
    elk_fault_code += uint16_bit[10];
  } else {
    /*do nothing*/
  }

  // bit 11
  // vehicle_service模块节点通讯丢失
  if ((GetContext.mutable_state_info()->vehicle_service_node_valid == false)) {
    elk_fault_code += uint16_bit[11];
  } else {
    /*do nothing*/
  }

  // bit 12
  // 定位模块节点通讯丢失
  if ((GetContext.mutable_state_info()->localization_info_node_valid ==
       false)) {
    elk_fault_code += uint16_bit[12];
  } else {
    /*do nothing*/
  }

  return elk_fault_code & GetContext.get_param()->elk_fault_code_maskcode;
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

  // // Condition1
  // // 判断左转向灯处于关闭时长是否满足
  // if (GetContext.mutable_state_info()->left_turn_light_off_time <
  //     elk_param_.supp_turn_light_recovery_time) {
  //   // elk_left_suppression_code += uint16_bit[0];
  // } else {
  //   /*do nothing*/
  // }

  // Condition2
  // 判断是否处于道线允许左侧报警区域内
  bool in_left_line_aera_flag = false;
  bool in_left_roadedge_aera_flag = false;
  if ((GetContext.get_road_info()->current_lane.left_line.valid == true) &&
      (GetContext.mutable_state_info()->fl_wheel_distance_to_line >
       elk_param_.latest_warning_line) &&
      (GetContext.mutable_state_info()->fl_wheel_distance_to_line <
       elk_param_.earliest_warning_line)) {
    in_left_line_aera_flag = true;
  } else {
    in_left_line_aera_flag = false;
  }
  // 判断是否处于路沿允许左侧报警区域内
  if ((GetContext.get_road_info()->current_lane.left_roadedge.valid == true) &&
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
    elk_left_suppression_code += uint16_bit[1];
  }

  // Condition3
  // 距上次触发报警后,越过了重置线
  if (left_suppress_repeat_warning_flag_ == true) {
    elk_left_suppression_code += uint16_bit[2];
  } else {
    /*do nothing*/
  }
  // bit 3
  // 车辆偏离本车道，满足触发阈值,暂时保留
  if ((elk_left_intervention_ == false) &&
      (elk_state_ !=
       iflyauto::ELKFunctionFSMWorkState::
           ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION)) {
    elk_left_suppression_code += uint16_bit[3];
  } else {
    /*do nothing*/
  }

  // bit4
  // 驾驶员手力矩条件,手力矩绝对值<=1.5Nm，持续超过0.5s

  if (fabs(GetContext.mutable_state_info()->driver_hand_trq) <
      GetContext.get_param()->ELK_suppression_driver_hand_trq) {
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
    elk_left_suppression_code += uint16_bit[4];
  } else {
    /*do nothing*/
  }

  // bit 5
  // 距离上一次LDp报警结束后，冷却超过3s
  if (elk_state_ != iflyauto::ELKFunctionFSMWorkState::
                        ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION &&
      elk_state_ != iflyauto::ELKFunctionFSMWorkState::
                        ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION) {
    LDP_CoolingTime_duration_ += GetContext.get_param()->dt;
    if (LDP_CoolingTime_duration_ > 60.0) {
      LDP_CoolingTime_duration_ = 60.0;
    } else {
      /*do nothing*/
    }
  } else {
    LDP_CoolingTime_duration_ = 0.0;
  }
  if ((LDP_CoolingTime_duration_ < 3.0) &&
      fabs(GetContext.get_state_info()->driver_hand_trq) >
          GetContext.get_param()->ELK_supp_CoolingTime_handtrq_thr) {
    elk_left_suppression_code += uint16_bit[5];
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
    elk_left_suppression_code += uint16_bit[6];
  } else {
    /*do nothing*/
  }

  // bit 7
  // 纠偏侧为虚拟道线
  if (GetContext.get_road_info()->current_lane.left_line.line_type ==
      context::Enum_LineType::Enum_LineType_Virtual) {
    elk_left_suppression_code += uint16_bit[7];
  } else {
    /*do nothing*/
  }
  // bit 8
  // 纠偏侧为分流口
  if ((GetContext.get_road_info()->current_lane.left_sideway_exist_flag ||
       GetContext.get_road_info()->current_lane.right_sideway_exist_flag) &&
      (GetContext.get_param()->force_no_sideway_switch == false)) {
    // 左侧有分流口
    elk_left_suppression_code += uint16_bit[8];
  }
  // bit 9
  // 旁侧or前方有车，且有碰撞风险
  if (GetContext.get_road_info()->current_lane.right_parallel_car_flag ==
          true ||
      GetContext.get_road_info()->current_lane.right_front_car_flag == true) {
    // 右侧有并行车或右前方一定区域内有车、或者正前方一定区域内有车，且有碰撞风险
    elk_left_suppression_code += uint16_bit[9];
  }

  // bit 10
  // 压线行驶抑制
  if (GetContext.get_road_info()->close_to_left_line_flag) {
    elk_left_suppression_code += uint16_bit[10];
  } else {
    /*do nothing*/
  }

  return elk_left_suppression_code &
         GetContext.get_param()->elk_left_suppression_code_maskcode;
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

  //  // Condition1
  //   if (vehicle_service_output_info_ptr->left_turn_light_state == true) {
  //     elk_left_kickdown_code += uint16_bit[0];
  //   } else {
  //     /*do nothing*/
  //   }

  // bit 1
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
  // bit 2
  // 纠偏持续时间>最大纠偏时长(8s)
  if (elk_left_warning_time_ > elk_param_.warning_time_max) {
    elk_left_kickdown_code += uint16_bit[2];
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
      elk_state_ == iflyauto::ELKFunctionFSMWorkState::
                        ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION) {
    elk_left_kickdown_lat_v_duration_ += GetContext.get_param()->dt;
    if (elk_left_kickdown_lat_v_duration_ > 60.0) {
      elk_left_kickdown_lat_v_duration_ = 60.0;
    } else {
      /*do nothing*/
    }
  } else {
    elk_left_kickdown_lat_v_duration_ = 0.0;
  }
  if ((elk_left_kickdown_lat_v_duration_ >
       GetContext.get_param()->ldp_kickdown_lat_v_dur) &&
      (GetContext.mutable_state_info()->veh_left_departure_speed > 0.05)) {
    elk_left_kickdown_code += uint16_bit[3];
  } else {
    /*do nothing*/
  }

  // bit 4
  // 驾驶员手力矩条件,反向手力矩>3.5Nm 或 同向手力矩>2.5Nm
  // 驾驶员手力矩 单位:Nm 左转弯为正，右转弯为负
  bool ldp_handtrq_abs_flag = false;
  if (fabs(GetContext.mutable_state_info()->driver_hand_trq) >
          GetContext.get_param()->ELK_kickdown_abs_hand_trq &&
      elk_state_ == iflyauto::ELKFunctionFSMWorkState::
                        ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION) {
    elk_left_handtrq_kickdown_duration_ += GetContext.get_param()->dt;
    if (elk_left_handtrq_kickdown_duration_ > 60.0) {
      elk_left_handtrq_kickdown_duration_ = 60.0;
    } else {
      /*do nothing*/
    }
  } else {
    elk_left_handtrq_kickdown_duration_ = 0.0;
  }
  if (elk_left_handtrq_kickdown_duration_ >
      GetContext.get_param()->ELK_kickdown_hand_trq_dur) {
    ldp_handtrq_abs_flag = true;
  } else {
    ldp_handtrq_abs_flag = false;
  }

  if (GetContext.mutable_state_info()->driver_hand_trq >
          GetContext.get_param()->ELK_kickdown_oppodir_hand_trq ||
      GetContext.mutable_state_info()->driver_hand_trq <
          -GetContext.get_param()->ELK_kickdown_samedir_hand_trq ||
      ldp_handtrq_abs_flag) {
    elk_left_kickdown_code += uint16_bit[4];
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
    elk_left_kickdown_code += uint16_bit[6];
  } else {
    /*do nothing*/
  }

  // bit 7
  // 偏离侧道线为虚拟线
  if (GetContext.get_road_info()->current_lane.left_line.line_type ==
      context::Enum_LineType::Enum_LineType_Virtual) {
    elk_left_kickdown_code += uint16_bit[7];
  } else {
    /*do nothing*/
  }

  return elk_left_kickdown_code &
         GetContext.get_param()->elk_left_kickdown_code_maskcode;
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

  // // Condition1
  // // 判断右转向灯处于关闭时长是否满足
  // if (GetContext.mutable_state_info()->right_turn_light_off_time <
  //     elk_param_.supp_turn_light_recovery_time) {
  //   // elk_right_suppression_code += uint16_bit[0];
  // } else {
  //   /*do nothing*/
  // }

  // Condition2
  // 判断是否处于允许右侧报警区域内
  bool in_right_line_aera_flag = false;
  bool in_right_roadedge_aera_flag = false;
  if ((GetContext.get_road_info()->current_lane.right_line.valid == true) &&
      (GetContext.mutable_state_info()->fr_wheel_distance_to_line <
       (-1.0 * elk_param_.latest_warning_line)) &&
      (GetContext.mutable_state_info()->fr_wheel_distance_to_line >
       (-1.0 * elk_param_.earliest_warning_line))) {
    in_right_line_aera_flag = true;
  } else {
    in_right_line_aera_flag = false;
  }
  // 判断是否处于路沿允许右侧报警区域内
  if ((GetContext.get_road_info()->current_lane.right_roadedge.valid == true) &&
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
    elk_right_suppression_code += uint16_bit[1];
  }

  // bit 3
  // 车辆偏离本车道，满足触发阈值
  if ((elk_right_intervention_ == false) &&
      (elk_state_ !=
       iflyauto::ELKFunctionFSMWorkState::
           ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION)) {
    elk_right_suppression_code += uint16_bit[3];
  } else {
    /*do nothing*/
  }

  // bit 4
  // 驾驶员手力矩条件
  if (fabs(GetContext.mutable_state_info()->driver_hand_trq) <
      GetContext.get_param()->ELK_suppression_driver_hand_trq) {
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
    elk_right_suppression_code += uint16_bit[4];
  } else {
    /*do nothing*/
  }

  // bit5
  // 距离上一次LDW报警结束后，冷却超过3s
  if (elk_state_ != iflyauto::ELKFunctionFSMWorkState::
                        ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION &&
      elk_state_ != iflyauto::ELKFunctionFSMWorkState::
                        ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION) {
    LDP_CoolingTime_duration_ += GetContext.get_param()->dt;
    if (LDP_CoolingTime_duration_ > 60.0) {
      LDP_CoolingTime_duration_ = 60.0;
    } else {
      /*do nothing*/
    }
  } else {
    LDP_CoolingTime_duration_ = 0.0;
  }
  if ((LDP_CoolingTime_duration_ < 3.0) &&
      fabs(GetContext.get_state_info()->driver_hand_trq) >
          GetContext.get_param()->ELK_supp_CoolingTime_handtrq_thr) {
    elk_right_suppression_code += uint16_bit[5];
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
    elk_right_suppression_code += uint16_bit[6];
  } else {
    /*do nothing*/
  }

  // bit 7
  // 触发侧为虚拟道线
  if (GetContext.get_road_info()->current_lane.right_line.line_type ==
      context::Enum_LineType::Enum_LineType_Virtual) {
    elk_right_suppression_code += uint16_bit[7];
  } else {
    /*do nothing*/
  }

  // bit 8
  // 触发侧为分流口
  if ((GetContext.get_road_info()->current_lane.left_sideway_exist_flag ||
       GetContext.get_road_info()->current_lane.right_sideway_exist_flag) &&
      (GetContext.get_param()->force_no_sideway_switch == false)) {
    // 右侧有分流口
    elk_right_suppression_code += uint16_bit[8];
  }
  // bit9
  // 左侧有并行车或左前方一定区域内有车、或者正前方一定区域内有车，且有碰撞风险
  if (GetContext.get_road_info()->current_lane.left_parallel_car_flag == true ||
      GetContext.get_road_info()->current_lane.left_front_car_flag == true) {
    // 左侧有并行车或左前方一定区域内有车、或者正前方一定区域内有车，且有碰撞风险
    elk_right_suppression_code += uint16_bit[9];
  }

  // bit 10
  // 压线行驶抑制
  if (GetContext.get_road_info()->close_to_right_line_flag) {
    elk_right_suppression_code += uint16_bit[10];
  } else {
    /*do nothing*/
  }

  return elk_right_suppression_code &
         GetContext.get_param()->elk_right_suppression_code_maskcode;
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

  // // Condition1

  // if (vehicle_service_output_info_ptr->right_turn_light_state == true) {
  //    elk_right_kickdown_code += uint16_bit[0];
  // } else {
  //   /*do nothing*/
  // }

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

  // bit 2
  if (elk_right_warning_time_ > elk_param_.warning_time_max) {
    elk_right_kickdown_code += uint16_bit[2];
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
      elk_state_ == iflyauto::ELKFunctionFSMWorkState::
                        ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION) {
    elk_right_kickdown_lat_v_duration_ += GetContext.get_param()->dt;
    if (elk_right_kickdown_lat_v_duration_ > 60.0) {
      elk_right_kickdown_lat_v_duration_ = 60.0;
    } else {
      /*do nothing*/
    }
  } else {
    elk_right_kickdown_lat_v_duration_ = 0.0;
  }
  if ((elk_right_kickdown_lat_v_duration_ >
       GetContext.get_param()->ldp_kickdown_lat_v_dur) &&
      (GetContext.mutable_state_info()->veh_right_departure_speed < -0.05)) {
    elk_right_kickdown_code += uint16_bit[3];
  } else {
    /*do nothing*/
  }

  // bit 4,手力矩条件
  // 反向手力矩>3.5Nm 或 同向手力矩>2.5Nm. 单位:Nm 左转弯为正，右转弯为负

  bool ldp_handtrq_abs_flag = false;
  if (fabs(GetContext.mutable_state_info()->driver_hand_trq) >
          GetContext.get_param()->ELK_kickdown_abs_hand_trq &&
      elk_state_ == iflyauto::ELKFunctionFSMWorkState::
                        ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION) {
    elk_right_kickdown_lat_v_duration_ += GetContext.get_param()->dt;
    if (elk_right_kickdown_lat_v_duration_ > 60.0) {
      elk_right_kickdown_lat_v_duration_ = 60.0;
    } else {
      /*do nothing*/
    }
  } else {
    elk_right_kickdown_lat_v_duration_ = 0.0;
  }
  if (elk_right_kickdown_lat_v_duration_ >
      GetContext.get_param()->ELK_kickdown_hand_trq_dur) {
    ldp_handtrq_abs_flag = true;
  } else {
    ldp_handtrq_abs_flag = false;
  }

  if (GetContext.mutable_state_info()->driver_hand_trq <
          -GetContext.get_param()->ELK_kickdown_oppodir_hand_trq ||
      GetContext.mutable_state_info()->driver_hand_trq >
          GetContext.get_param()->ELK_kickdown_samedir_hand_trq ||
      ldp_handtrq_abs_flag) {
    elk_right_kickdown_code += uint16_bit[4];
  } else {
    /*do nothing*/
  }

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
    elk_right_kickdown_code += uint16_bit[6];
  } else {
    /*do nothing*/
  }

  // bit 7
  if (GetContext.get_road_info()->current_lane.right_line.line_type ==
      context::Enum_LineType::Enum_LineType_Virtual) {
    elk_right_kickdown_code += uint16_bit[7];
  } else {
    /*do nothing*/
  }

  return elk_right_kickdown_code &
         GetContext.get_param()->elk_right_kickdown_code_maskcode;
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

  if (((elk_state_ ==
        iflyauto::ELKFunctionFSMWorkState::
            ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION) &&
       (elk_left_has_risk_for_hmi_flag_ == true)) ||
      ((elk_state_ ==
        iflyauto::ELKFunctionFSMWorkState::
            ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION) &&
       (elk_right_has_risk_for_hmi_flag_ == true))) {
    GetContext.mutable_output_info()->elk_output_info_.elk_risk_obj_.obj_valid =
        true;
    GetContext.mutable_output_info()->elk_output_info_.elk_risk_obj_.id =
        elk_obj_id_;
  }

  else {
    GetContext.mutable_output_info()->elk_output_info_.elk_risk_obj_.obj_valid =
        false;
  }

  return;
}

uint16 ElkCore::RiskAlertJudge(const context::FusionObjExtractInfo &obj,
                               double &elk_obj_tlc) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  //新增&elk_obj_tlc，存储各个目标车辆的tlc信息，用以分析问题
  uint16 risk_condition_code = 0;  //=0,有风险
  double collision_x = 0.0;
  double obj_collision_x = 0.0;
  bool requre_obj_speed_direction_positive = true;
  if (obj.relative_position_x > 0) {  // 目标车在前方
    collision_x =
        GetContext.get_param()->origin_2_front_bumper;  // 后轴中心到前保
    requre_obj_speed_direction_positive = false;
    obj_collision_x = obj.relative_position_x_down;  // 定义目标车后方到本车
  } else {
    collision_x = -1.0 * GetContext.get_param()->origin_2_rear_bumper;
    obj_collision_x = obj.relative_position_x_up;  // 定义目标车前方到本车
  }
  // 相对速度
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
  // std::vector<double> ttc_collision_thrd_tab = {2.0, 2.5, 3.0};
  // std::vector<double> obj_relative_speed_tab = {5.0, 10.0, 15.0};

  ttc_collision_thrd_tmp =
      pnc::mathlib::Interp1(GetContext.get_param()->obj_relative_speed_tab,
                            GetContext.get_param()->ttc_collision_thrd_tab,
                            std::fabs(obj.relative_speed_x));
  ttc_collision_tmp =
      std::fabs((obj_collision_x - collision_x) / obj.relative_speed_x);
  if (ttc_collision_tmp > ttc_collision_thrd_tmp) {
    risk_condition_code += uint16_bit[1];
  }
  // // 判断是否为大车场景
  // bool is_large_vehicle = false;
  // bool is_small_vehicle = false;
  // if (obj.type >= 4 && obj.type <= 8) {
  //   bool is_large_vehicle = true;
  // } else {
  //   /*do nothing*/
  // }
  // if (obj.type == 3) {
  //   is_small_vehicle = true;
  // } else {
  //   /*do nothing*/
  // }
  // // 判断是否有横向碰撞的风险
  // // 定义大车中心距离本车道线距离
  // double distance_sidecar_to_lane =
  //     fabs(obj.relative_position_y -
  //          GetContext.get_road_info()->current_lane.right_line.c0 -
  //          obj.width);

  // if (is_large_vehicle = true && (distance_sidecar_to_lane > 0.6)) {
  //   risk_condition_code += uint16_bit[2];
  // }
  // if (is_small_vehicle = true && (distance_sidecar_to_lane > 0.4)) {
  //   risk_condition_code += uint16_bit[2];
  // }
  elk_obj_tlc = ttc_collision_tmp;
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

double ElkCore::UpdateTlcThreshold(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  double tlc_calculate_base = GetContext.get_param()->elk_tlc_thrd;
  tlc_calculate_base = pnc::mathlib::Interp1(
      GetContext.get_param()->lka_vel_vector,
      GetContext.get_param()->lka_tlc_vector,
      (GetContext.get_state_info()->display_vehicle_speed * 3.6));

  // 定义弯道tlc减少
  double tlc_dec_by_curv = 0.0;
  double C2_temp_thr = 0.0;
  double R_temp_thr = 0.0;
  if (GetContext.get_road_info()->current_lane.left_line.valid == true) {
    C2_temp_thr = GetContext.get_road_info()->current_lane.left_line.c2;
  } else if (GetContext.get_road_info()->current_lane.right_line.valid ==
             true) {
    C2_temp_thr = GetContext.get_road_info()->current_lane.right_line.c2;
  } else {
    C2_temp_thr = 0.00005;
  }
  if (fabs(C2_temp_thr) < 0.00005) {
    R_temp_thr = 10000.0;
  } else {
    R_temp_thr = 0.5 / fabs(C2_temp_thr);
  }

  tlc_dec_by_curv = pnc::mathlib::Interp1(
      GetContext.get_param()->lka_r_vector,
      GetContext.get_param()->lka_dec_tlc_by_c2_vector, R_temp_thr);

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

void ElkCore::RunOnce(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;

  // 更新tlc_to_line_threshold_
  elk_tlc_threshold_ = UpdateTlcThreshold();

  // ELK—Roadedge场景，触发线逻辑修改
  //更新elk_roadedge_offset,修改为根据横向速度查表
  double elk_roadedge_offset = 0.0;
  double elk_roadedge_offset_temp = 0.0;
  if ((GetContext.get_road_info()->current_lane.left_roadedge.valid) &&
      (GetContext.get_road_info()->current_lane.left_line.valid)) {
    elk_roadedge_offset_temp =
        fabs(GetContext.get_state_info()->veh_left_departure_speed);
  } else if ((GetContext.get_road_info()->current_lane.right_roadedge.valid) &&
             (GetContext.get_road_info()->current_lane.right_line.valid)) {
    elk_roadedge_offset_temp =
        fabs(GetContext.get_state_info()->veh_right_departure_speed);
  } else { /*do nothing*/
  }

  elk_roadedge_offset = pnc::mathlib::Interp1(
      GetContext.get_param()->elk_roadedge_departure_V_vector,
      GetContext.get_param()->elk_roadedge_offset_vector,
      elk_roadedge_offset_temp);

  // ELK—Roadedge场景，触发线逻辑修改
  // 若压线行驶，触发线外移
  double preview_y_gap_Vy_offset = 0.0;
  if (GetContext.get_road_info()->close_to_right_line_flag ||
      GetContext.get_road_info()->close_to_left_line_flag) {
    preview_y_gap_Vy_offset = 0.2;
  } else {
    preview_y_gap_Vy_offset = 0.0;
  }
  // ELK—Solid line场景，触发线逻辑修改
  // 弯道，触发线外移
  double y_gap_dec_by_curv = 0.0;
  double C2_temp_thr = 0.0;
  double R_temp_thr = 0.0;
  if (GetContext.get_road_info()->current_lane.left_line.valid == true) {
    C2_temp_thr = GetContext.get_road_info()->current_lane.left_line.c2;
  } else if (GetContext.get_road_info()->current_lane.right_line.valid ==
             true) {
    C2_temp_thr = GetContext.get_road_info()->current_lane.right_line.c2;
  } else {
    C2_temp_thr = 0.00005;
  }

  if (fabs(C2_temp_thr) < 0.00005) {
    R_temp_thr = 10000.0;
  } else {
    R_temp_thr = 0.5 / fabs(C2_temp_thr);
  }
  y_gap_dec_by_curv = pnc::mathlib::Interp1(
      GetContext.get_param()->lka_r_vector,
      GetContext.get_param()->lka_dec_y_gap_by_c2_vector, R_temp_thr);

  // ELK—Roadedge场景，触发线逻辑修改
  // 弯道，触发线外移
  double roadedge_y_gap_dec_by_curv = 0.0;
  double roadedge_C2_temp_thr = 0.0;
  double roadedge_R_temp_thr = 0.0;
  if (GetContext.get_road_info()->current_lane.left_roadedge.valid == true) {
    roadedge_C2_temp_thr =
        GetContext.get_road_info()->current_lane.left_roadedge.c2;
  } else if (GetContext.get_road_info()->current_lane.right_roadedge.valid ==
             true) {
    roadedge_C2_temp_thr =
        GetContext.get_road_info()->current_lane.right_roadedge.c2;
  } else {
    roadedge_C2_temp_thr = 0.00005;
  }

  if (fabs(roadedge_C2_temp_thr) < 0.00005) {
    roadedge_R_temp_thr = 10000.0;
  } else {
    roadedge_R_temp_thr = 0.5 / fabs(roadedge_C2_temp_thr);
  }
  roadedge_y_gap_dec_by_curv = pnc::mathlib::Interp1(
      GetContext.get_param()->lka_r_vector,
      GetContext.get_param()->roadedge_dec_y_gap_by_c2_vector,
      roadedge_R_temp_thr);

  // ELK—Roadedge场景，路沿的可触发区域逻辑修改
  // 弯道场景，路沿的可触发区域调整
  double roadegge_earl_warning_line_offset_curv = 0.0;
  roadegge_earl_warning_line_offset_curv = pnc::mathlib::Interp1(
      GetContext.get_param()->lka_r_vector,
      GetContext.get_param()->elk_roadedge_earliest_line_c2_vector,
      roadedge_R_temp_thr);
  bool in_left_roadedge_aera_flag = false;
  bool in_right_roadedge_aera_flag = false;
  if ((GetContext.get_road_info()->current_lane.left_roadedge.valid == true) &&
      (GetContext.mutable_state_info()->fl_wheel_distance_to_roadedge > 0.0) &&
      (GetContext.mutable_state_info()->fl_wheel_distance_to_roadedge <
       (elk_param_.roadedge_earliest_warning_line -
        roadegge_earl_warning_line_offset_curv))) {
    in_left_roadedge_aera_flag = true;
  } else {
    in_left_roadedge_aera_flag = false;
  }

  if ((GetContext.get_road_info()->current_lane.right_roadedge.valid == true) &&
      (GetContext.mutable_state_info()->fr_wheel_distance_to_roadedge < 0.0) &&
      (GetContext.mutable_state_info()->fr_wheel_distance_to_roadedge >
       (-1.0 * (elk_param_.roadedge_earliest_warning_line -
                roadegge_earl_warning_line_offset_curv)))) {
    in_right_roadedge_aera_flag = true;
  } else {
    in_right_roadedge_aera_flag = false;
  }
  // ELK—Roadedge场景，路沿的可触发区域逻辑修改
  // 弯道路沿场景，功能不触发
  // R300 4秒
  double left_roadedge_C2_temp =
      GetContext.get_road_info()->current_lane.left_roadedge.c2;
  double right_roadedge_C2_temp =
      GetContext.get_road_info()->current_lane.right_roadedge.c2;
  if (((fabs(left_roadedge_C2_temp) <
        0.5 / GetContext.get_param()->elk_roadedge_supp_curv_r_thr) &&
       ((GetContext.get_road_info()->current_lane.left_roadedge.valid ==
         true))) ||
      ((fabs(right_roadedge_C2_temp) <
        0.5 / GetContext.get_param()->elk_roadedge_supp_curv_r_thr) &&
       (GetContext.get_road_info()->current_lane.right_roadedge.valid ==
        true))) {
    elk_roadedge_curve_supp_duration_ += GetContext.get_param()->dt;
    if (elk_roadedge_curve_supp_duration_ > 60.0) {
      elk_roadedge_curve_supp_duration_ = 60.0;
    } else {
      /*do nothing*/
    }
  } else {
    elk_roadedge_curve_supp_duration_ = 0.0;
  }
  bool elk_leftroadedge_left_curv_flag = false;  //左侧
  bool elk_roadedge_left_curv_supp_flag = false;

  if (GetContext.get_road_info()->current_lane.left_roadedge.valid == true &&
      left_roadedge_C2_temp < 0.0) {
    elk_leftroadedge_left_curv_flag = true;
  }
  if ((elk_roadedge_curve_supp_duration_ <
       GetContext.get_param()->elk_roadedge_supp_curv_r_dur) &&
      (elk_leftroadedge_left_curv_flag == true ||
       GetContext.get_param()->elk_roadedge_testswitch_temp_ == true)) {
    elk_roadedge_left_curv_supp_flag = true;
  } else {
    /*do nothing*/
  }

  bool elk_rightroadedge_right_curv_flag = false;  //右侧
  bool elk_roadedge_right_curv_supp_flag = false;

  if (GetContext.get_road_info()->current_lane.right_roadedge.valid == true &&
      right_roadedge_C2_temp > 0.0) {
    elk_rightroadedge_right_curv_flag = true;
  }
  if (elk_roadedge_curve_supp_duration_ <
          GetContext.get_param()->elk_roadedge_supp_curv_r_dur &&
      (elk_rightroadedge_right_curv_flag == true ||
       GetContext.get_param()->elk_roadedge_testswitch_temp_ == true)) {
    elk_roadedge_right_curv_supp_flag = true;
  } else {
    /*do nothing*/
  }

  // 更新elk_left_intervention_by_line和elk_left_intervention_by_roadedge
  // 此处更新触发线offset逻辑
  double preview_left_y_gap =
      adas_function::LkasLineLeftIntervention(elk_tlc_threshold_);
  // double preview_left_roadedge_y_gap =
  //     adas_function::LkasRoadedgeLeftIntervention(
  //         elk_roadedge_tlc_threshold_,
  //         GetContext.get_param()->elk_roadedge_offset);
  double preview_left_roadedge_y_gap =
      adas_function::LkasRoadedgeLeftIntervention(elk_roadedge_tlc_threshold_,
                                                  elk_roadedge_offset);

  bool elk_left_intervention_by_line = false;      //
  bool elk_left_intervention_by_roadedge = false;  //
  if ((preview_left_y_gap + preview_y_gap_Vy_offset + y_gap_dec_by_curv) <
      0.0) {
    elk_left_intervention_by_line = true;
  }
  if ((preview_left_roadedge_y_gap + roadedge_y_gap_dec_by_curv) < 0.0 &&
      GetContext.get_road_info()->current_lane.left_roadedge.end_x >
          GetContext.get_param()->ego_length &&
      in_left_roadedge_aera_flag == true &&
      elk_roadedge_left_curv_supp_flag == false) {
    elk_left_intervention_by_roadedge = true;
  }
  // 更新left_has_risk_code
  //  bit0：左旁侧区域内有车 bit1:左侧有Oncoming车 bit2:左侧有Overtaking车
  uint16 left_has_risk_code = 0;
  uint16 left_risk_temp_code = 0;
  if (GetContext.get_objs_info()->objs_selected.ml_objs.vehicle_info_valid) {
    left_has_risk_code += uint16_bit[0];
    elk_obj_ml_tlc_ = 0.0;
    elk_obj_id_ =
        GetContext.get_objs_info()->objs_selected.ml_objs.vehicle_info.id;
  } else {
    elk_obj_ml_tlc_ = 10.0;
  }
  if (GetContext.get_objs_info()->objs_selected.fl_objs.vehicle_info_valid) {
    left_risk_temp_code = RiskAlertJudge(
        GetContext.get_objs_info()->objs_selected.fl_objs.vehicle_info,
        elk_obj_fl_tlc_);  // 通过RiskAlertJudge，把左前车的信息输入进去判断是否存在风险，==0是有风险
    if (left_risk_temp_code == 0) {
      left_has_risk_code += uint16_bit[1];
      elk_obj_id_ =
          GetContext.get_objs_info()->objs_selected.fl_objs.vehicle_info.id;
    }
  } else {
    elk_obj_fl_tlc_ = 10.0;
  }

  if (GetContext.get_objs_info()->objs_selected.rl_objs.vehicle_info_valid) {
    left_risk_temp_code = RiskAlertJudge(
        GetContext.get_objs_info()->objs_selected.rl_objs.vehicle_info,
        elk_obj_rl_tlc_);
    if (left_risk_temp_code == 0) {
      left_has_risk_code += uint16_bit[2];
      elk_obj_id_ =
          GetContext.get_objs_info()->objs_selected.rl_objs.vehicle_info.id;
    }
  } else {
    elk_obj_rl_tlc_ = 10.0;
  }
  // 更新elk_left_intervention_
  if (elk_left_intervention_by_roadedge) {
    elk_left_intervention_ = true;
  } else if (elk_left_intervention_by_line && left_has_risk_code != 0) {
    elk_left_intervention_ = true;
    bool elk_left_has_risk_for_hmi_flag_ = true;

  } else if (elk_left_intervention_by_line &&
             (GetContext.get_road_info()->current_lane.left_line.line_type ==
              context::Enum_LineType::Enum_LineType_Solid) &&
             (vehicle_service_output_info_ptr->left_turn_light_state ==
              false)) {
    elk_left_intervention_ = true;
  } else {
    elk_left_intervention_ = false;
    bool elk_left_has_risk_for_hmi_flag_ = false;
  }

  // 更新elk_right_intervention_by_line和elk_right_intervention_by_roadedge
  double preview_right_y_gap =
      adas_function::LkasLineRightIntervention(elk_tlc_threshold_);
  // double preview_right_roadedge_y_gap =
  //     adas_function::LkasRoadedgeRightIntervention(
  //         elk_roadedge_tlc_threshold_,
  //         GetContext.get_param()->elk_roadedge_offset);
  double preview_right_roadedge_y_gap =
      adas_function::LkasRoadedgeRightIntervention(elk_roadedge_tlc_threshold_,
                                                   elk_roadedge_offset);
  bool elk_right_intervention_by_line = false;
  bool elk_right_intervention_by_roadedge = false;
  if ((preview_right_y_gap - preview_y_gap_Vy_offset - y_gap_dec_by_curv) >
      0.0) {
    elk_right_intervention_by_line = true;
  }
  if ((preview_right_roadedge_y_gap - roadedge_y_gap_dec_by_curv) > 0.0 &&
      GetContext.get_road_info()->current_lane.right_roadedge.end_x >
          GetContext.get_param()->ego_length &&
      in_right_roadedge_aera_flag == true &&
      elk_roadedge_right_curv_supp_flag == false) {
    elk_right_intervention_by_roadedge = true;
  }
  // 更新right_has_risk_code
  //  bit0：右旁侧区域内有车 bit1:右侧有Oncoming车 bit2:右侧有Overtaking车
  uint16 right_has_risk_code = 0;
  uint16 right_risk_temp_code = 0;
  if (GetContext.get_objs_info()->objs_selected.mr_objs.vehicle_info_valid) {
    right_has_risk_code += uint16_bit[0];
    elk_obj_mr_tlc_ = 0.0;
    elk_obj_id_ =
        GetContext.get_objs_info()->objs_selected.mr_objs.vehicle_info.id;
  } else {
    elk_obj_mr_tlc_ = 10.0;
  }
  if (GetContext.get_objs_info()->objs_selected.fr_objs.vehicle_info_valid) {
    right_risk_temp_code = RiskAlertJudge(
        GetContext.get_objs_info()->objs_selected.fr_objs.vehicle_info,
        elk_obj_fr_tlc_);  // 通过RiskAlertJudge，把左前车的信息输入进去判断是否存在风险，==0是有风险
    if (right_risk_temp_code == 0) {
      right_has_risk_code += uint16_bit[1];
      elk_obj_id_ =
          GetContext.get_objs_info()->objs_selected.fr_objs.vehicle_info.id;
    }
  } else {
    elk_obj_fr_tlc_ = 10.0;
  }
  if (GetContext.get_objs_info()->objs_selected.rr_objs.vehicle_info_valid) {
    right_risk_temp_code = RiskAlertJudge(
        GetContext.get_objs_info()->objs_selected.rr_objs.vehicle_info,
        elk_obj_rr_tlc_);
    if (right_risk_temp_code == 0) {
      right_has_risk_code += uint16_bit[2];
      elk_obj_id_ =
          GetContext.get_objs_info()->objs_selected.rr_objs.vehicle_info.id;
    }
  } else {
    elk_obj_rr_tlc_ = 10.0;
  }
  // 更新elk_right_intervention_
  if (elk_right_intervention_by_roadedge) {
    elk_right_intervention_ = true;
  } else if (elk_right_intervention_by_line && right_has_risk_code != 0) {
    elk_right_intervention_ = true;
    elk_right_has_risk_for_hmi_flag_ = true;
  } else if (elk_right_intervention_by_line &&
             (GetContext.get_road_info()->current_lane.right_line.line_type ==
              context::Enum_LineType::Enum_LineType_Solid) &&
             (vehicle_service_output_info_ptr->right_turn_light_state ==
              false)) {
    elk_right_intervention_ = true;
  } else {
    elk_right_intervention_ = false;
    elk_right_has_risk_for_hmi_flag_ = false;
  }

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

  JSON_DEBUG_VALUE("left_has_risk_code", left_has_risk_code);
  JSON_DEBUG_VALUE("right_has_risk_code", right_has_risk_code);

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

  // JSON_DEBUG_VALUE("elk_roadedge_offset",
  //                  GetContext.get_param()->elk_roadedge_offset);
  JSON_DEBUG_VECTOR("elk_preview_ego_pos_vec", preview_ego_pos_vec, 2);
  JSON_DEBUG_VALUE("elk_obj_fl_tlc_", elk_obj_fl_tlc_);
  JSON_DEBUG_VALUE("elk_obj_ml_tlc_", elk_obj_ml_tlc_);
  JSON_DEBUG_VALUE("elk_obj_rl_tlc_", elk_obj_rl_tlc_);
  JSON_DEBUG_VALUE("elk_obj_fr_tlc_", elk_obj_fr_tlc_);
  JSON_DEBUG_VALUE("elk_obj_mr_tlc_", elk_obj_mr_tlc_);
  JSON_DEBUG_VALUE("elk_obj_rr_tlc_", elk_obj_rr_tlc_);
}

}  // namespace elk_core
}  // namespace adas_function