#include "lane_depart_prevention_context.h"
namespace planning {
void LaneDepartPrevention::Init(planning::LkasInput *lkas_input) {
  lkas_input_ = lkas_input;
  calribration_str_.enable_roadedge_switch =
      false;  // 是否使能功能的路沿场景开关 0:不使能  1:使能
  calribration_str_.enable_vehspd_display_min =
      5.0F / 3.6F;  // 激活的最小仪表车速，单位：m/s
  calribration_str_.enable_vehspd_display_max =
      150.0F / 3.6F;  // 激活的最大仪表车速，单位：m/s
  calribration_str_.disable_vehspd_display_min =
      2.0F / 3.6F;  // 退出的最小仪表车速，单位：m/s
  calribration_str_.disable_vehspd_display_max =
      155.0F / 3.6F;  // 退出的最大仪表车速，单位：m/s
  calribration_str_.supp_turn_light_recovery_time =
      2000;  // 转向灯抑制恢复时长，单位：ms
  calribration_str_.earliest_warning_line = 0.75F;  // 触发的最早报警线，单位：m
  calribration_str_.latest_warning_line = -0.3F;  // 触发的最晚报警线，单位：m
  calribration_str_.reset_warning_line = 0.15F;  // 触发的报警重置线，单位：m
  calribration_str_.warning_time_max = 2000;  // 最大报警时长，单位：ms
  calribration_str_.tlc_line_far =
      1.0F;  // 针对道线触发报警的高灵敏度阈值，单位：s
  calribration_str_.tlc_line_medium =
      0.6F;  // 针对道线触发报警的中灵敏度阈值，单位：s
  calribration_str_.tlc_line_near =
      0.2F;  // 针对道线触发报警的低灵敏度阈值，单位：s
  calribration_str_.suppression_driver_hand_trq =
      2.0F;  // 抑制报警的驾驶员手力矩(绝对值)阈值，单位：Nm
  calribration_str_.kickdown_driver_hand_trq =
      2.5F;  // 打断报警的驾驶员手力矩(绝对值)阈值，单位：Nm
}
void LaneDepartPrevention::Update() {
  measurement_str_.main_switch = lkas_input_->vehicle_info.ldp_main_switch;
  ;  // LDP功能开关状态 0:Off  1:On
  measurement_str_.tlc_line_threshold =
      calribration_str_
          .tlc_line_far;  // 依据驾驶员选择的灵敏度等级,设定触发阈值
  /*measurement_str_.left_intervention*/
  if (lkas_input_->road_info.left_line_valid == true) {
    measurement_str_.left_intervention = LKALineLeftIntervention(
        measurement_str_.tlc_line_threshold, lkas_input_);
  } else if ((calribration_str_.enable_roadedge_switch == true) &&
             (lkas_input_->road_info.left_roadedge_valid == true)) {
    measurement_str_.left_intervention = LKARoadEdgeLeftIntervention(
        measurement_str_.tlc_line_threshold, lkas_input_);
  } else {
    measurement_str_.left_intervention = false;
  }
  /*measurement_str_.right_intervention*/
  if (lkas_input_->road_info.right_line_valid == true) {
    measurement_str_.right_intervention = LKALineRightIntervention(
        measurement_str_.tlc_line_threshold, lkas_input_);
  } else if ((calribration_str_.enable_roadedge_switch == true) &&
             (lkas_input_->road_info.right_roadedge_valid == true)) {
    measurement_str_.right_intervention = LKARoadEdgeRightIntervention(
        measurement_str_.tlc_line_threshold, lkas_input_);
  } else {
    measurement_str_.right_intervention = false;
  }
}
void LaneDepartPrevention::RunOnce() {
  Update();
  measurement_str_.enable_code = EnableCode();
  measurement_str_.disable_code = DisableCode();
  measurement_str_.fault_code = FaultCode();
  measurement_str_.left_suppression_code = LeftSuppressionCode();
  measurement_str_.left_kickdown_code = LeftKickDownCode();
  measurement_str_.right_suppression_code = RightSuppressionCode();
  measurement_str_.right_kickdown_code = RightKickDownCode();
  measurement_str_.state = StateMachine();  // LDP状态机跳转
  set_ldp_output_info();

  JSON_DEBUG_VALUE("lkas_function::ldp::left_intervention",
                   measurement_str_.left_intervention);
  JSON_DEBUG_VALUE("lkas_function::ldp::right_intervention",
                   measurement_str_.right_intervention);
  JSON_DEBUG_VALUE("lkas_function::ldp::main_switch",
                   measurement_str_.main_switch);
  JSON_DEBUG_VALUE("lkas_function::ldp::enable_code",
                   measurement_str_.enable_code);
  JSON_DEBUG_VALUE("lkas_function::ldp::disable_code",
                   measurement_str_.disable_code);
  JSON_DEBUG_VALUE("lkas_function::ldp::fault_code",
                   measurement_str_.fault_code);
  JSON_DEBUG_VALUE("lkas_function::ldp::left_suppression_code",
                   measurement_str_.left_suppression_code);
  JSON_DEBUG_VALUE("lkas_function::ldp::left_kickdown_code",
                   measurement_str_.left_kickdown_code);
  JSON_DEBUG_VALUE("lkas_function::ldp::right_suppression_code",
                   measurement_str_.right_suppression_code);
  JSON_DEBUG_VALUE("lkas_function::ldp::right_kickdown_code",
                   measurement_str_.right_kickdown_code);
  JSON_DEBUG_VALUE("lkas_function::ldp::state", measurement_str_.state);
}
uint16 LaneDepartPrevention::EnableCode() {
  uint16 ldp_enable_code_temp = 0;
  // Condition1
  // 判断车速是否处于工作车速范围内
  if (lkas_input_->vehicle_info.veh_display_speed <
      calribration_str_.enable_vehspd_display_min) {
    ldp_enable_code_temp += uint16_bit[0];
  } else if (lkas_input_->vehicle_info.veh_display_speed >
             calribration_str_.enable_vehspd_display_max) {
    ldp_enable_code_temp += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // Condition2
  // 判断是否至少识别到一侧道线或一侧路沿
  if (lkas_input_->road_info.left_line_valid == true) {  // 识别到左侧道线
                                                         /*do nothing*/
  } else if (lkas_input_->road_info.right_line_valid ==
             true) {  // 识别到右侧道线
                      /*do nothing*/
  } else if (calribration_str_.enable_roadedge_switch ==
             true) {  // 使能路沿场景
    if (lkas_input_->road_info.left_roadedge_valid == true) {  // 识别到左侧路沿
                                                               /*do nothing*/
    } else if (lkas_input_->road_info.right_roadedge_valid ==
               true) {  // 识别到右侧路沿
                        /*do nothing*/
    } else {
      ldp_enable_code_temp += uint16_bit[1];
    }
  } else {
    ldp_enable_code_temp += uint16_bit[1];
  }

  // Condition3
  // 判断当前车道宽度是否满足激活条件
  if (lkas_input_->road_info.lane_width_valid == true) {
    if (lkas_input_->road_info.lane_width < k_lka_enable_lane_width_min) {
      ldp_enable_code_temp += uint16_bit[2];
    } else if (lkas_input_->road_info.lane_width >
               k_lka_enable_lane_width_max) {
      ldp_enable_code_temp += uint16_bit[2];
    } else {
      /*do nothing*/
    }
  } else {
    /*do nothing*/
  }
  return ldp_enable_code_temp;
}
uint16 LaneDepartPrevention::DisableCode() {
  uint16 ldp_disable_code_temp = 0;

  // Condition1
  // 判断车速是否处于工作车速范围内
  if (lkas_input_->vehicle_info.veh_display_speed <
      calribration_str_.disable_vehspd_display_min) {
    ldp_disable_code_temp += uint16_bit[0];
  } else if (lkas_input_->vehicle_info.veh_display_speed >
             calribration_str_.disable_vehspd_display_max) {
    ldp_disable_code_temp += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // Condition2
  // 判断是否至少识别到一侧道线或一侧路沿
  if (lkas_input_->road_info.left_line_valid == true) {  // 识别到左侧道线
                                                         /*do nothing*/
  } else if (lkas_input_->road_info.right_line_valid ==
             true) {  // 识别到右侧道线
                      /*do nothing*/
  } else if (calribration_str_.enable_roadedge_switch ==
             true) {  // 使能路沿场景
    if (lkas_input_->road_info.left_roadedge_valid == true) {  // 识别到左侧路沿
                                                               /*do nothing*/
    } else if (lkas_input_->road_info.right_roadedge_valid ==
               true) {  // 识别到右侧路沿
                        /*do nothing*/
    } else {
      ldp_disable_code_temp += uint16_bit[1];
    }
  } else {
    ldp_disable_code_temp += uint16_bit[1];
  }

  // Condition3
  // 判断当前车道宽度是否满足激活条件
  if (lkas_input_->road_info.lane_width_valid == true) {
    if (lkas_input_->road_info.lane_width < k_lka_disable_lane_width_min) {
      ldp_disable_code_temp += uint16_bit[2];
    } else if (lkas_input_->road_info.lane_width >
               k_lka_disable_lane_width_max) {
      ldp_disable_code_temp += uint16_bit[2];
    } else {
      /*do nothing*/
    }
  } else {
    /*do nothing*/
  }

  return ldp_disable_code_temp;
}
uint16 LaneDepartPrevention::FaultCode() {
  uint16 ldp_fault_code_temp = 0;
  return ldp_fault_code_temp;
}
uint16 LaneDepartPrevention::LeftSuppressionCode() {
  uint16 ldp_left_suppression_code_temp = 0;

  static uint16 left_turn_light_off_duration =
      3000;  // 左转向灯处于关闭状态的时长，单位:ms
  if (lkas_input_->vehicle_info.left_turn_light_state == true) {
    left_turn_light_off_duration = 0;  // 转向灯开启,转向灯关闭状态计时器清零
  } else {
    left_turn_light_off_duration += Common_Cycle_Time;
    if (left_turn_light_off_duration >= 60000) {
      left_turn_light_off_duration = 60000;
    } else {
      /*do nothing*/
    }
  }

  static bool left_suppress_repeat_warning_flag =
      0;  // 是否抑制重复报警的标志位 0:不抑制 1:抑制
  if (measurement_str_.state == 4) {  // 检测到左侧报警
    left_suppress_repeat_warning_flag = true;
  } else {  // 检测到左侧报警结束
    if (left_suppress_repeat_warning_flag == true) {
      if (lkas_input_->road_info.left_line_valid == true) {  // 左侧道线有效
        if (lkas_input_->wheel_to_line.fl_wheel_distance_to_line >
            calribration_str_.reset_warning_line) {  // 检测到越过了重置线
          left_suppress_repeat_warning_flag = false;
        } else {
          /*do nothing*/
        }
      } else if ((calribration_str_.enable_roadedge_switch == true) &&
                 (lkas_input_->road_info.left_roadedge_valid == true) &&
                 (lkas_input_->wheel_to_line.fl_wheel_distance_to_roadedge >
                  calribration_str_.reset_warning_line)) {
        left_suppress_repeat_warning_flag = false;
      } else {
        /*do nothing*/
      }
    } else {
      /*do nothing*/
    }
  }

  // Condition1
  // 判断左转向灯处于关闭时长是否满足
  if (left_turn_light_off_duration <
      calribration_str_.supp_turn_light_recovery_time) {
    ldp_left_suppression_code_temp += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // Condition2
  // 判断是否处于允许左侧报警区域内
  if (lkas_input_->road_info.left_line_valid == true) {  // 检测到左侧道线
    if (lkas_input_->wheel_to_line.fl_wheel_distance_to_line < 0.0F) {
      ldp_left_suppression_code_temp += uint16_bit[1];
    } else if (lkas_input_->wheel_to_line.fl_wheel_distance_to_line >
               calribration_str_.earliest_warning_line) {
      ldp_left_suppression_code_temp += uint16_bit[1];
    } else {
      /*do nothing*/
    }
  } else if ((calribration_str_.enable_roadedge_switch == true) &&
             (lkas_input_->road_info.left_roadedge_valid ==
              true)) {  // 使能路沿场景&&检测到路沿
    if (lkas_input_->wheel_to_line.fl_wheel_distance_to_roadedge < 0.0F) {
      ldp_left_suppression_code_temp += uint16_bit[1];
    } else if (lkas_input_->wheel_to_line.fl_wheel_distance_to_roadedge >
               calribration_str_.earliest_warning_line) {
      ldp_left_suppression_code_temp += uint16_bit[1];
    } else {
      /*do nothing*/
    }
  } else {
    ldp_left_suppression_code_temp += uint16_bit[1];
  }

  // Condition3
  // 距上次触发报警后,越过了重置线
  if (left_suppress_repeat_warning_flag == true) {
    ldp_left_suppression_code_temp += uint16_bit[2];
  } else {
    /*do nothing*/
  }

  // Condition4
  // 驾驶员手力矩条件
  if (fabs(lkas_input_->vehicle_info.driver_hand_torque) >
      calribration_str_.suppression_driver_hand_trq) {
    ldp_left_suppression_code_temp += uint16_bit[3];
  } else {
    /*do nothing*/
  }

  return ldp_left_suppression_code_temp;
}
uint16 LaneDepartPrevention::LeftKickDownCode() {
  uint16 ldp_left_kickdown_code_temp = 0;

  static uint16 left_warning_duration = 0;  // 处于左侧报警状态的时长，单位:ms
  if (measurement_str_.state == 4) {
    left_warning_duration += Common_Cycle_Time;
    if (left_warning_duration >= 60000) {
      left_warning_duration = 60000;
    } else {
      /*do nothing*/
    }
  } else {
    left_warning_duration = 0;
  }

  // Condition1
  if (lkas_input_->vehicle_info.left_turn_light_state == true) {
    ldp_left_kickdown_code_temp += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // Condition2
  // 判断是否处于允许左侧报警区域内
  if (lkas_input_->road_info.left_line_valid == true) {  // 检测到左侧道线
    if (lkas_input_->wheel_to_line.fl_wheel_distance_to_line <
        calribration_str_.latest_warning_line) {
      ldp_left_kickdown_code_temp += uint16_bit[1];
    } else if (lkas_input_->wheel_to_line.fl_wheel_distance_to_line >
               calribration_str_.earliest_warning_line) {
      ldp_left_kickdown_code_temp += uint16_bit[1];
    } else {
      /*do nothing*/
    }
  } else if ((calribration_str_.enable_roadedge_switch == true) &&
             (lkas_input_->road_info.left_roadedge_valid ==
              true)) {  // 使能路沿场景&&检测到路沿
    if (lkas_input_->wheel_to_line.fl_wheel_distance_to_roadedge <
        calribration_str_.latest_warning_line) {
      ldp_left_kickdown_code_temp += uint16_bit[1];
    } else if (lkas_input_->wheel_to_line.fl_wheel_distance_to_roadedge >
               calribration_str_.earliest_warning_line) {
      ldp_left_kickdown_code_temp += uint16_bit[1];
    } else {
      /*do nothing*/
    }
  } else {
    ldp_left_kickdown_code_temp += uint16_bit[1];
  }

  // Condition3
  if (left_warning_duration > calribration_str_.warning_time_max) {
    ldp_left_kickdown_code_temp += uint16_bit[2];
  } else {
    /*do nothing*/
  }

  // Condition4
  if (fabs(lkas_input_->vehicle_info.driver_hand_torque) >
      calribration_str_.kickdown_driver_hand_trq) {
    ldp_left_kickdown_code_temp += uint16_bit[3];
  } else {
    /*do nothing*/
  }

  return ldp_left_kickdown_code_temp;
}
uint16 LaneDepartPrevention::RightSuppressionCode() {
  uint16 ldp_right_suppression_code_temp = 0;

  static uint16 right_turn_light_off_duration =
      3000;  // 右转向灯处于关闭状态的时长，单位:ms
  if (lkas_input_->vehicle_info.right_turn_light_state == true) {
    right_turn_light_off_duration = 0;  // 转向灯开启,转向灯关闭状态计时器清零
  } else {
    right_turn_light_off_duration += Common_Cycle_Time;
    if (right_turn_light_off_duration >= 60000) {
      right_turn_light_off_duration = 60000;
    } else {
      /*do nothing*/
    }
  }

  static bool right_suppress_repeat_warning_flag =
      0;  // 是否抑制重复报警的标志位 0:不抑制 1:抑制
  if (measurement_str_.state == 5) {  // 检测到右侧报警
    right_suppress_repeat_warning_flag = true;
  } else {  // 检测到右侧报警结束
    if (right_suppress_repeat_warning_flag == true) {
      if (lkas_input_->road_info.right_line_valid == true) {  // 右侧道线有效
        if (lkas_input_->wheel_to_line.fr_wheel_distance_to_line <
            (-1.0 *
             calribration_str_.reset_warning_line)) {  // 检测到越过了重置线
          right_suppress_repeat_warning_flag = false;
        } else {
          /*do nothing*/
        }
      } else if ((calribration_str_.enable_roadedge_switch == true) &&
                 (lkas_input_->road_info.right_roadedge_valid == true) &&
                 (lkas_input_->wheel_to_line.fr_wheel_distance_to_roadedge <
                  (-1.0 * calribration_str_.reset_warning_line))) {
        right_suppress_repeat_warning_flag = false;
      } else {
        /*do nothing*/
      }
    } else {
      /*do nothing*/
    }
  }

  // Condition1
  // 判断右转向灯处于关闭时长是否满足
  if (right_turn_light_off_duration <
      calribration_str_.supp_turn_light_recovery_time) {
    ldp_right_suppression_code_temp += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // Condition2
  // 判断是否处于允许右侧报警区域内
  if (lkas_input_->road_info.right_line_valid == true) {  // 检测到右侧道线
    if (lkas_input_->wheel_to_line.fr_wheel_distance_to_line > 0.0F) {
      ldp_right_suppression_code_temp += uint16_bit[1];
    } else if (lkas_input_->wheel_to_line.fr_wheel_distance_to_line <
               (-1.0 * calribration_str_.earliest_warning_line)) {
      ldp_right_suppression_code_temp += uint16_bit[1];
    } else {
      /*do nothing*/
    }
  } else if ((calribration_str_.enable_roadedge_switch == true) &&
             (lkas_input_->road_info.right_roadedge_valid ==
              true)) {  // 使能路沿场景&&检测到路沿
    if (lkas_input_->wheel_to_line.fr_wheel_distance_to_roadedge > 0.0F) {
      ldp_right_suppression_code_temp += uint16_bit[1];
    } else if (lkas_input_->wheel_to_line.fr_wheel_distance_to_roadedge <
               (-1.0 * calribration_str_.earliest_warning_line)) {
      ldp_right_suppression_code_temp += uint16_bit[1];
    } else {
      /*do nothing*/
    }
  } else {
    ldp_right_suppression_code_temp += uint16_bit[1];
  }

  // Condition3
  if (right_suppress_repeat_warning_flag == true) {
    ldp_right_suppression_code_temp += uint16_bit[2];
  } else {
    /*do nothing*/
  }

  // Condition4
  // 驾驶员手力矩条件
  if (fabs(lkas_input_->vehicle_info.driver_hand_torque) >
      calribration_str_.suppression_driver_hand_trq) {
    ldp_right_suppression_code_temp += uint16_bit[3];
  } else {
    /*do nothing*/
  }

  return ldp_right_suppression_code_temp;
}
uint16 LaneDepartPrevention::RightKickDownCode() {
  uint16 ldp_right_kickdown_code_temp = 0;

  static uint16 right_warning_duration = 0;  // 处于右侧报警状态的时长，单位:ms
  if (measurement_str_.state == 5) {
    right_warning_duration += Common_Cycle_Time;
    if (right_warning_duration >= 60000) {
      right_warning_duration = 60000;
    } else {
      /*do nothing*/
    }
  } else {
    right_warning_duration = 0;
  }

  // Condition1
  if (lkas_input_->vehicle_info.right_turn_light_state == true) {
    ldp_right_kickdown_code_temp += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // Condition2
  // 判断是否处于允许右侧报警区域内
  if (lkas_input_->road_info.right_line_valid == true) {  // 检测到右侧道线
    if (lkas_input_->wheel_to_line.fr_wheel_distance_to_line >
        (-1.0 * calribration_str_.latest_warning_line)) {
      ldp_right_kickdown_code_temp += uint16_bit[1];
    } else if (lkas_input_->wheel_to_line.fr_wheel_distance_to_line <
               (-1.0 * calribration_str_.earliest_warning_line)) {
      ldp_right_kickdown_code_temp += uint16_bit[1];
    } else {
      /*do nothing*/
    }
  } else if ((calribration_str_.enable_roadedge_switch == true) &&
             (lkas_input_->road_info.right_roadedge_valid ==
              true)) {  // 使能路沿场景&&检测到路沿
    if (lkas_input_->wheel_to_line.fr_wheel_distance_to_roadedge >
        (-1.0 * calribration_str_.latest_warning_line)) {
      ldp_right_kickdown_code_temp += uint16_bit[1];
    } else if (lkas_input_->wheel_to_line.fr_wheel_distance_to_roadedge <
               (-1.0 * calribration_str_.earliest_warning_line)) {
      ldp_right_kickdown_code_temp += uint16_bit[1];
    } else {
      /*do nothing*/
    }
  } else {
    ldp_right_kickdown_code_temp += uint16_bit[1];
  }

  // Condition3
  if (right_warning_duration > calribration_str_.warning_time_max) {
    ldp_right_kickdown_code_temp += uint16_bit[2];
  } else {
    /*do nothing*/
  }

  // Condition4
  if (fabs(lkas_input_->vehicle_info.driver_hand_torque) >
      calribration_str_.kickdown_driver_hand_trq) {
    ldp_right_kickdown_code_temp += uint16_bit[3];
  } else {
    /*do nothing*/
  }

  return ldp_right_kickdown_code_temp;
}
uint8 LaneDepartPrevention::StateMachine() {
  static uint8 ldp_state_machine_init_flag =
      0;  // ldp状态机初始化状态 0:未初始化过 1:已完成过初始化
  static uint8 ldp_state_fault_off_standby_active =
      0;  // ldp一级主状态 FAULT OFF STANDBY ACTIVE
  static uint8 ldp_state_active_substate =
      0;  // ldp ACTIVE状态子状态 NO_ACTIVE_CHILD NoIntervention
          // LeftIntervention RightIntervention
  uint8 ldp_state_temp;
  if (ldp_state_machine_init_flag == 0) {
    // 状态机处于初始化状态 根据开关状态,决定第一个周期是输出OFF还是STANDBY
    ldp_state_machine_init_flag = 1;
    if (!measurement_str_.main_switch) {
      ldp_state_fault_off_standby_active = LKA_StateMachine_IN_OFF;
      ldp_state_temp = 1;
    } else {
      ldp_state_fault_off_standby_active = LKA_StateMachine_IN_STANDBY;
      ldp_state_temp = 2;
    }
  } else {
    // 状态机处于完成过初始化的状态
    switch (ldp_state_fault_off_standby_active) {
      case LKA_StateMachine_IN_ACTIVE:
        if (!measurement_str_.main_switch) {  // ACTIVE->OFF
          ldp_state_active_substate = LKA_StateMachine_IN_NO_ACTIVE_CHILD;
          ldp_state_fault_off_standby_active = LKA_StateMachine_IN_OFF;
          ldp_state_temp = 1;
        } else if (measurement_str_.fault_code) {  // ACTIVE->FAULT
          ldp_state_active_substate = LKA_StateMachine_IN_NO_ACTIVE_CHILD;
          ldp_state_fault_off_standby_active = LKA_StateMachine_IN_FAULT;
          ldp_state_temp = 0;
        } else if (measurement_str_.disable_code) {  // ACTIVE->STANDBY
          ldp_state_active_substate = LKA_StateMachine_IN_NO_ACTIVE_CHILD;
          ldp_state_fault_off_standby_active = LKA_StateMachine_IN_STANDBY;
          ldp_state_temp = 2;
        } else {  // 继续维持在ACTIVE状态
          switch (ldp_state_active_substate) {
            case LKA_StateMachine_IN_LeftIntervention:
              ldp_state_temp = 4;
              if (measurement_str_
                      .left_kickdown_code) {  // LeftIntervention->NoIntervention
                ldp_state_active_substate = LKA_StateMachine_IN_NoIntervention;
                ldp_state_temp = 3;
              }
              break;
            case LKA_StateMachine_IN_NoIntervention:
              ldp_state_temp = 3;
              if (measurement_str_.right_intervention &&
                  (!measurement_str_
                        .right_suppression_code)) {  // NoIntervention->RightIntervention
                ldp_state_active_substate =
                    LKA_StateMachine_IN_RightIntervention;
                ldp_state_temp = 5;
              } else if (
                  measurement_str_.left_intervention &&
                  (!measurement_str_
                        .left_suppression_code)) {  // NoIntervention->LeftIntervention
                ldp_state_active_substate =
                    LKA_StateMachine_IN_LeftIntervention;
                ldp_state_temp = 4;
              }
              break;
            default:
              ldp_state_temp = 5;
              if (measurement_str_
                      .right_kickdown_code) {  // RightIntervention->NoIntervention
                ldp_state_active_substate = LKA_StateMachine_IN_NoIntervention;
                ldp_state_temp = 3;
              }
              break;
          }
        }
        break;
      case LKA_StateMachine_IN_FAULT:
        ldp_state_temp = 0;
        if (!measurement_str_.main_switch) {  // FAULT->OFF
          ldp_state_fault_off_standby_active = LKA_StateMachine_IN_OFF;
          ldp_state_temp = 1;
        } else if (!measurement_str_.fault_code) {  // FAULT->STANDBY
          ldp_state_fault_off_standby_active = LKA_StateMachine_IN_STANDBY;
          ldp_state_temp = 2;
        }
        break;
      case LKA_StateMachine_IN_OFF:
        ldp_state_temp = 1;
        if (measurement_str_.main_switch) {  // OFF->STANDBY
          ldp_state_fault_off_standby_active = LKA_StateMachine_IN_STANDBY;
          ldp_state_temp = 2;
        }
        break;
      default:
        ldp_state_temp = 2;
        if (!measurement_str_.main_switch) {  // STANDBY->OFF
          ldp_state_fault_off_standby_active = LKA_StateMachine_IN_OFF;
          ldp_state_temp = 1;
        } else if (measurement_str_.fault_code) {  // STANDBY->FAULT
          ldp_state_fault_off_standby_active = LKA_StateMachine_IN_FAULT;
          ldp_state_temp = 0;
        } else if (measurement_str_.enable_code == 0) {  // STANDBY->ACTIVE
          ldp_state_fault_off_standby_active = LKA_StateMachine_IN_ACTIVE;
          ldp_state_active_substate = LKA_StateMachine_IN_NoIntervention;
          ldp_state_temp = 3;
        }
        break;
    }
  }
  return ldp_state_temp;
}
}  // namespace planning