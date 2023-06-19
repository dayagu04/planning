#include "lane_depart_warning_context.h"
namespace planning {
void LaneDepartWarning::Init(planning::LkasInput *lkas_input) {
  lkas_input_ = lkas_input;
  calribration_str_.enable_roadedge_switch = FALSE;  // 是否使能功能的路沿场景开关 0:不使能  1:使能
  calribration_str_.enable_vehspd_display_min = 10.0F / 3.6F;    // 激活的最小仪表车速，单位：m/s
  calribration_str_.enable_vehspd_display_max = 150.0F / 3.6F;   // 激活的最大仪表车速，单位：m/s
  calribration_str_.disable_vehspd_display_min = 5.0F / 3.6F;    // 退出的最小仪表车速，单位：m/s
  calribration_str_.disable_vehspd_display_max = 155.0F / 3.6F;  // 退出的最大仪表车速，单位：m/s
  calribration_str_.supp_turn_light_recovery_time = 2000;        // 转向灯抑制恢复时长，单位：ms
  calribration_str_.earliest_warning_line = 0.75F;               // 触发的最早报警线，单位：m
  calribration_str_.latest_warning_line = -0.3F;                 // 触发的最晚报警线，单位：m
  calribration_str_.reset_warning_line = 0.15F;                  // 触发的报警重置线，单位：m
  calribration_str_.warning_time_max = 2000;                     // 最大报警时长，单位：ms
  calribration_str_.tlc_line_far = 1.0F;                 // 针对道线触发报警的高灵敏度阈值，单位：s
  calribration_str_.tlc_line_medium = 0.6F;              // 针对道线触发报警的中灵敏度阈值，单位：s
  calribration_str_.tlc_line_near = 0.2F;                // 针对道线触发报警的低灵敏度阈值，单位：s
  calribration_str_.suppression_driver_hand_trq = 2.0F;  // 抑制报警的驾驶员手力矩(绝对值)阈值，单位：Nm
  calribration_str_.kickdown_driver_hand_trq = 2.5F;  // 打断报警的驾驶员手力矩(绝对值)阈值，单位：Nm
}
void LaneDepartWarning::Update() {
  // printf("aaa=%d\n", aaaa);
  /*运行LDW算法*/
  measurement_str_.main_switch = lkas_input_->vehicle_info.ldw_main_switch;  // LDW功能开关状态 0:Off  1:On
  // 依据驾驶员选择的灵敏度等级,设定触发阈值
  if (lkas_input_->vehicle_info.ldw_tlc_level == 0) {
    measurement_str_.tlc_line_threshold = calribration_str_.tlc_line_near;
  } else if (lkas_input_->vehicle_info.ldw_tlc_level == 1) {
    measurement_str_.tlc_line_threshold = calribration_str_.tlc_line_medium;
  } else {
    measurement_str_.tlc_line_threshold = calribration_str_.tlc_line_far;
  }

  /*measurement_str_.left_intervention*/
  if (lkas_input_->road_info.left_line_valid == TRUE) {
    measurement_str_.left_intervention = LKALineLeftIntervention(measurement_str_.tlc_line_threshold, lkas_input_);
  } else if ((calribration_str_.enable_roadedge_switch == TRUE) &&
             (lkas_input_->road_info.left_roadedge_valid == TRUE)) {
    measurement_str_.left_intervention = LKARoadEdgeLeftIntervention(measurement_str_.tlc_line_threshold, lkas_input_);
  } else {
    measurement_str_.left_intervention = FALSE;
  }
  /*measurement_str_.right_intervention*/
  if (lkas_input_->road_info.right_line_valid == TRUE) {
    measurement_str_.right_intervention = LKALineRightIntervention(measurement_str_.tlc_line_threshold, lkas_input_);
  } else if ((calribration_str_.enable_roadedge_switch == TRUE) &&
             (lkas_input_->road_info.right_roadedge_valid == TRUE)) {
    measurement_str_.right_intervention =
        LKARoadEdgeRightIntervention(measurement_str_.tlc_line_threshold, lkas_input_);
  } else {
    measurement_str_.right_intervention = FALSE;
  }
}
void LaneDepartWarning::RunOnce() {
  Update();
  measurement_str_.enable_code = EnableCode();
  measurement_str_.disable_code = DisableCode();
  measurement_str_.fault_code = FaultCode();
  measurement_str_.left_suppression_code = LeftSuppressionCode();
  measurement_str_.left_kickdown_code = LeftKickDownCode();
  measurement_str_.right_suppression_code = RightSuppressionCode();
  measurement_str_.right_kickdown_code = RightKickDownCode();
  measurement_str_.state = StateMachine();  // LDW状态机跳转
  set_ldw_output_info();

  JSON_DEBUG_VALUE("lkas_function::ldw::left_intervention", measurement_str_.left_intervention);
  JSON_DEBUG_VALUE("lkas_function::ldw::right_intervention", measurement_str_.right_intervention);
  JSON_DEBUG_VALUE("lkas_function::ldw::main_switch", measurement_str_.main_switch);
  JSON_DEBUG_VALUE("lkas_function::ldw::enable_code", measurement_str_.enable_code);
  JSON_DEBUG_VALUE("lkas_function::ldw::disable_code", measurement_str_.disable_code);
  JSON_DEBUG_VALUE("lkas_function::ldw::fault_code", measurement_str_.fault_code);
  JSON_DEBUG_VALUE("lkas_function::ldw::left_suppression_code", measurement_str_.left_suppression_code);
  JSON_DEBUG_VALUE("lkas_function::ldw::left_kickdown_code", measurement_str_.left_kickdown_code);
  JSON_DEBUG_VALUE("lkas_function::ldw::right_suppression_code", measurement_str_.right_suppression_code);
  JSON_DEBUG_VALUE("lkas_function::ldw::right_kickdown_code", measurement_str_.right_kickdown_code);
  JSON_DEBUG_VALUE("lkas_function::ldw::state", measurement_str_.state);
}

uint16 LaneDepartWarning::EnableCode() {
  uint16 ldw_enable_code_temp = 0;
  // Condition1
  // 判断车速是否处于工作车速范围内
  if (lkas_input_->vehicle_info.veh_display_speed < calribration_str_.enable_vehspd_display_min) {
    ldw_enable_code_temp += uint16_bit[0];
  } else if (lkas_input_->vehicle_info.veh_display_speed > calribration_str_.enable_vehspd_display_max) {
    ldw_enable_code_temp += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // Condition2
  // 判断是否至少识别到一侧道线或一侧路沿
  if (lkas_input_->road_info.left_line_valid == TRUE) {                // 识别到左侧道线
                                                                       /*do nothing*/
  } else if (lkas_input_->road_info.right_line_valid == TRUE) {        // 识别到右侧道线
                                                                       /*do nothing*/
  } else if (calribration_str_.enable_roadedge_switch == TRUE) {       // 使能路沿场景
    if (lkas_input_->road_info.left_roadedge_valid == TRUE) {          // 识别到左侧路沿
                                                                       /*do nothing*/
    } else if (lkas_input_->road_info.right_roadedge_valid == TRUE) {  // 识别到右侧路沿
                                                                       /*do nothing*/
    } else {
      ldw_enable_code_temp += uint16_bit[1];
    }
  } else {
    ldw_enable_code_temp += uint16_bit[1];
  }

  // Condition3
  // 判断当前车道宽度是否满足激活条件
  if (lkas_input_->road_info.lane_width_valid == TRUE) {
    if (lkas_input_->road_info.lane_width < k_lka_enable_lane_width_min) {
      ldw_enable_code_temp += uint16_bit[2];
    } else if (lkas_input_->road_info.lane_width > k_lka_enable_lane_width_max) {
      ldw_enable_code_temp += uint16_bit[2];
    } else {
      /*do nothing*/
    }
  } else {
    /*do nothing*/
  }
  return ldw_enable_code_temp;
}
uint16 LaneDepartWarning::DisableCode() {
  uint16 ldw_disable_code_temp = 0;

  // Condition1
  // 判断车速是否处于工作车速范围内
  if (lkas_input_->vehicle_info.veh_display_speed < calribration_str_.disable_vehspd_display_min) {
    ldw_disable_code_temp += uint16_bit[0];
  } else if (lkas_input_->vehicle_info.veh_display_speed > calribration_str_.disable_vehspd_display_max) {
    ldw_disable_code_temp += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // Condition2
  // 判断是否至少识别到一侧道线或一侧路沿
  if (lkas_input_->road_info.left_line_valid == TRUE) {                // 识别到左侧道线
                                                                       /*do nothing*/
  } else if (lkas_input_->road_info.right_line_valid == TRUE) {        // 识别到右侧道线
                                                                       /*do nothing*/
  } else if (calribration_str_.enable_roadedge_switch == TRUE) {       // 使能路沿场景
    if (lkas_input_->road_info.left_roadedge_valid == TRUE) {          // 识别到左侧路沿
                                                                       /*do nothing*/
    } else if (lkas_input_->road_info.right_roadedge_valid == TRUE) {  // 识别到右侧路沿
                                                                       /*do nothing*/
    } else {
      ldw_disable_code_temp += uint16_bit[1];
    }
  } else {
    ldw_disable_code_temp += uint16_bit[1];
  }

  // Condition3
  // 判断当前车道宽度是否满足激活条件
  if (lkas_input_->road_info.lane_width_valid == TRUE) {
    if (lkas_input_->road_info.lane_width < k_lka_disable_lane_width_min) {
      ldw_disable_code_temp += uint16_bit[2];
    } else if (lkas_input_->road_info.lane_width > k_lka_disable_lane_width_max) {
      ldw_disable_code_temp += uint16_bit[2];
    } else {
      /*do nothing*/
    }
  } else {
    /*do nothing*/
  }

  return ldw_disable_code_temp;
}
uint16 LaneDepartWarning::FaultCode() {
  uint16 ldw_fault_code_temp = 0;
  return ldw_fault_code_temp;
}
uint16 LaneDepartWarning::LeftSuppressionCode() {
  uint16 ldw_left_suppression_code_temp = 0;

  static uint16 left_turn_light_off_duration = 3000;  // 左转向灯处于关闭状态的时长，单位:ms
  if (lkas_input_->vehicle_info.left_turn_light_state == TRUE) {
    left_turn_light_off_duration = 0;  // 转向灯开启,转向灯关闭状态计时器清零
  } else {
    left_turn_light_off_duration += Common_Cycle_Time;
    if (left_turn_light_off_duration >= 60000) {
      left_turn_light_off_duration = 60000;
    } else {
      /*do nothing*/
    }
  }

  static boolean left_suppress_repeat_warning_flag = 0;  // 是否抑制重复报警的标志位 0:不抑制 1:抑制
  if (measurement_str_.state == 4) {                     // 检测到左侧报警
    left_suppress_repeat_warning_flag = TRUE;
  } else {  // 检测到左侧报警结束
    if (left_suppress_repeat_warning_flag == TRUE) {
      if (lkas_input_->road_info.left_line_valid == TRUE) {  // 左侧道线有效 && 检测到越过了重置线
        if (lkas_input_->wheel_to_line.fl_wheel_distance_to_line > calribration_str_.reset_warning_line) {
          left_suppress_repeat_warning_flag = FALSE;
        } else {
          /*do nothing*/
        }
      } else if ((calribration_str_.enable_roadedge_switch == TRUE) &&
                 (lkas_input_->road_info.left_roadedge_valid == TRUE) &&
                 (lkas_input_->wheel_to_line.fl_wheel_distance_to_roadedge > calribration_str_.reset_warning_line))  //?
      {
        left_suppress_repeat_warning_flag = FALSE;
      } else {
        /*do nothing*/
      }
    } else {
      /*do nothing*/
    }
  }

  // Condition1
  // 判断左转向灯处于关闭时长是否满足
  if (left_turn_light_off_duration < calribration_str_.supp_turn_light_recovery_time) {
    ldw_left_suppression_code_temp += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // Condition2
  // 判断是否处于允许左侧报警区域内
  if (lkas_input_->road_info.left_line_valid == TRUE) {  // 检测到左侧道线
    if (lkas_input_->wheel_to_line.fl_wheel_distance_to_line < 0.0F) {
      ldw_left_suppression_code_temp += uint16_bit[1];
    } else if (lkas_input_->wheel_to_line.fl_wheel_distance_to_line > calribration_str_.earliest_warning_line) {
      ldw_left_suppression_code_temp += uint16_bit[1];
    } else {
      /*do nothing*/
    }
  } else if ((calribration_str_.enable_roadedge_switch == TRUE) &&
             (lkas_input_->road_info.left_roadedge_valid == TRUE)) {  // 使能路沿场景&&检测到路沿
    if (lkas_input_->wheel_to_line.fl_wheel_distance_to_roadedge < 0.0F) {
      ldw_left_suppression_code_temp += uint16_bit[1];
    } else if (lkas_input_->wheel_to_line.fl_wheel_distance_to_roadedge > calribration_str_.earliest_warning_line) {
      ldw_left_suppression_code_temp += uint16_bit[1];
    } else {
      /*do nothing*/
    }
  } else {
    ldw_left_suppression_code_temp += uint16_bit[1];
  }

  // Condition3
  // 距上次触发报警后,越过了重置线
  if (left_suppress_repeat_warning_flag == TRUE) {
    ldw_left_suppression_code_temp += uint16_bit[2];
  } else {
    /*do nothing*/
  }

  return ldw_left_suppression_code_temp;
}
uint16 LaneDepartWarning::LeftKickDownCode() {
  uint16 ldw_left_kickdown_code_temp = 0;

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
  if (lkas_input_->vehicle_info.left_turn_light_state == TRUE) {
    ldw_left_kickdown_code_temp += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // Condition2
  // 判断是否处于允许左侧报警区域内
  if (lkas_input_->road_info.left_line_valid == TRUE) {  // 检测到左侧道线
    if (lkas_input_->wheel_to_line.fl_wheel_distance_to_line < calribration_str_.latest_warning_line) {
      ldw_left_kickdown_code_temp += uint16_bit[1];
    } else if (lkas_input_->wheel_to_line.fl_wheel_distance_to_line > calribration_str_.earliest_warning_line) {
      ldw_left_kickdown_code_temp += uint16_bit[1];
    } else {
      /*do nothing*/
    }
  } else if ((calribration_str_.enable_roadedge_switch == TRUE) &&
             (lkas_input_->road_info.left_roadedge_valid == TRUE)) {  // 使能路沿场景&&检测到路沿
    if (lkas_input_->wheel_to_line.fl_wheel_distance_to_roadedge < calribration_str_.latest_warning_line) {
      ldw_left_kickdown_code_temp += uint16_bit[1];
    } else if (lkas_input_->wheel_to_line.fl_wheel_distance_to_roadedge > calribration_str_.earliest_warning_line) {
      ldw_left_kickdown_code_temp += uint16_bit[1];
    } else {
      /*do nothing*/
    }
  } else {
    ldw_left_kickdown_code_temp += uint16_bit[1];
  }

  // Condition3
  if (left_warning_duration > calribration_str_.warning_time_max) {
    ldw_left_kickdown_code_temp += uint16_bit[2];
  }

  // Condition4
  if (measurement_str_.left_intervention == FALSE) {
    ldw_left_kickdown_code_temp += uint16_bit[3];
  }

  return ldw_left_kickdown_code_temp;
}
uint16 LaneDepartWarning::RightSuppressionCode() {
  uint16 ldw_right_suppression_code_temp = 0;

  static uint16 right_turn_light_off_duration = 3000;  // 右转向灯处于关闭状态的时长，单位:ms
  if (lkas_input_->vehicle_info.right_turn_light_state == TRUE) {
    right_turn_light_off_duration = 0;  // 转向灯开启,转向灯关闭状态计时器清零
  } else {
    right_turn_light_off_duration += Common_Cycle_Time;
    if (right_turn_light_off_duration >= 60000) {
      right_turn_light_off_duration = 60000;
    } else {
      /*do nothing*/
    }
  }

  static boolean right_suppress_repeat_warning_flag = 0;  // 是否抑制重复报警的标志位 0:不抑制 1:抑制
  if (measurement_str_.state == 5) {                      // 检测到右侧报警
    right_suppress_repeat_warning_flag = TRUE;
  } else {  // 检测到右侧报警结束
    if (right_suppress_repeat_warning_flag == TRUE) {
      if (lkas_input_->road_info.right_line_valid == TRUE) {  // 右侧道线有效
        if (lkas_input_->wheel_to_line.fr_wheel_distance_to_line <
            (-1.0 * calribration_str_.reset_warning_line)) {  // 检测到越过了重置线
          right_suppress_repeat_warning_flag = FALSE;
        } else {
          /*do nothing*/
        }
      } else if ((calribration_str_.enable_roadedge_switch == TRUE) &&
                 (lkas_input_->road_info.right_roadedge_valid == TRUE) &&
                 (lkas_input_->wheel_to_line.fr_wheel_distance_to_roadedge <
                  (-1.0 * calribration_str_.reset_warning_line))) {
        right_suppress_repeat_warning_flag = FALSE;
      } else {
        /*do nothing*/
      }
    } else {
      /*do nothing*/
    }
  }

  // Condition1
  // 判断右转向灯处于关闭时长是否满足
  if (right_turn_light_off_duration < calribration_str_.supp_turn_light_recovery_time) {
    ldw_right_suppression_code_temp += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // Condition2
  // 判断是否处于允许右侧报警区域内
  if (lkas_input_->road_info.right_line_valid == TRUE) {  // 检测到右侧道线
    if (lkas_input_->wheel_to_line.fr_wheel_distance_to_line > 0.0F) {
      ldw_right_suppression_code_temp += uint16_bit[1];
    } else if (lkas_input_->wheel_to_line.fr_wheel_distance_to_line <
               (-1.0 * calribration_str_.earliest_warning_line)) {
      ldw_right_suppression_code_temp += uint16_bit[1];
    } else {
      /*do nothing*/
    }
  } else if ((calribration_str_.enable_roadedge_switch == TRUE) &&
             (lkas_input_->road_info.right_roadedge_valid == TRUE)) {  // 使能路沿场景&&检测到路沿
    if (lkas_input_->wheel_to_line.fr_wheel_distance_to_roadedge > 0.0F) {
      ldw_right_suppression_code_temp += uint16_bit[1];
    } else if (lkas_input_->wheel_to_line.fr_wheel_distance_to_roadedge <
               (-1.0 * calribration_str_.earliest_warning_line)) {
      ldw_right_suppression_code_temp += uint16_bit[1];
    } else {
      /*do nothing*/
    }
  } else {
    ldw_right_suppression_code_temp += uint16_bit[1];
  }

  // Condition3
  if (right_suppress_repeat_warning_flag == TRUE) {
    ldw_right_suppression_code_temp += uint16_bit[2];
  } else {
    /*do nothing*/
  }

  return ldw_right_suppression_code_temp;
}
uint16 LaneDepartWarning::RightKickDownCode() {
  uint16 ldw_right_kickdown_code_temp = 0;

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
  if (lkas_input_->vehicle_info.right_turn_light_state == TRUE) {
    ldw_right_kickdown_code_temp += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // Condition2
  // 判断是否处于允许右侧报警区域内
  if (lkas_input_->road_info.right_line_valid == TRUE) {  // 检测到右侧道线
    if (lkas_input_->wheel_to_line.fr_wheel_distance_to_line > (-1.0 * calribration_str_.latest_warning_line)) {
      ldw_right_kickdown_code_temp += uint16_bit[1];
    } else if (lkas_input_->wheel_to_line.fr_wheel_distance_to_line <
               (-1.0 * calribration_str_.earliest_warning_line)) {
      ldw_right_kickdown_code_temp += uint16_bit[1];
    } else {
      /*do nothing*/
    }
  } else if ((calribration_str_.enable_roadedge_switch == TRUE) &&
             (lkas_input_->road_info.right_roadedge_valid == TRUE)) {  // 使能路沿场景&&检测到路沿
    if (lkas_input_->wheel_to_line.fr_wheel_distance_to_roadedge > (-1.0 * calribration_str_.latest_warning_line)) {
      ldw_right_kickdown_code_temp += uint16_bit[1];
    } else if (lkas_input_->wheel_to_line.fr_wheel_distance_to_roadedge <
               (-1.0 * calribration_str_.earliest_warning_line)) {
      ldw_right_kickdown_code_temp += uint16_bit[1];
    } else {
      /*do nothing*/
    }
  } else {
    ldw_right_kickdown_code_temp += uint16_bit[1];
  }

  // Condition3
  if (right_warning_duration > calribration_str_.warning_time_max) {
    ldw_right_kickdown_code_temp += uint16_bit[2];
  }

  // Condition4
  if (measurement_str_.right_intervention == FALSE) {
    ldw_right_kickdown_code_temp += uint16_bit[3];
  }

  return ldw_right_kickdown_code_temp;
}
uint8 LaneDepartWarning::StateMachine() {
  static uint8 ldw_state_machine_init_flag = 0;  // LDW状态机初始化状态 0:未初始化过 1:已完成过初始化
  static uint8 ldw_state_fault_off_standby_active = 0;  // LDW一级主状态 FAULT OFF STANDBY ACTIVE
  static uint8 ldw_state_active_substate =
      0;  // LDW ACTIVE状态子状态 NO_ACTIVE_CHILD NoIntervention LeftIntervention RightIntervention
  uint8 ldw_state_temp;
  if (ldw_state_machine_init_flag == 0) {
    // 状态机处于初始化状态 根据开关状态,决定第一个周期是输出OFF还是STANDBY
    ldw_state_machine_init_flag = 1;
    if (!measurement_str_.main_switch) {
      ldw_state_fault_off_standby_active = LKA_StateMachine_IN_OFF;
      ldw_state_temp = 1;
    } else {
      ldw_state_fault_off_standby_active = LKA_StateMachine_IN_STANDBY;
      ldw_state_temp = 2;
    }
  } else {
    // 状态机处于完成过初始化的状态
    switch (ldw_state_fault_off_standby_active) {
      case LKA_StateMachine_IN_ACTIVE:
        if (!measurement_str_.main_switch) {  // ACTIVE->OFF
          ldw_state_active_substate = LKA_StateMachine_IN_NO_ACTIVE_CHILD;
          ldw_state_fault_off_standby_active = LKA_StateMachine_IN_OFF;
          ldw_state_temp = 1;
        } else if (measurement_str_.fault_code) {  // ACTIVE->FAULT
          ldw_state_active_substate = LKA_StateMachine_IN_NO_ACTIVE_CHILD;
          ldw_state_fault_off_standby_active = LKA_StateMachine_IN_FAULT;
          ldw_state_temp = 0;
        } else if (measurement_str_.disable_code) {  // ACTIVE->STANDBY
          ldw_state_active_substate = LKA_StateMachine_IN_NO_ACTIVE_CHILD;
          ldw_state_fault_off_standby_active = LKA_StateMachine_IN_STANDBY;
          ldw_state_temp = 2;
        } else {  // 继续维持在ACTIVE状态
          switch (ldw_state_active_substate) {
            case LKA_StateMachine_IN_LeftIntervention:
              ldw_state_temp = 4;
              if (measurement_str_.left_kickdown_code) {  // LeftIntervention->NoIntervention
                ldw_state_active_substate = LKA_StateMachine_IN_NoIntervention;
                ldw_state_temp = 3;
              }
              break;
            case LKA_StateMachine_IN_NoIntervention:
              ldw_state_temp = 3;
              if (measurement_str_.right_intervention &&
                  (!measurement_str_.right_suppression_code)) {  // NoIntervention->RightIntervention
                ldw_state_active_substate = LKA_StateMachine_IN_RightIntervention;
                ldw_state_temp = 5;
              } else if (measurement_str_.left_intervention &&
                         (!measurement_str_.left_suppression_code)) {  // NoIntervention->LeftIntervention
                ldw_state_active_substate = LKA_StateMachine_IN_LeftIntervention;
                ldw_state_temp = 4;
              }
              break;
            default:
              ldw_state_temp = 5;
              if (measurement_str_.right_kickdown_code) {  // RightIntervention->NoIntervention
                ldw_state_active_substate = LKA_StateMachine_IN_NoIntervention;
                ldw_state_temp = 3;
              }
              break;
          }
        }
        break;
      case LKA_StateMachine_IN_FAULT:
        ldw_state_temp = 0;
        if (!measurement_str_.main_switch) {  // FAULT->OFF
          ldw_state_fault_off_standby_active = LKA_StateMachine_IN_OFF;
          ldw_state_temp = 1;
        } else if (!measurement_str_.fault_code) {  // FAULT->STANDBY
          ldw_state_fault_off_standby_active = LKA_StateMachine_IN_STANDBY;
          ldw_state_temp = 2;
        }
        break;
      case LKA_StateMachine_IN_OFF:
        ldw_state_temp = 1;
        if (measurement_str_.main_switch) {  // OFF->STANDBY
          ldw_state_fault_off_standby_active = LKA_StateMachine_IN_STANDBY;
          ldw_state_temp = 2;
        }
        break;
      default:
        ldw_state_temp = 2;
        if (!measurement_str_.main_switch) {  // STANDBY->OFF
          ldw_state_fault_off_standby_active = LKA_StateMachine_IN_OFF;
          ldw_state_temp = 1;
        } else if (measurement_str_.fault_code) {  // STANDBY->FAULT
          ldw_state_fault_off_standby_active = LKA_StateMachine_IN_FAULT;
          ldw_state_temp = 0;
        } else if (measurement_str_.enable_code == 0) {  // STANDBY->ACTIVE
          ldw_state_fault_off_standby_active = LKA_StateMachine_IN_ACTIVE;
          ldw_state_active_substate = LKA_StateMachine_IN_NoIntervention;
          ldw_state_temp = 3;
        }
        break;
    }
  }
  return ldw_state_temp;
}
}  // namespace planning
