#include "emergency_lane_keep_context.h"
namespace planning {
void EmergencyLaneKeep::Init(planning::LkasInput *lkas_input,
                             framework::Session *session) {
  session_ = session;
  lkas_input_ = lkas_input;
  calribration_str_.enable_roadedge_switch =
      false;  // 是否使能功能的路沿场景开关 0:不使能  1:使能
  calribration_str_.enable_vehspd_display_min =
      60.0 / 3.6;  // 激活的最小仪表车速，单位：m/s
  calribration_str_.enable_vehspd_display_max =
      150.0 / 3.6;  // 激活的最大仪表车速，单位：m/s
  calribration_str_.disable_vehspd_display_min =
      55.0 / 3.6;  // 退出的最小仪表车速，单位：m/s
  calribration_str_.disable_vehspd_display_max =
      155.0 / 3.6;  // 退出的最大仪表车速，单位：m/s
  calribration_str_.supp_turn_light_recovery_time =
      2000;  // 转向灯抑制恢复时长，单位：ms
  calribration_str_.earliest_warning_line = 1.5;  // 触发的最早报警线，单位：m
  calribration_str_.latest_warning_line = -0.3;  // 触发的最晚报警线，单位：m
  calribration_str_.reset_warning_line = 0.15;  // 触发的报警重置线，单位：m
  calribration_str_.warning_time_max = 8000;  // 最大报警时长，单位：ms
  calribration_str_.tlc_line_far =
      1.0;  // 针对道线触发报警的高灵敏度阈值，单位：s
  calribration_str_.tlc_line_medium =
      0.6;  // 针对道线触发报警的中灵敏度阈值，单位：s
  calribration_str_.tlc_line_near =
      0.2;  // 针对道线触发报警的低灵敏度阈值，单位：s
  calribration_str_.suppression_driver_hand_trq =
      2.0;  // 抑制报警的驾驶员手力矩(绝对值)阈值，单位：Nm
  calribration_str_.kickdown_driver_hand_trq =
      2.5;  // 打断报警的驾驶员手力矩(绝对值)阈值，单位：Nm
  bsd_lca_.Init(lkas_input_, session_);
}
void EmergencyLaneKeep::Update() {
  measurement_str_.main_switch =
      lkas_input_->vehicle_info.elk_main_switch;  // ELK功能开关状态 0:Off  1:On
  // measurement_str_.tlc_line_threshold =
  //     calribration_str_
  //         .tlc_line_far;  // 依据驾驶员选择的灵敏度等级,设定触发阈值
  measurement_str_.tlc_line_threshold = lkas_input_->param.ldp_tlc_thrd;
  /*ELK侧方危险区域划分*/
  // bsd_lca_.Init();
  /*ELK侧方危险判断*/
  uint8 elk_bsd_lca_state =
      0;  // 0,左右侧都没有主动换道意图；1，有左侧主动换道意图且有碰撞风险；2，有右侧主动换道意图且有碰撞风险；3，左右侧都有主动换道意图且都有碰撞风险
  elk_bsd_lca_state = bsd_lca_.RunOnce();
  // LOG_DEBUG("LaneKeepAssistManager::elk:: bsd_lca RunOnce \n");
  if ((elk_bsd_lca_state == 1) || (elk_bsd_lca_state == 3)) {
    measurement_str_.left_bsd_lca_code = 1;
  } else {
    /*do nothing*/
  }
  if ((elk_bsd_lca_state) == 2 || (elk_bsd_lca_state == 3)) {
    measurement_str_.right_bsd_lca_code = 1;
  } else {
    /*do nothing*/
  }
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
void EmergencyLaneKeep::RunOnce() {
  Update();
  measurement_str_.enable_code = EnableCode();
  measurement_str_.disable_code = DisableCode();
  measurement_str_.fault_code = FaultCode();
  measurement_str_.left_suppression_code = LeftSuppressionCode();
  measurement_str_.left_kickdown_code = LeftKickDownCode();
  measurement_str_.right_suppression_code = RightSuppressionCode();
  measurement_str_.right_kickdown_code = RightKickDownCode();
  measurement_str_.state = StateMachine();  // LDP状态机跳转
  ste_elk_output_info();

  JSON_DEBUG_VALUE("lkas_function::elk::left_intervention",
                   measurement_str_.left_intervention);
  JSON_DEBUG_VALUE("lkas_function::elk::right_intervention",
                   measurement_str_.right_intervention);
  JSON_DEBUG_VALUE("lkas_function::elk::main_switch",
                   measurement_str_.main_switch);
  JSON_DEBUG_VALUE("lkas_function::elk::enable_code",
                   measurement_str_.enable_code);
  JSON_DEBUG_VALUE("lkas_function::elk::disable_code",
                   measurement_str_.disable_code);
  JSON_DEBUG_VALUE("lkas_function::elk::fault_code",
                   measurement_str_.fault_code);
  JSON_DEBUG_VALUE("lkas_function::elk::left_suppression_code",
                   measurement_str_.left_suppression_code);
  JSON_DEBUG_VALUE("lkas_function::elk::left_kickdown_code",
                   measurement_str_.left_kickdown_code);
  JSON_DEBUG_VALUE("lkas_function::elk::right_suppression_code",
                   measurement_str_.right_suppression_code);
  JSON_DEBUG_VALUE("lkas_function::elk::right_kickdown_code",
                   measurement_str_.right_kickdown_code);
  JSON_DEBUG_VALUE("lkas_function::elk::state", measurement_str_.state);
  JSON_DEBUG_VALUE("elk_tlc_threshold", measurement_str_.tlc_line_threshold);
}
uint16 EmergencyLaneKeep::EnableCode() {
  uint16 elk_enable_code_temp = 0;
  // Condition1
  // 判断车速是否处于工作车速范围内
  if (lkas_input_->vehicle_info.veh_display_speed <
      calribration_str_.enable_vehspd_display_min) {
    elk_enable_code_temp += uint16_bit[0];
  } else if (lkas_input_->vehicle_info.veh_display_speed >
             calribration_str_.enable_vehspd_display_max) {
    elk_enable_code_temp += uint16_bit[0];
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
      elk_enable_code_temp += uint16_bit[1];
    }
  } else {
    elk_enable_code_temp += uint16_bit[1];
  }

  // Condition3
  // 判断当前车道宽度是否满足激活条件
  if (lkas_input_->road_info.lane_width_valid == true) {
    if (lkas_input_->road_info.lane_width < k_lka_enable_lane_width_min) {
      elk_enable_code_temp += uint16_bit[2];
    } else if (lkas_input_->road_info.lane_width >
               k_lka_enable_lane_width_max) {
      elk_enable_code_temp += uint16_bit[2];
    } else {
      /*do nothing*/
    }
  } else {
    elk_enable_code_temp += uint16_bit[2];
  }
  return elk_enable_code_temp;
}
uint16 EmergencyLaneKeep::DisableCode() {
  uint16 elk_disable_code_temp = 0;

  // Condition1
  // 判断车速是否处于工作车速范围内
  if (lkas_input_->vehicle_info.veh_display_speed <
      calribration_str_.disable_vehspd_display_min) {
    elk_disable_code_temp += uint16_bit[0];
  } else if (lkas_input_->vehicle_info.veh_display_speed >
             calribration_str_.disable_vehspd_display_max) {
    elk_disable_code_temp += uint16_bit[0];
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
      elk_disable_code_temp += uint16_bit[1];
    }
  } else {
    elk_disable_code_temp += uint16_bit[1];
  }

  // Condition3
  // 判断当前车道宽度是否满足激活条件
  if (lkas_input_->road_info.lane_width_valid == true) {
    if (lkas_input_->road_info.lane_width < k_lka_disable_lane_width_min) {
      elk_disable_code_temp += uint16_bit[2];
    } else if (lkas_input_->road_info.lane_width >
               k_lka_disable_lane_width_max) {
      elk_disable_code_temp += uint16_bit[2];
    } else {
      /*do nothing*/
    }
  } else {
    elk_disable_code_temp += uint16_bit[2];
  }

  return elk_disable_code_temp;
}
uint16 EmergencyLaneKeep::FaultCode() {
  uint16 elk_fault_code_temp = 0;
  return elk_fault_code_temp;
}
uint16 EmergencyLaneKeep::LeftSuppressionCode() {
  uint16 elk_left_suppression_code_temp = 0;

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

  uint8 left_line_type_flag = 0;  // 是否可跨越线型 1:不可跨越 0:可跨越
                                  // 偏离侧道线类型条件
  if (lkas_input_->road_info.left_line_valid == 1) {  // 检测到左侧道线
    if (lkas_input_->road_info.left_line_type == 1)   // 实线
    {
      left_line_type_flag = 1;
    } else {
      // 虚线
      /*do nothing*/
    }
  } else {  // 在active状态下道路宽是合理的
    // 左侧无线，则右侧一定有路沿或者线或者左侧有路沿，
    // 左侧一定是存在标志使路宽合理，但是这个标志不能跨越（
    // 1.有路沿，不能跨越；
    // 2，无路沿，存在特殊标志使路宽合理，不可跨越
    left_line_type_flag = 1;
  }

  // Condition1
  // 判断左转向灯处于关闭时长是否满足
  /*转向灯开启或者转向灯刚关闭一段时间判断为有主动转向意图，
  1.此时应该判断相应侧线行，如果是实线，则应该抑制主动转向意图,即不再抑制ELK主动纠偏能力
  2.此时应该判断相应侧是否有来车碰撞风险，如果有碰撞风险，则应该抑制主动转向意图,即不再抑制ELK主动纠偏能力
  代码上则使elk_left_suppression_code_temp==0代表不再抑制ELK主动纠偏能力
  */
  if ((left_turn_light_off_duration <
       calribration_str_.supp_turn_light_recovery_time) &&
      (left_line_type_flag == 0) && (measurement_str_.left_bsd_lca_code == 0)) {
    elk_left_suppression_code_temp += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // Condition2
  // 判断是否处于允许左侧报警区域内
  if (lkas_input_->road_info.left_line_valid == true) {  // 检测到左侧道线
    if (lkas_input_->wheel_to_line.fl_wheel_distance_to_line < 0.0) {
      elk_left_suppression_code_temp += uint16_bit[1];
    } else if (lkas_input_->wheel_to_line.fl_wheel_distance_to_line >
               calribration_str_.earliest_warning_line) {
      elk_left_suppression_code_temp += uint16_bit[1];
    } else {
      /*do nothing*/
    }
  } else if ((calribration_str_.enable_roadedge_switch == true) &&
             (lkas_input_->road_info.left_roadedge_valid ==
              true)) {  // 使能路沿场景&&检测到路沿
    if (lkas_input_->wheel_to_line.fl_wheel_distance_to_roadedge < 0.0) {
      elk_left_suppression_code_temp += uint16_bit[1];
    } else if (lkas_input_->wheel_to_line.fl_wheel_distance_to_roadedge >
               calribration_str_.earliest_warning_line) {
      elk_left_suppression_code_temp += uint16_bit[1];
    } else {
      /*do nothing*/
    }
  } else {
    elk_left_suppression_code_temp += uint16_bit[1];
  }

  // Condition3
  // 距上次触发报警后,越过了重置线
  // true未越过
  if (left_suppress_repeat_warning_flag == true) {
    elk_left_suppression_code_temp += uint16_bit[2];
  } else {
    /*do nothing*/
  }

  // Condition4
  // 驾驶员手力矩条件
  if (fabs(lkas_input_->vehicle_info.driver_hand_torque) >
      calribration_str_.suppression_driver_hand_trq) {
    elk_left_suppression_code_temp += uint16_bit[3];
  } else {
    /*do nothing*/
  }

  // Condition5
  if ((lkas_input_->function_state > iflyauto::FunctionalState_SCC) &&
      (lkas_input_->function_state <
       iflyauto::FunctionalState_PARK_IN_APA_IN)) {
    elk_left_suppression_code_temp += uint16_bit[4];
  }

  return elk_left_suppression_code_temp;
}
uint16 EmergencyLaneKeep::LeftKickDownCode() {
  uint16 elk_left_kickdown_code_temp = 0;

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
    elk_left_kickdown_code_temp += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // Condition2
  // 判断是否处于允许左侧报警区域内
  if (lkas_input_->road_info.left_line_valid == true) {  // 检测到左侧道线
    if (lkas_input_->wheel_to_line.fl_wheel_distance_to_line <
        calribration_str_.latest_warning_line) {
      elk_left_kickdown_code_temp += uint16_bit[1];
    } else if (lkas_input_->wheel_to_line.fl_wheel_distance_to_line >
               calribration_str_.earliest_warning_line) {
      elk_left_kickdown_code_temp += uint16_bit[1];
    } else {
      /*do nothing*/
    }
  } else if ((calribration_str_.enable_roadedge_switch == true) &&
             (lkas_input_->road_info.left_roadedge_valid ==
              true)) {  // 使能路沿场景&&检测到路沿
    if (lkas_input_->wheel_to_line.fl_wheel_distance_to_roadedge <
        calribration_str_.latest_warning_line) {
      elk_left_kickdown_code_temp += uint16_bit[1];
    } else if (lkas_input_->wheel_to_line.fl_wheel_distance_to_roadedge >
               calribration_str_.earliest_warning_line) {
      elk_left_kickdown_code_temp += uint16_bit[1];
    } else {
      /*do nothing*/
    }
  } else {
    elk_left_kickdown_code_temp += uint16_bit[1];
  }

  // Condition3
  if (left_warning_duration > calribration_str_.warning_time_max) {
    elk_left_kickdown_code_temp += uint16_bit[2];
  } else {
    /*do nothing*/
  }

  // Condition4
  if (fabs(lkas_input_->vehicle_info.driver_hand_torque) >
      calribration_str_.kickdown_driver_hand_trq) {
    elk_left_kickdown_code_temp += uint16_bit[3];
  } else {
    /*do nothing*/
  }

  // Condition5
  if ((lkas_input_->function_state > iflyauto::FunctionalState_SCC) &&
      (lkas_input_->function_state <
       iflyauto::FunctionalState_PARK_IN_APA_IN)) {
    elk_left_kickdown_code_temp += uint16_bit[4];
  }

  return elk_left_kickdown_code_temp;
}
uint16 EmergencyLaneKeep::RightSuppressionCode() {
  uint16 elk_right_suppression_code_temp = 0;

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

  uint8 right_line_type_flag = 0;  // 是否可跨越线型 1:不可跨越 0:可跨越
                                   // 偏离侧道线类型条件
  if (lkas_input_->road_info.right_line_valid == 1) {
    // 检测到左侧道线
    if (lkas_input_->road_info.right_line_type == 1)  // 实线,不可跨越线型
    {
      right_line_type_flag = 1;
    }
  } else {
    // 在active状态下道路宽是合理的
    // 右侧无线，则右侧一定有路沿或者线或者左侧有路沿，
    // 右侧一定是存在标志使路宽合理，但是这个标志不能跨越（
    // 1.有路沿，不能跨越；
    // 2，无路沿，存在特殊标志使路宽合理，不可跨越
    right_line_type_flag = 1;
  }

  // Condition1
  // 判断左转向灯处于关闭时长是否满足
  /*转向灯开启或者转向灯刚关闭一段时间判断为有主动转向意图，
  1.此时应该判断相应侧线行，如果是实线，则应该抑制主动转向意图,即不再抑制ELK主动纠偏能力
  2.此时应该判断相应侧是否有来车碰撞风险，如果有碰撞风险，则应该抑制主动转向意图,即不再抑制ELK主动纠偏能力
  代码上则使elk_right_suppression_code_temp==0代表不再抑制ELK主动纠偏能力
  */
  if (right_turn_light_off_duration <
          calribration_str_.supp_turn_light_recovery_time &&
      right_line_type_flag == 0 && measurement_str_.right_bsd_lca_code == 0) {
    elk_right_suppression_code_temp += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // Condition2
  // 判断是否处于允许右侧报警区域内
  if (lkas_input_->road_info.right_line_valid == true) {  // 检测到右侧道线
    if (lkas_input_->wheel_to_line.fr_wheel_distance_to_line > 0.0) {
      elk_right_suppression_code_temp += uint16_bit[1];
    } else if (lkas_input_->wheel_to_line.fr_wheel_distance_to_line <
               (-1.0 * calribration_str_.earliest_warning_line)) {
      elk_right_suppression_code_temp += uint16_bit[1];
    } else {
      /*do nothing*/
    }
  } else if ((calribration_str_.enable_roadedge_switch == true) &&
             (lkas_input_->road_info.right_roadedge_valid ==
              true)) {  // 使能路沿场景&&检测到路沿
    if (lkas_input_->wheel_to_line.fr_wheel_distance_to_roadedge > 0.0) {
      elk_right_suppression_code_temp += uint16_bit[1];
    } else if (lkas_input_->wheel_to_line.fr_wheel_distance_to_roadedge <
               (-1.0 * calribration_str_.earliest_warning_line)) {
      elk_right_suppression_code_temp += uint16_bit[1];
    } else {
      /*do nothing*/
    }
  } else {
    elk_right_suppression_code_temp += uint16_bit[1];
  }

  // Condition3
  if (right_suppress_repeat_warning_flag == true) {
    elk_right_suppression_code_temp += uint16_bit[2];
  } else {
    /*do nothing*/
  }

  // Condition4
  // 驾驶员手力矩条件
  if (fabs(lkas_input_->vehicle_info.driver_hand_torque) >
      calribration_str_.suppression_driver_hand_trq) {
    elk_right_suppression_code_temp += uint16_bit[3];
  } else {
    /*do nothing*/
  }

  // Condition5
  if ((lkas_input_->function_state > iflyauto::FunctionalState_SCC) &&
      (lkas_input_->function_state <
       iflyauto::FunctionalState_PARK_IN_APA_IN)) {
    elk_right_suppression_code_temp += uint16_bit[4];
  }

  return elk_right_suppression_code_temp;
}
uint16 EmergencyLaneKeep::RightKickDownCode() {
  uint16 elk_right_kickdown_code_temp = 0;

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
    elk_right_kickdown_code_temp += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // Condition2
  // 判断是否处于允许右侧报警区域内
  if (lkas_input_->road_info.right_line_valid == true) {  // 检测到右侧道线
    if (lkas_input_->wheel_to_line.fr_wheel_distance_to_line >
        (-1.0 * calribration_str_.latest_warning_line)) {
      elk_right_kickdown_code_temp += uint16_bit[1];
    } else if (lkas_input_->wheel_to_line.fr_wheel_distance_to_line <
               (-1.0 * calribration_str_.earliest_warning_line)) {
      elk_right_kickdown_code_temp += uint16_bit[1];
    } else {
      /*do nothing*/
    }
  } else if ((calribration_str_.enable_roadedge_switch == true) &&
             (lkas_input_->road_info.right_roadedge_valid ==
              true)) {  // 使能路沿场景&&检测到路沿
    if (lkas_input_->wheel_to_line.fr_wheel_distance_to_roadedge >
        (-1.0 * calribration_str_.latest_warning_line)) {
      elk_right_kickdown_code_temp += uint16_bit[1];
    } else if (lkas_input_->wheel_to_line.fr_wheel_distance_to_roadedge <
               (-1.0 * calribration_str_.earliest_warning_line)) {
      elk_right_kickdown_code_temp += uint16_bit[1];
    } else {
      /*do nothing*/
    }
  } else {
    elk_right_kickdown_code_temp += uint16_bit[1];
  }

  // Condition3
  if (right_warning_duration > calribration_str_.warning_time_max) {
    elk_right_kickdown_code_temp += uint16_bit[2];
  } else {
    /*do nothing*/
  }

  // Condition4
  if (fabs(lkas_input_->vehicle_info.driver_hand_torque) >
      calribration_str_.kickdown_driver_hand_trq) {
    elk_right_kickdown_code_temp += uint16_bit[3];
  } else {
    /*do nothing*/
  }

  // Condition5
  if ((lkas_input_->function_state > iflyauto::FunctionalState_SCC) &&
      (lkas_input_->function_state <
       iflyauto::FunctionalState_PARK_IN_APA_IN)) {
    elk_right_kickdown_code_temp += uint16_bit[4];
  }

  return elk_right_kickdown_code_temp;
}
uint8 EmergencyLaneKeep::StateMachine() {
  static uint8 elk_state_machine_init_flag =
      0;  // ldp状态机初始化状态 0:未初始化过 1:已完成过初始化
  static uint8 elk_state_fault_off_standby_active =
      0;  // ldp一级主状态 FAULT OFF STANDBY ACTIVE
  static uint8 elk_state_active_substate =
      0;  // ldp ACTIVE状态子状态 NO_ACTIVE_CHILD NoIntervention
          // LeftIntervention RightIntervention
  uint8 elk_state_temp;
  if (elk_state_machine_init_flag == 0) {
    // 状态机处于初始化状态 根据开关状态,决定第一个周期是输出OFF还是STANDBY
    elk_state_machine_init_flag = 1;
    if (!measurement_str_.main_switch) {
      elk_state_fault_off_standby_active = LKA_StateMachine_IN_OFF;
      elk_state_temp = 1;
    } else {
      elk_state_fault_off_standby_active = LKA_StateMachine_IN_STANDBY;
      elk_state_temp = 2;
    }
  } else {
    // 状态机处于完成过初始化的状态
    switch (elk_state_fault_off_standby_active) {
      case LKA_StateMachine_IN_ACTIVE:
        if (!measurement_str_.main_switch) {  // ACTIVE->OFF
          elk_state_active_substate = LKA_StateMachine_IN_NO_ACTIVE_CHILD;
          elk_state_fault_off_standby_active = LKA_StateMachine_IN_OFF;
          elk_state_temp = 1;
        } else if (measurement_str_.fault_code) {  // ACTIVE->FAULT
          elk_state_active_substate = LKA_StateMachine_IN_NO_ACTIVE_CHILD;
          elk_state_fault_off_standby_active = LKA_StateMachine_IN_FAULT;
          elk_state_temp = 0;
        } else if (measurement_str_.disable_code) {  // ACTIVE->STANDBY
          elk_state_active_substate = LKA_StateMachine_IN_NO_ACTIVE_CHILD;
          elk_state_fault_off_standby_active = LKA_StateMachine_IN_STANDBY;
          elk_state_temp = 2;
        } else {  // 继续维持在ACTIVE状态
          switch (elk_state_active_substate) {
            case LKA_StateMachine_IN_LeftIntervention:
              elk_state_temp = 4;
              if (measurement_str_
                      .left_kickdown_code) {  // LeftIntervention->NoIntervention
                elk_state_active_substate = LKA_StateMachine_IN_NoIntervention;
                elk_state_temp = 3;
              }
              break;
            case LKA_StateMachine_IN_NoIntervention:
              elk_state_temp = 3;
              if (measurement_str_.right_intervention &&
                  (!measurement_str_
                        .right_suppression_code)) {  // NoIntervention->RightIntervention
                elk_state_active_substate =
                    LKA_StateMachine_IN_RightIntervention;
                elk_state_temp = 5;
              } else if (
                  measurement_str_.left_intervention &&
                  (!measurement_str_
                        .left_suppression_code)) {  // NoIntervention->LeftIntervention
                elk_state_active_substate =
                    LKA_StateMachine_IN_LeftIntervention;
                elk_state_temp = 4;
              }
              break;
            default:
              elk_state_temp = 5;
              if (measurement_str_
                      .right_kickdown_code) {  // RightIntervention->NoIntervention
                elk_state_active_substate = LKA_StateMachine_IN_NoIntervention;
                elk_state_temp = 3;
              }
              break;
          }
        }
        break;
      case LKA_StateMachine_IN_FAULT:
        elk_state_temp = 0;
        if (!measurement_str_.main_switch) {  // FAULT->OFF
          elk_state_fault_off_standby_active = LKA_StateMachine_IN_OFF;
          elk_state_temp = 1;
        } else if (!measurement_str_.fault_code) {  // FAULT->STANDBY
          elk_state_fault_off_standby_active = LKA_StateMachine_IN_STANDBY;
          elk_state_temp = 2;
        }
        break;
      case LKA_StateMachine_IN_OFF:
        elk_state_temp = 1;
        if (measurement_str_.main_switch) {  // OFF->STANDBY
          elk_state_fault_off_standby_active = LKA_StateMachine_IN_STANDBY;
          elk_state_temp = 2;
        }
        break;
      default:
        elk_state_temp = 2;
        if (!measurement_str_.main_switch) {  // STANDBY->OFF
          elk_state_fault_off_standby_active = LKA_StateMachine_IN_OFF;
          elk_state_temp = 1;
        } else if (measurement_str_.fault_code) {  // STANDBY->FAULT
          elk_state_fault_off_standby_active = LKA_StateMachine_IN_FAULT;
          elk_state_temp = 0;
        } else if (measurement_str_.enable_code == 0) {  // STANDBY->ACTIVE
          elk_state_fault_off_standby_active = LKA_StateMachine_IN_ACTIVE;
          elk_state_active_substate = LKA_StateMachine_IN_NoIntervention;
          elk_state_temp = 3;
        }
        break;
    }
  }
  return elk_state_temp;
}
}  // namespace planning