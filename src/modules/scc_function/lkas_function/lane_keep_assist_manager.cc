#include "lane_keep_assist_manager.h"

namespace planning {
void LaneKeepAssistManager::Init(framework::Session *session) {
  // session init
  session_ = session;
  // ldw init
  lane_depart_warning_.Init(&lkas_input_);
  // ldp init
  lane_depart_prevention_.Init(&lkas_input_);
  // elk init
  emergency_lane_keep_.Init(&lkas_input_, session_);

  LOG_DEBUG("LaneKeepAssistManager::ldw_ldp_elk has been inited \n");
}
void LaneKeepAssistManager::Update() {
  // input update
  lkas_input_.road_info.left_line_valid =
      0;  // 本车道左侧道线有效性 0:Invalid 1:Valid
  lkas_input_.road_info.left_line_type =
      0;  // 本车道左侧车道线类型 0：虚线  1：实线
  lkas_input_.road_info.left_line_c0 = 0.0F;  // 本车道左侧道线方程系数c0
  lkas_input_.road_info.left_line_c1 = 0.0F;  // 本车道左侧道线方程系数c1
  lkas_input_.road_info.left_line_c2 = 0.0F;  // 本车道左侧道线方程系数c3
  lkas_input_.road_info.left_line_c3 = 0.0F;  // 本车道左侧道线方程系数c3
  lkas_input_.road_info.right_line_valid =
      0;  // 本车道右侧道线有效性 0:Invalid 1:Valid
  lkas_input_.road_info.right_line_type =
      0;  // 本车道右侧车道线类型 0：虚线  1：实线
  lkas_input_.road_info.right_line_c0 = 0.0F;  // 本车道右侧道线方程系数c0
  lkas_input_.road_info.right_line_c1 = 0.0F;  // 本车道右侧道线方程系数c1
  lkas_input_.road_info.right_line_c2 = 0.0F;  // 本车道右侧道线方程系数c2
  lkas_input_.road_info.right_line_c3 = 0.0F;  // 本车道右侧道线方程系数c3
  lkas_input_.road_info.left_roadedge_valid =
      0;  // 本车道左侧路缘有效性 0:Invalid 1:Valid
  lkas_input_.road_info.left_roadedge_c0 = 0.0F;  // 本车道左侧路缘方程系数c0
  lkas_input_.road_info.left_roadedge_c1 = 0.0F;  // 本车道左侧路缘方程系数c1
  lkas_input_.road_info.left_roadedge_c2 = 0.0F;  // 本车道左侧路缘方程系数c3
  lkas_input_.road_info.left_roadedge_c3 = 0.0F;  // 本车道左侧路缘方程系数c3
  lkas_input_.road_info.right_roadedge_valid =
      0;  // 本车道右侧路缘有效性 0:Invalid 1:Valid
  lkas_input_.road_info.right_roadedge_c0 = 0.0F;  // 本车道右侧路缘方程系数c0
  lkas_input_.road_info.right_roadedge_c1 = 0.0F;  // 本车道右侧路缘方程系数c1
  lkas_input_.road_info.right_roadedge_c2 = 0.0F;  // 本车道右侧路缘方程系数c2
  lkas_input_.road_info.right_roadedge_c3 = 0.0F;  // 本车道右侧路缘方程系数c3
  lkas_input_.road_info.lane_width_valid =
      0;  // 当前车道宽度信息有效性 0:Invalid 1:Valid
  lkas_input_.road_info.lane_width = 0.0F;  // 当前车道宽度,单位:m

  // 车道线信息初始化
  // 当前车道地址
  // auto ptr_environmental_model = session_->mutable_environmental_model();
  auto ptr_current_lane = session_->mutable_environmental_model()
                              ->get_virtual_lane_manager()
                              ->get_current_lane();
  // 左侧边界结构体
  auto ptr_current_lane_left_boundary =
      ptr_current_lane->get_left_lane_boundary();
  // 判断边界是路沿还是道线
  uint8 left_boundary_existence = ptr_current_lane_left_boundary.existence();
  /*
  LineType_LINE_TYPE_UNKNOWN = 0,
  LineType_LINE_TYPE_LANELINE = 1,//车道线
  LineType_LINE_TYPE_BORDERLINE = 2, 路沿，边沿线，物理隔离障碍物统称
  LineType_LINE_TYPE_CENTER = 3
   */
  uint8 left_boundary_type = ptr_current_lane_left_boundary.type();

  left_boundary_type = 1;

  if ((left_boundary_type == 1) && left_boundary_existence == 1) {
    lkas_input_.road_info.left_line_valid = 1;
  } else {
    lkas_input_.road_info.left_line_valid = 0;
  }

  if (left_boundary_type == 2 && left_boundary_existence == 1) {
    lkas_input_.road_info.left_roadedge_valid = 1;
  } else {
    lkas_input_.road_info.left_roadedge_valid = 0;
  }

  if (lkas_input_.road_info.left_line_valid == 1)  // 判断为道线
  {
    // LaneBoundaryType_MARKING_UNKNOWN = 0, /* 未知线型 */
    // LaneBoundaryType_MARKING_DASHED = 1, /* 虚线 */
    // LaneBoundaryType_MARKING_SOLID = 2, /* 实线 */
    // LaneBoundaryType_MARKING_SHORT_DASHED = 3, /* 短虚线 */
    // LaneBoundaryType_MARKING_DOUBLE_DASHED = 4, /* 双虚线 */
    // LaneBoundaryType_MARKING_DOUBLE_SOLID = 5, /* 双实线 */
    // LaneBoundaryType_MARKING_LEFT_DASHED_RIGHT_SOLID = 6, /* 左虚右实线 */
    // LaneBoundaryType_MARKING_LEFT_SOLID_RIGHT_DASHED = 7 /* 左实右虚线 */
    auto left_line_type = ptr_current_lane_left_boundary.segment(0).type();
    if (left_line_type == 1 || left_line_type == 3 || left_line_type == 4 ||
        left_line_type == 7) {
      lkas_input_.road_info.left_line_type = 0;  // 可跨越道线
    } else {
      lkas_input_.road_info.left_line_type = 1;  // 不可跨越道线
    }

    lkas_input_.road_info.left_line_c0 =
        ptr_current_lane_left_boundary.poly_coefficient(0);
    lkas_input_.road_info.left_line_c1 =
        ptr_current_lane_left_boundary.poly_coefficient(1);
    lkas_input_.road_info.left_line_c2 =
        ptr_current_lane_left_boundary.poly_coefficient(2);
    lkas_input_.road_info.left_line_c3 =
        ptr_current_lane_left_boundary.poly_coefficient(3);
  } else if (lkas_input_.road_info.left_roadedge_valid == 1)  // 判断为路沿
  {
    lkas_input_.road_info.left_roadedge_c0 =
        ptr_current_lane_left_boundary.poly_coefficient(0);
    lkas_input_.road_info.left_roadedge_c1 =
        ptr_current_lane_left_boundary.poly_coefficient(1);
    lkas_input_.road_info.left_roadedge_c2 =
        ptr_current_lane_left_boundary.poly_coefficient(2);
    lkas_input_.road_info.left_roadedge_c3 =
        ptr_current_lane_left_boundary.poly_coefficient(3);
  } else {
    /*do nothing*/
  }
  // 右侧边界
  auto ptr_current_lane_right_boundary =
      ptr_current_lane->get_right_lane_boundary();
  uint8 right_boundary_existence = ptr_current_lane_right_boundary.existence();
  /*
  LineType_LINE_TYPE_UNKNOWN = 0,
  LineType_LINE_TYPE_LANELINE = 1,//车道线
  LineType_LINE_TYPE_BORDERLINE = 2, 路沿，边沿线，物理隔离障碍物统称
  LineType_LINE_TYPE_CENTER = 3
   */
  uint8 right_boundary_type = ptr_current_lane_right_boundary.type();
  right_boundary_type = 1;
  if ((right_boundary_type == 1) && right_boundary_existence == 1) {
    lkas_input_.road_info.right_line_valid = 1;
  } else {
    lkas_input_.road_info.right_line_valid = 0;
  }

  if (right_boundary_type == 2 && right_boundary_existence == 1) {
    lkas_input_.road_info.right_roadedge_valid = 1;
  } else {
    lkas_input_.road_info.right_roadedge_valid = 0;
  }

  if (lkas_input_.road_info.right_line_valid == 1) {
    // LaneBoundaryType_MARKING_UNKNOWN = 0, /* 未知线型 */
    // LaneBoundaryType_MARKING_DASHED = 1, /* 虚线 */
    // LaneBoundaryType_MARKING_SOLID = 2, /* 实线 */
    // LaneBoundaryType_MARKING_SHORT_DASHED = 3, /* 短虚线 */
    // LaneBoundaryType_MARKING_DOUBLE_DASHED = 4, /* 双虚线 */
    // LaneBoundaryType_MARKING_DOUBLE_SOLID = 5, /* 双实线 */
    // LaneBoundaryType_MARKING_LEFT_DASHED_RIGHT_SOLID = 6, /* 左虚右实线 */
    // LaneBoundaryType_MARKING_LEFT_SOLID_RIGHT_DASHED = 7 /* 左实右虚线 */
    auto right_line_type = ptr_current_lane_right_boundary.segment(0).type();
    if ((right_line_type == 1) || (right_line_type == 3) ||
        (right_line_type == 4) || (right_line_type == 6)) {
      lkas_input_.road_info.right_line_type = 0;
    } else {
      lkas_input_.road_info.right_line_type = 1;
    }
    lkas_input_.road_info.right_line_c0 =
        ptr_current_lane_right_boundary.poly_coefficient(0);
    lkas_input_.road_info.right_line_c1 =
        ptr_current_lane_right_boundary.poly_coefficient(1);
    lkas_input_.road_info.right_line_c2 =
        ptr_current_lane_right_boundary.poly_coefficient(2);
    lkas_input_.road_info.right_line_c3 =
        ptr_current_lane_right_boundary.poly_coefficient(3);
  } else if (lkas_input_.road_info.right_roadedge_valid == 1) {
    lkas_input_.road_info.right_roadedge_c0 =
        ptr_current_lane_right_boundary.poly_coefficient(0);
    lkas_input_.road_info.right_roadedge_c1 =
        ptr_current_lane_right_boundary.poly_coefficient(1);
    lkas_input_.road_info.right_roadedge_c2 =
        ptr_current_lane_right_boundary.poly_coefficient(2);
    lkas_input_.road_info.right_roadedge_c3 =
        ptr_current_lane_right_boundary.poly_coefficient(3);
  } else {
    /*do nothing*/
  }
  // 道路宽度
  lkas_input_.road_info.lane_width = ptr_current_lane->width();

  // 车辆信息初始化、参数初始化
  auto ptr_ego_state_manager =
      session_->mutable_environmental_model()->get_ego_state_manager();
  lkas_input_.vehicle_info.veh_display_speed = ptr_ego_state_manager->ego_v();
  lkas_input_.vehicle_info.veh_yaw_rate = ptr_ego_state_manager->ego_yaw_rate();
  lkas_input_.vehicle_info.driver_hand_torque =
      ptr_ego_state_manager->driver_hand_torque();
  uint8 turn_light_flag = ptr_ego_state_manager->ego_blinker();

  // TURN_SIGNAL_TYPE_NONE = 0,              // 无请求
  // TURN_SIGNAL_TYPE_LEFT = 1,              // 请求左转向
  // TURN_SIGNAL_TYPE_RIGHT = 2,             // 请求右转向
  // TURN_SIGNAL_TYPE_EMERGENCY_FLASH = 3,   // 双闪
  // 左转向灯开启状态
  if (turn_light_flag == 1) {
    lkas_input_.vehicle_info.left_turn_light_state = 1;
  } else {
    lkas_input_.vehicle_info.left_turn_light_state = 0;
  }
  // 右转向灯开启状态
  if (turn_light_flag == 2) {
    lkas_input_.vehicle_info.right_turn_light_state = 1;
  } else {
    lkas_input_.vehicle_info.right_turn_light_state = 0;
  }
  // ldw switch
  lkas_input_.vehicle_info.ldw_main_switch =
      TRUE;  // session_->mutable_environmental_model()
             //->get_hmi_info()
             //.ldw_main_switch();
  // ldp switch
  lkas_input_.vehicle_info.ldp_main_switch =
      TRUE;  // session_->mutable_environmental_model()
             //  ->get_hmi_info()
             //  .ldp_main_switch();
  // elk switch
  lkas_input_.vehicle_info.elk_main_switch =
      TRUE;  // session_->mutable_environmental_model()
             //->get_hmi_info()
  // .elk_main_switch();
  // ldw sensitvity set
  lkas_input_.vehicle_info.ldw_tlc_level =
      session_->mutable_environmental_model()
          ->get_hmi_info()
          .ldw_set_sensitivity_level();

  lkas_input_.vehicle_info.common_front_over =
      session_->mutable_environmental_model()
          ->vehicle_param()
          .front_edge_to_center -
      session_->mutable_environmental_model()->vehicle_param().wheel_base;
  lkas_input_.vehicle_info.common_rear_over =
      session_->mutable_environmental_model()
          ->vehicle_param()
          .back_edge_to_center;
  lkas_input_.vehicle_info.common_wheel_base =
      session_->mutable_environmental_model()->vehicle_param().wheel_base;
  lkas_input_.vehicle_info.common_veh_width =
      session_->mutable_environmental_model()->vehicle_param().width;
  // update
  CalculateWheelToLine();
  JSON_DEBUG_VALUE("lkas_function::left_line_c0",
                   lkas_input_.road_info.left_line_c0);
  JSON_DEBUG_VALUE("lkas_function::left_line_c1",
                   lkas_input_.road_info.left_line_c1);
  JSON_DEBUG_VALUE("lkas_function::left_line_c2",
                   lkas_input_.road_info.left_line_c2);
  JSON_DEBUG_VALUE("lkas_function::left_line_c3",
                   lkas_input_.road_info.left_line_c3);
  JSON_DEBUG_VALUE("lkas_function::right_line_c0",
                   lkas_input_.road_info.right_line_c0);
  JSON_DEBUG_VALUE("lkas_function::right_line_c1",
                   lkas_input_.road_info.right_line_c1);
  JSON_DEBUG_VALUE("lkas_function::right_line_c2",
                   lkas_input_.road_info.right_line_c2);
  JSON_DEBUG_VALUE("lkas_function::right_line_c3",
                   lkas_input_.road_info.right_line_c3);
}
void LaneKeepAssistManager::RunOnce() {
  static uint16 ldw_l_warn_count = 0;
  static uint16 ldw_r_warn_count = 0;
  Update();
  lane_depart_warning_.RunOnce();
  lane_depart_prevention_.RunOnce();
  emergency_lane_keep_.RunOnce();
  set_lka_output_info();

  if (ldw_state_ == 4) {
    ldw_l_warn_count++;
  }
  if (ldw_state_ == 5) {
    ldw_l_warn_count++;
  }
  if (ldw_l_warn_count > 10000) {
    ldw_l_warn_count = 10000;
  }
  if (ldw_r_warn_count > 10000) {
    ldw_r_warn_count = 10000;
  }
  LOG_DEBUG("LaneKeepAssistManager::RunOnce \n");
  LOG_DEBUG("LaneKeepAssistManager::ldw_state = %d \n", ldw_state_);
  LOG_DEBUG("LaneKeepAssistManager::ldp_state = %d \n", ldp_state_);
  LOG_DEBUG("LaneKeepAssistManager::elk_state = %d \n", elk_state_);
  LOG_DEBUG("LaneKeepAssistManager::ldw_count_l = %d \n", ldw_l_warn_count);
  LOG_DEBUG("LaneKeepAssistManager::ldw_count_r = %d \n", ldw_r_warn_count);
}
void LaneKeepAssistManager::CalculateWheelToLine() {
  /*fl_wheel_distance_to_line*/
  lkas_input_.wheel_to_line.fl_wheel_distance_to_line =
      lkas_input_.road_info.left_line_c0 +
      lkas_input_.road_info.left_line_c1 *
          lkas_input_.vehicle_info.common_wheel_base +
      lkas_input_.road_info.left_line_c2 *
          lkas_input_.vehicle_info.common_wheel_base *
          lkas_input_.vehicle_info.common_wheel_base +
      lkas_input_.road_info.left_line_c3 *
          lkas_input_.vehicle_info.common_wheel_base *
          lkas_input_.vehicle_info.common_wheel_base *
          lkas_input_.vehicle_info.common_wheel_base -
      Common_FrontCamera_PosYL;
  /*fl_wheel_distance_to_roadedge*/
  lkas_input_.wheel_to_line.fl_wheel_distance_to_roadedge =
      lkas_input_.road_info.left_roadedge_c0 +
      lkas_input_.road_info.left_roadedge_c1 *
          lkas_input_.vehicle_info.common_wheel_base +
      lkas_input_.road_info.left_roadedge_c2 *
          lkas_input_.vehicle_info.common_wheel_base *
          lkas_input_.vehicle_info.common_wheel_base +
      lkas_input_.road_info.left_roadedge_c3 *
          lkas_input_.vehicle_info.common_wheel_base *
          lkas_input_.vehicle_info.common_wheel_base *
          lkas_input_.vehicle_info.common_wheel_base -
      Common_FrontCamera_PosYL;
  /*fr_wheel_distance_to_line*/
  lkas_input_.wheel_to_line.fr_wheel_distance_to_line =
      lkas_input_.road_info.right_line_c0 +
      lkas_input_.road_info.right_line_c1 *
          lkas_input_.vehicle_info.common_wheel_base +
      lkas_input_.road_info.right_line_c2 *
          lkas_input_.vehicle_info.common_wheel_base *
          lkas_input_.vehicle_info.common_wheel_base +
      lkas_input_.road_info.right_line_c3 *
          lkas_input_.vehicle_info.common_wheel_base *
          lkas_input_.vehicle_info.common_wheel_base *
          lkas_input_.vehicle_info.common_wheel_base +
      Common_FrontCamera_PosYR;
  /*fr_wheel_distance_to_roadedge*/
  lkas_input_.wheel_to_line.fr_wheel_distance_to_roadedge =
      lkas_input_.road_info.right_roadedge_c0 +
      lkas_input_.road_info.right_roadedge_c1 *
          lkas_input_.vehicle_info.common_wheel_base +
      lkas_input_.road_info.right_roadedge_c2 *
          lkas_input_.vehicle_info.common_wheel_base *
          lkas_input_.vehicle_info.common_wheel_base +
      lkas_input_.road_info.right_roadedge_c3 *
          lkas_input_.vehicle_info.common_wheel_base *
          lkas_input_.vehicle_info.common_wheel_base *
          lkas_input_.vehicle_info.common_wheel_base +
      Common_FrontCamera_PosYR;
}
void LaneKeepAssistManager::Output() {
  auto lka_output_str =
      session_->mutable_planning_output_context()->mutable_planning_hmi_info();
  /*
  0:Unavailable
  1:Off
  2:Standby
  3:Active(No Intervention)
  4:Active(Left Intervention)
  5:Active(Right Intervention)
  */
  /*update ldw result*/
  lka_output_str->mutable_ldw_output_info()->set_ldw_state(
      lane_depart_warning_.get_ldw_state());
  if (lane_depart_warning_.get_ldw_state() ==
      PlanningHMI::
          LDWOutputInfoStr_LDWFunctionFSMWorkState_LDW_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION) {
    lka_output_str->mutable_ldw_output_info()->set_ldw_left_warning(TRUE);
  } else {
    lka_output_str->mutable_ldw_output_info()->set_ldw_left_warning(FALSE);
  }
  if (lane_depart_warning_.get_ldw_state() ==
      PlanningHMI::
          LDWOutputInfoStr_LDWFunctionFSMWorkState_LDW_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION) {
    lka_output_str->mutable_ldw_output_info()->set_ldw_right_warning(TRUE);
  } else {
    lka_output_str->mutable_ldw_output_info()->set_ldw_right_warning(FALSE);
  }
  /*update ldp result*/  /////////////////////////
  lka_output_str->mutable_ldp_output_info()->set_ldp_state(
      lane_depart_prevention_.get_ldp_state());

  if (lane_depart_prevention_.get_ldp_state() ==
      PlanningHMI::
          LDPOutputInfoStr_LDPFunctionFSMWorkState_LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION) {
    lka_output_str->mutable_ldp_output_info()->set_ldp_left_intervention_flag(
        TRUE);
  } else {
    lka_output_str->mutable_ldp_output_info()->set_ldp_left_intervention_flag(
        FALSE);
  }
  if (lane_depart_prevention_.get_ldp_state() ==
      PlanningHMI::
          LDPOutputInfoStr_LDPFunctionFSMWorkState_LDP_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION) {
    lka_output_str->mutable_ldp_output_info()->set_ldp_right_intervention_flag(
        TRUE);
  } else {
    lka_output_str->mutable_ldp_output_info()->set_ldp_right_intervention_flag(
        FALSE);
  }
  /*update elk result*/  ////////////////////////
  lka_output_str->mutable_elk_output_info()->set_elk_state(
      emergency_lane_keep_.get_elk_state());
  if (emergency_lane_keep_.get_elk_state() ==
      PlanningHMI::
          ELKOutputInfoStr_ELKFunctionFSMWorkState_ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_LEFT_INTERVENTION) {
    lka_output_str->mutable_elk_output_info()->set_elk_left_intervention_flag(
        TRUE);
  } else {
    lka_output_str->mutable_elk_output_info()->set_elk_left_intervention_flag(
        FALSE);
  }
  if (emergency_lane_keep_.get_elk_state() ==
      PlanningHMI::
          ELKOutputInfoStr_ELKFunctionFSMWorkState_ELK_FUNCTION_FSM_WORK_STATE_ACTIVE_RIGHT_INTERVENTION) {
    lka_output_str->mutable_elk_output_info()->set_elk_right_intervention_flag(
        TRUE);
  } else {
    lka_output_str->mutable_elk_output_info()->set_elk_right_intervention_flag(
        FALSE);
  }
}

void LaneKeepAssistManager::set_lka_output_info() {
  /*for ldw output info*/
  ldw_state_ = lane_depart_warning_.get_ldw_state();
  ldw_left_warning_ = lane_depart_warning_.get_ldw_left_warning_info();
  ldw_right_warning_ = lane_depart_warning_.get_ldw_right_warning_info();
  /*for ldp output info*/
  ldp_state_ = lane_depart_prevention_.get_ldp_state();
  ldp_left_intervention_flag_ =
      lane_depart_prevention_.get_ldp_left_intervention_flag_info();
  ldp_right_intervention_flag_ =
      lane_depart_prevention_.get_ldp_right_intervention_flag_info();
  /*for elk output info*/
  elk_state_ = emergency_lane_keep_.get_elk_state();
  elk_left_intervention_flag_ =
      emergency_lane_keep_.get_elk_left_intervention_flag_info();
  elk_right_intervention_flag_ =
      emergency_lane_keep_.get_elk_right_intervention_flag_info();
}

}  // namespace planning