#include "emergency_lane_keep_alert_context.h"

#include "environmental_model.h"
namespace planning {
void EmergencyLaneKeepAlert::Init(planning::LkasInput *lkas_input,
                                  framework::Session *session) {
  lkas_input_ = lkas_input;
  session_ = session;
  // for aera&velocit init
  f_bsd_aera_vel_.area_length_distance = 0.0F;
  f_bsd_aera_vel_.b_x = 0.0F;
  f_bsd_aera_vel_.c_x = 0.0F;
  f_bsd_aera_vel_.f_y = 0.0F;
  f_bsd_aera_vel_.g_y = 0.0F;
  f_bsd_aera_vel_.l_y = 0.0F;
  f_bsd_aera_vel_.k_y = 0.0F;
  f_bsd_aera_vel_.obstacle_velocity_limit = 0.0F;
  r_bsd_aera_vel_.area_length_distance = 0.0F;
  r_bsd_aera_vel_.b_x = 0.0F;
  r_bsd_aera_vel_.c_x = 0.0F;
  r_bsd_aera_vel_.f_y = 0.0F;
  r_bsd_aera_vel_.g_y = 0.0F;
  r_bsd_aera_vel_.l_y = 0.0F;
  r_bsd_aera_vel_.k_y = 0.0F;
  r_bsd_aera_vel_.obstacle_velocity_limit = 0.0F;
  f_lca_aera_vel_.area_length_distance = 0.0F;
  f_lca_aera_vel_.b_x = 0.0F;
  f_lca_aera_vel_.c_x = 0.0F;
  f_lca_aera_vel_.f_y = 0.0F;
  f_lca_aera_vel_.g_y = 0.0F;
  f_lca_aera_vel_.l_y = 0.0F;
  f_lca_aera_vel_.k_y = 0.0F;
  f_lca_aera_vel_.obstacle_velocity_limit = 0.0F;
  r_lca_aera_vel_.area_length_distance = 0.0F;
  r_lca_aera_vel_.b_x = 0.0F;
  r_lca_aera_vel_.c_x = 0.0F;
  r_lca_aera_vel_.f_y = 0.0F;
  r_lca_aera_vel_.g_y = 0.0F;
  r_lca_aera_vel_.l_y = 0.0F;
  r_lca_aera_vel_.k_y = 0.0F;
  r_lca_aera_vel_.obstacle_velocity_limit = 0.0F;

  // LOG_DEBUG("LaneKeepAssistManager::elk:: bsd_lca has been inited \n");
}
void EmergencyLaneKeepAlert::Update() {
  // 区域划分
  if (lkas_input_->vehicle_info.veh_display_speed <= (60.0F / 3.6F)) {
    f_bsd_aera_vel_.area_length_distance = 4.0F;
    r_bsd_aera_vel_.area_length_distance = 4.0F;
    f_lca_aera_vel_.area_length_distance = 4.0F;
    r_lca_aera_vel_.area_length_distance = 4.0F;
  } else if ((lkas_input_->vehicle_info.veh_display_speed > (60.0F / 3.6F)) &&
             (lkas_input_->vehicle_info.veh_display_speed < (115.0F / 3.6F))) {
    f_bsd_aera_vel_.area_length_distance =
        0.2F * (lkas_input_->vehicle_info.veh_display_speed - 60.0F / 3.6F) +
        4.0F;
    r_bsd_aera_vel_.area_length_distance =
        0.2F * (lkas_input_->vehicle_info.veh_display_speed - 60.0F / 3.6F) +
        4.0F;
    f_lca_aera_vel_.area_length_distance =
        0.2F * (lkas_input_->vehicle_info.veh_display_speed - 60.0F / 3.6F) +
        4.0F;
    r_lca_aera_vel_.area_length_distance =
        0.2F * (lkas_input_->vehicle_info.veh_display_speed - 60.0F / 3.6F) +
        4.0F;
  } else {
    f_bsd_aera_vel_.area_length_distance = 15.0F;
    r_bsd_aera_vel_.area_length_distance = 15.0F;
    f_lca_aera_vel_.area_length_distance = 15.0F;
    r_lca_aera_vel_.area_length_distance = 15.0F;
  }

  f_bsd_aera_vel_.b_x = 0.0F;  // 1
  f_bsd_aera_vel_.c_x = f_bsd_aera_vel_.area_length_distance +
                        lkas_input_->vehicle_info.common_wheel_base +
                        lkas_input_->vehicle_info.common_front_over;  // 2
  r_bsd_aera_vel_.b_x = -f_bsd_aera_vel_.area_length_distance -
                        lkas_input_->vehicle_info.common_rear_over;   // 3
  r_bsd_aera_vel_.c_x = lkas_input_->vehicle_info.common_wheel_base;  // 4

  f_lca_aera_vel_.b_x = 70.0F;  // 5
  f_lca_aera_vel_.c_x = f_bsd_aera_vel_.area_length_distance +
                        lkas_input_->vehicle_info.common_wheel_base +
                        lkas_input_->vehicle_info.common_front_over;  // 6
  r_lca_aera_vel_.b_x = -f_bsd_aera_vel_.area_length_distance -
                        lkas_input_->vehicle_info.common_rear_over;  // 7
  r_lca_aera_vel_.c_x = -70.0F;                                      // 8

  f_bsd_aera_vel_.f_y =
      lkas_input_->vehicle_info.common_veh_width / 2.0F + width_distance_min;
  f_bsd_aera_vel_.g_y =
      lkas_input_->vehicle_info.common_veh_width / 2.0F + width_distance_max;
  f_bsd_aera_vel_.l_y =
      -lkas_input_->vehicle_info.common_veh_width / 2.0F - width_distance_max;
  f_bsd_aera_vel_.k_y =
      -lkas_input_->vehicle_info.common_veh_width / 2.0F - width_distance_min;
  r_bsd_aera_vel_.f_y =
      lkas_input_->vehicle_info.common_veh_width / 2.0F + width_distance_min;
  r_bsd_aera_vel_.g_y =
      lkas_input_->vehicle_info.common_veh_width / 2.0F + width_distance_max;
  r_bsd_aera_vel_.l_y =
      -lkas_input_->vehicle_info.common_veh_width / 2.0F - width_distance_max;
  r_bsd_aera_vel_.k_y =
      -lkas_input_->vehicle_info.common_veh_width / 2.0F - width_distance_min;

  f_lca_aera_vel_.f_y =
      lkas_input_->vehicle_info.common_veh_width / 2.0F + width_distance_min;
  f_lca_aera_vel_.g_y =
      lkas_input_->vehicle_info.common_veh_width / 2.0F + width_distance_max;
  f_lca_aera_vel_.l_y =
      -lkas_input_->vehicle_info.common_veh_width / 2.0F - width_distance_max;
  f_lca_aera_vel_.k_y =
      -lkas_input_->vehicle_info.common_veh_width / 2.0F - width_distance_min;
  r_lca_aera_vel_.f_y =
      lkas_input_->vehicle_info.common_veh_width / 2.0F + width_distance_min;
  r_lca_aera_vel_.g_y =
      lkas_input_->vehicle_info.common_veh_width / 2.0F + width_distance_max;
  r_lca_aera_vel_.l_y =
      -lkas_input_->vehicle_info.common_veh_width / 2.0F - width_distance_max;
  r_lca_aera_vel_.k_y =
      -lkas_input_->vehicle_info.common_veh_width / 2.0F - width_distance_min;
  // 目标物速度限制
  if (lkas_input_->vehicle_info.veh_display_speed < (10.0F / 3.6F)) {
    f_bsd_aera_vel_.obstacle_velocity_limit =
        10.0F / 3.6F;  // 单位需要统一转换成m/s
    r_bsd_aera_vel_.obstacle_velocity_limit = -10.0F / 3.6F;
  } else if ((lkas_input_->vehicle_info.veh_display_speed <= (30.0F / 3.6F)) &&
             (lkas_input_->vehicle_info.veh_display_speed >=
              (10.0F / 3.6F)))  // 根据本车车速设置速度阈值
  {
    f_bsd_aera_vel_.obstacle_velocity_limit =
        -((-15.0F / 3.6F + 10.0F / 3.6F) / (30.0F / 3.6F - 10.0F / 3.6F) *
              (lkas_input_->vehicle_info.veh_display_speed - 30.0F / 3.6F) -
          15.0F / 3.6F);
    r_bsd_aera_vel_.obstacle_velocity_limit =
        ((-15.0F / 3.6F + 10.0F / 3.6F) / (30.0F / 3.6F - 10.0F / 3.6F) *
             (lkas_input_->vehicle_info.veh_display_speed - 30.0F / 3.6F) -
         15.0F / 3.6F);
  } else {
    f_bsd_aera_vel_.obstacle_velocity_limit = (15.0F / 3.6F);
    r_bsd_aera_vel_.obstacle_velocity_limit = -(15.0F / 3.6F);
  }
  f_lca_aera_vel_.obstacle_velocity_limit =
      -30.0F / 3.6F;  // 单位需要统一转换成m/s
  r_lca_aera_vel_.obstacle_velocity_limit = 30.0F / 3.6F;
}
uint8 EmergencyLaneKeepAlert::RunOnce() {
  Update();
  uint16 elk_bsd_lca_code_temp = 0;
  static uint16 left_turn_light_off_count =
      0;  // 左转向灯处于关闭状态的时长，单位:ms
  if (lkas_input_->vehicle_info.left_turn_light_state == TRUE) {
    left_turn_light_off_count = 0;  // 转向灯开启,转向灯关闭状态计时器清零
  } else {
    left_turn_light_off_count += Common_Cycle_Time;
    if (left_turn_light_off_count >= 60000) {
      left_turn_light_off_count = 60000;
    } else {
      /*do nothing*/
    }
  }

  static uint16 right_turn_light_off_count =
      0;  // 左转向灯处于关闭状态的时长，单位:ms
  if (lkas_input_->vehicle_info.right_turn_light_state == TRUE) {
    right_turn_light_off_count = 0;  // 转向灯开启,转向灯关闭状态计时器清零
  } else {
    right_turn_light_off_count += Common_Cycle_Time;
    if (right_turn_light_off_count >= 60000) {
      right_turn_light_off_count = 60000;
    } else {
      /*do nothing*/
    }
  }

  if (left_turn_light_off_count < 60000) {
    // 探测到有左侧主动换向意图
    if (LeftAlertJudge()) {
      // 判断是否有危险出现
      elk_bsd_lca_code_temp += uint16_bit[0];
    } else {
      /*do nothing*/
    }
  } else {
    /*do nothing*/
  }

  if (right_turn_light_off_count < 60000) {
    // 探测到有右侧主动换向意图
    if (RightAlertJudge()) {
      // 判断是否有危险出现
      elk_bsd_lca_code_temp += uint16_bit[1];
    } else {
      /*do nothing*/
    }
  } else {
    /*do nothing*/
  }

  return elk_bsd_lca_code_temp;
}
// single side judge
boolean EmergencyLaneKeepAlert::LeftAlertJudge() {
  boolean FLAlertState = FALSE;
  FLAlertState = LeftAlertJudgeFL();
  boolean RLAlertState = FALSE;
  RLAlertState = LeftAlertJudgeRL();
  return (FLAlertState || RLAlertState);
}
boolean EmergencyLaneKeepAlert::RightAlertJudge() {
  boolean FRAlertState = FALSE;
  FRAlertState = RightAlertJudgeFR();
  boolean RRAlertState = FALSE;
  RRAlertState = RightAlertJudgeRR();
  return (FRAlertState || RRAlertState);
}
// sigle radar judge
boolean EmergencyLaneKeepAlert::LeftAlertJudgeFL() {
  // 雷达信息
  RadarObjData radar_obj_data = {0, 0, 0.0F, 0.0F, 0.0F, 0.0F};
  // 状态标志位
  uint8 select_state = 0;
  boolean condition_bsd_state = FALSE;
  boolean condition_lca_state = FALSE;

  auto &radar_vector_fl =
      session_->mutable_environmental_model()->get_prediction_info();
  // auto iter = radar_map.find('3');
  if (false) {
    // auto &radar_vector_fl = iter->second;
    uint8 objs_nmu = radar_vector_fl.size();

    if (objs_nmu > max_objs_num) {  // 限制校验目标物数量
      objs_nmu = max_objs_num;
    }

    for (uint8 i = 0; i < objs_nmu; i++) {  // 遍历目标物
      // 读取目标物信息
      // objs_nmu = 0;
      radar_obj_data.pos = 0;  // 0代表左，1代表右
      radar_obj_data.obj_class = radar_vector_fl[i].type;
      radar_obj_data.obj_x = radar_vector_fl[i].relative_position_x;
      radar_obj_data.obj_y = radar_vector_fl[i].relative_position_y;
      radar_obj_data.obj_vx = radar_vector_fl[i].relative_speed_x;
      radar_obj_data.obj_vx = radar_vector_fl[i].relative_speed_y;
      // 筛选
      select_state = ObjSelect(&radar_obj_data);
      // 条件判断
      condition_bsd_state = ObjBsdFL(&radar_obj_data);
      condition_lca_state = ObjLcaFL(&radar_obj_data);
      // 风险判断
      if ((select_state == 0) &&
          ((condition_bsd_state == TRUE) || (condition_lca_state == TRUE))) {
        return TRUE;
      }
    }
    return FALSE;
  } else {
    return FALSE;
  }
}
boolean EmergencyLaneKeepAlert::LeftAlertJudgeRL() {
  // 雷达信息
  RadarObjData radar_obj_data = {0, 0, 0.0F, 0.0F, 0.0F, 0.0F};
  // 状态标志位
  uint8 select_state = 0;
  boolean condition_bsd_state = FALSE;
  boolean condition_lca_state = FALSE;

  auto &radar_vector_rl =
      session_->mutable_environmental_model()->get_prediction_info();
  if (false) {
    // auto &radar_vector_rl = iter->second;
    uint8 objs_nmu = radar_vector_rl.size();

    // uint8 alert_state = 0;
    if (objs_nmu > max_objs_num) {  // 限制校验目标物数量
      objs_nmu = max_objs_num;
    }

    for (uint8 i = 0; i < objs_nmu; i++) {  // 遍历目标物
      // 读取目标物信息
      // objs_nmu = 0;
      radar_obj_data.pos = 0;
      radar_obj_data.obj_class = radar_vector_rl[i].type;
      radar_obj_data.obj_x = radar_vector_rl[i].relative_position_x;
      radar_obj_data.obj_y = radar_vector_rl[i].relative_position_y;
      radar_obj_data.obj_vx = radar_vector_rl[i].relative_speed_x;
      radar_obj_data.obj_vx = radar_vector_rl[i].relative_speed_y;
      // 筛选
      select_state = ObjSelect(&radar_obj_data);
      // 条件判断
      condition_bsd_state = ObjBsdRL(&radar_obj_data);
      condition_lca_state = ObjLcaRL(&radar_obj_data);
      // 风险判断
      if ((select_state == 0) &&
          ((condition_bsd_state == TRUE) || (condition_lca_state == TRUE))) {
        return TRUE;
      }
    }
    return FALSE;
  } else {
    return FALSE;
  }
}
boolean EmergencyLaneKeepAlert::RightAlertJudgeFR() {
  // 雷达信息
  RadarObjData radar_obj_data = {0, 0, 0.0F, 0.0F, 0.0F, 0.0F};
  // 状态标志位
  uint8 select_state = 0;
  boolean condition_bsd_state = FALSE;
  boolean condition_lca_state = FALSE;

  auto &radar_vector_fr =
      session_->mutable_environmental_model()->get_prediction_info();
  // auto iter = radar_map.find(5);
  if (false) {
    // auto &radar_vector_fr = iter->second;
    uint8 objs_nmu = radar_vector_fr.size();
    // uint8 alert_state = 0;
    if (objs_nmu > max_objs_num) {  // 限制校验目标物数量
      objs_nmu = max_objs_num;
    }

    for (uint8 i = 0; i < objs_nmu; i++) {  // 遍历目标物
      // 读取目标物信息
      radar_obj_data.pos = 1;
      radar_obj_data.obj_class = radar_vector_fr[i].type;
      radar_obj_data.obj_x = radar_vector_fr[i].relative_position_x;
      radar_obj_data.obj_y = radar_vector_fr[i].relative_position_y;
      radar_obj_data.obj_vx = radar_vector_fr[i].relative_speed_x;
      radar_obj_data.obj_vx = radar_vector_fr[i].relative_speed_y;
      // 筛选
      select_state = ObjSelect(&radar_obj_data);
      // 条件判断
      condition_bsd_state = ObjBsdFR(&radar_obj_data);
      condition_lca_state = ObjLcaFR(&radar_obj_data);
      // 风险判断
      if ((select_state == 0) &&
          ((condition_bsd_state == TRUE) || (condition_lca_state == TRUE))) {
        return TRUE;
      }
    }
    return FALSE;
  } else {
    return FALSE;
  }
}
boolean EmergencyLaneKeepAlert::RightAlertJudgeRR() {
  // 雷达信息
  RadarObjData radar_obj_data = {0, 0, 0.0F, 0.0F, 0.0F, 0.0F};
  // 状态标志位
  uint8 select_state = 0;
  boolean condition_bsd_state = FALSE;
  boolean condition_lca_state = FALSE;

  auto &radar_vector_rr =
      session_->mutable_environmental_model()->get_prediction_info();
  //   auto iter = radar_map.find(6);
  if (false) {
    // auto &radar_vector_rr = iter->second;
    uint8 objs_nmu = radar_vector_rr.size();

    // uint8 alert_state = 0;
    if (objs_nmu > max_objs_num) {  // 限制校验目标物数量
      objs_nmu = max_objs_num;
    }

    for (uint8 i = 0; i < objs_nmu; i++) {  // 遍历目标物
      // 读取目标物信息
      radar_obj_data.pos = 1;
      radar_obj_data.obj_class = radar_vector_rr[i].type;
      radar_obj_data.obj_x = radar_vector_rr[i].relative_position_x;
      radar_obj_data.obj_y = radar_vector_rr[i].relative_position_y;
      radar_obj_data.obj_vx = radar_vector_rr[i].relative_speed_x;
      radar_obj_data.obj_vx = radar_vector_rr[i].relative_speed_y;
      // 筛选
      select_state = ObjSelect(&radar_obj_data);
      // 条件判断
      condition_bsd_state = ObjBsdRR(&radar_obj_data);
      condition_lca_state = ObjLcaRR(&radar_obj_data);
      // 风险判断
      if ((select_state == 0) &&
          ((condition_bsd_state == TRUE) || (condition_lca_state == TRUE))) {
        return TRUE;
      }
    }
    return FALSE;
  } else {
    return FALSE;
  }
}
// single function judge of each radar
boolean EmergencyLaneKeepAlert::ObjBsdFL(RadarObjData *radar) {
  uint8 bsd_condition_code = 0;
  bsd_condition_code = ObjBsdConditionF(radar);
  return (bsd_condition_code == 0);
}
boolean EmergencyLaneKeepAlert::ObjLcaFL(RadarObjData *radar) {
  uint8 lca_condition_code = 0;
  uint8 lca_alert_code = 0;
  lca_condition_code = ObjLcaConditionF(radar);
  lca_alert_code = ObjLcaAlertF(radar);
  return ((lca_condition_code == 0) && (lca_alert_code == 0));
}
boolean EmergencyLaneKeepAlert::ObjBsdFR(RadarObjData *radar) {
  uint8 bsd_condition_code = 0;
  bsd_condition_code = ObjBsdConditionF(radar);
  return (bsd_condition_code == 0);
}
boolean EmergencyLaneKeepAlert::ObjLcaFR(RadarObjData *radar) {
  uint8 lca_condition_code = 0;
  uint8 lca_alert_code = 0;
  lca_condition_code = ObjLcaConditionF(radar);
  lca_alert_code = ObjLcaAlertF(radar);
  return ((lca_condition_code == 0) && (lca_alert_code == 0));
}
boolean EmergencyLaneKeepAlert::ObjBsdRL(RadarObjData *radar) {
  uint8 bsd_condition_code = 0;
  bsd_condition_code = ObjBsdConditionR(radar);
  return (bsd_condition_code == 0);
}
boolean EmergencyLaneKeepAlert::ObjLcaRL(RadarObjData *radar) {
  uint8 lca_condition_code = 0;
  uint8 lca_alert_code = 0;
  lca_condition_code = ObjLcaConditionR(radar);
  lca_alert_code = ObjLcaAlertR(radar);
  return ((lca_condition_code == 0) && (lca_alert_code == 0));
}
boolean EmergencyLaneKeepAlert::ObjBsdRR(RadarObjData *radar) {
  uint8 bsd_condition_code = 0;
  bsd_condition_code = ObjBsdConditionR(radar);
  return (bsd_condition_code == 0);
}
boolean EmergencyLaneKeepAlert::ObjLcaRR(RadarObjData *radar) {
  uint8 lca_condition_code = 0;
  uint8 lca_alert_code = 0;
  lca_condition_code = ObjLcaConditionR(radar);
  lca_alert_code = ObjLcaAlertR(radar);
  return ((lca_condition_code) == 0 && (lca_alert_code == 0));
}
// single obj judge
uint8 EmergencyLaneKeepAlert::ObjSelect(RadarObjData *radar) {
  uint16 class_code = 0;
  if (radar->obj_class >= 11 || radar->obj_class <= 2) {
    class_code += uint16_bit[0];
  }
  return (class_code);
}
uint8 EmergencyLaneKeepAlert::ObjBsdConditionF(RadarObjData *radar) {
  uint8 condition_code = 0;
  // uint8 velocity_code = 0;
  // uint8 approach_code = 0; // 航向角判断
  // condition1
  if ((radar->obj_x < f_bsd_aera_vel_.b_x) ||
      (radar->obj_x > f_bsd_aera_vel_.c_x)) {  // 纵向距离不满足要求
    condition_code += uint16_bit[0];
  }
  if (radar->pos == 0) {
    if ((radar->obj_y < f_bsd_aera_vel_.f_y) ||
        (radar->obj_y > f_bsd_aera_vel_.g_y)) {  // 横向距离不满足要求
      condition_code += uint16_bit[1];
    }
  } else {
    if ((radar->obj_y < f_bsd_aera_vel_.l_y) ||
        (radar->obj_y > f_bsd_aera_vel_.k_y)) {  // 横向距离不满足要求
      condition_code += uint16_bit[1];
    }
  }
  // condition2
  if ((radar->obj_vx > f_bsd_aera_vel_.obstacle_velocity_limit) ||
      (fabs(radar->obj_vx + lkas_input_->vehicle_info.veh_display_speed) <
       (5.0F / 3.6F))) {
    condition_code += uint16_bit[2];
  }
  // condition3
  if (fabs(radar->obj_vy) < 0.5) {
  } else {
    if (fabs(radar->obj_vx / radar->obj_vy) < 1.0F) {
      condition_code = uint16_bit[3];
    }
  }
  return (condition_code);
}
uint8 EmergencyLaneKeepAlert::ObjBsdConditionR(RadarObjData *radar) {
  uint8 condition_code = 0;
  // uint8 velocity_code = 0;
  // uint8 approach_code = 0; // 航向角判断
  // condition1
  if ((radar->obj_x < r_bsd_aera_vel_.b_x) ||
      (radar->obj_x > r_bsd_aera_vel_.c_x)) {  // 纵向距离不满足要求
    condition_code += uint16_bit[0];
  }
  if (radar->pos == 0) {  // 左侧
    if ((radar->obj_y < f_bsd_aera_vel_.f_y) ||
        (radar->obj_y > f_bsd_aera_vel_.g_y)) {  // 横向距离不满足要求
      condition_code += uint16_bit[1];
    }
  } else {  // 右侧
    if ((radar->obj_y < r_bsd_aera_vel_.l_y) ||
        (radar->obj_y > r_bsd_aera_vel_.k_y)) {  // 横向距离不满足要求
      condition_code += uint16_bit[1];
    }
  }
  // condition2
  if ((radar->obj_vx < r_bsd_aera_vel_.obstacle_velocity_limit) ||
      (fabs(radar->obj_vx + lkas_input_->vehicle_info.veh_display_speed) <
       (5.0F / 3.6F))) {
    condition_code += uint16_bit[2];
  }
  // condition3
  if (fabs(radar->obj_vy) < 0.5) {
  } else {
    if (fabs(radar->obj_vx / radar->obj_vy) < 1.0F) {
      condition_code = uint16_bit[3];
    }
  }
  return (condition_code);
}
uint8 EmergencyLaneKeepAlert::ObjLcaConditionF(RadarObjData *radar) {
  uint8 condition_code = 0;
  // condition1
  if ((radar->obj_x < f_lca_aera_vel_.c_x) ||
      (radar->obj_x > f_lca_aera_vel_.b_x)) {  // 纵向距离不满足要求
    condition_code += uint16_bit[0];
  }
  if (radar->pos == 0) {
    if ((radar->obj_y < f_lca_aera_vel_.f_y) ||
        (radar->obj_y > f_lca_aera_vel_.g_y)) {  // 横向距离不满足要求
      condition_code += uint16_bit[1];
    }
  } else {
    if ((radar->obj_y < f_lca_aera_vel_.l_y) ||
        (radar->obj_y > f_lca_aera_vel_.k_y)) {  // 横向距离不满足要求
      condition_code += uint16_bit[1];
    }
  }
  // condition2
  if ((radar->obj_vx > f_lca_aera_vel_.obstacle_velocity_limit) ||
      (fabs(radar->obj_vx + lkas_input_->vehicle_info.veh_display_speed) <
       (5.0F / 3.6F))) {
    condition_code += uint16_bit[2];
  }
  // condition3
  if (fabs(radar->obj_vy) < 0.5F) {
  } else {
    if (fabs(radar->obj_vx / radar->obj_vy) < 1.0F) {
      condition_code = uint16_bit[3];
    }
  }
  return (condition_code);
}
uint8 EmergencyLaneKeepAlert::ObjLcaAlertF(RadarObjData *radar) {
  uint8 alert_code = 0;
  // uint8 crash_code = 0;
  float32 lca_ttc = 0.0F;
  float32 ttc = 0.0F;
  float32 obj_to_front_x = 0;
  float32 crash_y = 0.0F;
  obj_to_front_x = radar->obj_x - lkas_input_->vehicle_info.common_wheel_base -
                   lkas_input_->vehicle_info.common_front_over;

  if ((radar->obj_x > f_lca_aera_vel_.c_x) && (radar->obj_x <= 25)) {
    lca_ttc = 2.5;
  } else if ((radar->obj_x < f_lca_aera_vel_.b_x) && (radar->obj_x > 25)) {
    lca_ttc = 3.5;
  } else {
    lca_ttc = 0;
  }
  ttc = fabs(obj_to_front_x / radar->obj_vx);

  crash_y = radar->obj_y + ttc * radar->obj_vy;
  // condition1
  if (ttc > lca_ttc) {
    alert_code += uint16_bit[0];
  }
  if (radar->pos = 0) {  // 左侧
    if ((crash_y < -0.5 * lkas_input_->vehicle_info.common_veh_width) ||
        (crash_y > (f_lca_aera_vel_.g_y + 1.0F))) {
      alert_code += uint16_bit[1];
    }
  } else {  // 右侧
    if ((crash_y > 0.5 * lkas_input_->vehicle_info.common_veh_width) ||
        (crash_y < -(f_lca_aera_vel_.l_y + 1.0F))) {
      alert_code += uint16_bit[1];
    }
  }

  return (alert_code);
}
uint8 EmergencyLaneKeepAlert::ObjLcaConditionR(RadarObjData *radar) {
  uint8 condition_code = 0;
  // uint8 velocity_code = 0;
  // uint8 approach_code = 0; // 航向角判断
  // condition1
  if ((radar->obj_x < r_lca_aera_vel_.c_x) &&
      (radar->obj_x > r_lca_aera_vel_.b_x)) {  // 纵向距离不满足要求
    condition_code += uint16_bit[0];
  }
  if (radar->pos == 0) {  // 左侧
    if ((radar->obj_y < r_lca_aera_vel_.f_y) ||
        (radar->obj_y > r_lca_aera_vel_.g_y)) {  // 横向距离不满足要求
      condition_code += uint16_bit[1];
    }
  } else {  // 右侧
    if ((radar->obj_y < f_lca_aera_vel_.l_y) ||
        (radar->obj_y > f_lca_aera_vel_.k_y)) {  // 横向距离不满足要求
      condition_code += uint16_bit[1];
    }
  }
  // condition2
  if (((radar->obj_vx + lkas_input_->vehicle_info.veh_display_speed) <
       f_lca_aera_vel_.obstacle_velocity_limit) ||
      (radar->obj_vx < 0.0F)) {
    condition_code += uint16_bit[2];
  }
  // condition3
  if (fabs(radar->obj_vy) < 0.5F) {
  } else {
    if (fabs(radar->obj_vx / radar->obj_vy) < 1.0F) {
      condition_code = uint16_bit[3];
    }
  }
  return (condition_code);
}
uint8 EmergencyLaneKeepAlert::ObjLcaAlertR(RadarObjData *radar) {
  uint8 alert_code = 0;
  float32 lca_ttc = 0.0F;
  float32 ttc = 0.0F;
  float32 obj_to_front_x = 0;
  float32 crash_y = 0.0F;
  obj_to_front_x = -radar->obj_x - lkas_input_->vehicle_info.common_rear_over;

  if ((radar->obj_x < r_lca_aera_vel_.b_x) && (radar->obj_x >= -25)) {
    lca_ttc = 2.5;
  } else if ((radar->obj_x < -25) && (radar->obj_x > r_lca_aera_vel_.c_x)) {
    lca_ttc = 3.5;
  } else {
    lca_ttc = 0;
  }
  ttc = fabs(obj_to_front_x / radar->obj_vx);

  crash_y = radar->obj_y + ttc * radar->obj_vy;
  // condition1
  if (ttc > lca_ttc) {
    alert_code += uint16_bit[0];
  }
  if (radar->pos = 0) {
    if ((crash_y < -0.5 * lkas_input_->vehicle_info.common_veh_width) ||
        (crash_y > (r_lca_aera_vel_.g_y + 1.0F))) {
      alert_code += uint16_bit[1];
    }
  } else {
    if ((crash_y > 0.5 * lkas_input_->vehicle_info.common_veh_width) ||
        (crash_y < -(r_lca_aera_vel_.l_y + 1.0F))) {
      alert_code += uint16_bit[1];
    }
  }

  return (alert_code);
}
}  // namespace planning