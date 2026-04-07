
#include "od_straight_scenario.h"
using namespace planning;
namespace adas_function {

void OdStraightScenario::Init() {
  scene_code_ = -1;
  obj_num_ = 0;
  interest_obj_info_ = {};
  collision_obj_info_ = {};
  final_collision_obj_info_ = {};
  brake_alert_ = false;
  last_collsion_num_ = 0;
};

int OdStraightScenario::SceneCode(void) {
  int temp_scene_code = 0;

  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  auto vehicle_service = GetContext.get_session()
                             ->environmental_model()
                             .get_local_view()
                             .vehicle_service_output_info;

  /*bit_0*/
  // 本车挡位校验, 档位不为D/R
  if ((vehicle_service.shift_lever_state != iflyauto::ShiftLeverState_D) &&
      (vehicle_service.shift_lever_state != iflyauto::ShiftLeverState_R)) {
    temp_scene_code += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  /*bit_1*/
  // 本车处于静止状态
  if ((fabs(vehicle_service.vehicle_speed) < 0.1)) {
    temp_scene_code += uint16_bit[1];
  } else {
    /*do nothing*/
  }

  /*bit_2*/
  // 本车运动速度过高
  if (fabs(vehicle_service.vehicle_speed) > (30.0 / 3.6)) {
    temp_scene_code += uint16_bit[2];
  } else {
    /*do nothing*/
  }

  return temp_scene_code;
};

int OdStraightScenario::SelcetInterestObject(MebTempObj &temp_obj) {
  int temp_interest_code = 0;

  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto parameters =
      adas_function::context::AdasFunctionContext::GetInstance().get_param();

  auto vehicle_service = GetContext.get_session()
                             ->environmental_model()
                             .get_local_view()
                             .vehicle_service_output_info;

  auto &meb_pre = adas_function::MebPreprocess::GetInstance();

  double signed_ego_v = meb_pre.GetInstance().GetMebInput().signed_ego_vel_mps;

  double obj_v =
      sqrt((temp_obj.rel_vx + signed_ego_v) * (temp_obj.rel_vx + signed_ego_v) +
           temp_obj.rel_vy * temp_obj.rel_vy);

  /*bit_0*/
  // 判断障碍物传感器来源,只选择视觉感知检测到的障碍物
  if (!(temp_obj.fusion_source & 0x1u)) {
    temp_interest_code += uint16_bit[0];
  }

  /*bit_1*/
  // 根据障碍物纵向位置进行筛选,障碍物离本车较远
  // 障碍物筛选区域--x(绝对值大小)
  double obj_select_area_x = std::max(5.0, vehicle_service.vehicle_speed * 3.0);
  if (vehicle_service.shift_lever_state ==
      iflyauto::ShiftLeverStateEnum::ShiftLeverState_D) {
    if (temp_obj.rel_x >
        (obj_select_area_x + parameters->origin_2_front_bumper)) {
      temp_interest_code += uint16_bit[1];
    }
  } else if (vehicle_service.shift_lever_state ==
             iflyauto::ShiftLeverStateEnum::ShiftLeverState_R) {
    if (temp_obj.rel_x <
        (-1.0 * (obj_select_area_x + parameters->origin_2_rear_bumper))) {
      temp_interest_code += uint16_bit[1];
    }
  } else {
    // do nothing
  }

  /*bit_2*/
  // 根据挡位判断,障碍物纵向位置是否处于本车运动方向前方
  if (vehicle_service.shift_lever_state ==
      iflyauto::ShiftLeverStateEnum::ShiftLeverState_D) {
    if (temp_obj.rel_x < parameters->origin_2_front_bumper) {
      temp_interest_code += uint16_bit[2];
    }
  } else if (vehicle_service.shift_lever_state ==
             iflyauto::ShiftLeverStateEnum::ShiftLeverState_R) {
    if (temp_obj.rel_x > (-1.0 * parameters->origin_2_rear_bumper)) {
      temp_interest_code += uint16_bit[2];
    }
  } else {
    // do nothing
  }

  /*bit_3*/
  // 判断障碍物是否在本车运动轨迹上
  float left_boundary_c0 = 1.0 * (parameters->ego_width / 2.0 + 0.5);
  float left_boundary_c1 = 0.0;
  float left_boundary_c2 = 0.5 * GetContext.get_state_info()->ego_curvature;
  float left_boundary_c3 = 0.0;
  float right_boundary_c0 = -1.0 * (parameters->ego_width / 2.0 + 0.5);
  float right_boundary_c1 = 0.0;
  float right_boundary_c2 = 0.5 * GetContext.get_state_info()->ego_curvature;
  float right_boundary_c3 = 0.0;

  float left_line_y =
      left_boundary_c0 + left_boundary_c1 * temp_obj.rel_x +
      left_boundary_c2 * temp_obj.rel_x * temp_obj.rel_x +
      left_boundary_c3 * temp_obj.rel_x * temp_obj.rel_x * temp_obj.rel_x;

  float right_line_y =
      right_boundary_c0 + right_boundary_c1 * temp_obj.rel_x +
      right_boundary_c2 * temp_obj.rel_x * temp_obj.rel_x +
      right_boundary_c3 * temp_obj.rel_x * temp_obj.rel_x * temp_obj.rel_x;

  if ((temp_obj.rel_y > left_line_y) || (temp_obj.rel_y < right_line_y)) {
    temp_interest_code += uint16_bit[3];
  }

  /*bit_4*/
  // 障碍物有横穿运动趋势,即有逃离本车正前方/正后方的运动趋势
  if ((obj_v > 1.0) &&
      (fabs(temp_obj.rel_vx + signed_ego_v) < fabs(temp_obj.rel_vy))) {
    temp_interest_code += uint16_bit[4];
  }

  /*bit_5*/
  // 过滤非感兴趣类型的目标
  if (temp_obj.type_for_meb == OdObjGroup::kDefault) {
    if ((temp_obj.type != iflyauto::OBJECT_TYPE_TRAFFIC_CONE) &&
        (temp_obj.type != iflyauto::OBJECT_TYPE_WATER_SAFETY_BARRIER)) {
      temp_interest_code += uint16_bit[5];
    }
  }

  return temp_interest_code;
};

uint64_t OdStraightScenario::FalseTriggerStratege(MebTempObj &obj) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  const auto param = *GetContext.get_param();
  auto &meb_pre = adas_function::MebPreprocess::GetInstance();
  double vel_speed_preprocess =
      meb_pre.GetInstance().GetMebInput().signed_ego_vel_mps;
  auto vehicle_service = GetContext.get_session()
                             ->environmental_model()
                             .get_local_view()
                             .vehicle_service_output_info;

  auto front_radar_key_obj_info = meb_pre.GetFrontRadarKeyObjInfo();

  // suppe_code初始化
  uint64_t suppe_code = 0;

  /*bit_0*/
  // 处于大转弯状态
  if (fabs(vehicle_service.steering_wheel_angle) > (90.0 / 57.3)) {
    suppe_code += uint32_bit[0];
  }

  /*bit_1*/
  // 处于快速打方向盘状态
  if (fabs(vehicle_service.steering_wheel_angle_speed) > (300.0 / 57.3)) {
    suppe_code += uint32_bit[1];
  }

  /*bit_2*/
  // 判断本车处于直行状态的持续时长
  if (ego_in_straight_state_duration_ < 2.0) {
    suppe_code += uint32_bit[2];
  }

  /*bit_3*/
  // 判断本车处于匀速状态的持续时长
  if (ego_in_constant_speed_state_duration_ < 2.0) {
    suppe_code += uint32_bit[3];
  }

  /*bit_4*/
  // 判断障碍物存活时间
  float obj_age_thrd = 1000;
  if (obj.type == iflyauto::OBJECT_TYPE_TRAFFIC_CONE ||
      obj.type == iflyauto::OBJECT_TYPE_WATER_SAFETY_BARRIER) {
    obj_age_thrd = 100;
  } else {
    obj_age_thrd = 1000;
  }
  if (obj.age < obj_age_thrd) {
    suppe_code += uint32_bit[4];
  }

  /*bit_5*/
  // 障碍物与本车当前重叠率较低 注:这样判断有缺陷,不适用于转弯场景
  if (fabs(obj.rel_y) > 0.75) {
    suppe_code += uint32_bit[5];
  }

  /*bit_6*/
  // 假定本车以固定向左的方向盘转速进行打方向,判断是否可以避免碰撞发生
  box_collision_.t_start = 0.0;
  box_collision_.t_end = 2.0;
  box_collision_.time_step = 0.1;
  box_collision_.boxs_info_.ego_a_x = vehicle_service.long_acceleration;
  box_collision_.boxs_info_.ego_backshaft_2_fbumper =
      param.origin_2_front_bumper;
  box_collision_.boxs_info_.ego_length = param.ego_length;
  box_collision_.boxs_info_.ego_radius =
      meb_pre.GetInstance().GetMebInput().ego_radius;
  box_collision_.boxs_info_.ego_v_x = vel_speed_preprocess;
  box_collision_.boxs_info_.ego_width = param.ego_width;
  box_collision_.boxs_info_.obj_a_x = 0.0;
  box_collision_.boxs_info_.obj_a_y = 0.0;
  box_collision_.boxs_info_.obj_heading_angle = obj.rel_heading_angle;
  box_collision_.boxs_info_.obj_length = obj.length;
  box_collision_.boxs_info_.obj_v_x =
      obj.rel_vx + vel_speed_preprocess;  // 需要考虑自车前进及后退
  box_collision_.boxs_info_.obj_v_y = obj.rel_vy;
  box_collision_.boxs_info_.obj_width = obj.width;
  box_collision_.boxs_info_.obj_x = obj.rel_x;
  box_collision_.boxs_info_.obj_y = obj.rel_y;
  double ego_steer_angle_speed_for_avoid_collision = 40.0 / 57.3;
  double ego_steer_angle_max_abs = 500.0 / 57.3;
  bool ego_avoid_collision_by_left_steer_result =
      box_collision_.GetCollisionResultBySimEgoSteerAngleSpeed(
          box_collision_.boxs_info_, box_collision_.t_start,
          box_collision_.t_end, box_collision_.time_step, param.steer_ratio,
          param.wheel_base, vehicle_service.steering_wheel_angle,
          ego_steer_angle_speed_for_avoid_collision, ego_steer_angle_max_abs);
  // if (ego_avoid_collision_by_left_steer_result == false) {
  //   obj.suppe_code += uint32_bit[6];
  // }

  /*bit_7*/
  // 假定本车以固定向右的方向盘转速进行打方向,判断是否可以避免碰撞发生
  ego_steer_angle_speed_for_avoid_collision = -40.0 / 57.3;
  ego_steer_angle_max_abs = 500.0 / 57.3;
  bool ego_avoid_collision_by_right_steer_result =
      box_collision_.GetCollisionResultBySimEgoSteerAngleSpeed(
          box_collision_.boxs_info_, box_collision_.t_start,
          box_collision_.t_end, box_collision_.time_step, param.steer_ratio,
          param.wheel_base, vehicle_service.steering_wheel_angle,
          ego_steer_angle_speed_for_avoid_collision, ego_steer_angle_max_abs);
  // if (ego_avoid_collision_by_right_steer_result == false) {
  //   obj.suppe_code += uint32_bit[7];
  // }

  /*bit_8*/
  // 判断障碍物类型,暂时先不针对此障碍物触发
  if ((obj.type == iflyauto::OBJECT_TYPE_TRAFFIC_CONE) ||
      (obj.type == iflyauto::OBJECT_TYPE_WATER_SAFETY_BARRIER)) {
    suppe_code += uint32_bit[8];
  }

  /*bit_9*/
  // ttc过小
  double obj_min_dist =
      obj.rel_x - 0.5 * obj.length - param.origin_2_front_bumper;
  double ttc;
  if ((vehicle_service.shift_lever_state ==
       iflyauto::ShiftLeverStateEnum::ShiftLeverState_D) &&
      (obj.rel_vx < -0.1)) {
    // Todo:未考虑障碍物航向角，这样简化计算有点问题
    if (obj_min_dist > 0.0) {
      ttc = fabs(obj_min_dist / obj.rel_vx);
    } else {
      ttc = 0.0;
    }
    if (ttc < 0.3) {
      suppe_code += uint32_bit[9];
    }
  }

  /*bit_10*/
  // 障碍物有横穿趋势
  if (fabs(obj.rel_vy) > 0.6) {
    suppe_code += uint32_bit[10];
  }

  /*bit_11*/
  // 判断本车处于运动状态的持续时长
  if (ego_in_motion_state_duration_ < 3.0) {
    suppe_code += uint32_bit[11];
  }

  /*bit_12*/
  // 判断障碍物航向角
  if (obj.type_for_meb == OdObjGroup::kCar) {
    if (fabs(obj.rel_heading_angle) > 0.785) {
      suppe_code += uint32_bit[12];
    }
  }

  // *bit_13*/
  double speed_angle = atan2(obj.rel_vy, obj.abs_vx);
  if (obj.abs_v > 1.0) {
    if ((speed_angle > M_PI / 4 && speed_angle < M_PI) ||
        (speed_angle > -M_PI && speed_angle < -M_PI / 4)) {
      suppe_code += uint32_bit[13];
    }
  }

  // *bit_14*/
  // 通过判断与雷达障碍物是否有碰撞风险,进行双重校验
  // 当前仅针对行人做此校验,泊车感知下无前雷达
  // if ((obj.type_for_meb == OdObjGroup::kPeople) ||
  //     (obj.type_for_meb == OdObjGroup::kMotor)) {
  if ((vehicle_service.shift_lever_state ==
           iflyauto::ShiftLeverStateEnum::ShiftLeverState_D ||
       vehicle_service.shift_lever_state ==
           iflyauto::ShiftLeverStateEnum::ShiftLeverState_M) &&
      (!meb_pre.GetMebInput().park_mode) && (obj.fusion_source == 1)) {
    if (obj.type_for_meb == OdObjGroup::kPeople) {
      if (obj.age < 10000) {
        if (collision_result_for_front_radar_obj_ == false) {
          suppe_code += uint32_bit[14];
        }
      }
    } else {
      if (collision_result_for_front_radar_obj_ == false) {
        suppe_code += uint32_bit[14];
      }
    }

    /*bit_15*/
    // 障碍物与本车当前重叠率较低 通过雷达障碍物进行双重校验
    // 注:这样判断有缺陷,不适用于转弯场景
    if ((obj.type_for_meb == OdObjGroup::kPeople) && (obj.fusion_source == 1)) {
      if (fabs(front_radar_key_obj_info.key_obj_relative_y) > 0.75) {
        suppe_code += uint32_bit[15];
      }
    }
  }

  /*bit_16*/
  // 防止出地库在坡道上时,因感知误识别导致误触发
  if (vehicle_service.shift_lever_state ==
      iflyauto::ShiftLeverStateEnum::ShiftLeverState_D) {
    if ((vehicle_service.vehicle_speed > 2.5) &&
        (vehicle_service.long_acceleration > 0.45)) {
      suppe_code += uint32_bit[16];
    }
  }

  // }

  // 如果误触发策略关闭，则suppe_code清零
  if (param.meb_false_trigger_switch == false) {
    suppe_code = 0;
  }

  return suppe_code;
}

bool OdStraightScenario::CollisionCalculateForFrontRadarObj(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  const auto &param = *GetContext.get_param();

  auto &meb_pre = adas_function::MebPreprocess::GetInstance();

  auto front_radar_key_obj_info = meb_pre.GetFrontRadarKeyObjInfo();

  float32 radius = meb_pre.GetInstance().GetMebInput().ego_radius;
  float32 meb_acc_collision_thrd = param.meb_acc_collision_thrd;
  float32 ego_backshaft_2_fbumper = param.origin_2_front_bumper;
  float32 ego_length = param.ego_length;
  float32 ego_width = param.ego_width;
  float32 ego_acc = GetContext.get_state_info()->vel_acc;

  double shift_direction_index =
      meb_pre.GetInstance().GetMebInput().shift_direction_index;
  double vel_speed_preprocess =
      meb_pre.GetInstance().GetMebInput().signed_ego_vel_mps;

  //修正本车的加速度,只考虑本车减速工况,不考虑本车加速工况
  if (vel_speed_preprocess > 0.0) {
    // 本车处于前进状态
    if (ego_acc > 0.0) {
      ego_acc = 0.0;
    }
  } else {
    // 本车处于倒车状态
    if (ego_acc < 0.0) {
      ego_acc = 0.0;
    }
  }

  // 1.标定信息
  box_collision_.t_start = 0.0;
  box_collision_.t_end = 2.0;
  box_collision_.time_step = 0.1;
  box_collision_.dec_request = meb_acc_collision_thrd * shift_direction_index;

  // 2.车辆信息
  box_collision_.boxs_info_.ego_a_x = ego_acc;
  box_collision_.boxs_info_.ego_backshaft_2_fbumper = ego_backshaft_2_fbumper;
  box_collision_.boxs_info_.ego_length = ego_length;
  box_collision_.boxs_info_.ego_radius = radius;
  box_collision_.boxs_info_.ego_v_x = vel_speed_preprocess;
  box_collision_.boxs_info_.ego_width = ego_width;

  if (GetContext.get_state_info()->shift_lever_state ==
      iflyauto::ShiftLeverStateEnum::ShiftLeverState_R) {
    return false;
  }

  if (front_radar_key_obj_info.key_obj_index >= FUSION_OBJECT_MAX_NUM) {
    return false;
  }

  // 设置的刹停安全距离 单位:m
  double stop_distance_buffer = 1.3;

  box_collision_.time_dealy =
      GetContext.get_param()->meb_actuator_act_time +
      stop_distance_buffer /
          std::max(0.5, GetContext.get_state_info()->vehicle_speed);

  box_collision_.boxs_info_.obj_a_x = 0.0;  // obs.relative_acceleration.x;
  box_collision_.boxs_info_.obj_a_y = 0.0;  // obs.relative_acceleration.y;
  box_collision_.boxs_info_.obj_heading_angle =
      front_radar_key_obj_info.key_obj_relative_heading_angle;
  box_collision_.boxs_info_.obj_length =
      front_radar_key_obj_info.key_obj_length;
  // 需要考虑自车前进及后退
  box_collision_.boxs_info_.obj_v_x =
      front_radar_key_obj_info.key_obj_relative_v_x + vel_speed_preprocess;
  box_collision_.boxs_info_.obj_v_y =
      front_radar_key_obj_info.key_obj_relative_v_y;
  box_collision_.boxs_info_.obj_width = front_radar_key_obj_info.key_obj_width;
  box_collision_.boxs_info_.obj_x = front_radar_key_obj_info.key_obj_relative_x;
  box_collision_.boxs_info_.obj_y = front_radar_key_obj_info.key_obj_relative_y;

  bool is_collision_result = box_collision_.GetCollisionResultBySimEgoDec(
      box_collision_.boxs_info_, box_collision_.t_start, box_collision_.t_end,
      box_collision_.time_step, box_collision_.time_dealy,
      box_collision_.dec_request);

  //==========================================debug_code======================================================================//
  // if (is_collision_result) {
  //   std::cout << "跟雷达障碍物有碰撞风险" << std::endl;
  // } else {
  //   std::cout << "跟雷达障碍物无碰撞风险" << std::endl;
  // }
  // if (is_collision_result) {
  //   // 已有碰撞风险,安全距离设置为多少时,会认为没有碰撞风险
  //   for (int i = 0; i < 30; i++) {
  //     std::cout << "正在查找:" << std::endl;
  //     // 1.标定信息
  //     box_collision_.t_start = 0.0;
  //     box_collision_.t_end = 2.0;
  //     box_collision_.time_step = 0.1;
  //     box_collision_.dec_request =
  //         meb_acc_collision_thrd * shift_direction_index;

  //     // 2.车辆信息
  //     box_collision_.boxs_info_.ego_a_x = ego_acc;
  //     box_collision_.boxs_info_.ego_backshaft_2_fbumper =
  //         ego_backshaft_2_fbumper;
  //     box_collision_.boxs_info_.ego_length = ego_length;
  //     box_collision_.boxs_info_.ego_radius = radius;
  //     box_collision_.boxs_info_.ego_v_x = vel_speed_preprocess;
  //     box_collision_.boxs_info_.ego_width = ego_width;

  //     double safe_dist = 2.0 - i * 0.1;
  //     if (safe_dist < 0.1) {
  //       safe_dist = 0.1;
  //     }
  //     box_collision_.time_dealy =
  //         GetContext.get_param()->meb_actuator_act_time +
  //         safe_dist / std::max(0.5,
  //         GetContext.get_state_info()->vehicle_speed);

  //     box_collision_.boxs_info_.obj_a_x = 0.0;  //
  //     obs.relative_acceleration.x; box_collision_.boxs_info_.obj_a_y = 0.0;
  //     // obs.relative_acceleration.y;
  //     box_collision_.boxs_info_.obj_heading_angle =
  //         front_radar_key_obj_info.key_obj_relative_heading_angle;
  //     box_collision_.boxs_info_.obj_length =
  //         front_radar_key_obj_info.key_obj_length;
  //     // 需要考虑自车前进及后退
  //     box_collision_.boxs_info_.obj_v_x =
  //         front_radar_key_obj_info.key_obj_relative_v_x +
  //         vel_speed_preprocess;
  //     box_collision_.boxs_info_.obj_v_y =
  //         front_radar_key_obj_info.key_obj_relative_v_y;
  //     box_collision_.boxs_info_.obj_width =
  //         front_radar_key_obj_info.key_obj_width;
  //     box_collision_.boxs_info_.obj_x =
  //         front_radar_key_obj_info.key_obj_relative_x;
  //     box_collision_.boxs_info_.obj_y =
  //         front_radar_key_obj_info.key_obj_relative_y;

  //     double current_dist = box_collision_.boxs_info_.obj_x -
  //                           box_collision_.boxs_info_.ego_backshaft_2_fbumper
  //                           - 0.5 * box_collision_.boxs_info_.obj_length;
  //     std::cout << "与障碍物当前的剩余距离为:" << current_dist << std::endl;

  //     bool is_collision_result_temp =
  //         box_collision_.GetCollisionResultBySimEgoDec(
  //             box_collision_.boxs_info_, box_collision_.t_start,
  //             box_collision_.t_end, box_collision_.time_step,
  //             box_collision_.time_dealy, box_collision_.dec_request);
  //     std::cout << "safe_dist = " << safe_dist << std::endl;
  //     std::cout << "box_collision_.time_dealy = " <<
  //     box_collision_.time_dealy
  //               << std::endl;

  //     if (is_collision_result_temp == false) {
  //       std::cout << "最小允许设置的安全距离为:" << safe_dist << std::endl;
  //       break;
  //     } else {
  //       std::cout << "仍然会发生碰撞:" << safe_dist << std::endl;
  //     }
  //   }
  // }
  //==========================================debug_code======================================================================//

  return is_collision_result;
}

// straight_scenario.cc
// 1. 筛选直行相关的障碍物
// 2. 根据障碍物类型(人/车)计算 TTC
// 3. 返回决策结果
void OdStraightScenario::Process(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto vehicle_service = GetContext.get_session()
                             ->environmental_model()
                             .get_local_view()
                             .vehicle_service_output_info;

  auto fusion_obj = GetContext.get_session()
                        ->environmental_model()
                        .get_local_view()
                        .fusion_objects_info;

  auto &meb_pre = adas_function::MebPreprocess::GetInstance();

  double signed_ego_v = meb_pre.GetInstance().GetMebInput().signed_ego_vel_mps;

  auto rear_key_obj_info = meb_pre.GetRearKeyObjInfo();

  auto front_radar_key_obj_info = meb_pre.GetFrontRadarKeyObjInfo();

  // 初始化所有成员变量默认值
  Init();

  // 本车处于直行状态的持续时长 单位:s
  bool ego_in_straight_state_flag = true;
  if (vehicle_service.shift_lever_state == iflyauto::ShiftLeverState_D ||
      vehicle_service.shift_lever_state == iflyauto::ShiftLeverState_M) {
    if ((vehicle_service.vehicle_speed > (1.0 / 3.6)) &&
        (fabs(vehicle_service.steering_wheel_angle) > (45.0 / 57.3))) {
      ego_in_straight_state_flag = false;
    }
    if ((vehicle_service.vehicle_speed > (1.0 / 3.6)) &&
        (fabs(vehicle_service.steering_wheel_angle_speed) > (60.0 / 57.3))) {
      ego_in_straight_state_flag = false;
    }
  } else if (vehicle_service.shift_lever_state == iflyauto::ShiftLeverState_R) {
    if ((vehicle_service.vehicle_speed > (1.0 / 3.6)) &&
        (fabs(vehicle_service.steering_wheel_angle) > (45.0 / 57.3))) {
      ego_in_straight_state_flag = false;
    }
    if ((vehicle_service.vehicle_speed > (1.0 / 3.6)) &&
        (fabs(vehicle_service.steering_wheel_angle_speed) > (170.0 / 57.3))) {
      ego_in_straight_state_flag = false;
    }
  }

  if (ego_in_straight_state_flag) {
    ego_in_straight_state_duration_ += GetContext.get_param()->dt;
  } else {
    ego_in_straight_state_duration_ = 0.0;
  }
  if (ego_in_straight_state_duration_ > 60.0) {
    ego_in_straight_state_duration_ = 60.0;
  }

  //本车处于匀速状态的持续时长 单位:s
  bool ego_in_constant_speed_state_flag = true;
  if (vehicle_service.long_acceleration < -1.0) {
    ego_in_constant_speed_state_flag = false;
  }
  if (ego_in_constant_speed_state_flag) {
    ego_in_constant_speed_state_duration_ += GetContext.get_param()->dt;
  } else {
    ego_in_constant_speed_state_duration_ = 0.0;
  }
  if (ego_in_constant_speed_state_duration_ > 60.0) {
    ego_in_constant_speed_state_duration_ = 60.0;
  }

  //本车处于运动状态的持续时长 单位:s
  if (vehicle_service.vehicle_speed > (1.0 / 3.6)) {
    ego_in_motion_state_duration_ += GetContext.get_param()->dt;
    if (ego_in_motion_state_duration_ > 60.0) {
      ego_in_motion_state_duration_ = 60.0;
    }
  } else {
    ego_in_motion_state_duration_ = 0.0;
  }

  // 获取场景抑制码
  scene_code_ = SceneCode();
  if (scene_code_ != 0) {
    return;
  }

  // 障碍物总个数
  obj_num_ = fusion_obj.fusion_object_size;

  // 障碍物数量有效性判断
  if ((obj_num_ <= 0) || (obj_num_ > FUSION_OBJECT_MAX_NUM)) {
    return;
  }

  // 目标筛选:遍历所有融合障碍物,筛选出感兴趣目标
  interest_obj_info_.valid_num = 0;
  interest_obj_info_.interest_obj_vec_.clear();
  interest_obj_info_.interest_obj_vec_.reserve(obj_num_);
  MebTempObj temp_obj;
  // od box obj
  for (int i = 0; i < fusion_obj.fusion_object_size; i++) {
    // 存储fusion_obj原始信息
    temp_obj.rel_x =
        fusion_obj.fusion_object[i].common_info.relative_center_position.x;
    temp_obj.rel_y =
        fusion_obj.fusion_object[i].common_info.relative_center_position.y;
    temp_obj.rel_heading_angle =
        fusion_obj.fusion_object[i].common_info.relative_heading_angle;
    temp_obj.rel_vx =
        fusion_obj.fusion_object[i].common_info.relative_velocity.x;
    temp_obj.abs_vx =
        (fusion_obj.fusion_object[i].common_info.relative_velocity.x +
         signed_ego_v);
    temp_obj.rel_vy =
        fusion_obj.fusion_object[i].common_info.relative_velocity.y;
    temp_obj.abs_v = sqrt(temp_obj.abs_vx * temp_obj.abs_vx +
                          temp_obj.rel_vy * temp_obj.rel_vy);
    temp_obj.rel_acc_x =
        fusion_obj.fusion_object[i].common_info.relative_acceleration.x;
    temp_obj.rel_acc_y =
        fusion_obj.fusion_object[i].common_info.relative_acceleration.y;
    temp_obj.width = fusion_obj.fusion_object[i].common_info.shape.width;
    temp_obj.length = fusion_obj.fusion_object[i].common_info.shape.length;
    temp_obj.height = fusion_obj.fusion_object[i].common_info.shape.height;
    temp_obj.conf = fusion_obj.fusion_object[i].additional_info.confidence;
    temp_obj.age = fusion_obj.fusion_object[i].additional_info.track_age;
    temp_obj.track_id = fusion_obj.fusion_object[i].additional_info.track_id;
    temp_obj.index = i;
    temp_obj.fusion_source =
        fusion_obj.fusion_object[i].additional_info.fusion_source;

    // hotfix 为了解决感知对静止物体测出
    // 动态速度的bug,这里强行对障碍物速度赋值成静止

    // if (fabs(temp_obj.abs_vx) < 1.4 && (temp_obj.abs_vx * signed_ego_v) <
    // 0.0) {
    //   temp_obj.rel_vx = signed_ego_v * -1.0;
    //   temp_obj.abs_vx = 0.0;
    //   temp_obj.abs_v = sqrt(temp_obj.abs_vx * temp_obj.abs_vx +
    //                         temp_obj.rel_vy * temp_obj.rel_vy);
    // }

    // 存储fusion_obj计算处理后得到的信息
    temp_obj.type_for_meb =
        GetOdObjGroup(fusion_obj.fusion_object[i].common_info.type);
    temp_obj.type = fusion_obj.fusion_object[i].common_info.type;
    temp_obj.interest_code = SelcetInterestObject(temp_obj);

    // 障碍物通过打方向可以避免发生碰撞的向心加速度数值。给横穿用
    float32 key_obj_steering_safe_lat_y =
        1.0;  // 假设障碍物完成转向后，与本车之间剩余的安全横向距离 单位:m
    float32 key_obj_avoid_by_steering_y =
        (fabs(temp_obj.rel_y) - 0.5 * temp_obj.width -
         0.5 * GetContext.get_param()->ego_width - key_obj_steering_safe_lat_y);
    if (key_obj_avoid_by_steering_y < 0.01) {
      key_obj_avoid_by_steering_y = 0.01;
    }

    float32 key_obj_an_avoid_by_steering =
        (temp_obj.abs_v * temp_obj.abs_v *
         (1 - MyCosRad(temp_obj.rel_heading_angle))) /
        key_obj_avoid_by_steering_y;
    if (key_obj_an_avoid_by_steering > 10.0) {
      key_obj_an_avoid_by_steering = 10.0;
    } else {
      // do nothing
    }
    temp_obj.an_avoid_by_steering = key_obj_an_avoid_by_steering;

    // 评判scp 工况， 低速前车是否可以通过减速度避免碰撞的ay数值。
    float32 key_obj_min_y_for_acc = fabs(temp_obj.rel_y) -
                                    0.5 * temp_obj.length -
                                    0.5 * GetContext.get_param()->ego_width;
    if (key_obj_min_y_for_acc < 0.01) {
      key_obj_min_y_for_acc = 0.01;
    } else {
    }
    float32 key_obj_ay_avoid_by_accelerating =
        (temp_obj.rel_vy * temp_obj.abs_v) / 2.0 / key_obj_min_y_for_acc;
    if (key_obj_ay_avoid_by_accelerating > 10.0) {
      key_obj_ay_avoid_by_accelerating = 10.0;
    } else {
    }
    temp_obj.ay_avoid_by_accelerating = key_obj_ay_avoid_by_accelerating;

    temp_obj.collision_point_y = 0.0;  // 直行不使用该变量

    // hack 若为大卡车，则认为距离偏远0.3m，晚点触发
    if (temp_obj.type == iflyauto::OBJECT_TYPE_TRUCK) {
      temp_obj.rel_x = temp_obj.rel_x + 0.3;
    }

    // 若为感兴趣目标,则添加至interest_obj_info_中
    if (temp_obj.interest_code == 0) {
      interest_obj_info_.valid_num++;
      interest_obj_info_.interest_obj_vec_.emplace_back(temp_obj);
    }
  }

  // 回收未使用的interest_obj_info_内存
  interest_obj_info_.interest_obj_vec_.shrink_to_fit();

  // 针对感兴趣的障碍物进行碰撞检测
  if (interest_obj_info_.valid_num <= 0) {
    return;
  } else {
    collision_obj_info_.valid_num = 0;
    collision_obj_info_.interest_obj_vec_.clear();
    collision_obj_info_.interest_obj_vec_.reserve(interest_obj_info_.valid_num);
    double stop_distance_buffer_reduction = 0.0;
    if ((vehicle_service.shift_lever_state == iflyauto::ShiftLeverState_D) &&
        (rear_key_obj_info.key_obj_index < FUSION_OBJECT_MAX_NUM)) {
      // 存在较近范围内有后方车辆,则认为是交通拥堵场景,适当降低剩余安全距离
      if (rear_key_obj_info.key_obj_relative_x > -10.0) {
        stop_distance_buffer_reduction = 0.3;
      } else {
        stop_distance_buffer_reduction = 0.0;
      }
    }
    CollisionCalculate(stop_distance_buffer_reduction, true);
  }

  collision_result_for_front_radar_obj_ = CollisionCalculateForFrontRadarObj();

  // 针对有碰撞风险的障碍物,配合误触发策略,决策是否需要产生制动
  if (collision_obj_info_.valid_num <= 0) {
    return;
  } else {
    // 每个碰撞物体对应的抑制码为0, 为0则触发aeb
    final_collision_obj_info_.valid_num = 0;
    final_collision_obj_info_.interest_obj_vec_.clear();
    final_collision_obj_info_.interest_obj_vec_.reserve(
        collision_obj_info_.valid_num);
    for (auto &collision_obs : collision_obj_info_.interest_obj_vec_) {
      collision_obs.suppe_code = FalseTriggerStratege(collision_obs);
      // result:需要针对此物体制动来避免碰撞发生
      if (collision_obs.suppe_code == 0) {
        final_collision_obj_info_.valid_num++;
        final_collision_obj_info_.interest_obj_vec_.emplace_back(collision_obs);
      }
    }

    // 回收未使用的final_collision_obj_info_内存
    final_collision_obj_info_.interest_obj_vec_.shrink_to_fit();
  }

  // set_brake_alert
  if (final_collision_obj_info_.valid_num > 0) {
    brake_alert_ = true;
  } else {
    brake_alert_ = false;
  }

  if (last_collsion_num_ == 0 && collision_obj_info_.valid_num > 0 &&
      final_collision_obj_info_.valid_num != collision_obj_info_.valid_num) {
    std::vector<double> suppe_code_vector;
    for (auto &collision_obs : final_collision_obj_info_.interest_obj_vec_) {
      suppe_code_vector.push_back(
          static_cast<double>(collision_obs.suppe_code));
    }
    JSON_DEBUG_VALUE("od_straight_scenario_is_suppressed", (int)1);
    JSON_DEBUG_VECTOR("od_straight_scenario_supp_code", suppe_code_vector, 2);
  }
  last_collsion_num_ = collision_obj_info_.valid_num;
}

}  // namespace adas_function
