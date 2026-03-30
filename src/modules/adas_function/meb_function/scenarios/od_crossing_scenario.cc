
#include "od_crossing_scenario.h"
using namespace planning;
namespace adas_function {

void OdCrossingScenario::Init() {
  scene_code_ = -1;
  obj_num_ = 0;
  interest_obj_info_ = {};
  collision_obj_info_ = {};
  final_collision_obj_info_ = {};
  brake_alert_ = false;
};

int OdCrossingScenario::SceneCode(void) {
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

  /*bit_3*/
  // 本车不属于直行状态
  if (fabs(vehicle_service.steering_wheel_angle) > (90.0 / 57.3)) {
    temp_scene_code += uint16_bit[3];
  } else {
    /*do nothing*/
  }

  return temp_scene_code;
};

int OdCrossingScenario::SelcetInterestObject(MebTempObj &temp_obj) {
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
  // 障碍物横向位置过远
  if (fabs(temp_obj.rel_y) > 50.0) {
    temp_interest_code += uint16_bit[3];
  } else {
    // do nothing
  }

  /*bit_4*/
  // 障碍物横穿速度较低
  if (fabs(temp_obj.rel_vy) < (3.0 / 3.6)) {
    temp_interest_code += uint16_bit[4];
  } else {
    // do nothing
  }

  /*bit_5*/
  // 判断障碍物是否在朝着本车这边运动,过滤掉在远离本车的运动物体
  if ((temp_obj.rel_y * temp_obj.rel_vy) > 0.0) {
    temp_interest_code += uint16_bit[5];
  } else {
    // do nothing
  }

  /*bit_6*/
  // 障碍物为非标准横穿状态
  if ((obj_v > 1.0) &&
      (fabs(temp_obj.rel_vx + signed_ego_v) > fabs(temp_obj.rel_vy))) {
    temp_interest_code += uint16_bit[6];
  } else {
    // do nothing
  }

  return temp_interest_code;
};

uint64_t OdCrossingScenario::FalseTriggerStratege(const MebTempObj obj) {
  // suppe_code初始化
  uint64_t suppe_code = 0;

  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  const auto param = *GetContext.get_param();

  auto vehicle_service = GetContext.get_session()
                             ->environmental_model()
                             .get_local_view()
                             .vehicle_service_output_info;

  auto &meb_pre = adas_function::MebPreprocess::GetInstance();

  double signed_ego_v = meb_pre.GetInstance().GetMebInput().signed_ego_vel_mps;

  float32 key_obj_absolute_vel =
      sqrt((obj.rel_vx + signed_ego_v) * (obj.rel_vx + signed_ego_v) +
           obj.rel_vy * obj.rel_vy);

  /*bit_0*/
  // 处于大转弯状态
  if (fabs(vehicle_service.steering_wheel_angle) > (45.0 / 57.3)) {
    suppe_code += uint32_bit[0];
  }

  /*bit_1*/
  // 处于快速打方向盘状态
  if (fabs(vehicle_service.steering_wheel_angle_speed) > (100.0 / 57.3)) {
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
  if (obj.age < 500) {
    suppe_code += uint32_bit[4];
  }

  /*bit_5*/
  // 评估障碍物是否可以轻易通过转向避让
  // 计算障碍物转向避让需要的横向加速度
  // 设置阈值
  double key_obj_an_avoid_by_steering_thrd = 2.0;
  if (obj.type_for_meb == OdObjGroup::kCar) {
    key_obj_an_avoid_by_steering_thrd = 3.0;
  } else if ((obj.type_for_meb == OdObjGroup::kPeople) ||
             (obj.type_for_meb == OdObjGroup::kMotor)) {
    std::vector<double> obj_speed_kph_table = {10.0, 20.0, 30.0};
    std::vector<double> key_obj_an_avoid_by_steering_thrd_table = {4.0, 3.0,
                                                                   2.5};
    double obj_v_absolute_value = fabs(obj.abs_v) * 3.6;
    key_obj_an_avoid_by_steering_thrd = pnc::mathlib::Interp1(
        obj_speed_kph_table, key_obj_an_avoid_by_steering_thrd_table,
        obj_v_absolute_value);
  } else {
    key_obj_an_avoid_by_steering_thrd = 2.0;
  }

  if (fabs(obj.an_avoid_by_steering) < key_obj_an_avoid_by_steering_thrd) {
    suppe_code += uint32_bit[5];
  } else {
    /*do nothing*/
  }

  /*bit_6*/
  // 障碍物有较大的纵向运动速度
  if (fabs(obj.rel_vx + signed_ego_v) > (8.0 / 3.6)) {
    suppe_code += uint32_bit[6];
  } else {
    /*do nothing*/
  }

  /*bit_7*/
  // 判断障碍物的运动速度角,过滤掉非标准横穿障碍物
  float32 speed_direction_deg =
      fabs(atan2(obj.rel_vy + 1e-3, (obj.rel_vx + signed_ego_v) + 1e-3)) * 57.3;
  float32 horizontal_angle_thres_deg = 90.0;
  float32 horizontal_angle_diff_thres_deg = 15.5;
  if (fabs(speed_direction_deg - horizontal_angle_thres_deg) >
      horizontal_angle_diff_thres_deg) {
    suppe_code += uint32_bit[7];
  } else {
    /*do nothing*/
  }

  /*bit_8*/
  // 障碍物处于左路沿以外
  float32 curb_c0 = GetContext.get_road_info()->current_lane.left_roadedge.c0;
  float32 curb_c1 = GetContext.get_road_info()->current_lane.left_roadedge.c1;
  float32 curb_c2 = GetContext.get_road_info()->current_lane.left_roadedge.c2;
  float32 curb_c3 = GetContext.get_road_info()->current_lane.left_roadedge.c3;
  float32 roadedge_begin =
      GetContext.get_road_info()->current_lane.left_roadedge.begin_x;
  float32 roadedge_end =
      GetContext.get_road_info()->current_lane.left_roadedge.end_x;
  float32 left_roadedge_y = curb_c0 + curb_c1 * obj.rel_x +
                            curb_c2 * obj.rel_x * obj.rel_x +
                            curb_c3 * obj.rel_x * obj.rel_x * obj.rel_x;
  if ((GetContext.get_road_info()->current_lane.left_roadedge.valid) &&
      (obj.rel_x > roadedge_begin) && (obj.rel_x < roadedge_end) &&
      (obj.rel_y > left_roadedge_y)) {
    // suppe_code += uint32_bit[8];
  } else {
    // do nothing
  }

  /*bit_9*/
  // 障碍物处于右路沿以外
  curb_c0 = GetContext.get_road_info()->current_lane.right_roadedge.c0;
  curb_c1 = GetContext.get_road_info()->current_lane.right_roadedge.c1;
  curb_c2 = GetContext.get_road_info()->current_lane.right_roadedge.c2;
  curb_c3 = GetContext.get_road_info()->current_lane.right_roadedge.c3;
  roadedge_begin =
      GetContext.get_road_info()->current_lane.right_roadedge.begin_x;
  roadedge_end = GetContext.get_road_info()->current_lane.right_roadedge.end_x;
  float32 right_roadedge_y = curb_c0 + curb_c1 * obj.rel_x +
                             curb_c2 * obj.rel_x * obj.rel_x +
                             curb_c3 * obj.rel_x * obj.rel_x * obj.rel_x;
  if ((GetContext.get_road_info()->current_lane.right_roadedge.valid) &&
      (obj.rel_x > roadedge_begin) && (obj.rel_x < roadedge_end) &&
      (obj.rel_y < right_roadedge_y)) {
    // suppe_code += uint32_bit[9];
  } else {
    // do nothing
  }

  /*bit_10*/
  // 处于泊车感知下

  /*bit_11*/
  // 本车前进时,正前方近距离范围内有运动的CIPV

  // 如果误触发策略关闭，则suppe_code清零
  if (param.meb_false_trigger_switch == false) {
    suppe_code = 0;
  }

  return suppe_code;
}

// crossing_scenario.cc
// 1. 筛选横穿相关的障碍物
// 2. 根据障碍物类型(人/车)计算 TTC
// 3. 返回决策结果
void OdCrossingScenario::Process(void) {
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

  // 初始化所有成员变量默认值
  Init();

  // 本车处于直行状态的持续时长 单位:s
  bool ego_in_straight_state_flag = true;
  if ((vehicle_service.vehicle_speed > (1.0 / 3.6)) &&
      (fabs(vehicle_service.steering_wheel_angle) > (45.0 / 57.3))) {
    ego_in_straight_state_flag = false;
  }
  if ((vehicle_service.vehicle_speed > (1.0 / 3.6)) &&
      (fabs(vehicle_service.steering_wheel_angle_speed) > (45.0 / 57.3))) {
    ego_in_straight_state_flag = false;
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

    // 存储fusion_obj计算处理后得到的信息
    temp_obj.type_for_meb =
        GetOdObjGroup(fusion_obj.fusion_object[i].common_info.type);
    temp_obj.type = fusion_obj.fusion_object[i].common_info.type;
    temp_obj.interest_code = SelcetInterestObject(temp_obj);
    // 障碍物通过打方向可以避免发生碰撞的向心加速度数值。
    float32 key_obj_steering_safe_lat_y =
        0.1;  // 假设障碍物完成转向后，与本车之间剩余的安全横向距离 单位:m
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
    CollisionCalculate(stop_distance_buffer_reduction);
  }

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
}

}  // namespace adas_function
