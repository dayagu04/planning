#include "occ_straight_scenario.h"
using namespace planning;
namespace adas_function {

void OccStraightScenario::Init() {
  scene_code_ = -1;
  obj_num_ = 0;
  interest_obj_info_ = {};
  collision_obj_info_ = {};
  final_collision_obj_info_ = {};
};
// 1. 筛选直行相关的障碍物
// 2. 根据障碍物类型(人/车)计算 TTC
// 3. 返回决策结果

// straight_scenario.cc
void OccStraightScenario::Process(void) {
  // 根据感知算法来源 知悉障碍物有效数量
  Init();
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  // 获取场景抑制码
  scene_code_ = SceneCode();
  if (scene_code_ != 0) {
    return;
  }

  auto occ_obj = GetContext.get_session()
                     ->environmental_model()
                     .get_local_view()
                     .fusion_occupancy_objects_info;
  // 障碍物总个数
  obj_num_ = occ_obj.fusion_object_size;

  if (obj_num_ <= 0) {
    return;
  }

  // occ 障碍物个数不对，待办。

  // 遍历感兴趣障碍物
  interest_obj_info_.valid_num = 0;
  interest_obj_info_.interest_obj_vec_.clear();
  interest_obj_info_.interest_obj_vec_.reserve(obj_num_);
  MebTempObj temp_obj;

  // occ obj
  for (int i = 0;
       i < std::min(occ_obj.fusion_object_size,
                    static_cast<uint8>(FUSION_OCCUPANCY_OBJECTS_MAX_NUM));
       i++) {
    temp_obj.rel_x = occ_obj.fusion_object[i]
                         .common_occupancy_info.relative_center_position.x;
    temp_obj.rel_y = occ_obj.fusion_object[i]
                         .common_occupancy_info.relative_center_position.y;
    temp_obj.rel_heading_angle =
        occ_obj.fusion_object[i].common_occupancy_info.heading_angle;
    temp_obj.rel_vx =
        occ_obj.fusion_object[i].common_occupancy_info.relative_velocity.x;
    temp_obj.rel_vy =
        occ_obj.fusion_object[i].common_occupancy_info.relative_velocity.y;
    temp_obj.age = occ_obj.fusion_object[i].additional_occupancy_info.track_age;
    temp_obj.track_id =
        occ_obj.fusion_object[i].additional_occupancy_info.track_id;
    temp_obj.height =
        occ_obj.fusion_object[i].common_occupancy_info.shape.height;
    temp_obj.width = occ_obj.fusion_object[i].common_occupancy_info.shape.width;
    temp_obj.length =
        occ_obj.fusion_object[i].common_occupancy_info.shape.length;
    temp_obj.type_for_meb = OdObjGroup::kDefault;
    temp_obj.index = i;
    temp_obj.fusion_source =
        occ_obj.fusion_object[i].additional_occupancy_info.fusion_source;
    temp_obj.interest_code = SelcetInterestObject(temp_obj);
    if (temp_obj.interest_code == 0) {
      interest_obj_info_.valid_num++;
      interest_obj_info_.interest_obj_vec_.emplace_back(temp_obj);
    }
  }

  // 回收未使用的内存
  interest_obj_info_.interest_obj_vec_.shrink_to_fit();

  if (interest_obj_info_.valid_num <= 0) {
    return;
  } else {
    collision_obj_info_.valid_num = 0;
    collision_obj_info_.interest_obj_vec_.clear();
    collision_obj_info_.interest_obj_vec_.reserve(interest_obj_info_.valid_num);
    CollisionCalculate(0.0, false);
  }

  // 计算得到一组碰撞的数据点
}

int OccStraightScenario::SceneCode(void) {
  // 1. 筛选直行相关的障碍物
  // 2. 根据障碍物类型(人/车)计算 TTC
  // 3. 返回决策结果
  int temp_scene_code = 0;

  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  auto vehicle_service = GetContext.get_session()
                             ->environmental_model()
                             .get_local_view()
                             .vehicle_service_output_info;

  /*bit_0*/
  // 本车挡位校验, 档位不为D,N,P
  if ((vehicle_service.shift_lever_state == iflyauto::ShiftLeverState_N) ||
      (vehicle_service.shift_lever_state == iflyauto::ShiftLeverState_P)) {
    temp_scene_code += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  if ((fabs(vehicle_service.vehicle_speed) < 0.1)) {
    temp_scene_code += uint16_bit[1];
  } else {
    /*do nothing*/
  }

  return temp_scene_code;
};

int OccStraightScenario::SelcetInterestObject(MebTempObj &temp_obj) {
  // 1. 筛选直行相关的障碍物
  // 2. 根据障碍物类型(人/车)计算 TTC
  // 3. 返回决策结果
  int temp_interest_code = 0;
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  auto parameters =
      adas_function::context::AdasFunctionContext::GetInstance().get_param();

  auto vehicle_service = GetContext.get_session()
                             ->environmental_model()
                             .get_local_view()
                             .vehicle_service_output_info;

  // if (!(temp_obj.fusion_source & 0x1u)) {
  //   temp_interest_code += uint16_bit[0];
  // }

  if ((temp_obj.rel_x > 20.0) || (temp_obj.rel_x < -20.0)) {
    temp_interest_code += uint16_bit[1];
  }

  if ((temp_obj.rel_x < parameters->origin_2_front_bumper) &&
      (temp_obj.rel_x > -1.0 * parameters->origin_2_rear_bumper)) {
    temp_interest_code += uint16_bit[2];
  }

  return temp_interest_code;
};

uint64_t OccStraightScenario::FalseTriggerStratege(MebTempObj &obj) {
  // 1. 筛选直行相关的障碍物
  // 2. 根据障碍物类型(人/车)计算 TTC
  // 3. 返回决策结果
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  const auto param = *GetContext.get_param();
  // common code 1-16位
  uint64_t suppe_code = 0;

  if (GetContext.get_state_info()->vehicle_speed > 30.0) {
    suppe_code += uint32_bit[0];
  }

  if (obj.suppe_code == 0) {
    final_collision_obj_info_.valid_num++;
    final_collision_obj_info_.interest_obj_vec_.emplace_back(obj);
  }

  // 如果误触发策略关闭，则suppe_code清零
  if (param.meb_false_trigger_switch == false) {
    suppe_code = 0;
  }
  return suppe_code;
}

}  // namespace adas_function
