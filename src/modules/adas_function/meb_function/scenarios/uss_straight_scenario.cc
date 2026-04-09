
#include "uss_straight_scenario.h"
using namespace planning;
namespace adas_function {

void UssStraightScenario::Init() {
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
void UssStraightScenario::Process(void) {
  // 根据感知算法来源 知悉障碍物有效数量
  Init();
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  // 获取场景抑制码
  scene_code_ = SceneCode();
  if (scene_code_ != 0) {
    return;
  }

  auto uss_obj = GetContext.get_session()
                     ->environmental_model()
                     .get_local_view()
                     .uss_percept_info;
  // 障碍物总个数
  obj_num_ = uss_obj.out_line_dataori[0].obj_pt_cnt;

  if (obj_num_ <= 0) {
    return;
  }

  // 遍历感兴趣障碍物
  interest_obj_info_.valid_num = 0;
  interest_obj_info_.interest_obj_vec_.clear();
  interest_obj_info_.interest_obj_vec_.reserve(obj_num_);
  MebTempObj temp_obj;

  // uss obj
  for (int i = 0;
       i < std::min(uss_obj.out_line_dataori[0].obj_pt_cnt,
                    static_cast<uint32>(USS_PERCEPTION_APA_SLOT_OBJ_MAX_NUM));
       i++) {
    temp_obj.rel_x = uss_obj.out_line_dataori[0].obj_pt_local[i].x;
    temp_obj.rel_y = uss_obj.out_line_dataori[0].obj_pt_local[i].y;
    temp_obj.rel_heading_angle = 0;
    temp_obj.rel_vx = 0;
    temp_obj.rel_vy = 0;
    temp_obj.age = 0;
    temp_obj.track_id = 0;
    temp_obj.width = 0.1;
    temp_obj.length = 0.1;
    temp_obj.height = uss_obj.out_line_dataori[0].point_high[i];
    temp_obj.type_for_meb = OdObjGroup::kDefault;
    temp_obj.index = i;
    temp_obj.fusion_source = 0;
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

int UssStraightScenario::SceneCode(void) {
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

  return temp_scene_code;
};

int UssStraightScenario::SelcetInterestObject(MebTempObj &temp_obj) {
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

uint64_t UssStraightScenario::FalseTriggerStratege(MebTempObj &obj) {
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

  // 如果误触发策略关闭，则suppe_code清零
  if (param.meb_false_trigger_switch == false) {
    suppe_code = 0;
  }
  return suppe_code;
}

}  // namespace adas_function
