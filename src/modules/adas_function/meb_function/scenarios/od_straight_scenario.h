// straight_scenario.h
#ifndef Od_STRAIGHT_SCENARIO_H_
#define Od_STRAIGHT_SCENARIO_H_
#include "meb_scenario_base.h"

using namespace planning;
namespace adas_function {
class OdStraightScenario : public MebScenarioBase {
 public:
  OdStraightScenario(){};
  ~OdStraightScenario() = default;
  void Process(void) override;
  uint64_t FalseTriggerStratege(MebTempObj &obj) override;
  int SceneCode(void) override;
  int SelcetInterestObject(MebTempObj &temp_obj) override;
  int obj_num_;
  void Init(void) override;
  bool CollisionCalculateForFrontRadarObj(void);

 private:
  //本车处于直行状态的持续时长 单位:s
  double ego_in_straight_state_duration_ = 0.0;

  //本车处于匀速状态的持续时长 单位:s
  double ego_in_constant_speed_state_duration_ = 0.0;

  //本车处于运动状态的持续时长 单位:s
  double ego_in_motion_state_duration_ = 0.0;

  // 是否会与前雷达障碍物发生碰撞风险
  bool collision_result_for_front_radar_obj_ = false;
};
}  // namespace adas_function
#endif  // STRAIGHT_SCENARIO_H_