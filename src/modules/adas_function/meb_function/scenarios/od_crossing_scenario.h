// straight_scenario.h
#ifndef Od_CROSSING_SCENARIO_H_
#define Od_CROSSING_SCENARIO_H_
#include "meb_scenario_base.h"

using namespace planning;
namespace adas_function {
class OdCrossingScenario : public MebScenarioBase {
 public:
  OdCrossingScenario(){};
  ~OdCrossingScenario() = default;
  void Process(void) override;
  uint64_t FalseTriggerStratege(const MebTempObj obj) override;
  int SceneCode(void) override;
  int SelcetInterestObject(MebTempObj &temp_obj) override;
  int obj_num_;
  void Init(void) override;

 private:
  //本车处于直行状态的持续时长 单位:s
  double ego_in_straight_state_duration_ = 0.0;

  //本车处于匀速状态的持续时长 单位:s
  double ego_in_constant_speed_state_duration_ = 0.0;
};
}  // namespace adas_function
#endif  // STRAIGHT_SCENARIO_H_