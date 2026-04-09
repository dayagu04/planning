// straight_scenario.h
#ifndef USS_STRAIGHT_SCENARIO_H_
#define USS_STRAIGHT_SCENARIO_H_
#include "meb_scenario_base.h"

using namespace planning;
namespace adas_function {
class UssStraightScenario : public MebScenarioBase {
 public:
  UssStraightScenario(){};
  ~UssStraightScenario() = default;
  void Process(void) override;
  uint64_t FalseTriggerStratege(MebTempObj &obj) override;
  int SceneCode(void) override;
  int SelcetInterestObject(MebTempObj &temp_obj) override;
  int obj_num_;
  void Init(void) override;

 private:
};
}  // namespace adas_function
#endif  // STRAIGHT_SCENARIO_H_