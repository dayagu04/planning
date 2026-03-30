// straight_scenario.h
#ifndef OCC_STRAIGHT_SCENARIO_H_
#define OCC_STRAIGHT_SCENARIO_H_
#include "meb_scenario_base.h"

using namespace planning;
namespace adas_function {
class OccStraightScenario : public MebScenarioBase {
 public:
  OccStraightScenario(){};
  ~OccStraightScenario() = default;
  void Process(void) override;
  uint64_t FalseTriggerStratege(const MebTempObj obj) override;
  int SceneCode(void) override;
  int SelcetInterestObject(MebTempObj &temp_obj) override;
  int obj_num_;
  void Init(void) override;

 private:
};
}  // namespace adas_function
#endif  // STRAIGHT_SCENARIO_H_