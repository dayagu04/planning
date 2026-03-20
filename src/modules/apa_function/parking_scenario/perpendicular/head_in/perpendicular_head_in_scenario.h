#ifndef __PERPENDICULAR_PARK_HEADING_IN_PLANNER_H__
#define __PERPENDICULAR_PARK_HEADING_IN_PLANNER_H__

#include "perpendicular_tail_in_scenario.h"

namespace planning {
namespace apa_planner {

class PerpendicularHeadInScenario : public PerpendicularTailInScenario {
 public:
  PerpendicularHeadInScenario() {
    scenario_type_ = ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN;
  };
  PerpendicularHeadInScenario(const std::shared_ptr<ApaWorld>& apa_world_ptr) {
    SetApaWorldPtr(apa_world_ptr);
    scenario_type_ = ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN;
  }
  virtual std::string GetName() override { return typeid(this).name(); }
};

}  // namespace apa_planner
}  // namespace planning

#endif
