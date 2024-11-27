#pragma once

#include <memory>
#include <string>

#include "./../parking_scenario.h"

namespace planning {

class PerpendicularTailOutScenario : public apa_planner::ParkingScenario {
 public:
  void Init() override;

 private:
 private:
  bool init_ = false;
};

}  // namespace planning
