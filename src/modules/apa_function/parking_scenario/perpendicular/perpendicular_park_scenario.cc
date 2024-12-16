#include "perpendicular_park_scenario.h"

namespace planning {
namespace apa_planner {
void PerpendicularParkScenario::Reset() { frame_.Reset();
  ParkingScenario::Reset();
}

void PerpendicularParkScenario::ExcutePathPlanningTask() {}

const bool PerpendicularParkScenario::UpdateEgoSlotInfo() { return false; }

void PerpendicularParkScenario::GenTlane() {}

void PerpendicularParkScenario::GenObstacles() {}

const uint8_t PerpendicularParkScenario::PathPlanOnce() {
  return PathPlannerResult::PLAN_FAILED;
}

const bool PerpendicularParkScenario::CheckSegCompleted() { return false; }

const bool PerpendicularParkScenario::CheckUssStucked() { return false; }

const bool PerpendicularParkScenario::CheckColDetStucked() { return false; }

const bool PerpendicularParkScenario::CheckDynamicUpdate() { return false; }

const bool PerpendicularParkScenario::CheckReplan() { return false; }

const bool PerpendicularParkScenario::CheckFinished() { return false; }

const bool PerpendicularParkScenario::PostProcessPathAccordingLimiter() {
  return false;
}

const bool PerpendicularParkScenario::PostProcessPathAccordingObs(
    const double car_remain_dist) {
  return false;
}

void PerpendicularParkScenario::Log() const {}
}  // namespace apa_planner
}  // namespace planning