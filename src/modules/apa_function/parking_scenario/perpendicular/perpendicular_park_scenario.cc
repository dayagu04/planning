#include "perpendicular_park_scenario.h"

namespace planning {
namespace apa_planner {
void PerpendicularParkScenario::Reset() {
  frame_.Reset();
  ParkingScenario::Reset();
}

void PerpendicularParkScenario::ExcutePathPlanningTask() {}

const bool PerpendicularParkScenario::UpdateEgoSlotInfo() { return false; }

const bool PerpendicularParkScenario::GenTlane() { return true; }

const bool PerpendicularParkScenario::GenObstacles() { return true; }

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

void PerpendicularParkScenario::Log() const {}
}  // namespace apa_planner
}  // namespace planning