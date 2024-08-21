#include "perpendicular_park_planner.h"

namespace planning {
namespace apa_planner {
void PerpendicularParkPlanner::Reset() { frame_.Reset(); }

void PerpendicularParkPlanner::PlanCore() {}

const bool PerpendicularParkPlanner::UpdateEgoSlotInfo() { return false; }

void PerpendicularParkPlanner::GenTlane() {}

void PerpendicularParkPlanner::GenObstacles() {}

const uint8_t PerpendicularParkPlanner::PathPlanOnce() {
  return PathPlannerResult::PLAN_FAILED;
}

const bool PerpendicularParkPlanner::CheckSegCompleted() { return false; }

const bool PerpendicularParkPlanner::CheckUssStucked() { return false; }

const bool PerpendicularParkPlanner::CheckColDetStucked() { return false; }

const bool PerpendicularParkPlanner::CheckDynamicUpdate() { return false; }

const bool PerpendicularParkPlanner::CheckReplan() { return false; }

const bool PerpendicularParkPlanner::CheckFinished() { return false; }

const bool PerpendicularParkPlanner::PostProcessPathAccordingLimiter() {
  return false;
}

const bool PerpendicularParkPlanner::PostProcessPathAccordingObs(
    const double car_remain_dist) {
  return false;
}

void PerpendicularParkPlanner::Log() const {}
}  // namespace apa_planner
}  // namespace planning