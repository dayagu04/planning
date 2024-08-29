#include "hybrid_astar_park_planner.h"

namespace planning {
namespace apa_planner {

void HybridAStarParkPlanner::Reset() { return; }

const bool HybridAStarParkPlanner::CheckReplan() { return false; }

const bool HybridAStarParkPlanner::CheckFinished() { return false; }

void HybridAStarParkPlanner::PlanCore() { return; }

void HybridAStarParkPlanner::Log() const { return; }

void HybridAStarParkPlanner::GenTlane() { return; }

void HybridAStarParkPlanner::GenObstacles() { return; }

const bool HybridAStarParkPlanner::UpdateEgoSlotInfo() { return false; }

const uint8_t HybridAStarParkPlanner::PathPlanOnce() { return false; }

}  // namespace apa_planner
}  // namespace planning