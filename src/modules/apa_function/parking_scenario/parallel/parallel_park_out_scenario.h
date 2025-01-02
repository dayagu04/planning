#pragma once

#include <memory>
#include <string>

#include "apa_world/apa_world.h"
#include "parallel_out_path_generator.h"
#include "parking_scenario.h"

namespace planning {
namespace apa_planner {

class ParallelParkOutScenario : public ParkingScenario {
 public:
  ParallelParkOutScenario() = default;
  ParallelParkOutScenario(const std::shared_ptr<ApaWorld>& apa_world_ptr) {
    SetApaWorldPtr(apa_world_ptr);
  }

  virtual void Reset() override;
  virtual void ExcutePathPlanningTask() override;
  virtual const bool GenTlane() override;
  virtual const bool CheckFinished() override;
  virtual const uint8_t PathPlanOnce() override;
  virtual const bool UpdateEgoSlotInfo() override;
  void GenTBoundaryObstacles();

  const ParallelPathGenerator::Tlane& GetTlane() const { return tlane_; }
  const ParallelOutPathGenerator& GetPathPlanner() const {
    return parallel_out_path_planner_;
  }

  ParallelOutPathGenerator* GetMutablePathPlanner() {
    return &parallel_out_path_planner_;
  }

 private:
  void Log() const override;
  const bool CheckReplan() override;
  const bool CheckSegCompleted();
  virtual const bool GenObstacles() override;

 private:
  ParallelOutPathGenerator::Tlane tlane_;
  ParallelOutPathGenerator parallel_out_path_planner_;
};
}  // namespace apa_planner
}  // namespace planning
