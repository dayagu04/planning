#pragma once

#include <memory>
#include <string>

#include "./../parking_scenario.h"
#include "apa_world/apa_world.h"
#include "parallel_out_path_generator.h"

namespace planning {

namespace apa_planner {

class ParallelParkOutScenario : public ParkingScenario {
 public:
  ParallelParkOutScenario() = default;
  ParallelParkOutScenario(const std::shared_ptr<ApaWorld>& apa_world_ptr) {
    SetApaWorldPtr(apa_world_ptr);
  }

  virtual void Reset() override;

  virtual void PlanCore() override;
  virtual void GenTlane() override;
  virtual const bool CheckFinished() override;
  virtual const uint8_t PathPlanOnce() override;
  virtual const bool UpdateEgoSlotInfo() override;

  void GenTBoundaryObstacles();
  const ParallelPathGenerator::Tlane& GetTlane() { return tlane_; }
  const ParallelOutPathGenerator& GetPathPlanner() {
    return parallel_out_path_planner_;
  }

  ParallelOutPathGenerator& SetPathPlanner() {
    return parallel_out_path_planner_;
  }

 private:
  const bool CheckReplan() override;
  const bool CheckSegCompleted();
  void Log() const override;
  virtual void GenObstacles() override;

  ParallelOutPathGenerator::Tlane tlane_;
  ParallelOutPathGenerator parallel_out_path_planner_;
};
}  // namespace apa_planner
}  // namespace planning
