#ifndef __PARALLEL_PARK_IN_PLANNER_H__
#define __PARALLEL_PARK_IN_PLANNER_H__

#include <cstdint>
#include <memory>

#include "apa_world.h"
#include "dubins_lib.h"
#include "parallel_path_generator.h"
#include "src/modules/apa_function/parking_scenario/parking_scenario.h"

namespace planning {
namespace apa_planner {

class ParallelParkInScenario : public ParkingScenario {
 public:
  ParallelParkInScenario() = default;
  ParallelParkInScenario(const std::shared_ptr<ApaWorld>& apa_world_ptr) {
    SetApaWorldPtr(apa_world_ptr);
  }

  virtual void Reset() override;
  virtual std::string GetName() override { return typeid(this).name(); };

  const double CalcSlotOccupiedRatio(const Eigen::Vector2d& terminal_err,
                                     const double slot_width,
                                     const bool is_right_side) const;
  virtual const bool UpdateEgoSlotInfo() override;
  virtual const bool GenTlane() override;
  void GenTBoundaryObstacles();
  virtual const uint8_t PathPlanOnce() override;
  const ParallelPathGenerator::Tlane& GetTlane() { return t_lane_; }

  const bool CheckSegCompleted();
  virtual const bool CheckReplan() override;
  virtual const bool CheckFinished() override;

 private:
  // virtual func

  virtual const bool GenObstacles() override;
  virtual void ExcutePathPlanningTask() override;
  virtual void Log() const override;

  void UpdateTlaneOnceInSlot();

  ParallelPathGenerator::Tlane t_lane_;
  ParallelPathGenerator parallel_path_planner_;
};

}  // namespace apa_planner
}  // namespace planning

#endif
