#ifndef __PERPENDICULAR_PARK_PLANNER_H__
#define __PERPENDICULAR_PARK_PLANNER_H__

#include "src/modules/apa_function/parking_scenario/parking_scenario.h"

namespace planning {
namespace apa_planner {
class PerpendicularParkScenario : public ParkingScenario {
 public:
  virtual void Reset() override;
  virtual std::string GetName() override { return typeid(this).name(); }

  virtual ~PerpendicularParkScenario() = default;

 protected:
  // virtual func
  virtual const uint8_t PathPlanOnce() override;
  virtual const bool UpdateEgoSlotInfo() override;
  virtual void ExcutePathPlanningTask() override;
  virtual const bool GenTlane() override;
  virtual const bool GenObstacles() override;
  virtual void Log() const override;
  virtual const bool CheckReplan() override;
  virtual const bool CheckFinished() override;

  virtual const bool PostProcessPathAccordingLimiter();
  virtual const bool CheckSegCompleted();
  virtual const bool CheckUssStucked();
  virtual const bool CheckColDetStucked();
  virtual const bool CheckDynamicUpdate();
};
}  // namespace apa_planner
}  // namespace planning

#endif