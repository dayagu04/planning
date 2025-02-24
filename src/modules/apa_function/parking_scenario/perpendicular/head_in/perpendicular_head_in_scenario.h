#ifndef __PERPENDICULAR_PARK_HEADING_IN_PLANNER_H__
#define __PERPENDICULAR_PARK_HEADING_IN_PLANNER_H__

#include "perpendicular_head_in_path_generator.h"
#include "perpendicular_park_scenario.h"

namespace planning {
namespace apa_planner {

class PerpendicularHeadInScenario : public PerpendicularParkScenario {
 public:
  PerpendicularHeadInScenario() = default;
  PerpendicularHeadInScenario(const std::shared_ptr<ApaWorld>& apa_world_ptr) {
    SetApaWorldPtr(apa_world_ptr);
  }
  virtual void Reset() override;
  virtual std::string GetName() override { return typeid(this).name(); }

 private:
  void ExcutePathPlanningTask() override;
  const bool GenTlane() override;
  const bool GenObstacles() override;
  virtual void Log() const override;
  const bool UpdateEgoSlotInfo() override;
  const uint8_t PathPlanOnce() override;
  void RealTimeDynamicColDet(const EgoInfoUnderSlot& ego_slot_info);

  // virtual func
  virtual const double CalRemainDistFromPath() override;
  virtual const bool CheckFinished() override;
  virtual const bool PostProcessPath() override;
  const bool PostProcessPathAccordingLimiter() override;
 private:
  PerpendicularPathHeadingInPlanner perpendicular_path_planner_;

  std::vector<pnc::geometry_lib::PathSegment> current_plan_path_vec_;
};

}  // namespace apa_planner
}  // namespace planning

#endif
