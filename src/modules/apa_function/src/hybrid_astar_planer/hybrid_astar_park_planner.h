#ifndef __HYBRID_ASTAR_PARK_H__
#define __HYBRID_ASTAR_PARK_H__

#include "apa_plan_base.h"

namespace planning {
namespace apa_planner {

class HybridAStarParkPlanner : public ApaPlannerBase {
 public:
  HybridAStarParkPlanner() = default;
  HybridAStarParkPlanner(const std::shared_ptr<ApaWorld>& apa_world_ptr) {
    SetApaWorldPtr(apa_world_ptr);
  }
  virtual void Reset() override;
  virtual std::string GetName() override { return typeid(this).name(); }

 private:
  virtual const bool CheckReplan() override;
  virtual const bool CheckFinished() override;
  virtual void PlanCore() override;
  virtual void Log() const override;
  virtual void GenTlane() override;
  virtual void GenObstacles() override;
  virtual const bool UpdateEgoSlotInfo() override;
  virtual const uint8_t PathPlanOnce() override;
};

}  // namespace apa_planner
}  // namespace planning

#endif
