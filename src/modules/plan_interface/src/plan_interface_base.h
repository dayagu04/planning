#ifndef __PLAN_INTERFACE_BASE_H__
#define __PLAN_INTERFACE_BASE_H__

#include "frame.h"
#include "local_view.h"
#include "planning_debug_info.pb.h"
#include "planning_def.h"
#include "planning_plan.pb.h"

namespace planning {
namespace plan_interface {

class PlanInterfaceBase {
 public:
  virtual void Init() = 0;
  virtual void Reset() = 0;
  virtual const bool Update(const LocalView* local_view_ptr) = 0;
  virtual void SyncParameters() = 0;
  // an isolation between pybind (simulation) and real test
  virtual const bool UpdateFrame(framework::Frame* const frame) = 0;

  const planning::common::PlanningDebugInfo& GetPlanningDebugInfo() const {
    return planning_debug_info_;
  }

  const PlanningOutput::PlanningOutput& GetPlaningOutput() const {
    return planning_output_;
  }

  const LocalView* GetLocalViewPtr() { return local_view_ptr_; }

 protected:
  const LocalView* local_view_ptr_ = nullptr;  // note that pointer to constant

  // only for simulation
  planning::common::PlanningDebugInfo planning_debug_info_;
  PlanningOutput::PlanningOutput planning_output_;
};

}  // namespace plan_interface
}  // namespace planning

#endif
