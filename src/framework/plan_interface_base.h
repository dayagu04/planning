#ifndef __PLAN_INTERFACE_BASE_H__
#define __PLAN_INTERFACE_BASE_H__

#include "frame.h"
#include "local_view.h"
#include "plan_data.h"
#include "planning_debug_info.pb.h"
#include "planning_def.h"
#include "planning_plan.pb.h"

namespace planning {
namespace plan_interface {

class PlanInterfaceBase {
 public:
  virtual void Init(
      const std::shared_ptr<plan_interface::PlanData> plan_data_ptr) = 0;

  virtual void Reset() = 0;

  virtual const bool Update(
      const std::shared_ptr<LocalView> local_view_ptr) = 0;

  virtual void SyncParameters() = 0;

  const planning::common::PlanningDebugInfo& GetPlanningDebugInfo() const {
    return planning_debug_info_;
  }

  const PlanningOutput::PlanningOutput& GetPlaningOutput() const {
    return planning_output_;
  }

 protected:
  PlanningOutput::PlanningOutput planning_output_;
  std::shared_ptr<plan_interface::PlanData> plan_data_ptr_;

  // for simulation
  planning::common::PlanningDebugInfo planning_debug_info_;
};

}  // namespace plan_interface
}  // namespace planning

#endif
