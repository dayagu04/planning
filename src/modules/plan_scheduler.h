#ifndef __PLAN_SCHEDULER_H__
#define __PLAN_SCHEDULER_H__

#include <cstdint>
#include <memory>
#include <vector>

#include "plan_data.h"
#include "plan_interface_base.h"
namespace planning {
namespace plan_scheduler {

class PlanScheduler {
 public:
  void Init(const std::shared_ptr<LocalView> local_view_ptr);

  void Reset();

  const bool Update();

  // tmp
  const bool IsApa() const;

  const std::shared_ptr<plan_interface::PlanData> GetPlanDataPtr() const {
    return plan_data_ptr_;
  }

 private:
  // tmp
  // const bool IsApa() const;

  std::shared_ptr<plan_interface::PlanData> plan_data_ptr_;

  std::vector<std::shared_ptr<plan_interface::PlanInterfaceBase>>
      planner_interface_stack_;

  std::shared_ptr<LocalView> local_view_ptr_;
};

}  // namespace plan_scheduler
}  // namespace planning

#endif
