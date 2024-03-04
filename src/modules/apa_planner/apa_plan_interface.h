#ifndef __APA_PLAN_INTERFACE_H__
#define __APA_PLAN_INTERFACE_H__

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "apa_plan_base.h"
#include "apa_world.h"
#include "frame.h"
#include "local_view.h"
#include "plan_interface_base.h"

namespace planning {
namespace apa_planner {

class ApaPlanInterface : public plan_interface::PlanInterfaceBase {
 public:
  virtual void Init(
      const std::shared_ptr<plan_interface::PlanData> plan_data_ptr) override;

  virtual void Reset() override;

  virtual const bool Update(
      const std::shared_ptr<LocalView> local_view_ptr) override;

  virtual void SyncParameters() override;

  const std::vector<std::shared_ptr<ApaPlannerBase>>& GetPlannerStack() const {
    return apa_planner_stack_;
  }

  // only for simulation
  void UpdateDebugInfo();

 private:
  std::shared_ptr<ApaPlannerBase> GetPlannerByType(
      const uint8_t apa_planner_id);

  const bool ApaPlanOnce(const uint8_t planner_type);
  void AddReleasedSlotInfo(PlanningOutput::PlanningOutput& planning_output);

  std::vector<std::shared_ptr<ApaPlannerBase>> apa_planner_stack_;
  std::shared_ptr<ApaWorld> apa_world_ptr_ = nullptr;
  std::shared_ptr<ApaPlannerBase> planner_ptr_ = nullptr;
};

}  // namespace apa_planner
}  // namespace planning

#endif
