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
#include "planning_debug_info.pb.h"
#include "planning_def.h"
#include "planning_plan.pb.h"

namespace planning {
namespace apa_planner {

class ApaPlanInterface {
 public:
  void Init();
  void Reset();
  const bool Update(const LocalView* local_view_ptr);

  // an isolation between pybind (simulation) and real test
  const bool UpdateFrame(framework::Frame* const frame);

  const std::vector<std::shared_ptr<ApaPlannerBase>>& GetPlannerStack() const {
    return apa_planner_stack_;
  }

  const PlanningOutput::PlanningOutput GetPlaningOutput() const {
    if (planner_ptr_ != nullptr) {
      return planner_ptr_->GetOutput();
    } else {
      std::cout << "planner_ptr_ is null!" << std::endl;
      PlanningOutput::PlanningOutput planning_output;
      planning_output.Clear();

      return planning_output;
    }
  }

  // only for simulation
  void UpdateDebugInfo();

  const planning::common::PlanningDebugInfo& GetPlanningDebugInfo() const {
    return planning_debug_info_;
  }

 private:
  std::shared_ptr<ApaPlannerBase> GetPlannerByType(
      const uint8_t apa_planner_id);

  const bool ApaPlanOnce(const uint8_t planner_type);
  void AddReleasedSlotInfo(PlanningOutput::PlanningOutput& planning_output);

  std::vector<std::shared_ptr<ApaPlannerBase>> apa_planner_stack_;
  std::shared_ptr<ApaWorld> apa_world_ptr_ = nullptr;
  std::shared_ptr<ApaPlannerBase> planner_ptr_ = nullptr;

  const LocalView* local_view_ptr_ = nullptr;  // note that pointer to constant

  // only for simulation
  planning::common::PlanningDebugInfo planning_debug_info_;
};

}  // namespace apa_planner
}  // namespace planning

#endif
