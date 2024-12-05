#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "apa_data.h"
#include "parking_scenario/parking_scenario.h"
#include "apa_world.h"
#include "local_view.h"
#include "plan_data.h"
#include "planning_hmi_c.h"
#include "session.h"
#include "parking_scenario/parking_scenario_manager.h"

namespace planning {
namespace apa_planner {

class ApaPlanInterface {
 public:
  void Init(const bool is_simulation = false);

  void Reset();

  const bool Update(const LocalView* local_view_ptr);

  // only for simulation
  void UpdateDebugInfo();

  const planning::common::LateralPathOptimizerOutput& GetOutputDebugInfo()
      const {
    return apa_world_ptr_->GetLateralPathOptimizerPtr()->GetOutputDebugInfo();
  }

  const planning::common::LateralPathOptimizerInput& GetInputDebugInfo() const {
    return apa_world_ptr_->GetLateralPathOptimizerPtr()->GetInputDebugInfo();
  }

  const planning::common::PlanningDebugInfo& GetPlanningDebugInfo() const {
    return planning_debug_info_;
  }

  const iflyauto::PlanningOutput& GetPlaningOutput() const {
    return planning_output_;
  }
  const iflyauto::APAHMIData& GetAPAHmi() const { return apa_hmi_; }

  void SetSimuParam(const SimulationParam& param) {
    apa_world_ptr_->GetApaDataPtr()->simu_param = param;
  }

  std::shared_ptr<ParkingScenario> GetPlannerByType(
      const ParkingScenarioType type);

 private:
  void AddReleasedSlotInfo(iflyauto::PlanningOutput& planning_output);

  void RecordNodeReceiveTime(const LocalView* local_view_ptr);

  // environment information
  std::shared_ptr<ApaWorld> apa_world_ptr_ = nullptr;

  // scenario manager information
  ParkingScenarioManager scenario_manager_;

  iflyauto::PlanningOutput planning_output_;
  iflyauto::APAHMIData apa_hmi_;

  // for simulation
  planning::common::PlanningDebugInfo planning_debug_info_;
};

}  // namespace apa_planner
}  // namespace planning
