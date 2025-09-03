#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include "parking_scenario.h"
#include "planning_hmi_c.h"
#include "planning_plan_c.h"

namespace planning {
namespace apa_planner {

class ParkingScenarioManager final {
 public:
  bool Init(const std::shared_ptr<ApaWorld>& apa_world);

  void UpdateScenarioType();

  void Process();

  // slot cruise state: reset.
  void Reset();

  void ClearHistoryPreparePlanTraj() {
    history_prepare_plan_traj_.trajectory_points_size = 0;
    history_prepare_plan_traj_.available = false;
  }

  const iflyauto::PlanningOutput& GetPlanningOutput() const {
    return planning_output_;
  }

  const iflyauto::APAHMIData& GetAPAHmiData() const { return apa_hmi_data_; }

  const ParkingScenarioType GetScenarioType() const { return scenario_type_; }

  const ParkingScenarioStatus GetScenarioStatus() const {
    return scenario_status_;
  }

  std::shared_ptr<ParkingScenario> GetScenarioByType(
      const ParkingScenarioType type);

  ParkingScenario* MutableScenario() {
    if (current_scenario_ != nullptr) {
      return current_scenario_.get();
    }

    return nullptr;
  }

  std::shared_ptr<ParkingScenario> MutableScenarioPtr() {
    return current_scenario_;
  }

  void PubPreparePlanState();

  void PublishPreparePlanInfo();

  void RecommendParkingDirection();
  void PubStopReason();

 private:
  // if user select a slot id, autonomous system will call this to try plan.
  void ScenarioTry();

  void ScenarioRunning();

  const bool IsSlotReleaseByHybridAstar();
  void ClearPlanningOutput();

  // Do not publish path to HMI per frame for stable display.
  // If path is changed too much, publish it.
  const bool PubPreparePathByStableStrategy();

  void ScenarioSuspend();

 private:
  // reset if scenario is null or slot is not release.
  iflyauto::PlanningOutput planning_output_;
  iflyauto::APAHMIData apa_hmi_data_;
  std::shared_ptr<ParkingScenario> current_scenario_;
  ParkingScenarioType scenario_type_;
  ParkingScenarioStatus scenario_status_;

  iflyauto::Trajectory history_prepare_plan_traj_;

  std::unordered_map<ParkingScenarioType, std::shared_ptr<ParkingScenario>>
      scenario_list_;

  bool init_ = false;
  std::shared_ptr<ApaWorld> apa_world_;
};
}  // namespace apa_planner
}  // namespace planning
