#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include "parking_scenario.h"
#include "planning_hmi_c.h"
#include "planning_plan_c.h"

namespace planning {
namespace apa_planner {

// todo: move all scenario manager related in here.

enum class ParkingScenarioType {
  SCENARIO_UNKNOWN = 0,
  SCENARIO_PERPENDICULAR_TAIL_IN = 1,
  SCENARIO_PERPENDICULAR_HEAD_IN = 2,
  SCENARIO_PERPENDICULAR_TAIL_OUT = 3,
  SCENARIO_PERPENDICULAR_HEAD_OUT = 4,
  SCENARIO_PARALLEL_IN = 5,
  SCENARIO_PARALLEL_OUT = 6,
  SCENARIO_SLANT_TAIL_IN = 7,
  SCENARIO_SLANT_HEAD_IN = 8,
  SCENARIO_SLANT_TAIL_OUT = 9,
  SCENARIO_SLANT_HEAD_OUT = 10,
  // todo, remove narrow space scenario
  SCENARIO_NARROW_SPACE = 11,
};

void PrintApaScenarioType(const ParkingScenarioType scenario_type);

const std::string GetApaScenarioTypeString(
    const ParkingScenarioType scenario_type);

class ParkingScenarioManager final {
 public:
  bool Init(const std::shared_ptr<ApaWorld>& apa_world);

  void Excute();

  void Process();

  void Reset();

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

 private:
  // if user select a slot id, autonomous system will call this to try plan.
  void ScenarioTry();

  void ScenarioRunning();

  const bool IsSlotReleaseByHybridAstar();

 private:
  iflyauto::PlanningOutput planning_output_;
  iflyauto::APAHMIData apa_hmi_data_;
  std::shared_ptr<ParkingScenario> current_scenario_;
  ParkingScenarioType scenario_type_;
  ParkingScenarioStatus scenario_status_;

  std::unordered_map<ParkingScenarioType, std::shared_ptr<ParkingScenario>>
      scenario_list_;

  bool init_ = false;
  std::shared_ptr<ApaWorld> apa_world_;
};
}  // namespace apa_planner
}  // namespace planning
