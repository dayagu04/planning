#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include "parking_scenario.h"
#include "src/modules/apa_function/apa_world/apa_data.h"

namespace planning {
namespace apa_planner {

// todo: move all scenario manager related in here.
class ParkingScenarioManager final {
 public:
  bool Init(const std::shared_ptr<ApaWorld>& apa_world);

  ParkingScenarioStatus Excute(std::shared_ptr<apa_planner::ApaData> frame);

  void Reset();

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
  void ScenarioTry();

 private:
  std::shared_ptr<ParkingScenario> current_scenario_;
  std::shared_ptr<ParkingScenario> default_scenario_;
  ParkingScenarioType type_;

  std::unordered_map<ParkingScenarioType, std::shared_ptr<ParkingScenario>>
      scenario_list_;

  bool init_ = false;
  std::shared_ptr<ApaWorld> apa_world_;
};
}  // namespace apa_planner
}  // namespace planning
