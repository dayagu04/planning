#pragma once

#include "apa_slot.h"
#include "hybrid_astar_common.h"
#include "parking_task.h"

namespace planning {

// 目前的调用逻辑是：点击车位后，进行预规划，几何规划器失败，才调用Hybrid astar预规划;
// 根据预规划结果，在正式泊车中选择几何还是A星;
// 暂时只有车尾入库会调用A star, 其他场景不会调用A star.
class NarrowScenarioDecider : public ParkingTask {
 public:
  NarrowScenarioDecider() = default;
  ~NarrowScenarioDecider() = default;

  void Process(const apa_planner::SlotType slot_type);

  void Clear();

  const bool IsNarrowSpaceScenario() const;

  const bool IsNeedAstar() const;

  void SetAstarState(const AstarSearchState state);

  const AstarSearchState GetAstarState() { return astar_search_state_; }

 private:
  void UpdateNarrowScenario(const ParkSpaceType slot_type);

 private:
  ParkSpaceType slot_type_;
  AstarSearchState astar_search_state_;
  bool is_narrow_space_;
  bool is_need_astar_;
};

}  // namespace planning