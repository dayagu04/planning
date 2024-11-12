#pragma once

#include "apa_data.h"
#include "hybrid_astar_common.h"
#include "parking_task.h"

namespace planning {

// 目前的调用逻辑是：几何规划器第一次规划失败，调用Hybrid astar;
// 如果几何规划中途失败，不再调用A星;

class NarrowScenarioDecider : public ParkingTask{
 public:
  NarrowScenarioDecider() = default;
  ~NarrowScenarioDecider() = default;

  void Process(const uint8_t slot_type, const uint8_t park_task,
               const uint8_t plan_reason, const uint8_t planning_status,
               const apa_planner::ParkingScenarioType scene_type);

  void Clear();

  const bool IsNarrowSpaceScenario() const;

  const bool IsNeedAstar() const;

  static NarrowScenarioDecider* GetNarrowScenarioDecider() {
    static NarrowScenarioDecider astar_scheduler_;

    return &astar_scheduler_;
  }

  void SetAstarState(const AstarSearchState state);

  private:
   void UpdateNarrowScenario(const ParkSpaceType slot_type,
                             const PlanningReason plan_reason);

  private:
   ParkSpaceType slot_type_;
   PlanningReason plan_reason_;
   AstarSearchState astar_search_state_;
   bool is_narrow_space_;
   bool is_need_astar_;
};
}  // namespace planning