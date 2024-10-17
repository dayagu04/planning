#pragma once

#include <bits/stdint-uintn.h>
#include "hybrid_astar_common.h"

namespace planning {

class AstarScheduler {
 public:
  AstarScheduler() = default;
  ~AstarScheduler() = default;

  void UpdateScheduler(const ParkSpaceType slot_type,
                       const ParkingTask park_task,
                       const PlanningReason plan_reason);

  void Process(const uint8_t slot_type, const uint8_t park_task,
               const uint8_t plan_reason, const uint8_t planning_status,
               const uint8_t planner_type);

  void Clear();

  const bool IsNeedAstarSearch() const;

  static AstarScheduler* GetAstarScheduler() {
    static AstarScheduler astar_scheduler_;

    return &astar_scheduler_;
  }

  void SetSchedulerState(const AstarSearchState state);

 private:
  ParkSpaceType slot_type_;
  ParkingTask park_task_;
  PlanningReason plan_reason_;
  AstarSearchState astar_search_state_;
  bool need_astar_search_;
};
}  // namespace planning