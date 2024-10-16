#pragma once

#include "hybrid_astar_common.h"

namespace planning {

class AstarScheduler {
 public:
  AstarScheduler() = default;
  ~AstarScheduler() = default;

  void Process(const ParkSpaceType slot_type, const ParkingTask park_task,
               const PlanningReason plan_reason);

 private:
  ParkSpaceType slot_type_;
  ParkingTask park_task_;
  PlanningReason plan_reason_;
};
}