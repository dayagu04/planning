#pragma once

#include "modules/context/planning_context.h"

namespace planning {

class SplitSelectHmiDecider {
 public:
  SplitSelectHmiDecider() = default;

  explicit SplitSelectHmiDecider(framework::Session* session);
  ~SplitSelectHmiDecider() = default;

  bool Execute();

 private:
  void UpdateHMIInfo();

  void UpdateIntersection();
  
 private:
  framework::Session* session_ = nullptr;
  bool ego_in_intersection_state_ = false;
  int intersection_count_ = 0;
};

}  // namespace planning