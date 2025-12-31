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

 private:
  framework::Session* session_ = nullptr;
};

}  // namespace planning