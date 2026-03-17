#pragma once
#include "planning_context.h"
#include "session.h"

namespace planning {
class ConstructionTakeoverHMIDecider {

 public:
  ConstructionTakeoverHMIDecider(framework::Session* session);
  ~ConstructionTakeoverHMIDecider() = default;

  bool Execute();

 private:
  bool HasCipvStaticObs();

  framework::Session* session_;
};
}  // namespace planning