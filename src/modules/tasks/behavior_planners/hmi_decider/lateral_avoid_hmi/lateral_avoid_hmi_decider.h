#pragma once

#include "planning_context.h"
#include "session.h"

namespace planning {

class LateralAvoidHMIDecider {
 public:
  LateralAvoidHMIDecider(framework::Session*);

  ~LateralAvoidHMIDecider() = default;

  bool Execute();

 private:
  void InitInfo();

  bool GenerateHMIInfoForRADS();

 private:
  framework::Session* session_ = nullptr;
};
}  // namespace planning