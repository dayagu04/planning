#pragma once

#include "planning_context.h"
#include "session.h"

namespace planning {

class NarrowSpaceHMIDecider {
 public:
  NarrowSpaceHMIDecider(framework::Session*);

  ~NarrowSpaceHMIDecider() = default;

  bool Execute();

 private:
  void InitInfo();

  bool GenerateHMIInfo();

 private:
  framework::Session* session_ = nullptr;
  iflyauto::NSACompleteReason last_complete_reason_;
};
}  // namespace planning