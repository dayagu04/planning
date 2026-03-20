#pragma once

#include "planning_context.h"
#include "session.h"

namespace planning {

class ObstacleBrakeHMIDecider {
 public:
 ObstacleBrakeHMIDecider(framework::Session* session, const HmiDeciderConfig& config);

  ~ObstacleBrakeHMIDecider() = default;

  bool Execute();

 private:

  bool GenerateHMIInfoForRADS();

 private:
  framework::Session* session_ = nullptr;
  HmiDeciderConfig config_;
};
}  // namespace planning