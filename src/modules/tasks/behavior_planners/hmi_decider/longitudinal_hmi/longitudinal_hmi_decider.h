
#pragma once
#include "modules/context/planning_context.h"
#include "framework/session.h"
namespace planning {
class LongitudinalHmiDecider {
 public:
  LongitudinalHmiDecider() = default;
  LongitudinalHmiDecider(framework::Session* session, const HmiDeciderConfig& config);
  bool Execute();

 private:
  void IntersectionLeftRightLaneTakeOverProc();
  framework::Session* session_ = nullptr;
  HmiDeciderConfig config_;
};

}  // namespace planning
