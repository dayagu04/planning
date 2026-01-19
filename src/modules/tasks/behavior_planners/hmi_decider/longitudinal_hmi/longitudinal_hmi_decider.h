
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
  framework::Session* session_ = nullptr;
  HmiDeciderConfig config_;

  std::pair<int32_t, int32_t> lon_collision_count_{0, 0};
};

}  // namespace planning
