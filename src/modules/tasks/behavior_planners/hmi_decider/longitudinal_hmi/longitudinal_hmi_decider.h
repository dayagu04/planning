
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

  std::pair<int32_t, int32_t> lon_collision_count_{0, 0};
  bool intersection_red_light_stop_active_ = false;
  int32_t intersection_red_light_stop_exit_count_ = 0;
  bool intersection_green_light_go_active_ = false;
  int32_t intersection_green_light_go_exit_count_ = 0;
};

}  // namespace planning
