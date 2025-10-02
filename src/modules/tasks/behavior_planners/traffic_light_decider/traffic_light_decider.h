#pragma once

#include <memory>
#include <vector>
#include "debug_info_log.h"
#include "planning_context.h"
#include "tasks/task.h"
#include "traffic_light_decision_manager.h"

namespace planning {

class TrafficLightDecider : public Task {
 public:
  explicit TrafficLightDecider(const EgoPlanningConfigBuilder *config_builder,
                               framework::Session *session);

  virtual ~TrafficLightDecider() = default;

  // 1. if
  bool Execute();

  const bool& GetIsSmallFrontIntersection() { return is_small_front_intersection_;}

 private:
  // add virtual agent to agent manager
  bool AddVirtualObstacle();

  // intersection before is small intersection or not
  bool IsSmallFrontIntersection();

  // small intersection is matchable with tfl or not
  bool IsIntersectionMatchTFL();

  // have a risk of running a red light or not when human drive
  bool IsRunningRedTFL();

  // have a risk of staying still when green light on or not when human drive
  bool IsStayingStillGreenTFL();

  TrafficLightDeciderConfig config_;
  bool is_first_car_ = false;
  // bool is_pass_stopline_ = false;
  bool can_pass_ = true;
  double green_light_timer_ = 0.0;
  double green_blink_timer_ = 0.0;
  double yellow_light_timer_ = 0.0;
  bool is_small_front_intersection_ = false;
};
}  // namespace planning