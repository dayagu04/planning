#pragma once

#include <memory>
#include <vector>
#include "debug_info_log.h"
#include "tasks/task.h"
#include "traffic_light_decision_manager.h"

namespace planning {

class TrafficLightDecider : public Task {
 public:
  explicit TrafficLightDecider(const EgoPlanningConfigBuilder *config_builder,
                                 framework::Session *session);

  virtual ~TrafficLightDecider() = default;
  
  //1. if 
  bool Execute();

 private:
  
  //add virtual agent to agent manager
  bool AddVirtualObstacle(); 

  TrafficLightDeciderConfig config_;
  bool is_first_car_ = false;
  bool is_pass_stopline_ = false;

};
}