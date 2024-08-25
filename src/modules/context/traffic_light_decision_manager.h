#pragma once
#include "ego_planning_config.h"
#include "local_view.h"
#include "session.h"

namespace planning {

class TrafficLightDecisionManager {
 public:
  TrafficLightDecisionManager(const EgoPlanningConfigBuilder *config_builder,
                              planning::framework::Session *session);

  ~TrafficLightDecisionManager(){};

  bool Update(const iflyauto::CameraPerceptionTsrInfo &tsr_info);

  const iflyauto::CameraPerceptionTrafficStatus &GetTrafficStatus() {
    return traffic_status_;
  };

  const std::vector<iflyauto::CameraPerceptionTrafficLight>
      &GetTrafficLightsInfo() {
    return traffic_lights_info_;
  };

 private:
  planning::framework::Session *session_ = nullptr;
  EgoPlanningTrafficLightDecisionManagerConfig config_;

  iflyauto::CameraPerceptionTrafficStatus traffic_status_;
  std::vector<iflyauto::CameraPerceptionTrafficLight> traffic_lights_info_;
};
}  // namespace planning