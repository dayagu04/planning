#include "traffic_light_decision_manager.h"
#include "debug_info_log.h"

namespace planning {

TrafficLightDecisionManager::TrafficLightDecisionManager(
    const EgoPlanningConfigBuilder* config_builder,
    planning::framework::Session* session) {
  session_ = session;
  config_ = config_builder->cast<EgoPlanningTrafficLightDecisionManagerConfig>();
}

bool TrafficLightDecisionManager::Update(const iflyauto::CameraPerceptionTsrInfo &tsr_info) {
  traffic_lights_info_.clear();
  int traffic_lights_num = tsr_info.traffic_light_size;
  for(int i = 0; i < traffic_lights_num; i++) {
    traffic_lights_info_.emplace_back(tsr_info.traffic_lights[i]);
  }

  traffic_status_ = tsr_info.traffic_status;
  JSON_DEBUG_VALUE("traffic_status_straight", int(traffic_status_.go_straight));

  return true;
}

}
