#include "traffic_light_decision_manager.h"
#include "debug_info_log.h"

namespace planning {

TrafficLightDecisionManager::TrafficLightDecisionManager(
    const EgoPlanningConfigBuilder* config_builder,
    planning::framework::Session* session) {
  session_ = session;
  SetConfig(config_builder);
}

void TrafficLightDecisionManager::SetConfig(
    const EgoPlanningConfigBuilder* config_builder) {
  config_ =
      config_builder->cast<EgoPlanningTrafficLightDecisionManagerConfig>();
}

bool TrafficLightDecisionManager::Update(
    const iflyauto::CameraPerceptionTsrInfo& tsr_info) {
  traffic_lights_info_.clear();
  int traffic_lights_num = tsr_info.traffic_lights_size;
  for (int i = 0; i < traffic_lights_num; i++) {
    traffic_lights_info_.emplace_back(tsr_info.traffic_lights[i]);
  }

  traffic_status_ = tsr_info.traffic_status;
  JSON_DEBUG_VALUE("traffic_status_straight", int(traffic_status_.go_straight));

  UpdateNearestTFLDis();

  return true;
}

bool TrafficLightDecisionManager::UpdateNearestTFLDis() {
  double ego_to_nearest_tfl_dis = NL_NMAX;
  for (int i = 0; i < traffic_lights_info_.size(); i++) {
    if (traffic_lights_info_[i].traffic_light_x > 0 &&
        traffic_lights_info_[i].traffic_light_x < ego_to_nearest_tfl_dis) {
      ego_to_nearest_tfl_dis = traffic_lights_info_[i].traffic_light_x;
    }
  }

  nearest_tfl_dis_window_.pop_front();
  nearest_tfl_dis_window_.push_back(ego_to_nearest_tfl_dis);

  if (nearest_tfl_dis_window_[0] < 200.0 && nearest_tfl_dis_window_[1] < 200.0 &&
    nearest_tfl_dis_window_[2] < 200.0) {
    distance_to_nearest_tfl_ = ego_to_nearest_tfl_dis;
  } else if (nearest_tfl_dis_window_[0] > 200.0 && nearest_tfl_dis_window_[1] > 200.0 &&
    nearest_tfl_dis_window_[2] > 200.0) {
    distance_to_nearest_tfl_ = NL_NMAX;
  }
  return true;
}

}  // namespace planning
