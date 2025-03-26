#pragma once
#include "ego_planning_config.h"
#include "local_view.h"
#include "session.h"
#include <queue>
namespace planning {

class TrafficLightDecisionManager {
 public:
  TrafficLightDecisionManager(const EgoPlanningConfigBuilder *config_builder,
                              planning::framework::Session *session);

  ~TrafficLightDecisionManager(){};

  void SetConfig(const EgoPlanningConfigBuilder *config_builder);

  bool Update(const iflyauto::CameraPerceptionTsrInfo &tsr_info);

  const iflyauto::CameraPerceptionTrafficStatus &GetTrafficStatus() {
    return traffic_status_;
  };

  const std::vector<iflyauto::CameraPerceptionTrafficLight>
      &GetTrafficLightsInfo() {
    return traffic_lights_info_;
  };

  const double GetNearestTFLDis() {
    return distance_to_nearest_tfl_;
  }

 private:
  bool UpdateNearestTFLDis();

  planning::framework::Session *session_ = nullptr;
  EgoPlanningTrafficLightDecisionManagerConfig config_;

  iflyauto::CameraPerceptionTrafficStatus traffic_status_;
  std::vector<iflyauto::CameraPerceptionTrafficLight> traffic_lights_info_;
  double distance_to_nearest_tfl_ = NL_NMAX;
  std::deque<double> nearest_tfl_dis_window_ = {NL_NMAX, NL_NMAX, NL_NMAX};
};
}  // namespace planning