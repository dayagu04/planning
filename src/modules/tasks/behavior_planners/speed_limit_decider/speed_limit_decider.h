#pragma once

#include "ego_planning_config.h"
#include "speed_limit_decider_output.h"
#include "tasks/task.h"
#include "virtual_lane_manager.h"

namespace planning {

class SpeedLimitDecider : public Task {
 public:
  SpeedLimitDecider(const EgoPlanningConfigBuilder *config_builder,
                    framework::Session *session);
  virtual ~SpeedLimitDecider() = default;

  bool Execute() override;

 private:
  void CalculateMapSpeedLimit();

  void CalculateCurveSpeedLimit();

  void CalculateStaticAgentLimit();

  void CalculateIntersectionSpeedLimit();

  void CalculatePerceptVisibSpeedLimit();

  void CalculatePOISpeedLimit();

  // used in curv speed limit
  const std::vector<double> _A_TOTAL_MAX_BP{0., 20., 40.};
  const std::vector<double> _A_TOTAL_MAX_V{1.5, 1.9, 3.2};
  const std::vector<double> _AY_MAX_ABS_BP{5.0, 10.0, 15.0, 30.0};
  const std::vector<double> _AY_MAX_STEERS{2.0, 1.8, 1.6, 1.6};
  const std::vector<double> _AY_MAX_CURV_BP{50, 100, 200, 300, 400};
  const std::vector<double> _AY_MAX_CURV_V{2.2, 1.6, 1.1, 0.9, 0.8};

  SpeedLimitConfig speed_limit_config_;  // all configs
  double v_target_;                      // final v target
  SpeedLimitType v_target_type_;         // final v target type

  // used in intersection speed limit
  planning::common::IntersectionState last_intersection_state_ =
      planning::common::UNKNOWN;
  planning::common::IntersectionState current_intersection_state_ =
      planning::common::UNKNOWN;
  double v_limit_with_intersection_ = 0.0;
};

}  // namespace planning
