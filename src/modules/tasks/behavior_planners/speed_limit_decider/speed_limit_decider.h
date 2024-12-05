#pragma once

#include "tasks/task.h"

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
};

}  // namespace planning
