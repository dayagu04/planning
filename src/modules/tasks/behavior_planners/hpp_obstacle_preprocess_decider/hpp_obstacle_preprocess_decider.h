#pragma once

#include <memory>
#include <vector>

#include "tasks/task.h"
#include "ego_planning_config.h"
#include "session.h"

namespace planning {

class Obstacle;

namespace agent {
class Agent;
}  // namespace agent

class HppObstaclePreprocessDecider : public Task {
 public:
  HppObstaclePreprocessDecider(const EgoPlanningConfigBuilder *config_builder,
                               framework::Session *session);
  virtual ~HppObstaclePreprocessDecider() = default;

  bool Execute() override;

 private:
  void ProcessGroundLines();
  void ProcessDynamicObstacles();

  void CreateVirtualAgentFromGroundLine(const Obstacle *obstacle, double s,
                                        double l, int id);
  void GenerateConstantVelocityTrajectory(agent::Agent *agent);

  const double kGroundLineVirtualAgentLength = 0.5;
  const double kGroundLineVirtualAgentWidth = 2.0;
  const double kDynamicObstacleSpeedThreshold = 1.0;  // m/s
  const double kPredictionHorizon = 5.0;              // s
  const double kTimeResolution = 0.2;                 // s
};

}  // namespace planning
