#pragma once

#include <memory>
#include <vector>

#include "tasks/task.h"
#include "ego_planning_config.h"
#include "session.h"

namespace planning {

class Obstacle;
struct TrajectoryPoint;

namespace planning_math {
class Vec2d;
}  // namespace planning_math

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

  void CreateVirtualAgentFromGroundLine(
      const std::vector<planning_math::Vec2d> &hit_points,
      const TrajectoryPoint &anchor_traj_point, int id);
  void GenerateCurrentPoseTrajectory(agent::Agent *agent);

  const double kGroundLineVirtualAgentLength = 0.5;
  const double kGroundLineVirtualAgentWidth = 2.0;
  const double kPredictionHorizon = 5.0;              // s
  const double kTimeResolution = 0.2;                 // s
  LongitudinalDeciderV3Config lon_config_;
};

}  // namespace planning
