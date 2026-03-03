#pragma once

#include <cstdint>

#include "agent/agent.h"
#include "define/geometry.h"
#include "ego_planning_config.h"
#include "hpp_speed_limit_decider_output.h"
#include "tasks/task.h"
#include "tasks/task_interface/crossing_agent_decider_output.h"
#include "traffic_light_decision_manager.h"

namespace planning {
class HPPSpeedLimitDecider : public Task {
 public:
  HPPSpeedLimitDecider(const EgoPlanningConfigBuilder* config_builder,
                       framework::Session* session);
  virtual ~HPPSpeedLimitDecider() = default;
  bool Execute() override;

 private:
  void CalculateUserSpeedLimit();
  void CalculateMapSpeedLimit();
  void CalculateCurveSpeedLimit();
  void CalculateNarrowAreaSpeedLimit();
  void CalculateAvoidLimit();

  const double ComputeMaxLatAcceleration();
  const double ComputeCurvatureSpeedLimit(const TrajectoryPoints& traj_points,
                                          double ego_velocity,
                                          double max_lat_acceleration,
                                          double& vlimit_jerk,
                                          double& time_to_brake,
                                          double& out_max_curvature);

  const double ComputeAvoidVelocityLimit(const framework::Session& session);

 private:
  LongitudinalDeciderV3Config
      hpp_speed_limit_config_;    // Temporarily use the original version of
                                  // config
  double v_target_;               // final v target
  SpeedLimitType v_target_type_;  // final v target type

  double max_curvature_;
};
}  // namespace planning