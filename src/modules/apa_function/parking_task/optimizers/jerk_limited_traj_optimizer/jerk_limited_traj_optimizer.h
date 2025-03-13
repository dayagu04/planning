#pragma once
#include "parking_task.h"
#include "src/modules/common/speed/speed_data.h"
#include "dp_speed_common.h"
#include "jerk_limited_trajectory.h"
#include "jerk_limited_trajectory_define.h"
#include "optimizer_common.h"

namespace planning {

// ref:Jerk-limited Real-time Trajectory Generation
// with Arbitrary Target States.
class JerkLimitedTrajOptimizer : public ParkingTask {
 public:
  JerkLimitedTrajOptimizer() = default;

  virtual ~JerkLimitedTrajOptimizer() = default;

  bool Init();

  void Execute(const SVPoint& init_point, const double path_len);

  const SpeedOptimizerState GetSolverState() const { return solver_state_; }

  const SpeedData& GetSpeedData() const { return speed_data_; }

 private:
  // use sampling method to generate speed.
  // todo: use adaptive acc boundary to generate speed.
  void UpdateSolver(const SVPoint& init_point, const double s_des,
                    const double v_des);

  bool SamplingAccLowerBound(const planning::jlt::PointState& init_point,
                             const planning::jlt::StateLimitParam& state_limit,
                             jlt::JerkLimitedTrajectory* solver);

  void CopySpeedData(jlt::JerkLimitedTrajectory* solver);

  void DebugJLTSpeed(jlt::JerkLimitedTrajectory* solver,
                     const planning::jlt::StateLimitParam* constraints);

  void UpdateFallbackTraj(const SVPoint& init_point, const double s_des);

  void RecordDebugInfo();

 private:
  SpeedOptimizerState solver_state_;

  SpeedData speed_data_;

  double delta_time_;
};
}  // namespace planning