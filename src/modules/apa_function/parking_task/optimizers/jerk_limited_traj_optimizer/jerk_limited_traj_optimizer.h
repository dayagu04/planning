#pragma once
#include "dp_speed_common.h"
#include "jerk_limited_traj_optimizer/jerk_limited_traj_config.h"
#include "jerk_limited_trajectory.h"
#include "jerk_limited_trajectory_define.h"
#include "parking_task.h"
#include "speed/apa_speed_decision.h"
#include "src/modules/common/speed/speed_data.h"

namespace planning {
namespace apa_planner {

enum class JLTFailCode {
  NONE = 0,
  SHORT_PATH = 1,
  SOLVER_FAIL,
  NEGATIVE_SPEED,
  PATH_OVERSHOOT,
};

// ref:Jerk-limited Real-time Trajectory Generation
// with Arbitrary Target States.
// JLT planner is an absolutely safe speed planning task.
// It must return a success solution.
// If control is overshoot, this module must generate a stopping trajectory.
class JerkLimitedTrajOptimizer : public ParkingTask {
 public:
  JerkLimitedTrajOptimizer() = default;

  virtual ~JerkLimitedTrajOptimizer() = default;

  bool Init() override;

  /**
   * [in]: init_point, path, speed_decisions
   * [out]
   */
  void Execute(const SVPoint& stitch_speed_point,
               const SVPoint& ego_speed_point,
               const std::vector<pnc::geometry_lib::PathPoint>& path,
               const SpeedDecisions* speed_decisions);

  const SpeedData& GetSpeedData() const { return speed_data_; }

 private:
  // use sampling method to generate speed.
  // todo: use adaptive acc boundary to generate speed.
  void UpdatePositionTargetSolver(const SVPoint& init_point, const double s_des,
                                  const double v_des);

  bool SamplingAccLowerBound(const planning::jlt::PointState& init_point,
                             const planning::jlt::StateLimitParam& state_limit,
                             jlt::JerkLimitedTrajectory* solver);

  void CopySpeedData(jlt::JerkLimitedTrajectory* solver);

  void DebugJLTSpeed(jlt::JerkLimitedTrajectory* solver,
                     const planning::jlt::StateLimitParam* constraints);

  void GenerateFallbackTraj(const SVPoint& init_point, const double s_des);

  void RecordDebugInfo();

  const bool GenerateJLTSpeed(const SVPoint& init_point, const double s_des,
                              const double v_des);

  void TaskDebug() override;

  void ClearDebugInfo();

  void GenerateStandstillTraj(const SVPoint& init_point, const double s_des);

  void GenerateStartingAndStoppingTraj(const SVPoint& init_point,
                                       const double s_des);

  void GenerateStoppingTraj(const SVPoint& init_point, const double s_des);

  void FallbackTrajByExpectState();

 private:
  SpeedData speed_data_;

  jlt::JerkLimitedTrajectory solver_;

  JerkLimitedTrajConfig config_;

  JLTFailCode fail_code_;
};
}  // namespace apa_planner
}  // namespace planning