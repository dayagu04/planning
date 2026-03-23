#pragma once

#include <cmath>
#include <memory>
#include <vector>

#include "ego_planning_config.h"
#include "joint_motion_obstacles_selector.h"
#include "joint_motion_planning_problem.h"
#include "session.h"
#include "tasks/behavior_planners/lat_lon_joint_planner_decider/lat_lon_joint_planner_decider_output.h"

namespace planning {

class JointMotionInputBuilder {
 public:
  JointMotionInputBuilder(const EgoPlanningConfigBuilder* config_builder,
                          framework::Session* session);
  ~JointMotionInputBuilder() = default;

  void BuildInput(
      planning::common::JointMotionPlanningInput& planning_input,
      std::shared_ptr<pnc::joint_motion_planning::JointMotionPlanningProblem>
          planning_problem_ptr);

  void SetObstaclesSelector(
      std::shared_ptr<JointMotionObstaclesSelector> obstacles_selector);

  const std::vector<double>& GetKeyAgentIds() const { return key_agent_ids_; }

 private:
  void BuildEgoAndWeightInfo(
      planning::common::JointMotionPlanningInput& planning_input);

  void BuildObsInfo(planning::common::JointMotionPlanningInput& planning_input);

  void BuildRoadInfo(
      planning::common::JointMotionPlanningInput& planning_input,
      std::shared_ptr<pnc::joint_motion_planning::JointMotionPlanningProblem>
          planning_problem_ptr);

  struct JointTrajParams {
    double s0;
    double T;
    double a;
    double b;
    double b_max;
    double b_hard;
    double delta;
    double max_accel_jerk;
    double min_decel_jerk;
    double max_decel_jerk;
    double cool_factor;
    double default_front_distance;
    double delay_time_buffer;
  };

  double CalculateIdmAcceleration(double current_acc, double current_vel,
                                  double current_s, double front_acc,
                                  double front_vel, double front_s, double tau,
                                  double v0, double decel_jerk) const;

  void GenerateReferenceTrajectory(
      planning::common::JointMotionPlanningInput& planning_input);

 private:
  framework::Session* session_;
  JointMotionPlannerConfig config_;
  SpeedPlannerConfig speed_planning_config_;
  std::shared_ptr<JointMotionObstaclesSelector> obstacles_selector_;
  JointTrajParams joint_traj_params_;
  std::vector<JointPlannerTrajectoryPoint> ref_trajectory_;

  const std::vector<double> _L_SLOPE_BP{0.0, 40.0};
  const std::vector<double> _L_SLOPE_V{0.35, 0.08};
  const std::vector<double> _P_SLOPE_BP{0., 40.0};
  const std::vector<double> _P_SLOPE_V{0.8, 0.2};

  const std::vector<double> _EGO_VEL_TABLE{0.0,   3.33,  10.0,
                                           16.67, 26.67, 36.67};
  const std::vector<double> _TIME_HEADWAY_TABLE{1.15, 1.15, 1.35,
                                                1.5,  1.75, 2.0};

  static constexpr double kPlanningTimeHorizon = 5.0;
  static constexpr double kPlanningTimeStep = 0.2;
  static constexpr int kPlanningTimeSteps = 26;
  int32 lead_one_id_ = -1;
  std::vector<double> key_agent_ids_;
};

}  // namespace planning