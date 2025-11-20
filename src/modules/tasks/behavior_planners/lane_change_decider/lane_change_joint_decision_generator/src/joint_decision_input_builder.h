#pragma once

#include <cmath>
#include <memory>
#include <vector>

#include "ego_planning_config.h"
#include "joint_decision_obstacles_selector.h"
#include "joint_decision_planning_problem.h"
#include "joint_decision_speed_limit.h"
#include "joint_decision_planner.pb.h"
#include "session.h"
#include "tasks/behavior_planners/lane_change_decider/lane_change_joint_decision_generator/lat_lon_joint_decision_output.h"
namespace planning {
namespace lane_change_joint_decision {
class JointDecisionInputBuilder {
 public:
  JointDecisionInputBuilder(const EgoPlanningConfigBuilder* config_builder,
                            framework::Session* session);
  ~JointDecisionInputBuilder() = default;

  void BuildLaneChangeInput(
      planning::common::JointDecisionPlanningInput& planning_input,
      std::shared_ptr<
          pnc::lane_change_joint_decision::JointDecisionPlanningProblem>
          planning_problem_ptr,
      const LaneChangeDecisionInfo& lc_info);
  void SetObstaclesSelector(
      std::shared_ptr<JointDecisionObstaclesSelector> obstacles_selector);

  const std::vector<double>& GetKeyAgentIds() const { return key_agent_ids_; }

 private:
  void BuildLaneChangeWeightInfo(
      planning::common::JointDecisionPlanningInput& planning_input);
  void BuildLaneChangeEgoInfo(
      planning::common::JointDecisionPlanningInput& planning_input,
      const LaneChangeDecisionInfo& lc_info);

  void BuildObsInfo(planning::common::JointDecisionPlanningInput& planning_input,
                    const LaneChangeDecisionInfo& lc_info);

  void BuildRoadInfo(
      planning::common::JointDecisionPlanningInput& planning_input,
      std::shared_ptr<
          pnc::lane_change_joint_decision::JointDecisionPlanningProblem>
          planning_problem_ptr);

  struct ComfortParameters {
    double v0;
    double s0;
    double T;
    double a;
    double b_max;
    double b;
    double b_hard;
    double delta;
    double max_accel_jerk;
    double max_decel_jerk;
    double cool_factor;
    double eps;
  };

  double CalculateComfortAcceleration(const double current_acc,
                                      const double current_vel,
                                      const double current_s,
                                      const double front_vel,
                                      const double front_s) const;

 private:
  framework::Session* session_;
  JointDecisionPlannerConfig lc_decision_config_;
  std::shared_ptr<JointDecisionObstaclesSelector> obstacles_selector_;
  std::unique_ptr<JointDecisionSpeedLimit> speed_limit_calculator_;
  std::vector<JointDecisionTrajectoryPoint> ref_trajectory_;
  ComfortParameters comfort_params_;

  const std::vector<double> _EGO_VEL_TABLE{0.0,   3.33,  10.0,
                                           16.67, 26.67, 36.67};
  const std::vector<double> _TIME_HEADWAY_TABLE{1.15, 1.15, 1.35,
                                                1.5,  1.75, 2.0};

  static constexpr double kPlanningTimeHorizon = 5.0;
  static constexpr double kPlanningTimeStep = 0.2;
  static constexpr int kPlanningTimeSteps = 26;
  int32 lead_one_id_ = -1;
  std::vector<double> key_agent_ids_;

  double last_tau_ = 1.2;
  int32 last_lead_one_id_ = -1;
  bool is_in_lane_change_last_ = false;
};

}  // namespace lane_change_joint_decision
}  // namespace planning