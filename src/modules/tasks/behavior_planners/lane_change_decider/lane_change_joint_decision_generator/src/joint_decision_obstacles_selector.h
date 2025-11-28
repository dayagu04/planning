#pragma once

#include <cstdint>
#include <vector>

#include "agent/agent.h"
#include "session.h"
#include "src/modules/tasks/behavior_planners/lane_change_decider/lane_change_joint_decision_generator/lat_lon_joint_decision_output.h"
#include "utils/kd_path.h"
namespace planning {
namespace lane_change_joint_decision {
enum LongitudinalLabel {
  IGNORE = 0,
  OVERTAKE = 1,
  YIELD = 2,
  EGO_OVERTAKE = 3,
};

struct LaneChangeKeyObstacle {
  int32_t agent_id;
  LongitudinalLabel longitudinal_label;
  double length;
  double width;
  std::vector<double> ref_x_vec;
  std::vector<double> ref_y_vec;
  std::vector<double> ref_theta_vec;
  std::vector<double> ref_delta_vec;
  std::vector<double> ref_vel_vec;
  std::vector<double> ref_acc_vec;
  std::vector<double> ref_s_vec;

  double init_x;
  double init_y;
  double init_theta;
  double init_delta;
  double init_vel;
  double init_acc;
  double init_s;
  double init_l;
};

class JointDecisionObstaclesSelector {
 public:
  JointDecisionObstaclesSelector(framework::Session* session);
  ~JointDecisionObstaclesSelector() = default;

  void SelectObstacles(
      const std::vector<JointDecisionTrajectoryPoint>& prior_trajectory,
      int32_t lead_one_id);

  void SelectLaneChangeObstacles(
      const std::vector<JointDecisionTrajectoryPoint>& prior_trajectory,
      const LaneChangeDecisionInfo& lc_info);

  bool JudgeOverlapWithPriorTrajectory(
      const std::shared_ptr<agent::Agent>& agent, const double agent_l,
      const PlanningInitPoint init_point,
      const std::shared_ptr<planning_math::KDPath>& planned_path,
      const std::vector<JointDecisionTrajectoryPoint>& prior_trajectory,
      bool* is_in_front = nullptr);

  std::vector<LaneChangeKeyObstacle> GetKeyObstacles() const;
  // 检查后车行为
  void UpdateRearAgentConfidence(const std::shared_ptr<agent::Agent>& agent);
  bool ShouldIgnoreRearAgent(
      const std::shared_ptr<agent::Agent>& agent,
      const std::shared_ptr<ReferencePath>& ego_reference_path);

 private:
  LaneChangeKeyObstacle CreateKeyObstacle(
      const std::shared_ptr<agent::Agent>& agent,
      const std::shared_ptr<planning_math::KDPath>& ego_lane_coord,
      LongitudinalLabel longitudinal_label = IGNORE);
  void CalculateAgentSLBoundary(
      const std::shared_ptr<planning_math::KDPath>& planned_path,
      const planning_math::Box2d& agent_box, double* const ptr_min_s,
      double* const ptr_max_s, double* const ptr_min_l,
      double* const ptr_max_l);

  framework::Session* session_;
  std::vector<LaneChangeKeyObstacle> key_obstacles_;
  std::vector<std::shared_ptr<agent::Agent>> surrounding_agents_;
  // 记录后车id
  int rear_agent_id_;
  // 记录后车运动趋势 置信度
  double rear_agent_confidence_{1.0};
};

}  // namespace lane_change_joint_decision
}  // namespace planning