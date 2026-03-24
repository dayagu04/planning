#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "agent/agent.h"
#include "behavior_planners/lat_lon_joint_planner_decider/lat_lon_joint_planner_decider_output.h"
#include "session.h"
#include "utils/kd_path.h"
#include "utils/path_point.h"

enum LongitudinalLabel {
  IGNORE = 0,
  OVERTAKE = 1,
  YIELD = 2,
};

namespace planning {

struct KeyObstacle {
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
  agent::AgentType type;
};

class JointMotionObstaclesSelector {
 public:
  JointMotionObstaclesSelector(framework::Session* session);
  ~JointMotionObstaclesSelector() = default;

  void SelectObstacles(
      const std::vector<JointPlannerTrajectoryPoint>& prior_trajectory,
      int32_t lead_one_id);

  struct EgoTrajectoryFramet {
    std::vector<double> min_s_vec;
    std::vector<double> max_s_vec;
    std::vector<double> min_l_vec;
    std::vector<double> max_l_vec;
    std::vector<double> vel_vec;
  };

  bool JudgeOverlapWithPriorTrajectory(
      const KeyObstacle& key_obstacle, const double agent_l,
      const PlanningInitPoint init_point,
      const std::shared_ptr<planning_math::KDPath>& planned_path,
      const std::vector<JointPlannerTrajectoryPoint>& prior_trajectory,
      const EgoTrajectoryFramet& ego_frenet_data, bool* is_in_front = nullptr);

  EgoTrajectoryFramet PrecomputeEgoTrajectoryFrenet(
      const std::vector<JointPlannerTrajectoryPoint>& prior_trajectory,
      const std::shared_ptr<planning_math::KDPath>& planned_path);

  std::vector<std::shared_ptr<KeyObstacle>> GetKeyObstacles() const;

 private:

  KeyObstacle* CreateKeyObstacle(
      const std::shared_ptr<agent::Agent>& agent,
      const std::shared_ptr<planning_math::KDPath>& ego_lane_coord,
      double ego_s, double ego_l,
      LongitudinalLabel longitudinal_label = IGNORE);

  void CalculateAgentSLBoundary(
      const std::shared_ptr<planning_math::KDPath>& planned_path,
      const planning_math::Box2d& agent_box, double* const ptr_min_s,
      double* const ptr_max_s, double* const ptr_min_l,
      double* const ptr_max_l);

  void CorrectTrajectoryInConfluenceArea(std::shared_ptr<KeyObstacle>& key_obstacle,
                                         bool is_left_side);

  framework::Session* session_;
  std::vector<std::shared_ptr<KeyObstacle>> key_obstacles_;
  std::vector<std::shared_ptr<agent::Agent>> surrounding_agents_;
};

}  // namespace planning
