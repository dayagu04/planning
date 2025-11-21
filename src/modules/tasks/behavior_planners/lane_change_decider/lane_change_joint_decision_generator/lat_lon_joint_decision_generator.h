#pragma once

#include <memory>
#include <string>

#include "context/planning_context.h"
#include "joint_decision_planner.pb.h"
#include "lat_lon_joint_decision_output.h"
#include "src/joint_decision_input_builder.h"
#include "src/joint_decision_obstacles_selector.h"
#include "src/joint_decision_planning_problem.h"
#include "tasks/task.h"

namespace planning {
class LatLonJointDecision : public Task {
 public:
  LatLonJointDecision(const EgoPlanningConfigBuilder* config_builder,
                      framework::Session* session);
  void Init();
  bool Execute() override;
  void SetLaneChangeDecisionInfo(
      const lane_change_joint_decision::LaneChangeDecisionInfo& lc_info);
  void ClearLaneChangeDecisionInfo();
  void SetAgentTrajecTory();

 private:
  void Update();
  std::string name_;
  std::shared_ptr<pnc::lane_change_joint_decision::JointDecisionPlanningProblem>
      planning_problem_ptr_;
  planning::common::JointDecisionPlanningInput planning_input_;
  std::unique_ptr<
      planning::lane_change_joint_decision::JointDecisionInputBuilder>
      input_builder_;
  lane_change_joint_decision::LatLonJointDecisionOutput
      lat_lon_decision_output_;
  std::shared_ptr<
      planning::lane_change_joint_decision::JointDecisionObstaclesSelector>
      obstacles_selector_;
  lane_change_joint_decision::LaneChangeDecisionInfo lc_prior_info_;
};
}  // namespace planning
