#pragma once

#include <memory>
#include <string>

#include "context/planning_context.h"
#include "joint_motion_planner.pb.h"
#include "src/joint_motion_input_builder.h"
#include "src/joint_motion_obstacles_selector.h"
#include "src/joint_motion_planning_problem.h"
#include "tasks/task.h"

namespace planning {
class LatLonJointPlannerDecider : public Task {
 public:
  LatLonJointPlannerDecider(const EgoPlanningConfigBuilder* config_builder,
                            framework::Session* session);
  void Init();
  bool Execute() override;

 private:
  void Update();
  std::string name_;
  std::shared_ptr<pnc::joint_motion_planning::JointMotionPlanningProblem>
      planning_problem_ptr_;
  planning::common::JointMotionPlanningInput planning_input_;
  std::unique_ptr<JointMotionInputBuilder> input_builder_;
  LatLonJointPlannerDeciderOutput lat_lon_planning_output_;
  std::shared_ptr<JointMotionObstaclesSelector> obstacles_selector_;
};
}  // namespace planning