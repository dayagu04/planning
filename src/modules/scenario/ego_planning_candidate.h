#pragma once
#include <iostream>

#include "frame.h"
#include "common/config/basic_type.h"
#include "context/ego_planning_config.h"
#include "scenario/scenario_state.h"

namespace planning {

class TaskPipelineContext;
class PlanningResult;
class EgoPlanningCandidate {
 public:
  EgoPlanningCandidate(planning::framework::Frame *frame);
  const CoarsePlanningInfo &coarse_planning_info() const {
    return coarse_planning_info_;
  }

  // prepare candidate information
  void set_coarse_planning_info(
      const StateTransitionContext &transition_context);
  void set_last_planning_result(
      const std::shared_ptr<PlanningResult> &planning_result);

  double CalProjectPoint(Eigen::Vector2d &current_pos);

  // check and refine
  bool pre_check();
  void refine(const std::shared_ptr<TaskPipeline> &task_pipeline);

  // planning result
  bool success() const { return success_; }
  const PlanningResult &planning_result() const { return planning_result_; }

  void copy_to_planning_context();

  int id() const { return coarse_planning_info_.target_lane_id; }

 protected:
  void interpolate_with_last_trajectory_points();

 private:
  CoarsePlanningInfo coarse_planning_info_;
  std::shared_ptr<PlanningResult> last_planning_result_ =
      nullptr;
  std::shared_ptr<TaskPipelineContext> pipeline_context_ = nullptr;

  bool success_ = false;
  PlanningResult planning_result_;
  ResultTrajectoryType result_trajectory_type_;
  planning::framework::Frame *frame_;
  EgoPlanningCandidateConfig config_;
};

using EgoPlanningCandidates = std::vector<EgoPlanningCandidate>;

}  // namespace planning
