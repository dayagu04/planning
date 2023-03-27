#pragma once

#include "src/framework/frame.h"
#include "src/framework/session.h"
#include "src/modules/context/reference_path.h"
// #include "src/modules/context/traffic_light_decider.h"
#include "src/modules/tasks/task_basic_types.h"

namespace planning {

struct TaskPipelineContext {
 public:
  void Init(planning::framework::Frame *frame,
            const std::shared_ptr<ReferencePath> &reference_path,
            const TrajectoryPoints &traj_points,
            const CoarsePlanningInfo &coarse_planning_info_) {
    Clear();
    // Step 1) reference path
    reference_path_ptr = reference_path;

    // Step 2) traj points
    planning_result.raw_traj_points = traj_points;
    (void)reference_path_ptr->transform_trajectory_points(
        planning_result.raw_traj_points);
    auto delta_s = reference_path->get_frenet_ego_state()
                       .planning_init_point()
                       .frenet_state.s -
                   planning_result.raw_traj_points.front().s;
    for (auto &pt : planning_result.raw_traj_points) {
      pt.s = pt.s + delta_s;
    }
    for (size_t i = 1; i < planning_result.raw_traj_points.size(); i++) {
      auto &pre_pt = planning_result.raw_traj_points[i - 1];
      auto &pt = planning_result.raw_traj_points[i];
      pt.s = std::fmax(pt.s, pre_pt.s + 0.01);
    }
    planning_result.traj_points = planning_result.raw_traj_points;

    // Step 3) coarse planning info
    coarse_planning_info = coarse_planning_info_;

    // Step 4) target state
    planning_target_state = coarse_planning_info.target_state;

    // Step 5) status info
    status_info = frame->mutable_session()
                      ->mutable_planning_context()
                      ->status_info();

    // Step 6) traffic light decision
    // const auto &traffic_light_decider = frame->mutable_session()
    //                                         ->mutable_planning_context()
    //                                         ->traffic_light_decider();
    // planning_info.traffic_light_decision.stop_flag =
    //     traffic_light_decider->get_stop_flag();
    // planning_info.traffic_light_decision.stop_distance =
    //     traffic_light_decider->get_stop_distance();
    // planning_info.traffic_light_decision.velocity_limit =
    //     traffic_light_decider->get_velocity_limit();
  }

  void Clear() {
    planning_result.raw_traj_points.clear();
    planning_result.traj_points.clear();

    planning_info = {};
    status_info.error_info = {};
    status_info.debug_info = {};
  }

  bool success = false;
  PlanningResult planning_result;
  PlanningInfo planning_info;
  StatusInfo status_info;
  std::shared_ptr<ReferencePath> reference_path_ptr = nullptr;
  ScenarioStateEnum planning_target_state;
  CoarsePlanningInfo coarse_planning_info;
};

}  // namespace planning
