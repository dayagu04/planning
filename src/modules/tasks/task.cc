#include "task.h"

#include "tasks/behavior_planners/gap_selector_decider/gap_selector_decider.h"
#include "tasks/behavior_planners/general_lateral_decider/general_lateral_decider.h"
#include "tasks/behavior_planners/general_longitudinal_decider/general_longitudinal_decider.h"
#include "tasks/behavior_planners/hpp_general_lateral_decider/hpp_general_lateral_decider.h"
#include "tasks/behavior_planners/lateral_obstacle_decider/lateral_obstacle_decider.h"
#include "tasks/behavior_planners/lateral_offset_decider/lateral_offset_decider.h"
#include "tasks/behavior_planners/scc_lon_behavior_planner/scc_lon_behavior_planner.h"
#include "tasks/behavior_planners/vision_only_lateral_behavior_planner/vision_lateral_behavior_planner.h"
#include "tasks/behavior_planners/vision_only_longitudinal_behavior_planner/vision_longitudinal_behavior_planner.h"
#include "tasks/motion_planners/lateral_motion_planner/lateral_motion_planner.h"
#include "tasks/motion_planners/longitudinal_motion_planner/longitudinal_motion_planner.h"
#include "tasks/motion_planners/longitudinal_motion_planner/pwj_longitudinal_motion_planner.h"
#include "tasks/motion_planners/scc_lon_motion_planner/scc_longitudinal_motion_planner.h"
#include "tasks/motion_planners/vision_only_lateral_motion_planner/lateral_motion_planner_real_time.h"
#include "trajectory_generator/result_trajectory_generator.h"
#include "vehicle_config_context.h"

namespace planning {

Task::Task(const EgoPlanningConfigBuilder *config_builder,
           framework::Session *session)
    : session_(session) {}

bool Task::PreCheck() {
  const auto &reference_path = session_->planning_context()
                                   .lane_change_decider_output()
                                   .coarse_planning_info.reference_path;
  if (reference_path == nullptr) {
    return false;
  }

  const auto &frenet_coord = reference_path->get_frenet_coord();
  if (frenet_coord == nullptr) {
    return false;
  }

  return true;
}

}  // namespace planning