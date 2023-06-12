#include "task.h"

#include "behavior_planners/general_lateral_decider/general_lateral_decider.h"
#include "behavior_planners/general_longitudinal_decider/general_longitudinal_decider.h"
#include "behavior_planners/vision_only_lateral_behavior_planner/vision_lateral_behavior_planner.h"
#include "behavior_planners/vision_only_longitudinal_behavior_planner/vision_longitudinal_behavior_planner.h"
#include "motion_planners/lateral_motion_planner/lateral_motion_planner.h"
#include "motion_planners/longitudinal_motion_planner/longitudinal_motion_planner.h"
#include "motion_planners/longitudinal_motion_planner/pwj_longitudinal_motion_planner.h"
#include "motion_planners/vision_only_lateral_motion_planner/lateral_motion_planner_real_time.h"
#include "trajectory_generator/result_trajectory_generator.h"
#include "vehicle_config_context.h"

namespace planning {

Task::Task(const EgoPlanningConfigBuilder *config_builder, const std::shared_ptr<TaskPipelineContext> &pipeline_context)
    : pipeline_context_(pipeline_context),
      vehicle_param_(VehicleConfigurationContext::Instance()->get_vehicle_param()) {}

bool Task::Execute(framework::Frame *frame) {
  frame_ = frame;
  reference_path_ptr_ = pipeline_context_->reference_path_ptr;
  // planning_loop_ = pipeline_context_->status_info.planning_loop; //
  // 没有被使用到
  if (reference_path_ptr_ != nullptr) {
    frenet_coord_ = reference_path_ptr_->get_frenet_coord();
  }
  auto &frenet_ego_state = reference_path_ptr_->get_frenet_ego_state();

  // if (reference_path_ptr_ == nullptr or frenet_coord_ == nullptr or
  //     not frenet_ego_state.planning_init_point_valid()) {
  // hack: 目前planning_init_point校验有问题
  if (reference_path_ptr_ == nullptr or frenet_coord_ == nullptr) {
    printf(
        "baseline_info_ is %d, frenet_coord_ is %d, planning_init_point_valid "
        "is %d",
        int(reference_path_ptr_ != nullptr), int(frenet_coord_ != nullptr),
        int(frenet_ego_state.planning_init_point_valid()));
    return false;
  } else {
    return true;
  }
}

std::shared_ptr<Task> Task::Make(const TaskType &task_type, const EgoPlanningConfigBuilder *config_builder,
                                 const std::shared_ptr<TaskPipelineContext> &pipeline_context) {
  switch (task_type) {
    case TaskType::GENERAL_LATERAL_DECIDER: {
      return std::make_shared<GeneralLateralDecider>(config_builder, pipeline_context);
    }

    // ilqr based lateral motion planner
    case TaskType::LATERAL_MOTION_PLANNER: {
      return std::make_shared<LateralMotionPlanner>(config_builder, pipeline_context);
    }

    // ilqr based longitudinal motion planner
    case TaskType::LONGITUDINAL_MOTION_PLANNER: {
      return std::make_shared<LongitudinalMotionPlanner>(config_builder, pipeline_context);
    }

    case TaskType::LATERAL_DECIDER: {
      return std::make_shared<VisionLateralBehaviorPlanner>(config_builder, pipeline_context);
    }
    case TaskType::VISION_LATERAL_MOTION_PLANNER: {
      return std::make_shared<VisionLateralMotionPlanner>(config_builder, pipeline_context);
    }
    // case TaskType::LATERAL_OPTIMIZER_V2: {
    //   return std::make_shared<LateralOptimizerV2>(config_builder,
    //                                               pipeline_context);
    // }
    case TaskType::GENERAL_LONGITUDINAL_DECIDER: {
      return std::make_shared<GeneralLongitudinalDecider>(config_builder, pipeline_context);
    }
    case TaskType::PWJ_LONGITUDINAL_MOTION_PLANNER: {
      return std::make_shared<LongitudinalOptimizerV3>(config_builder, pipeline_context);
    }

    case TaskType::VISION_ONLY_LONGITUDINAL_BEHAVIOR_PLANNER: {
      return std::make_shared<VisionLongitudinalBehaviorPlanner>(config_builder, pipeline_context);
    }

    case TaskType::RESULT_TRAJECTORY_GENERATOR: {
      return std::make_shared<ResultTrajectoryGenerator>(config_builder, pipeline_context);
    }
    default: { /*LOG_ERROR*/
      return nullptr;
    }
  }
  return nullptr;
}

}  // namespace planning