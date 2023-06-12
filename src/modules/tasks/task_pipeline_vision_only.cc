#include "task_pipeline_vision_only.h"

#include "ifly_time.h"
// #include "trace.h"
#include "ego_planning_candidate.h"
#include "task.h"

namespace planning {

TaskPipelineVisionOnly::TaskPipelineVisionOnly(
    const EgoPlanningConfigBuilder *config_builder, framework::Frame *frame)
    : TaskPipeline(config_builder, frame) {
  name_ = "TaskPipelineVisionOnly";
  config_ = config_builder->cast<EgoPlanningTaskPipelineVisionOnlyConfig>();
  version_to_tasks_["v1"] = {
      // TaskType::OBSTACLE_DECIDER,
      TaskType::LATERAL_DECIDER,
      TaskType::VISION_LATERAL_MOTION_PLANNER,
      TaskType::VISION_ONLY_LONGITUDINAL_BEHAVIOR_PLANNER,
      // TaskType::RESULT_TRAJECTORY_GENERATOR
  };
  CreatePlanningTasks(config_builder);
}

bool TaskPipelineVisionOnly::Run(const EgoPlanningCandidate &candidate) {
  // NTRACE_CALL(6);

  auto &coarse_planning_info = candidate.coarse_planning_info();
  auto &reference_path = candidate.coarse_planning_info().reference_path;
  auto &trajectory_points = candidate.coarse_planning_info().trajectory_points;
  if (reference_path == nullptr) {
    // NTRACE_FAIL("reference_path is null");
    return false;
  }
  if (trajectory_points.empty()) {
    // NTRACE_FAIL("trajectory_points is empty");
    return false;
  }

  pipeline_context_->Init(frame_, reference_path, trajectory_points,
                          coarse_planning_info);

  auto &ego_prediction_status_info = pipeline_context_->status_info;

  double total_start_time = IflyTime::Now_ms();
  for (const auto &task : planning_tasks_) {
    auto start_timestamp = IflyTime::Now_ms();
    if (task.second->Execute(frame_)) {
      auto end_timestamp = IflyTime::Now_ms();
      printf("%s| TASK_DONE =============== TIME_COST: [%f] ms \n",
             task.second->Name().c_str(), (end_timestamp - start_timestamp));
    } else {
      ego_prediction_status_info.error_info =
          "ERROR|" + task.second->Name() + "|" +
          ego_prediction_status_info.error_info;
      printf("%s| FAIL =============== \n", task.second->Name().c_str());
      return false;
    }
  }
  double total_end_time = IflyTime::Now_ms();
  auto total_time_diff = total_end_time - total_start_time;
  // MDEBUG_JSON_ADD_ITEM(ego_prediction_task_latency, total_time_diff,
  //                      ego_prediction_tasks_latency)

  pipeline_context_->success = true;
  return true;
}

void TaskPipelineVisionOnly::CreatePlanningTasks(
    const EgoPlanningConfigBuilder *config_builder) {
  auto version_it = version_to_tasks_.find(config_.pipeline_version);
  assert(version_it != version_to_tasks_.end());
  if (version_it != version_to_tasks_.end()) {
    for (auto &task_type : version_it->second) {
      std::shared_ptr<Task> task_ptr =
          Task::Make(task_type, config_builder, pipeline_context_);
      planning_tasks_.emplace_back(task_type, task_ptr);
    }
  }
}

}  // namespace planning