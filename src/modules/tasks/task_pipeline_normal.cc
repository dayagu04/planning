#include "task_pipeline_normal.h"

#include "ifly_time.h"
// #include "trace.h"
#include "ego_planning_candidate.h"
#include "task.h"

namespace planning {

TaskPipelineNormal::TaskPipelineNormal(
    const EgoPlanningConfigBuilder *config_builder, framework::Frame *frame)
    : TaskPipeline(config_builder, frame) {
  name_ = "TaskPipelineNormal";
  config_ = config_builder->cast<EgoPlanningTaskPipelineNormalConfig>();
  version_to_tasks_["v1"] = {
      // TaskType::OBSTACLE_DECIDER,
      TaskType::GAP_SELECTOR, TaskType::GENERAL_LATERAL_DECIDER,
      TaskType::LATERAL_MOTION_PLANNER,
      // TaskType::LATERAL_OPTIMIZER_V2,
      TaskType::GENERAL_LONGITUDINAL_DECIDER,
      // TaskType::PWJ_LONGITUDINAL_MOTION_PLANNER,
      TaskType::LONGITUDINAL_MOTION_PLANNER,
      TaskType::RESULT_TRAJECTORY_GENERATOR};
  CreatePlanningTasks(config_builder);
}

bool TaskPipelineNormal::Run(const EgoPlanningCandidate &candidate) {
  // NTRACE_CALL(6);

  auto &coarse_planning_info = candidate.coarse_planning_info();
  auto &reference_path = candidate.coarse_planning_info().reference_path;
  auto &trajectory_points = candidate.coarse_planning_info().trajectory_points;
  if (reference_path == nullptr) {
    LOG_ERROR("reference_path is null! \n");
    return false;
  }
  if (trajectory_points.empty()) {
    LOG_ERROR("trajectory_points is empty! \n");
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

void TaskPipelineNormal::CreatePlanningTasks(
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