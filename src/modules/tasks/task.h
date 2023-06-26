#pragma once

#include <assert.h>
#include <math.h>

#include <memory>
#include <string>
#include <vector>

#include "config_context.h"
#include "ego_planning_config.h"
#include "frame.h"
#include "task_pipeline_context.h"

namespace planning {

enum TaskType {
  LATERAL_DECIDER,
  GENERAL_LATERAL_DECIDER,
  LATERAL_MOTION_PLANNER,
  LONGITUDINAL_MOTION_PLANNER,
  VISION_LATERAL_MOTION_PLANNER,
  GENERAL_LONGITUDINAL_DECIDER,
  PWJ_LONGITUDINAL_MOTION_PLANNER,
  VISION_ONLY_LONGITUDINAL_BEHAVIOR_PLANNER,
  RESULT_TRAJECTORY_GENERATOR,
  ADAS_FUNCTION_TASK,
};
using PlanningTaskTypes = std::vector<TaskType>;

class Task {
 public:
  explicit Task(const EgoPlanningConfigBuilder *config_builder,
                const std::shared_ptr<TaskPipelineContext> &pipeline_context);

  virtual ~Task() = default;

  const std::string &Name() const { return name_; }

  // 虚函数，基类本身也有自己的函数实体
  virtual bool Execute(framework::Frame *frame);

  static std::shared_ptr<Task> Make(
      const TaskType &task_type, const EgoPlanningConfigBuilder *config_builder,
      const std::shared_ptr<TaskPipelineContext> &pipeline_context);

 protected:
  // PlanningContext *pnc_context_ = nullptr; // 删除此变量
  std::shared_ptr<TaskPipelineContext> pipeline_context_;
  std::shared_ptr<ReferencePath> reference_path_ptr_ = nullptr;
  std::shared_ptr<FrenetCoordinateSystem> frenet_coord_ =
      nullptr;  // 这个是否还需要？
  std::string name_{};
  // int planning_loop_; // 没有被使用到
  VehicleParam vehicle_param_;  // 这个是否还需要？应该去掉
  framework::Frame *frame_;
};

}  // namespace planning
