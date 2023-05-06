#pragma once

#include <limits.h>

#include <map>

// #include "core/modules/common/config/basic_types.h"
#include "tasks/task.h"
#include "tasks/task_basic_types.h"

namespace planning {

class ResultTrajectoryGenerator : public Task {
 public:
  explicit ResultTrajectoryGenerator(
      const EgoPlanningConfigBuilder *config_builder,
      const std::shared_ptr<TaskPipelineContext> &pipeline_context);
  virtual ~ResultTrajectoryGenerator() = default;

  bool Execute(planning::framework::Frame *frame) override;

  inline bool is_abnormal_number(double number) {
    return (isnan(number) == 1) || (isinf(number) != 0);
  }

 private:
  ResultTrajectoryGeneratorConfig config_;
};

}  // namespace planning
