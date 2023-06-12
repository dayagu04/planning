#pragma once

#include <limits.h>

#include <map>

// #include "core/modules/common/config/basic_types.h"
#include "task.h"
#include "task_basic_types.h"

namespace planning {

class ResultTrajectoryGenerator : public Task {
 public:
  explicit ResultTrajectoryGenerator(
      const EgoPlanningConfigBuilder *config_builder,
      const std::shared_ptr<TaskPipelineContext> &pipeline_context);
  virtual ~ResultTrajectoryGenerator() = default;

  bool Execute(planning::framework::Frame *frame) override;

  bool GenerateTrajecotry(planning::framework::Frame *frame);
  bool GenerateTrajecotryVisionOnly(planning::framework::Frame *frame);
  void Init();

  inline bool is_abnormal_number(double number) {
    return (isnan(number) == 1) || (isinf(number) != 0);
  }

 private:
  ResultTrajectoryGeneratorConfig config_;
  std::vector<double> t_vec_;
  std::vector<double> s_vec_;
  std::vector<double> l_vec_;
  std::vector<double> curvature_vec_;
  std::vector<double> dkappa_vec_;
  std::vector<double> ddkappa_vec_;

};

}  // namespace planning
