#pragma once

#include <limits.h>

#include <map>

// #include "core/modules/common/config/basic_types.h"
#include "task_basic_types.h"
#include "tasks/task.h"

namespace planning {

class ResultTrajectoryGenerator : public Task {
 public:
  explicit ResultTrajectoryGenerator(
      const EgoPlanningConfigBuilder* config_builder,
      framework::Session* session);
  virtual ~ResultTrajectoryGenerator() = default;

  bool Execute() override;

  bool TrajectoryGenerator();
  bool RealtimeTrajectoryGenerator();

  void Init();

  void UpdateTurnSignal();

  inline bool is_abnormal_number(double number) {
    return (std::isnan(number) == 1) || (std::isinf(number) != 0);
  }

 private:
  ResultTrajectoryGeneratorConfig config_;
};

}  // namespace planning
