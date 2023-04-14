#include "src/modules/tasks/behavior_planners/general_longitudinal_decider/general_longitudinal_decider.h"
#include "src/modules/tasks/task_pipeline_context.h"

#include <array>
#include <cmath>

#include "gtest/gtest.h"

namespace planning {

TEST(TestLongBehavior, general_longitudinal_decider) {
  auto kk = 10.0;
  printf("TestLongBehavior: general_longitudinal_decider");
  EgoPlanningConfigBuilder *config_builder;
  framework::Frame *frame;
  std::shared_ptr<TaskPipelineContext> pipeline_context;

  auto long_behavior_planner_ptr = std::make_shared<GeneralLongitudinalDecider>(
      config_builder, pipeline_context);

  long_behavior_planner_ptr->Execute(frame);


}
}  // namespace planning