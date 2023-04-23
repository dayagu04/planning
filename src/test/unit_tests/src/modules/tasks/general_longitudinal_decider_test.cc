#include "src/modules/tasks/behavior_planners/general_longitudinal_decider/general_longitudinal_decider.h"

#include <array>
#include <cmath>

#include "gtest/gtest.h"
#include "src/modules/tasks/task_pipeline_context.h"
#include "src/common/log.h"

namespace planning {

TEST(TestLongBehavior, general_longitudinal_decider) {
  std::string log_file = "/asw/Planning/log/planning_log";
  bst::Log::getInstance().setConfig("Planning_Log", log_file.c_str(), bst::DEBUG);
  
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