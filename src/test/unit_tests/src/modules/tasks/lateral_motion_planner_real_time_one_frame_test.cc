#include <array>
#include <cmath>

#include "framework/session.h"
#include "gtest/gtest.h"
#include "src/modules/tasks/motion_planners/general_lateral_motion_planner/lateral_motion_planner_real_time.h"
#include "src/modules/tasks/task_pipeline_context.h"

namespace planning {

TEST(TestLatMotionPlannerRealTime, LateralMotionPlannerV1) {
  std::string log_file = "/home/ros/asw/Planning/log/planning_log";
  bst::Log::getInstance().setConfig("Planning_Log", log_file.c_str(),
                                    bst::DEBUG);
  auto kk = 10.0;

  printf("TestLongMotion: general_lateral_motion_planner");

  EgoPlanningConfigBuilder *config_builder;
  framework::Frame *frame;
  std::shared_ptr<TaskPipelineContext> pipeline_context;

  auto lat_motion_planner_ptr = std::make_shared<LateralMotionPlannerV1>(
      config_builder, pipeline_context);

  lat_motion_planner_ptr->Execute(frame);
}
}  // namespace planning