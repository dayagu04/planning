#include <array>
#include <cmath>

#include "gtest/gtest.h"
#include "session.h"

namespace planning {

TEST(TestLatMotionPlannerRealTime, VisionLateralMotionPlanner) {
  std::string log_file = "/home/ros/asw/Planning/log/planning_log";
  bst::Log::getInstance().setConfig("Planning_Log", log_file.c_str(),
                                    bst::DEBUG);
  auto kk = 10.0;

  printf("TestLongMotion: lateral_motion_planner");

  EgoPlanningConfigBuilder *config_builder;
  framework::Session *session;

  // auto lat_motion_planner_ptr =
  //     std::make_shared<VisionLateralMotionPlanner>(config_builder);

  // lat_motion_planner_ptr->Execute(session);
}
}  // namespace planning