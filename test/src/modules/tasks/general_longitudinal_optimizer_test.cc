#include "motion_planners/longitudinal_motion_planner/pwj_longitudinal_motion_planner.h"

#include <array>
#include <cmath>

#include "gtest/gtest.h"

namespace planning {

TEST(TestLongMotion, longitudinal_motion_planner) {
  auto kk = 10.0;
  printf("TestLongMotion: longitudinal_motion_planner");
  EgoPlanningConfigBuilder *config_builder;
  framework::Session *session;

  auto long_motion_planner_ptr =
      std::make_shared<LongitudinalOptimizerV3>(config_builder);

  long_motion_planner_ptr->Execute(session);
}
}  // namespace planning