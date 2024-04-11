#include "tasks/behavior_planners/general_longitudinal_decider/general_longitudinal_decider.h"

#include <array>
#include <cmath>

#include "gtest/gtest.h"
#include "log.h"

namespace planning {

TEST(TestLongBehavior, general_longitudinal_decider) {
  std::string log_file = "/asw/Planning/log/planning_log";
  bst::Log::getInstance().setConfig("Planning_Log", log_file.c_str(),
                                    bst::DEBUG);

  auto kk = 10.0;
  printf("TestLongBehavior: general_longitudinal_decider");
  EgoPlanningConfigBuilder *config_builder;
  framework::Session *session

      auto long_behavior_planner_ptr =
          std::make_shared<GeneralLongitudinalDecider>(config_builder);

  long_behavior_planner_ptr->Execute(session);
}
}  // namespace planning