#include <array>
#include <cmath>

#include "define/debug_output.h"
#include "general_planning.h"
#include "gtest/gtest.h"
#include "local_view.h"
#include "common/config_context.h"

namespace planning {

TEST(GeneralPlanning, RunOnce) {
  LocalView local_view_;
  PlanningOutput::PlanningOutput planning_output;
  DebugOutput debug_output;
  PlanningHMI::PlanningHMIOutputInfoStr planning_hmi_Info;

  std::unique_ptr<GeneralPlanning> planning_base =
      std::make_unique<GeneralPlanning>();
  std::cout << "==============The planning enters RunOnce============="
            << std::endl;
  planning_base->RunOnce(local_view_, &planning_output, debug_output, planning_hmi_Info);
}
}  // namespace planning