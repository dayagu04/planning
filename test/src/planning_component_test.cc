#include "src/planning_component.h"

#include <array>
#include <cmath>

#include "gtest/gtest.h"
#include "src/common/Platform_Types.h"
#include "src/modules/common/config_context.h"
#include "src/modules/general_planning.h"

namespace planning {

TEST(GeneralPlanning, RunOnce) {
  LocalView local_view_;
  PlanningOutput::PlanningOutput planning_output;
  DebugOutput debug_output;

  std::unique_ptr<GeneralPlanning> planning_base =
      std::make_unique<GeneralPlanning>();
  std::cout << "==============The planning enters RunOnce============="
            << std::endl;
  planning_base->RunOnce(local_view_, &planning_output, debug_output);
}
}  // namespace planning