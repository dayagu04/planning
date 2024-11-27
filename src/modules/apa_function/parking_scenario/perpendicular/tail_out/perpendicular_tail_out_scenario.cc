#include "perpendicular_tail_out_scenario.h"

namespace planning {

void PerpendicularTailOutScenario::Init() {
  if (init_) {
    return;
  }

  init_ = true;
  return;
}

}  // namespace planning
