#include "parallel_out_scenario.h"

namespace planning {

void ParallelOutScenario::Init() {
  if (init_) {
    return;
  }

  init_ = true;
  return;
}

}  // namespace planning
