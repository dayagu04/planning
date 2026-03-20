#pragma once

#include <cstdint>
#include <limits>

namespace planning {

struct CipvLostProhibitAccelerationDeciderOutput {
  bool prohibit_acceleration_ = false;
  double speed_limit_ = std::numeric_limits<double>::max();
};

}  // namespace planning
