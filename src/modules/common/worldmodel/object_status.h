#pragma once
#include "../common/common.h"

namespace planning {

struct MSDObjectStatus {
  float const_velocity_confidence;
  float const_acceleration_confidence;
  float still_confidence;
  float turn_confidence;
};

} // namespace planning
