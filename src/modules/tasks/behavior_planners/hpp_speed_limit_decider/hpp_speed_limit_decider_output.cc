#include "hpp_speed_limit_decider_output.h"

namespace planning {

bool HPPSpeedLimitDeciderOutput::GetSpeedLimit(
    double* const limited_speed,
    SpeedLimitType* const speed_limit_type) const {
  (*speed_limit_type) = hpp_speed_limit_type_final_.first;
  (*limited_speed) = hpp_speed_limit_type_final_.second;

  return true;
}

void HPPSpeedLimitDeciderOutput::SetSpeedLimit(
    const double limited_speed, const SpeedLimitType& speed_limit_type) {
  hpp_speed_limit_type_final_.first = speed_limit_type;
  hpp_speed_limit_type_final_.second = limited_speed;
}

bool HPPSpeedLimitDeciderOutput::GetSpeedLimitByType(
    const SpeedLimitType& speed_limit_type, double* const limited_speed) const {
  const auto& speed_limit_info = hpp_speed_limit_map_.find(speed_limit_type);
  if (speed_limit_info != hpp_speed_limit_map_.end()) {
    (*limited_speed) = speed_limit_info->second;
    return true;
  } else {
    return false;
  }
}

void HPPSpeedLimitDeciderOutput::SetSpeedLimitIntoMap(
    const double limited_speed, const SpeedLimitType& speed_limit_type) {
  if (hpp_speed_limit_map_.find(speed_limit_type) == hpp_speed_limit_map_.end()) {
    hpp_speed_limit_map_.insert(std::make_pair(speed_limit_type, limited_speed));
  } else {
    hpp_speed_limit_map_[speed_limit_type] = limited_speed;
  }
}
}  // namespace planning
