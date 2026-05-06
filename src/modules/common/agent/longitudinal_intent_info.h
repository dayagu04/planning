#pragma once

namespace planning {
namespace longitudinal_intention {

enum class LongitudinalIntent {
  DECEL = 0,
  CRUISE = 1,
  ACCEL = 2,
  UNKNOWN = 3,
};

struct LongitudinalIntentInfo {
  LongitudinalIntent intent = LongitudinalIntent::UNKNOWN;
  double decel_prob = 0.0;
  double cruise_prob = 0.0;
  double accel_prob = 0.0;
};

}  // namespace longitudinal_intention
}  // namespace planning
