#pragma once

namespace planning {

/**
 * @brief mrc request type from mff
 */
typedef enum {
  NO_RQT,
  PULL_OVER_RQT,
  INLANE_BRAKE_RQT,
  BLIND_BRAKE_RQT
} MrcRequestType;

/**
 * @brief mrc request type given to mff
 */
typedef enum {
  NOT_EXECUTE,
  PULL_OVER,
  INLANE_BRAKE,
  BLIND_BRAKE
} MrcExecuteType;

/**
 * @brief mrc brake type
 */
typedef enum {
  NOT_BRAKE,
  SLOW_BRAKE,
  HARD_BRAKE,
  EMERGENCY_BRAKE
} MrcBrakeType;

/**
 * @brief mrc config
 */
struct MrcConfig {
  static constexpr double lc_finish_dist_threshold{0.65};
};

}  // namespace planning