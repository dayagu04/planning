#pragma once

namespace planning {

enum StaticSteeringStatus {
  kStaticSteeringWait = 0,
  kStaticSteeringExecution,
  kStaticSteeringReady,
  kStaticSteeringComplete,
  kStaticSteeringCancel
};

enum StaticSteeringFailedReason {
  NO_FAILURE = 0,
  NOT_IN_FUNCTION,
  NOT_STATIONARY,
  NO_TARGET_OBSTACLE,
  NO_TURNING_DIRECTION,
  LANE_LINE_TYPE_SOLID,
  STEERING_SAFETY_CHECK_FAILED,
  STEER_ANGLE_INVALID,
};

struct SteeringWheelStationaryDeciderOutput {
  StaticSteeringStatus static_steering_status;
  StaticSteeringFailedReason static_steering_failed_reason;
  bool is_need_steering_wheel_stationary = false;
  bool is_need_start_slowly = false;
  bool is_static_lane_change = false;
  bool is_static_lane_borrow = false;
  bool is_static_lane_avoid = false;
  bool is_need_replan = false;
  double target_steering_angle = 0.0;  // rad
  void Clear() {
    static_steering_status = kStaticSteeringWait;
    static_steering_failed_reason = NO_FAILURE;
    is_need_steering_wheel_stationary = false;
    is_need_start_slowly = false;
    is_static_lane_change = false;
    is_static_lane_borrow = false;
    is_static_lane_avoid = false;
    is_need_replan = false;
    target_steering_angle = 0.0;
  }
};
}  // namespace planning