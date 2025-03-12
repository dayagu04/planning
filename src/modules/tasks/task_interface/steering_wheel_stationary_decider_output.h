#pragma once

namespace planning {
struct SteeringWheelStationaryDeciderOutput {
  bool is_need_steering_wheel_stationary = false;
  double target_steering_angle = 0.0;  // rad
  void Clear() {
    is_need_steering_wheel_stationary = false;
    target_steering_angle = 0.0;
  }
};
}  // namespace planning