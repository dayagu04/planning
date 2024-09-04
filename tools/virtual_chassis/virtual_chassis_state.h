#pragma once
#include <cmath>

namespace planning {

enum class ChassisGearMode {
  park = 0,
  normal,
  drive,
  reverse
};

enum class ChassisControlMode {
  none = 0,
  speed,
  acc,
};

// 虚拟底盘的实现，在收到指令后，零延时响应：方向盘、油门。
struct ChassisState {
  /* data */
  double x_;
  double y_;
  double z_;
  // -pi, pi
  double heading_;

  // if forward, v is positive; if backward driving, v is negative
  double v_;
  double acc_;
  // -pi, +pi, front wheel angle
  // todo: left turn is positive, keep same with apollo.
  double steer_;

  ChassisGearMode gear_;
};

struct ChassisCommand {
  /* data */
  // front wheel, -pi, +pi. left turn is positive value
  double steer_;
  // if speed up, acc is positive, or else acc is negative
  double acc_;
  // if forward, v is positive; if backward driving, v is negative
  double v_;

  ChassisGearMode gear_;
  ChassisControlMode control_mode_;
};

// 误差模型
enum class ChassisErrorMode {
  perfect_response,
  time_delay,
  time_delay_and_overshoot,
};

}  // namespace planning