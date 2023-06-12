#pragma once

#include <cstdint>
#include <memory>
#include <vector>

namespace planning {
struct MSDActuatorMeta {
  uint64_t timestamp_us;
};

enum MSDTurnSignalAvailable {
  MSD_TURN_LIGHT_STATE = 1 << 0,
};

enum MSDTurnSignal {
  MSD_TURN_SIGNAL_NONE = 0,
  MSD_TURN_SIGNAL_LEFT = 1,
  MSD_TURN_SIGNAL_RIGHT = 2,
  MSD_TURN_SIGNAL_EMERGENCY_FLASHER = 3,
};

struct MSDTurnState {
  uint8_t available;
  MSDTurnSignal control_format;
};

enum MSDThrottleAvailable {
  MSD_THROTTLE_VALUE_PEDAL = 1 << 0,
};

struct MSDThrottle {
  uint8_t available;
  double value_pedal;  // 0.0 - 1.0
};

enum MSDBrakeAvailable {
  MSD_BRAKE_VALUE_PEDAL = 1 << 0,
};

struct MSDBrake {
  uint8_t available;
  double value_pedal;  // 0.0 - 1.0
};

enum MSDSteerAvailable {
  MSD_STEER_VALUE_ANGLE = 1 << 0,
};

struct MSDSteer {
  uint8_t available;
  double value_angle;  // [-470, 470]
};

struct MSDActuator {
  MSDActuatorMeta meta;
  MSDThrottle throttle;
  MSDBrake brake;
  MSDSteer steer;
  MSDTurnState turn_state;
};
}  // namespace planning
