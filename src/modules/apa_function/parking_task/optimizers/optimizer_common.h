#pragma once

namespace planning {

enum class SpeedOptimizerState {
  NONE = 0,
  FAIL = 1,
  TIME_OUT = 2,
  MAX_ITERATION = 3,
  SUCCESS = 4,
};
}