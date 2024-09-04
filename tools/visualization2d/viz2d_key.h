#pragma once

#include <iostream>
#include <memory>

#include "opencv_viz.h"

namespace planning {

struct cv_direction_key {
  bool left;
  bool right;
  bool up;
  bool low;
};

void cv_key_direction_init(cv_direction_key* key);

bool has_cv_direction_key(cv_direction_key* key);
}  // namespace planning