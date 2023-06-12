#pragma once

#include "common.h"

namespace planning {

enum ObjectSourceEnum {
OBJECT_SOURCE_VISION = 0,
OBJECT_SOURCE_LIDAR = 1,
OBJECT_SOURCE_RADAR = 2,
OBJECT_SOURCE_ULTRASONIC = 3,
};

struct ObjectSourceData {
ObjectSourceEnum object_source;
u_int64_t track_id;
std::string reserved_info;
};

}  // namespace planning