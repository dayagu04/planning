#pragma once

#include <string>
#include "pose2d.h"

namespace planning {

enum class RSPathRequestType {
  NONE,
  ALL_PATH_FORBID_REVERSE,
  ALL_PATH_FORBID_FORWARD,
  FIRST_PATH_FORBID_REVERSE,
  FIRST_PATH_FORBID_FORWARD,
  LAST_PATH_FORBID_FORWARD,
  LAST_PATH_FORBID_REVERSE,
  FORBID_GEAR_CHANGE,
  // gear switch < 2,
  GEAR_SWITCH_LESS_THAN_TWICE,
  MAX_NUM
};

#define RS_REQUST_MAX_NUM (8)
struct RSPathRequest {
  size_t request_size;
  RSPathRequestType request[RS_REQUST_MAX_NUM];

  // all path segment must bigger than segment_min_length.
  bool enable_segment_length_check;
  double segment_min_length;

  // first path must bigger than first_segment_min_length;
  bool enable_first_segment_len_check;
  double first_segment_min_length;

  Pose2D start;
  Pose2D end;

  double min_radius;
};

std::string GetRSRequestType(const RSPathRequestType type);

}  // namespace planning